import CJolt
import Foundation
import simd
import Synchronization

// MARK: - JoltEngine

public struct JoltInitializationError: Error {}

/// Owns Jolt's global state. Jolt is initialized while at least one engine is alive.
public final class JoltEngine: Sendable {
    private static let liveCount = Mutex(0)

    // joltc keeps a single global TempAllocator, which is not thread safe. Every call that can
    // use it (PhysicsSystem update, CharacterVirtual update) must hold this lock, even across
    // separate worlds.
    private static let tempAllocatorLock = Mutex(())

    static func withTempAllocator<Result>(_ body: () -> Result) -> Result {
        tempAllocatorLock.withLock { _ in body() }
    }

    public init() throws(JoltInitializationError) {
        let succeeded = Self.liveCount.withLock { count in
            if count == 0 {
                guard JPH_Init() else {
                    return false
                }
            }
            count += 1
            return true
        }
        guard succeeded else {
            throw JoltInitializationError()
        }
    }

    deinit {
        Self.liveCount.withLock { count in
            count -= 1
            if count == 0 {
                JPH_Shutdown()
            }
        }
    }
}

// MARK: - Shape

public enum Shape: Sendable {
    case box(halfExtents: SIMD3<Float>, convexRadius: Float = 0.05)
    case sphere(radius: Float)
    /// A capsule along the Y axis. Total height is `2 * halfHeightOfCylinder + 2 * radius`.
    case capsule(halfHeightOfCylinder: Float, radius: Float)
    /// A capsule along the Y axis with different end radii. The sphere centers are at
    /// `y = ±halfHeight`.
    case taperedCapsule(halfHeight: Float, topRadius: Float, bottomRadius: Float)
    /// A cylinder along the Y axis.
    case cylinder(halfHeight: Float, radius: Float)
    /// A triangle mesh. `indices` holds three vertex indices per triangle, counter-clockwise when
    /// seen from the front. Jolt only supports mesh shapes on static and kinematic bodies.
    case mesh(vertices: [SIMD3<Float>], indices: [UInt32])
    /// A square grid of heights. `samples` has `sampleCount * sampleCount` entries, row by row along Z.
    /// Sample `(x, z)` sits at local position `offset + scale * (x, samples[z * sampleCount + x], z)`.
    /// `sampleCount` must be a multiple of 2. Static and kinematic bodies only.
    case heightField(samples: [Float], sampleCount: Int, offset: SIMD3<Float> = .zero, scale: SIMD3<Float> = .one)
    /// An infinite-looking ground plane: the surface where `dot(normal, p) == offset`, solid on the
    /// side opposite `normal`. `halfExtent` bounds it for the broad phase. Static bodies only.
    case plane(normal: SIMD3<Float> = [0, 1, 0], offset: Float = 0, halfExtent: Float = 1000)
    /// A single triangle. Static and kinematic bodies only.
    case triangle(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>)
    /// The convex hull of `points`. Needs at least 4 points that are not all on one plane.
    case convexHull(points: [SIMD3<Float>], convexRadius: Float = 0.05)
    /// Several shapes combined into one rigid shape. Cannot change after creation.
    case compound([CompoundChild])
    /// A compound whose children can be added, removed and moved after the body exists, with
    /// `PhysicsWorld.addCompoundChild` and related methods. Slower to simulate than `.compound`.
    case mutableCompound([CompoundChild])

    func createHandle() -> OpaquePointer {
        switch self {
        case .box(let halfExtents, let convexRadius):
            var he = JPH_Vec3(halfExtents)
            return JPH_BoxShape_Create(&he, convexRadius)!
        case .sphere(let radius):
            return JPH_SphereShape_Create(radius)!
        case .capsule(let halfHeightOfCylinder, let radius):
            return JPH_CapsuleShape_Create(halfHeightOfCylinder, radius)!
        case .taperedCapsule(let halfHeight, let topRadius, let bottomRadius):
            // Jolt refuses equal radii; that is a plain capsule.
            if topRadius == bottomRadius {
                return JPH_CapsuleShape_Create(halfHeight, topRadius)!
            }
            let settings = JPH_TaperedCapsuleShapeSettings_Create(halfHeight, topRadius, bottomRadius)!
            defer { JPH_ShapeSettings_Destroy(settings) }
            guard let shape = JPH_TaperedCapsuleShapeSettings_CreateShape(settings) else {
                preconditionFailure("Invalid tapered capsule: halfHeight \(halfHeight), radii \(topRadius)/\(bottomRadius)")
            }
            return shape
        case .cylinder(let halfHeight, let radius):
            return JPH_CylinderShape_Create(halfHeight, radius)!
        case .mesh(let vertices, let indices):
            precondition(indices.count.isMultiple(of: 3), "Mesh indices must come in groups of three")
            precondition(indices.allSatisfy { Int($0) < vertices.count }, "Mesh index out of range")
            let jphVertices = vertices.map(JPH_Vec3.init)
            let triangles = stride(from: 0, to: indices.count, by: 3).map { index in
                JPH_IndexedTriangle(i1: indices[index], i2: indices[index + 1], i3: indices[index + 2], materialIndex: 0, userData: 0)
            }
            let settings = JPH_MeshShapeSettings_Create2(jphVertices, UInt32(jphVertices.count), triangles, UInt32(triangles.count))!
            defer { JPH_ShapeSettings_Destroy(settings) }
            JPH_MeshShapeSettings_Sanitize(settings)
            return JPH_MeshShapeSettings_CreateShape(settings)!
        case .heightField(let samples, let sampleCount, let offset, let scale):
            precondition(samples.count == sampleCount * sampleCount, "Height field needs sampleCount * sampleCount samples")
            precondition(sampleCount >= 2 && sampleCount.isMultiple(of: 2), "Height field sampleCount must be a multiple of 2")
            var jphOffset = JPH_Vec3(offset)
            var jphScale = JPH_Vec3(scale)
            let settings = JPH_HeightFieldShapeSettings_Create(samples, &jphOffset, &jphScale, UInt32(sampleCount), nil)!
            defer { JPH_ShapeSettings_Destroy(settings) }
            return JPH_HeightFieldShapeSettings_CreateShape(settings)!
        case .plane(let normal, let offset, let halfExtent):
            // Jolt stores the plane as dot(normal, p) + constant == 0.
            var plane = JPH_Plane(normal: simd_normalize(normal), distance: -offset)
            return JPH_PlaneShape_Create(&plane, nil, halfExtent)!
        case .triangle(let v1, let v2, let v3):
            var a = JPH_Vec3(v1)
            var b = JPH_Vec3(v2)
            var c = JPH_Vec3(v3)
            // Jolt's convex radius had no visible effect on ray casts, shape casts or resting
            // contacts in testing, so it is not exposed.
            return JPH_TriangleShape_Create(&a, &b, &c, 0)!
        case .convexHull(let points, let convexRadius):
            let jphPoints = points.map(JPH_Vec3.init)
            let settings = JPH_ConvexHullShapeSettings_Create(jphPoints, UInt32(jphPoints.count), convexRadius)!
            defer { JPH_ShapeSettings_Destroy(settings) }
            guard let shape = JPH_ConvexHullShapeSettings_CreateShape(settings) else {
                preconditionFailure("Could not build a convex hull from \(points.count) points (too few, or all on one plane?)")
            }
            return shape
        case .compound(let children):
            precondition(!children.isEmpty, "A compound shape needs at least one child")
            let settings = JPH_StaticCompoundShapeSettings_Create()!
            defer { JPH_ShapeSettings_Destroy(settings) }
            for child in children {
                child.add(to: settings)
            }
            return JPH_StaticCompoundShape_Create(settings)!
        case .mutableCompound(let children):
            let settings = JPH_MutableCompoundShapeSettings_Create()!
            defer { JPH_ShapeSettings_Destroy(settings) }
            for child in children {
                child.add(to: settings)
            }
            return JPH_MutableCompoundShape_Create(settings)!
        }
    }
}

// MARK: - MotionType

public enum MotionType: Sendable {
    case `static`
    case kinematic
    case dynamic

    var jph: JPH_MotionType {
        switch self {
        case .static: return JPH_MotionType_Static
        case .kinematic: return JPH_MotionType_Kinematic
        case .dynamic: return JPH_MotionType_Dynamic
        }
    }
}

// MARK: - MotionQuality

public enum MotionQuality: Sendable {
    case discrete
    /// Continuous collision detection, for fast-moving bodies.
    case linearCast

    var jph: JPH_MotionQuality {
        switch self {
        case .discrete: return JPH_MotionQuality_Discrete
        case .linearCast: return JPH_MotionQuality_LinearCast
        }
    }
}

// MARK: - ObjectLayer

public enum ObjectLayer: JPH_ObjectLayer, Sendable {
    case nonMoving = 0
    case moving = 1
}

// MARK: - BodyID

public struct BodyID: Sendable, Hashable {
    public let rawValue: JPH_BodyID

    public init(rawValue: JPH_BodyID) {
        self.rawValue = rawValue
    }
}

// MARK: - RayHit

public struct RayHit: Sendable, Hashable {
    public var body: BodyID
    /// Distance along the ray as a fraction of the ray's direction vector, in 0...1.
    public var fraction: Float
    public var position: SIMD3<Float>
    /// Which part of a compound or mesh shape was hit. Used by `PhysicsWorld.surfaceNormal(at:)`.
    var subShapeID: JPH_SubShapeID

    init(_ result: JPH_RayCastResult, origin: SIMD3<Float>, direction: SIMD3<Float>) {
        body = BodyID(rawValue: result.bodyID)
        fraction = result.fraction
        position = origin + direction * result.fraction
        subShapeID = result.subShapeID2
    }
}

// MARK: - ShapeHit

public struct ShapeHit: Sendable {
    public var body: BodyID
    /// How far along the sweep the hit happened, as a fraction of the direction vector, in 0...1.
    public var fraction: Float
    /// World-space contact point on the body that was hit.
    public var contactPoint: SIMD3<Float>
    /// Unit surface normal of the hit body at the contact, pointing back toward the cast shape.
    public var normal: SIMD3<Float>
    /// Non-zero when the shape already overlapped the body at the start of the sweep.
    public var penetrationDepth: Float

    init(_ result: JPH_ShapeCastResult) {
        body = BodyID(rawValue: result.bodyID2)
        fraction = result.fraction
        contactPoint = result.contactPointOn2.simd
        let axis = result.penetrationAxis.simd
        normal = simd_length(axis) > 0 ? -simd_normalize(axis) : .zero
        penetrationDepth = result.penetrationDepth
    }
}

// MARK: - ShapeCollision

public struct ShapeCollision: Sendable {
    public var body: BodyID
    /// World-space contact point on the body.
    public var contactPoint: SIMD3<Float>
    /// Unit surface normal of the body at the contact, pointing toward the query shape.
    public var normal: SIMD3<Float>
    /// Overlap depth. Negative when the shape is separated but within `maxSeparationDistance`.
    public var penetrationDepth: Float

    init(_ result: JPH_CollideShapeResult) {
        body = BodyID(rawValue: result.bodyID2)
        contactPoint = result.contactPointOn2.simd
        let axis = result.penetrationAxis.simd
        normal = simd_length(axis) > 0 ? -simd_normalize(axis) : .zero
        penetrationDepth = result.penetrationDepth
    }
}

// MARK: - PhysicsWorld

/// A self-contained physics world. Handles layer setup, job system, and body management.
///
/// Usage:
/// ```swift
/// let engine = try JoltEngine()
/// let world = PhysicsWorld(engine: engine)
/// let floor = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
/// let box = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 10, 0], motionType: .dynamic)
/// world.update(deltaTime: 1.0 / 60.0)
/// let pos = world.position(of: box)
/// ```
public final class PhysicsWorld {
    public let engine: JoltEngine
    let system: OpaquePointer
    private let jobSystem: OpaquePointer
    private let jobThreadPool = JobThreadPool()
    let bodyInterface: OpaquePointer

    // Keep layer objects alive
    private let bpLayerInterface: OpaquePointer
    private let objectFilter: OpaquePointer
    private let bpFilter: OpaquePointer

    var contactListenerHandle: OpaquePointer?
    var constraints: [ObjectIdentifier: Constraint] = [:]
    var characters: [WeakCharacter] = []
    var activationListenerHandle: OpaquePointer?
    var activationListenerBox: Unmanaged<ActivationListenerBox>?

    /// Told when bodies wake up or go to sleep. See `ActivationListener` for threading rules.
    public var activationListener: (any ActivationListener)? {
        didSet { installActivationListener() }
    }
    var contactListenerBox: Unmanaged<ContactListenerBox>?

    /// Receives contact events. Callbacks run on Jolt's worker threads during `update`.
    public var contactListener: (any ContactListener)? {
        didSet { installContactListener() }
    }

    public init(
        engine: JoltEngine,
        gravity: SIMD3<Float> = [0, -9.81, 0],
        maxBodies: UInt32 = 10240,
        maxBodyPairs: UInt32 = 65536,
        maxContactConstraints: UInt32 = 10240
    ) {
        self.engine = engine

        // Layer setup: 2 layers (non-moving, moving)
        let bpLI = JPH_BroadPhaseLayerInterfaceTable_Create(2, 2)!
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLI, 0, 0)
        JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(bpLI, 1, 1)

        let objFilter = JPH_ObjectLayerPairFilterTable_Create(2)!
        JPH_ObjectLayerPairFilterTable_EnableCollision(objFilter, 0, 1) // static vs moving
        JPH_ObjectLayerPairFilterTable_EnableCollision(objFilter, 1, 1) // moving vs moving

        let bpF = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(bpLI, 2, objFilter, 2)!

        self.bpLayerInterface = bpLI
        self.objectFilter = objFilter
        self.bpFilter = bpF

        var settings = JPH_PhysicsSystemSettings(
            maxBodies: maxBodies,
            numBodyMutexes: 0,
            maxBodyPairs: maxBodyPairs,
            maxContactConstraints: maxContactConstraints,
            _padding: 0,
            broadPhaseLayerInterface: bpLI,
            objectLayerPairFilter: objFilter,
            objectVsBroadPhaseLayerFilter: bpF
        )
        self.system = JPH_PhysicsSystem_Create(&settings)

        var g = JPH_Vec3(gravity)
        JPH_PhysicsSystem_SetGravity(system, &g)

        // Jolt's own thread pool runs at Default QoS, causing priority inversions when updated from a
        // higher-QoS thread. Our pool runs its threads at user-interactive QoS instead.
        var jobConfig = JPH_JobSystemConfig(
            context: Unmanaged.passUnretained(jobThreadPool).toOpaque(),
            queueJob: { context, job, arg in
                guard let context, let job else { return }
                let pool = Unmanaged<JobThreadPool>.fromOpaque(context).takeUnretainedValue()
                pool.enqueue([PooledJob(job: job, arg: arg)])
            },
            queueJobs: { context, job, args, count in
                guard let context, let job, let args else { return }
                let pool = Unmanaged<JobThreadPool>.fromOpaque(context).takeUnretainedValue()
                pool.enqueue((0..<Int(count)).map { PooledJob(job: job, arg: args[$0]) })
            },
            maxConcurrency: UInt32(jobThreadPool.threadCount + 1),
            maxBarriers: 8
        )
        self.jobSystem = JPH_JobSystemCallback_Create(&jobConfig)

        self.bodyInterface = JPH_PhysicsSystem_GetBodyInterface(system)
    }

    deinit {
        for constraint in constraints.values {
            constraint.willRemove(from: system)
            JPH_PhysicsSystem_RemoveConstraint(system, constraint.handle)
        }
        JPH_PhysicsSystem_Destroy(system)
        releaseContactListener()
        releaseActivationListener()
        // Queued jobs can still hold job references after an update returns.
        jobThreadPool.shutdown()
        JPH_JobSystem_Destroy(jobSystem)
    }

    // MARK: - Bodies

    /// Add a body to the world. Returns a BodyID for later queries.
    @discardableResult
    public func addBody(
        _ shape: Shape,
        position: SIMD3<Float> = .zero,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        motionType: MotionType = .dynamic,
        restitution: Float = 0.3,
        friction: Float = 0.2
    ) -> BodyID {
        let shapeHandle = shape.createHandle()
        // createHandle returns an owned reference; the body takes its own.
        defer { JPH_Shape_Destroy(shapeHandle) }
        let layer: ObjectLayer = (motionType == .static) ? .nonMoving : .moving

        var pos = JPH_RVec3(position)
        var rot = JPH_Quat(rotation)

        let creationSettings = JPH_BodyCreationSettings_Create3(
            shapeHandle,
            &pos,
            &rot,
            motionType.jph,
            layer.rawValue
        )!

        let bodyID = JPH_BodyInterface_CreateAndAddBody(
            bodyInterface,
            creationSettings,
            JPH_Activation_Activate
        )

        JPH_BodyCreationSettings_Destroy(creationSettings)

        JPH_BodyInterface_SetRestitution(bodyInterface, bodyID, restitution)
        JPH_BodyInterface_SetFriction(bodyInterface, bodyID, friction)

        return BodyID(rawValue: bodyID)
    }

    /// Remove and destroy a body.
    public func removeBody(_ body: BodyID) {
        JPH_BodyInterface_RemoveAndDestroyBody(bodyInterface, body.rawValue)
    }

    // MARK: - Simulation

    /// Step the simulation forward.
    public func update(deltaTime: Float, collisionSteps: Int = 1) {
        JoltEngine.withTempAllocator {
            _ = JPH_PhysicsSystem_Update(system, deltaTime, Int32(collisionSteps), jobSystem)
        }
        postSimulateCharacters()
    }

    /// Optimize the broad phase (call after adding a batch of bodies).
    public func optimizeBroadPhase() {
        JPH_PhysicsSystem_OptimizeBroadPhase(system)
    }

    // MARK: - Queries

    public func position(of body: BodyID) -> SIMD3<Float> {
        var result = JPH_RVec3()
        JPH_BodyInterface_GetPosition(bodyInterface, body.rawValue, &result)
        return result.simd
    }

    public func rotation(of body: BodyID) -> simd_quatf {
        var result = JPH_Quat()
        JPH_BodyInterface_GetRotation(bodyInterface, body.rawValue, &result)
        return result.simd
    }

    public func worldTransform(of body: BodyID) -> simd_float4x4 {
        var result = JPH_RMat4()
        JPH_BodyInterface_GetWorldTransform(bodyInterface, body.rawValue, &result)
        return result.simd
    }

    public func setPosition(of body: BodyID, to position: SIMD3<Float>, activate: Bool = true) {
        var p = JPH_RVec3(position)
        JPH_BodyInterface_SetPosition(bodyInterface, body.rawValue, &p, activation(activate))
    }

    public func setRotation(of body: BodyID, to rotation: simd_quatf, activate: Bool = true) {
        var r = JPH_Quat(rotation)
        JPH_BodyInterface_SetRotation(bodyInterface, body.rawValue, &r, activation(activate))
    }

    public func setPositionAndRotation(
        of body: BodyID,
        position: SIMD3<Float>,
        rotation: simd_quatf,
        activate: Bool = true
    ) {
        var p = JPH_RVec3(position)
        var r = JPH_Quat(rotation)
        JPH_BodyInterface_SetPositionAndRotation(bodyInterface, body.rawValue, &p, &r, activation(activate))
    }

    /// Moves a kinematic body so it reaches the target after `deltaTime`, pushing dynamic bodies along the way.
    public func moveKinematic(_ body: BodyID, to position: SIMD3<Float>, rotation: simd_quatf, deltaTime: Float) {
        var p = JPH_RVec3(position)
        var r = JPH_Quat(rotation)
        JPH_BodyInterface_MoveKinematic(bodyInterface, body.rawValue, &p, &r, deltaTime)
    }

    public func angularVelocity(of body: BodyID) -> SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_BodyInterface_GetAngularVelocity(bodyInterface, body.rawValue, &result)
        return result.simd
    }

    public func setAngularVelocity(of body: BodyID, to velocity: SIMD3<Float>) {
        var v = JPH_Vec3(velocity)
        JPH_BodyInterface_SetAngularVelocity(bodyInterface, body.rawValue, &v)
    }

    private func activation(_ activate: Bool) -> JPH_Activation {
        activate ? JPH_Activation_Activate : JPH_Activation_DontActivate
    }

    public func linearVelocity(of body: BodyID) -> SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_BodyInterface_GetLinearVelocity(bodyInterface, body.rawValue, &result)
        return result.simd
    }

    public func setLinearVelocity(of body: BodyID, to velocity: SIMD3<Float>) {
        var v = JPH_Vec3(velocity)
        JPH_BodyInterface_SetLinearVelocity(bodyInterface, body.rawValue, &v)
    }

    public func addImpulse(to body: BodyID, impulse: SIMD3<Float>) {
        var imp = JPH_Vec3(impulse)
        JPH_BodyInterface_AddImpulse(bodyInterface, body.rawValue, &imp)
    }

    public func addForce(to body: BodyID, force: SIMD3<Float>) {
        var f = JPH_Vec3(force)
        JPH_BodyInterface_AddForce(bodyInterface, body.rawValue, &f)
    }

    // MARK: - Motion Properties

    // Static bodies have no motion properties, so these trap when called on one.
    private func withMotionProperties<Result>(
        of body: BodyID,
        _ action: (OpaquePointer) -> Result
    ) -> Result {
        let lockInterface = JPH_PhysicsSystem_GetBodyLockInterface(system)
        var lock = JPH_BodyLockWrite()
        JPH_BodyLockInterface_LockWrite(lockInterface, body.rawValue, &lock)
        defer { JPH_BodyLockInterface_UnlockWrite(lockInterface, &lock) }
        guard let jphBody = lock.body, let properties = JPH_Body_GetMotionProperties(jphBody) else {
            preconditionFailure("Body \(body.rawValue) has no motion properties (is it static or removed?)")
        }
        return action(properties)
    }

    public func linearDamping(of body: BodyID) -> Float {
        withMotionProperties(of: body) { JPH_MotionProperties_GetLinearDamping($0) }
    }

    public func setLinearDamping(of body: BodyID, to damping: Float) {
        withMotionProperties(of: body) { JPH_MotionProperties_SetLinearDamping($0, damping) }
    }

    public func angularDamping(of body: BodyID) -> Float {
        withMotionProperties(of: body) { JPH_MotionProperties_GetAngularDamping($0) }
    }

    public func setAngularDamping(of body: BodyID, to damping: Float) {
        withMotionProperties(of: body) { JPH_MotionProperties_SetAngularDamping($0, damping) }
    }

    /// Returns `.infinity` for bodies with zero inverse mass.
    public func mass(of body: BodyID) -> Float {
        let inverseMass = withMotionProperties(of: body) { JPH_MotionProperties_GetInverseMassUnchecked($0) }
        return inverseMass == 0 ? .infinity : 1 / inverseMass
    }

    /// Sets the mass and scales the inertia tensor to match.
    public func setMass(of body: BodyID, to mass: Float) {
        precondition(mass > 0, "Mass must be positive")
        withMotionProperties(of: body) { JPH_MotionProperties_ScaleToMass($0, mass) }
    }

    /// Diagonal of the inverse inertia tensor in the body's principal axes, and the rotation of those axes.
    public func inverseInertia(of body: BodyID) -> (diagonal: SIMD3<Float>, rotation: simd_quatf) {
        withMotionProperties(of: body) { properties in
            var diagonal = JPH_Vec3()
            var rotation = JPH_Quat()
            JPH_MotionProperties_GetInverseInertiaDiagonal(properties, &diagonal)
            JPH_MotionProperties_GetInertiaRotation(properties, &rotation)
            return (diagonal.simd, rotation.simd)
        }
    }

    public func setInverseInertia(of body: BodyID, diagonal: SIMD3<Float>, rotation: simd_quatf) {
        withMotionProperties(of: body) { properties in
            var d = JPH_Vec3(diagonal)
            var r = JPH_Quat(rotation)
            JPH_MotionProperties_SetInverseInertia(properties, &d, &r)
        }
    }

    public func gravityFactor(of body: BodyID) -> Float {
        JPH_BodyInterface_GetGravityFactor(bodyInterface, body.rawValue)
    }

    public func setGravityFactor(of body: BodyID, to factor: Float) {
        JPH_BodyInterface_SetGravityFactor(bodyInterface, body.rawValue, factor)
    }

    public func motionQuality(of body: BodyID) -> MotionQuality {
        JPH_BodyInterface_GetMotionQuality(bodyInterface, body.rawValue) == JPH_MotionQuality_LinearCast ? .linearCast : .discrete
    }

    public func setMotionQuality(of body: BodyID, to quality: MotionQuality) {
        JPH_BodyInterface_SetMotionQuality(bodyInterface, body.rawValue, quality.jph)
    }

    // MARK: - Ray Casting

    /// Returns the closest body hit by the ray. `direction` sets both the direction and the length of the ray.
    public func castRay(origin: SIMD3<Float>, direction: SIMD3<Float>) -> RayHit? {
        var o = JPH_RVec3(origin)
        var d = JPH_Vec3(direction)
        var result = JPH_RayCastResult()
        let query = JPH_PhysicsSystem_GetNarrowPhaseQuery(system)
        guard JPH_NarrowPhaseQuery_CastRay(query, &o, &d, &result, nil, nil, nil) else {
            return nil
        }
        return RayHit(result, origin: origin, direction: direction)
    }

    /// Returns every body hit by the ray, sorted from nearest to farthest.
    public func castRayAll(origin: SIMD3<Float>, direction: SIMD3<Float>) -> [RayHit] {
        var o = JPH_RVec3(origin)
        var d = JPH_Vec3(direction)
        var settings = JPH_RayCastSettings(
            backFaceModeTriangles: JPH_BackFaceMode_IgnoreBackFaces,
            backFaceModeConvex: JPH_BackFaceMode_IgnoreBackFaces,
            treatConvexAsSolid: true
        )
        var results: [JPH_RayCastResult] = []
        let query = JPH_PhysicsSystem_GetNarrowPhaseQuery(system)
        withUnsafeMutablePointer(to: &results) { resultsPointer in
            _ = JPH_NarrowPhaseQuery_CastRay3(
                query, &o, &d, &settings, JPH_CollisionCollectorType_AllHitSorted,
                { context, result in
                    guard let context, let result else {
                        return
                    }
                    context.assumingMemoryBound(to: [JPH_RayCastResult].self).pointee.append(result.pointee)
                },
                resultsPointer, nil, nil, nil, nil
            )
        }
        return results.map { RayHit($0, origin: origin, direction: direction) }
    }

    // MARK: - Shape Casting

    /// Sweeps `shape` from `position` along `direction` (whose length is the sweep distance) and returns the first hit.
    public func castShape(
        _ shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        direction: SIMD3<Float>
    ) -> ShapeHit? {
        castShape(shape, position: position, rotation: rotation, direction: direction, collector: JPH_CollisionCollectorType_ClosestHit).first
    }

    /// Sweeps `shape` along `direction` and returns every hit, sorted from nearest to farthest.
    public func castShapeAll(
        _ shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        direction: SIMD3<Float>
    ) -> [ShapeHit] {
        castShape(shape, position: position, rotation: rotation, direction: direction, collector: JPH_CollisionCollectorType_AllHitSorted)
    }

    private func castShape(
        _ shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf,
        direction: SIMD3<Float>,
        collector: JPH_CollisionCollectorType
    ) -> [ShapeHit] {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        var transform = simd_float4x4(rotation)
        transform.columns.3 = SIMD4(position, 1)
        var jphTransform = JPH_RMat4(transform)
        var jphDirection = JPH_Vec3(direction)
        var baseOffset = JPH_RVec3()
        var settings = JPH_ShapeCastSettings()
        JPH_ShapeCastSettings_Init(&settings)
        var results: [JPH_ShapeCastResult] = []
        let query = JPH_PhysicsSystem_GetNarrowPhaseQuery(system)
        withUnsafeMutablePointer(to: &results) { resultsPointer in
            _ = JPH_NarrowPhaseQuery_CastShape2(
                query, shapeHandle, &jphTransform, &jphDirection, &settings, &baseOffset, collector,
                { context, result in
                    guard let context, let result else {
                        return
                    }
                    context.assumingMemoryBound(to: [JPH_ShapeCastResult].self).pointee.append(result.pointee)
                },
                resultsPointer, nil, nil, nil, nil
            )
        }
        return results.map(ShapeHit.init)
    }

    // MARK: - Shape Overlap

    /// Returns a contact for each body that `shape` overlaps when placed at `position` and `rotation`.
    /// With `maxSeparationDistance` > 0, bodies within that distance are also reported.
    public func collideShape(
        _ shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        maxSeparationDistance: Float = 0
    ) -> [ShapeCollision] {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        // Jolt positions the shape by its center of mass, which is not always its origin.
        var centerOfMass = JPH_Vec3()
        JPH_Shape_GetCenterOfMass(shapeHandle, &centerOfMass)
        var transform = simd_float4x4(rotation)
        transform.columns.3 = SIMD4(position + rotation.act(centerOfMass.simd), 1)
        var jphTransform = JPH_RMat4(transform)
        var scale = JPH_Vec3(SIMD3<Float>.one)
        var baseOffset = JPH_RVec3()
        var settings = JPH_CollideShapeSettings()
        JPH_CollideShapeSettings_Init(&settings)
        settings.maxSeparationDistance = maxSeparationDistance
        var results: [JPH_CollideShapeResult] = []
        let query = JPH_PhysicsSystem_GetNarrowPhaseQuery(system)
        withUnsafeMutablePointer(to: &results) { resultsPointer in
            _ = JPH_NarrowPhaseQuery_CollideShape2(
                query, shapeHandle, &scale, &jphTransform, &settings, &baseOffset, JPH_CollisionCollectorType_AllHit,
                { context, result in
                    guard let context, let result else {
                        return
                    }
                    context.assumingMemoryBound(to: [JPH_CollideShapeResult].self).pointee.append(result.pointee)
                },
                resultsPointer, nil, nil, nil, nil
            )
        }
        return results.map(ShapeCollision.init)
    }

    // MARK: - Point Queries

    /// Returns every body whose shape contains `point`.
    public func bodies(containing point: SIMD3<Float>) -> [BodyID] {
        var p = JPH_RVec3(point)
        var results: [BodyID] = []
        let query = JPH_PhysicsSystem_GetNarrowPhaseQuery(system)
        withUnsafeMutablePointer(to: &results) { resultsPointer in
            _ = JPH_NarrowPhaseQuery_CollidePoint2(
                query, &p, JPH_CollisionCollectorType_AllHit,
                { context, result in
                    guard let context, let result else {
                        return
                    }
                    context.assumingMemoryBound(to: [BodyID].self).pointee.append(BodyID(rawValue: result.pointee.bodyID))
                },
                resultsPointer, nil, nil, nil, nil
            )
        }
        return results
    }

    // MARK: - Info

    public var bodyCount: UInt32 {
        JPH_PhysicsSystem_GetNumBodies(system)
    }

    public var activeBodyCount: UInt32 {
        JPH_PhysicsSystem_GetNumActiveBodies(system, JPH_BodyType_Rigid)
    }

    public var gravity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_PhysicsSystem_GetGravity(system, &result)
            return result.simd
        }
        set {
            var g = JPH_Vec3(newValue)
            JPH_PhysicsSystem_SetGravity(system, &g)
        }
    }
}

struct PooledJob: @unchecked Sendable {
    let job: @convention(c) (UnsafeMutableRawPointer?) -> Void
    let arg: UnsafeMutableRawPointer?

    func run() {
        job(arg)
    }
}

final class JobThreadPool: @unchecked Sendable {
    let threadCount = max(1, ProcessInfo.processInfo.activeProcessorCount - 1)
    private let condition = NSCondition()
    private var pending: [PooledJob] = []
    private var isShuttingDown = false
    private var runningThreads = 0

    init() {
        runningThreads = threadCount
        for index in 0..<threadCount {
            let thread = Thread { [self] in work() }
            thread.name = "JoltPhysics.job.\(index)"
            thread.qualityOfService = .userInteractive
            thread.start()
        }
    }

    func enqueue(_ jobs: [PooledJob]) {
        condition.lock()
        pending.append(contentsOf: jobs)
        condition.unlock()
        if jobs.count == 1 {
            condition.signal()
        } else {
            condition.broadcast()
        }
    }

    // Runs any remaining jobs, then waits for all threads to exit.
    func shutdown() {
        condition.lock()
        isShuttingDown = true
        condition.broadcast()
        while runningThreads > 0 {
            condition.wait()
        }
        condition.unlock()
    }

    private func work() {
        condition.lock()
        while true {
            if !pending.isEmpty {
                let job = pending.removeFirst()
                condition.unlock()
                job.run()
                condition.lock()
            } else if isShuttingDown {
                break
            } else {
                condition.wait()
            }
        }
        runningThreads -= 1
        condition.broadcast()
        condition.unlock()
    }
}
