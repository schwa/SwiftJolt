import CJolt
import simd

/// One rigid body of a ragdoll, matching the skeleton joint with the same index.
public struct RagdollPart: Sendable {
    public var shape: Shape
    public var position: SIMD3<Float>
    public var rotation: simd_quatf
    public var motionType: MotionType
    /// Overrides the mass computed from the shape.
    public var mass: Float?

    public init(
        shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        motionType: MotionType = .dynamic,
        mass: Float? = nil
    ) {
        self.shape = shape
        self.position = position
        self.rotation = rotation
        self.motionType = motionType
        self.mass = mass
    }
}

/// Shapes and joints for building ragdolls. One part per skeleton joint.
public final class RagdollSettings {
    public let skeleton: Skeleton
    let handle: OpaquePointer

    /// Add all joints to the skeleton first; the part count is fixed here.
    public init(skeleton: Skeleton) {
        self.skeleton = skeleton
        handle = JPH_RagdollSettings_Create()
        JPH_RagdollSettings_SetSkeleton(handle, skeleton.handle)
        JPH_RagdollSettings_ResizeParts(handle, Int32(skeleton.jointCount))
    }

    deinit {
        JPH_RagdollSettings_Destroy(handle)
    }

    public var engine: JoltEngine { skeleton.engine }

    public var partCount: Int {
        Int(JPH_RagdollSettings_GetPartCount(handle))
    }

    public func setPart(_ part: RagdollPart, at index: Int) {
        checkIndex(index)
        let shape = part.shape.createHandle()
        // createHandle returns an owned reference; the part takes its own.
        defer { JPH_Shape_Destroy(shape) }
        var position = JPH_RVec3(part.position)
        var rotation = JPH_Quat(part.rotation)
        let partIndex = Int32(index)
        JPH_RagdollSettings_SetPartShape(handle, partIndex, shape)
        JPH_RagdollSettings_SetPartPosition(handle, partIndex, &position)
        JPH_RagdollSettings_SetPartRotation(handle, partIndex, &rotation)
        JPH_RagdollSettings_SetPartMotionType(handle, partIndex, part.motionType.jph)
        let layer: ObjectLayer = part.motionType == .static ? .nonMoving : .moving
        JPH_RagdollSettings_SetPartObjectLayer(handle, partIndex, layer.rawValue)
        if let mass = part.mass {
            precondition(mass > 0, "Mass must be positive")
            JPH_RagdollSettings_SetPartMassProperties(handle, partIndex, mass)
        }
    }

    /// Joins part `index` to its parent's part with a swing-twist joint at a world-space point.
    /// The root part has no parent and must not get one.
    public func setPartToParent(
        _ index: Int,
        point: SIMD3<Float>,
        twistAxis: SIMD3<Float>,
        planeAxis: SIMD3<Float>? = nil,
        limits: SwingTwistLimits
    ) {
        checkIndex(index)
        precondition(skeleton.joint(at: index).parentIndex != nil, "Root part has no parent")
        let twist = simd_normalize(twistAxis)
        let plane = planeAxis.map(simd_normalize) ?? perpendicular(to: twist)
        precondition(abs(simd_dot(twist, plane)) < 1e-3, "planeAxis must be perpendicular to twistAxis")
        var settings = JPH_SwingTwistConstraintSettings()
        JPH_SwingTwistConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.position1 = JPH_RVec3(point)
        settings.position2 = JPH_RVec3(point)
        settings.twistAxis1 = JPH_Vec3(twist)
        settings.twistAxis2 = JPH_Vec3(twist)
        settings.planeAxis1 = JPH_Vec3(plane)
        settings.planeAxis2 = JPH_Vec3(plane)
        settings.swingType = limits.swingShape == .cone ? JPH_SwingType_Cone : JPH_SwingType_Pyramid
        settings.normalHalfConeAngle = limits.normalHalfConeAngle
        settings.planeHalfConeAngle = limits.planeHalfConeAngle
        settings.twistMinAngle = limits.twist.lowerBound
        settings.twistMaxAngle = limits.twist.upperBound
        settings.maxFrictionTorque = limits.maxFrictionTorque
        JPH_RagdollSettings_SetPartToParent(handle, Int32(index), &settings)
    }

    /// Evens out mass ratios between parts so the joints stay stable. Returns false on failure.
    @discardableResult
    public func stabilize() -> Bool {
        JPH_RagdollSettings_Stabilize(handle)
    }

    /// Stops parts from colliding with their parents. With `jointMatrices` (one per part), also stops
    /// collisions between non-adjacent parts closer than `minSeparationDistance` in that pose.
    public func disableParentChildCollisions(jointMatrices: [simd_float4x4] = [], minSeparationDistance: Float = 0) {
        if jointMatrices.isEmpty {
            JPH_RagdollSettings_DisableParentChildCollisions(handle, nil, minSeparationDistance)
            return
        }
        precondition(jointMatrices.count == partCount, "Expected \(partCount) joint matrices")
        let matrices = jointMatrices.map(JPH_Mat4.init)
        JPH_RagdollSettings_DisableParentChildCollisions(handle, matrices, minSeparationDistance)
    }

    /// Call before `constraintIndex(forBodyIndex:)`.
    public func calculateBodyIndexToConstraintIndex() {
        JPH_RagdollSettings_CalculateBodyIndexToConstraintIndex(handle)
    }

    /// The constraint joining a part to its parent, or nil for the root.
    public func constraintIndex(forBodyIndex index: Int) -> Int? {
        checkIndex(index)
        let constraint = JPH_RagdollSettings_GetConstraintIndexForBodyIndex(handle, Int32(index))
        return constraint >= 0 ? Int(constraint) : nil
    }

    public func calculateConstraintIndexToBodyIndexPair() {
        JPH_RagdollSettings_CalculateConstraintIndexToBodyIdxPair(handle)
    }

    private func checkIndex(_ index: Int) {
        precondition((0..<partCount).contains(index), "Part index out of range")
    }
}

/// A ragdoll's bodies and joints in a world. Created by `PhysicsWorld.createRagdoll`; not
/// simulated until `addToWorld()`. Removes and destroys its bodies when released.
public final class Ragdoll {
    public let settings: RagdollSettings
    /// The ragdoll's bodies live in this world, so it must outlive the ragdoll.
    public let world: PhysicsWorld
    let handle: OpaquePointer
    public private(set) var isInWorld = false

    init(settings: RagdollSettings, world: PhysicsWorld, collisionGroup: UInt32, userData: UInt64) {
        precondition(settings.engine === world.engine, "Ragdoll settings belong to a different engine")
        self.settings = settings
        self.world = world
        // DriveToPoseUsingMotors reads these tables and crashes if they are empty.
        settings.calculateBodyIndexToConstraintIndex()
        settings.calculateConstraintIndexToBodyIndexPair()
        handle = JPH_RagdollSettings_CreateRagdoll(settings.handle, world.system, collisionGroup, userData)
    }

    deinit {
        removeFromWorld()
        JPH_Ragdoll_Destroy(handle)
    }

    public func addToWorld(activate: Bool = true) {
        guard !isInWorld else {
            return
        }
        JPH_Ragdoll_AddToPhysicsSystem(handle, activate ? JPH_Activation_Activate : JPH_Activation_DontActivate, true)
        isInWorld = true
    }

    public func removeFromWorld() {
        guard isInWorld else {
            return
        }
        JPH_Ragdoll_RemoveFromPhysicsSystem(handle, true)
        isInWorld = false
    }

    public func activate() {
        JPH_Ragdoll_Activate(handle, true)
    }

    public var isActive: Bool {
        JPH_Ragdoll_IsActive(handle, true)
    }

    /// Clears cached constraint impulses, e.g. after teleporting the ragdoll.
    public func resetWarmStart() {
        JPH_Ragdoll_ResetWarmStart(handle)
    }

    public var bodyCount: Int {
        Int(JPH_Ragdoll_GetBodyCount(handle))
    }

    /// Bodies in part order, matching the skeleton joints.
    public var bodyIDs: [BodyID] {
        (0..<bodyCount).map { BodyID(rawValue: JPH_Ragdoll_GetBodyID(handle, Int32($0))) }
    }

    public var constraintCount: Int {
        Int(JPH_Ragdoll_GetConstraintCount(handle))
    }

    public var rootTransform: (position: SIMD3<Float>, rotation: simd_quatf) {
        var position = JPH_RVec3()
        var rotation = JPH_Quat()
        JPH_Ragdoll_GetRootTransform(handle, &position, &rotation, true)
        return (position.simd, rotation.simd)
    }

    /// Teleports the bodies to a pose. The pose's joint matrices must be up to date.
    public func setPose(_ pose: SkeletonPose) {
        checkPose(pose)
        JPH_Ragdoll_SetPose(handle, pose.handle, true)
    }

    public func setPose(rootOffset: SIMD3<Float>, jointMatrices: [simd_float4x4]) {
        precondition(jointMatrices.count == bodyCount, "Expected \(bodyCount) joint matrices")
        var offset = JPH_RVec3(rootOffset)
        JPH_Ragdoll_SetPose2(handle, &offset, jointMatrices.map(JPH_Mat4.init), true)
    }

    /// Writes the bodies' current transforms into `pose`.
    public func getPose(into pose: SkeletonPose) {
        checkPose(pose)
        JPH_Ragdoll_GetPose(handle, pose.handle, true)
    }

    public func currentPose() -> SkeletonPose {
        let pose = SkeletonPose(skeleton: settings.skeleton)
        getPose(into: pose)
        return pose
    }

    /// Sets joint motor targets toward `pose`. Joints without motors ignore this.
    public func driveToPoseUsingMotors(_ pose: SkeletonPose) {
        checkPose(pose)
        JPH_Ragdoll_DriveToPoseUsingMotors(handle, pose.handle)
    }

    /// Sets body velocities so they reach `pose` after `deltaTime`. Use with kinematic parts.
    public func driveToPoseUsingKinematics(_ pose: SkeletonPose, deltaTime: Float) {
        checkPose(pose)
        JPH_Ragdoll_DriveToPoseUsingKinematics(handle, pose.handle, deltaTime, true)
    }

    private func checkPose(_ pose: SkeletonPose) {
        precondition(pose.jointCount == bodyCount, "Pose joint count does not match the ragdoll")
    }
}

extension PhysicsWorld {
    /// Creates a ragdoll's bodies and joints. Call `addToWorld()` on the result to simulate it.
    public func createRagdoll(_ settings: RagdollSettings, collisionGroup: UInt32 = 0, userData: UInt64 = 0) -> Ragdoll {
        Ragdoll(settings: settings, world: self, collisionGroup: collisionGroup, userData: userData)
    }
}
