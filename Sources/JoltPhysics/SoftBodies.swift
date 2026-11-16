import CJolt
import simd

/// How a soft body resists bending across shared edges.
public enum SoftBodyBendType: Sendable {
    case noBending
    case distance
    case dihedral

    var jph: JPH_SoftBodyBendType {
        switch self {
        case .noBending: JPH_SoftBodyBendType_None
        case .distance: JPH_SoftBodyBendType_Distance
        case .dihedral: JPH_SoftBodyBendType_Dihedral
        }
    }
}

/// Rest-state geometry and constraints, shareable between soft body instances.
public final class SoftBodySharedSettings {
    public let engine: JoltEngine
    let handle: OpaquePointer
    public private(set) var isOptimized = false
    /// Set once a soft body uses these settings. Further edits would corrupt live bodies.
    public internal(set) var isInUse = false

    public init(engine: JoltEngine) {
        self.engine = engine
        handle = JPH_SoftBodySharedSettings_Create()
    }

    private func willMutate() {
        precondition(!isInUse, "Soft body settings can't change once a soft body uses them")
        isOptimized = false
    }

    deinit {
        JPH_SoftBodySharedSettings_Destroy(handle)
    }

    public var vertexCount: Int {
        Int(JPH_SoftBodySharedSettings_GetVertexCount(handle))
    }

    /// An inverse mass of 0 makes the vertex immovable.
    public func addVertex(_ position: SIMD3<Float>, inverseMass: Float = 1) {
        willMutate()
        var point = JPH_Vec3(position)
        JPH_SoftBodySharedSettings_AddVertex(handle, &point, inverseMass)
    }

    public func addVertices(_ positions: [SIMD3<Float>], inverseMass: Float = 1) {
        willMutate()
        let points = positions.map(JPH_Vec3.init)
        let inverseMasses = [Float](repeating: inverseMass, count: positions.count)
        JPH_SoftBodySharedSettings_AddVertices(handle, points, inverseMasses, UInt32(positions.count))
    }

    public func addFace(_ vertex1: Int, _ vertex2: Int, _ vertex3: Int) {
        willMutate()
        let count = vertexCount
        precondition([vertex1, vertex2, vertex3].allSatisfy { (0..<count).contains($0) }, "Face vertex index out of range")
        JPH_SoftBodySharedSettings_AddFace(handle, UInt32(vertex1), UInt32(vertex2), UInt32(vertex3))
    }

    /// Adds one face per three entries of `indices`.
    public func addFaces(_ indices: [UInt32]) {
        willMutate()
        precondition(indices.count.isMultiple(of: 3), "Face indices must come in groups of three")
        let count = vertexCount
        precondition(indices.allSatisfy { Int($0) < count }, "Face vertex index out of range")
        JPH_SoftBodySharedSettings_AddFaces(handle, indices, UInt32(indices.count / 3))
    }

    /// Pin vertices before calling `createConstraints`.
    public func pinVertex(_ index: Int) {
        willMutate()
        precondition((0..<vertexCount).contains(index), "Vertex index out of range")
        JPH_SoftBodySharedSettings_AddPinnedVertex(handle, UInt32(index))
    }

    /// Generates edge, shear, and bend constraints from the faces.
    public func createConstraints(compliance: Float = 1.0e-5, bendType: SoftBodyBendType = .distance) {
        willMutate()
        JPH_SoftBodySharedSettings_CreateConstraints(handle, compliance, bendType.jph)
    }

    /// Builds the constraint update groups the solver needs. `addSoftBody` calls this if needed.
    public func optimize() {
        precondition(!isInUse, "Soft body settings can't change once a soft body uses them")
        JPH_SoftBodySharedSettings_Optimize(handle)
        isOptimized = true
    }

    public func vertexPosition(at index: Int) -> SIMD3<Float> {
        precondition((0..<vertexCount).contains(index), "Vertex index out of range")
        var position = JPH_Vec3()
        JPH_SoftBodySharedSettings_GetVertexPosition(handle, UInt32(index), &position)
        return position.simd
    }
}

/// Per-instance soft body parameters. Defaults match Jolt's.
public struct SoftBodyOptions: Sendable, Equatable {
    public var iterations = 5
    public var linearDamping: Float = 0.1
    public var maxLinearVelocity: Float = 500
    public var friction: Float = 0.2
    public var restitution: Float = 0
    /// Internal pressure for closed shapes (n·R·T in Jolt terms). 0 disables it.
    public var pressure: Float = 0
    public var gravityFactor: Float = 1
    public var vertexRadius: Float = 0
    /// Moves the body position to follow the vertices.
    public var updatePosition = true
    public var makeRotationIdentity = true
    public var allowSleeping = true
    public var facesDoubleSided = false
    public var userData: UInt64 = 0
    public var objectLayer: ObjectLayer = .moving
    public var collisionGroup: CollisionGroup = .ungrouped

    public init() {}
}

extension PhysicsWorld {
    /// Create a soft body from shared settings and add it to the world.
    @discardableResult
    public func addSoftBody(
        _ settings: SoftBodySharedSettings,
        position: SIMD3<Float> = .zero,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        options: SoftBodyOptions = .init()
    ) -> BodyID {
        let body = withSoftBodyCreationSettings(settings, position, rotation, options) { creation in
            BodyID(rawValue: JPH_BodyInterface_CreateAndAddSoftBody(bodyInterface, creation, JPH_Activation_Activate))
        }
        softBodySettings[body] = settings
        return body
    }

    /// Creates a soft body without adding it to the simulation; see `createBody`. Returns nil when
    /// the world is full or `id` is already taken.
    public func createSoftBody(
        _ settings: SoftBodySharedSettings,
        id: BodyID? = nil,
        position: SIMD3<Float> = .zero,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        options: SoftBodyOptions = .init()
    ) -> BodyID? {
        let created = withSoftBodyCreationSettings(settings, position, rotation, options) { creation in
            if let id {
                return JPH_BodyInterface_CreateSoftBodyWithID(bodyInterface, id.rawValue, creation)
            }
            return JPH_BodyInterface_CreateSoftBody(bodyInterface, creation)
        }
        guard let created else {
            return nil
        }
        let body = BodyID(rawValue: JPH_Body_GetID(created))
        softBodySettings[body] = settings
        return body
    }

    private func withSoftBodyCreationSettings<Result>(
        _ settings: SoftBodySharedSettings,
        _ position: SIMD3<Float>,
        _ rotation: simd_quatf,
        _ options: SoftBodyOptions,
        _ action: (OpaquePointer) -> Result
    ) -> Result {
        precondition(settings.engine === engine, "Soft body settings belong to a different engine")
        if !settings.isOptimized {
            settings.optimize()
        }
        settings.isInUse = true
        var jphPosition = JPH_RVec3(position)
        var jphRotation = JPH_Quat(rotation)
        collisionLayers.checkLayer(options.objectLayer)
        let creation = JPH_SoftBodyCreationSettings_Create2(settings.handle, &jphPosition, &jphRotation, options.objectLayer.rawValue)!
        defer { JPH_SoftBodyCreationSettings_Destroy(creation) }
        registerGroupFilter(options.collisionGroup.filter)
        var group = options.collisionGroup.jph
        JPH_SoftBodyCreationSettings_SetCollisionGroup(creation, &group)

        JPH_SoftBodyCreationSettings_SetNumIterations(creation, UInt32(options.iterations))
        JPH_SoftBodyCreationSettings_SetLinearDamping(creation, options.linearDamping)
        JPH_SoftBodyCreationSettings_SetMaxLinearVelocity(creation, options.maxLinearVelocity)
        JPH_SoftBodyCreationSettings_SetFriction(creation, options.friction)
        JPH_SoftBodyCreationSettings_SetRestitution(creation, options.restitution)
        JPH_SoftBodyCreationSettings_SetPressure(creation, options.pressure)
        JPH_SoftBodyCreationSettings_SetGravityFactor(creation, options.gravityFactor)
        JPH_SoftBodyCreationSettings_SetVertexRadius(creation, options.vertexRadius)
        JPH_SoftBodyCreationSettings_SetUpdatePosition(creation, options.updatePosition)
        JPH_SoftBodyCreationSettings_SetMakeRotationIdentity(creation, options.makeRotationIdentity)
        JPH_SoftBodyCreationSettings_SetAllowSleeping(creation, options.allowSleeping)
        JPH_SoftBodyCreationSettings_SetFacesDoubleSided(creation, options.facesDoubleSided)
        JPH_SoftBodyCreationSettings_SetUserData(creation, options.userData)
        return action(creation)
    }
}
