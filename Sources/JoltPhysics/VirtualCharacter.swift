import CJolt
import simd

/// A character that is not a body in the simulation. It moves only when you call `update`,
/// slides along walls, and can walk up stairs and stick to the floor. Gravity is not applied
/// for you: add it to `linearVelocity` yourself each frame. Create one with
/// `PhysicsWorld.addVirtualCharacter`.
public final class VirtualCharacter: CharacterBase {
    var contactListenerHandle: OpaquePointer?
    var contactListenerBox: Unmanaged<CharacterContactListenerBox>?

    /// Receives this character's contact events. Held strongly.
    public var contactListener: (any CharacterContactListener)? {
        didSet { installContactListener() }
    }

    /// Collide with other virtual characters in the same world that also have this set.
    public var collidesWithCharacters = false {
        didSet { updateCharacterCollision(from: oldValue) }
    }

    deinit {
        // The base class destroys the Jolt character after this.
        if collidesWithCharacters {
            JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter(world.characterVsCharacterCollision, handle)
        }
        if let contactListenerHandle {
            JPH_CharacterVirtual_SetListener(handle, nil)
            JPH_CharacterContactListener_Destroy(contactListenerHandle)
        }
        contactListenerBox?.release()
        world.virtualCharacters[handle] = nil
    }

    public var userData: UInt64 {
        get { JPH_CharacterVirtual_GetUserData(handle) }
        set { JPH_CharacterVirtual_SetUserData(handle, newValue) }
    }

    public var position: SIMD3<Float> {
        get {
            var result = JPH_RVec3()
            JPH_CharacterVirtual_GetPosition(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_RVec3(newValue)
            JPH_CharacterVirtual_SetPosition(handle, &value)
        }
    }

    public var rotation: simd_quatf {
        get {
            var result = JPH_Quat()
            JPH_CharacterVirtual_GetRotation(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Quat(newValue)
            JPH_CharacterVirtual_SetRotation(handle, &value)
        }
    }

    public var linearVelocity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_CharacterVirtual_GetLinearVelocity(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_CharacterVirtual_SetLinearVelocity(handle, &value)
        }
    }

    /// Used when pushing dynamic bodies.
    public var mass: Float {
        get { JPH_CharacterVirtual_GetMass(handle) }
        set { JPH_CharacterVirtual_SetMass(handle, newValue) }
    }

    /// Maximum force the character can push other bodies with, in newtons.
    public var maxStrength: Float {
        get { JPH_CharacterVirtual_GetMaxStrength(handle) }
        set { JPH_CharacterVirtual_SetMaxStrength(handle, newValue) }
    }

    /// True if the character touched `body` during the last update.
    public func hasCollided(with body: BodyID) -> Bool {
        JPH_CharacterVirtual_HasCollidedWithBody(handle, body.rawValue)
    }

    public var worldTransform: simd_float4x4 {
        var result = JPH_RMat4()
        JPH_CharacterVirtual_GetWorldTransform(handle, &result)
        return result.simd
    }

    public var centerOfMassTransform: simd_float4x4 {
        var result = JPH_RMat4()
        JPH_CharacterVirtual_GetCenterOfMassTransform(handle, &result)
        return result.simd
    }

    /// Contacts found by the last update or `refreshContacts()`.
    public var activeContacts: [CharacterContact] {
        (0..<JPH_CharacterVirtual_GetNumActiveContacts(handle)).map { index in
            var contact = JPH_CharacterVirtualContact()
            JPH_CharacterVirtual_GetActiveContact(handle, index, &contact)
            return CharacterContact(contact)
        }
    }

    /// True if the last update found more than `maxHitCount` contacts and dropped some.
    public var maxHitsExceeded: Bool {
        JPH_CharacterVirtual_GetMaxHitsExceeded(handle)
    }

    public var maxHitCount: Int {
        get { Int(JPH_CharacterVirtual_GetMaxNumHits(handle)) }
        set {
            precondition(newValue > 0, "maxHitCount must be positive")
            JPH_CharacterVirtual_SetMaxNumHits(handle, UInt32(newValue))
        }
    }

    /// Contacts whose normals differ by less than this angle (as a cosine) are merged. -1 disables merging.
    public var hitReductionCosMaxAngle: Float {
        get { JPH_CharacterVirtual_GetHitReductionCosMaxAngle(handle) }
        set { JPH_CharacterVirtual_SetHitReductionCosMaxAngle(handle, newValue) }
    }

    /// Fraction of penetration resolved per update (0...1).
    public var penetrationRecoverySpeed: Float {
        get { JPH_CharacterVirtual_GetPenetrationRecoverySpeed(handle) }
        set { JPH_CharacterVirtual_SetPenetrationRecoverySpeed(handle, newValue) }
    }

    public var enhancedInternalEdgeRemoval: Bool {
        get { JPH_CharacterVirtual_GetEnhancedInternalEdgeRemoval(handle) }
        set { JPH_CharacterVirtual_SetEnhancedInternalEdgeRemoval(handle, newValue) }
    }

    /// Gap kept between the character and the geometry around it.
    public var characterPadding: Float {
        JPH_CharacterVirtual_GetCharacterPadding(handle)
    }

    /// Offset of the shape from the character's position.
    public var shapeOffset: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_CharacterVirtual_GetShapeOffset(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_CharacterVirtual_SetShapeOffset(handle, &value)
        }
    }

    /// The rigid body that follows the character so other bodies and queries can detect it.
    /// nil unless created with `innerBodyShape`.
    public var innerBody: BodyID? {
        let id = JPH_CharacterVirtual_GetInnerBodyID(handle)
        return id == invalidBodyID ? nil : BodyID(rawValue: id)
    }

    /// Changes the inner body's shape. Only valid when the character has an inner body.
    public func setInnerBodyShape(_ shape: Shape) {
        precondition(innerBody != nil, "Character has no inner body")
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        JPH_CharacterVirtual_SetInnerBodyShape(handle, shapeHandle)
    }

    /// Switches to `shape` if it fits where the character stands. Returns false (and keeps the old
    /// shape) when the new shape would penetrate more than `maxPenetrationDepth`. Pass
    /// `.infinity` to always switch.
    @discardableResult
    public func setShape(_ shape: Shape, maxPenetrationDepth: Float = 0.1) -> Bool {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        return JoltEngine.withTempAllocator {
            JPH_CharacterVirtual_SetShape(handle, shapeHandle, maxPenetrationDepth, layer, world.system, nil, nil)
        }
    }

    /// Re-detects contacts at the current position without moving.
    public func refreshContacts() {
        JoltEngine.withTempAllocator {
            JPH_CharacterVirtual_RefreshContacts(handle, layer, world.system, nil, nil)
        }
    }

    /// Recomputes `groundVelocity` from the body below. Call after moving the ground body.
    public func updateGroundVelocity() {
        JPH_CharacterVirtual_UpdateGroundVelocity(handle)
    }

    /// Removes the parts of `velocity` that would push the character up slopes steeper than `maxSlopeAngle`.
    public func cancelVelocityTowardsSteepSlopes(_ velocity: SIMD3<Float>) -> SIMD3<Float> {
        var desired = JPH_Vec3(velocity)
        var result = JPH_Vec3()
        JPH_CharacterVirtual_CancelVelocityTowardsSteepSlopes(handle, &desired, &result)
        return result.simd
    }

    /// True when the last update hit a step that `walkStairs` could climb at `velocity`.
    public func canWalkStairs(velocity: SIMD3<Float>) -> Bool {
        var v = JPH_Vec3(velocity)
        return JPH_CharacterVirtual_CanWalkStairs(handle, &v)
    }

    /// Tries to step up by `stepUp` and forward by `stepForward`. Use after a plain update when
    /// `canWalkStairs` is true. Returns true when the character moved.
    @discardableResult
    public func walkStairs(
        deltaTime: Float,
        stepUp: SIMD3<Float> = [0, 0.4, 0],
        stepForward: SIMD3<Float>,
        stepForwardTest: SIMD3<Float>,
        stepDownExtra: SIMD3<Float> = .zero
    ) -> Bool {
        var up = JPH_Vec3(stepUp)
        var forward = JPH_Vec3(stepForward)
        var forwardTest = JPH_Vec3(stepForwardTest)
        var downExtra = JPH_Vec3(stepDownExtra)
        return JoltEngine.withTempAllocator {
            JPH_CharacterVirtual_WalkStairs(handle, deltaTime, &up, &forward, &forwardTest, &downExtra, layer, world.system, nil, nil)
        }
    }

    /// Moves the character down by up to `stepDown` to the floor. Returns true when it found floor.
    @discardableResult
    public func stickToFloor(stepDown: SIMD3<Float> = [0, -0.5, 0]) -> Bool {
        var down = JPH_Vec3(stepDown)
        return JoltEngine.withTempAllocator {
            JPH_CharacterVirtual_StickToFloor(handle, &down, layer, world.system, nil, nil)
        }
    }

    private var layer: JPH_ObjectLayer {
        ObjectLayer.moving.rawValue
    }

    /// Moves the character by `linearVelocity * deltaTime`, sliding along anything it hits.
    /// With `stickToFloor`, it stays on the ground when walking down slopes and steps.
    /// With `walkStairs`, it climbs steps up to 0.4m high.
    public func update(deltaTime: Float, stickToFloor: Bool = true, walkStairs: Bool = true) {
        guard stickToFloor || walkStairs else {
            JoltEngine.withTempAllocator {
                JPH_CharacterVirtual_Update(handle, deltaTime, layer, world.system, nil, nil)
            }
            return
        }
        // Jolt's ExtendedUpdateSettings defaults. A zero vector turns a feature off.
        var settings = JPH_ExtendedUpdateSettings(
            stickToFloorStepDown: JPH_Vec3(stickToFloor ? [0, -0.5, 0] : .zero),
            walkStairsStepUp: JPH_Vec3(walkStairs ? [0, 0.4, 0] : .zero),
            walkStairsMinStepForward: 0.02,
            walkStairsStepForwardTest: 0.15,
            walkStairsCosAngleForwardContact: cos(75 * .pi / 180),
            walkStairsStepDownExtra: JPH_Vec3(.zero)
        )
        JoltEngine.withTempAllocator {
            JPH_CharacterVirtual_ExtendedUpdate(handle, deltaTime, &settings, layer, world.system, nil, nil)
        }
    }
}

private let invalidBodyID: JPH_BodyID = 0xFFFF_FFFF

/// A contact between a virtual character and a body or another character.
public struct CharacterContact: Sendable {
    /// nil when the contact is with another character.
    public var body: BodyID?
    public var position: SIMD3<Float>
    public var linearVelocity: SIMD3<Float>
    /// Points from the character toward the other object.
    public var contactNormal: SIMD3<Float>
    public var surfaceNormal: SIMD3<Float>
    /// Negative when penetrating.
    public var distance: Float
    public var fraction: Float
    public var motionType: MotionType
    public var isSensor: Bool
    public var userData: UInt64
    public var hadCollision: Bool
    public var wasDiscarded: Bool
    public var canPushCharacter: Bool

    init(_ contact: JPH_CharacterVirtualContact) {
        body = contact.bodyB == invalidBodyID ? nil : BodyID(rawValue: contact.bodyB)
        position = contact.position.simd
        linearVelocity = contact.linearVelocity.simd
        contactNormal = contact.contactNormal.simd
        surfaceNormal = contact.surfaceNormal.simd
        distance = contact.distance
        fraction = contact.fraction
        motionType = MotionType(contact.motionTypeB)
        isSensor = contact.isSensorB
        userData = contact.userData
        hadCollision = contact.hadCollision
        wasDiscarded = contact.wasDiscarded
        canPushCharacter = contact.canPushCharacter
    }
}

extension PhysicsWorld {
    public func addVirtualCharacter(
        shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        mass: Float = 70,
        maxStrength: Float = 100,
        maxSlopeAngle: Float = .pi / 4,
        innerBodyShape: Shape? = nil
    ) -> VirtualCharacter {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        let innerHandle = innerBodyShape?.createHandle()
        defer {
            if let innerHandle {
                JPH_Shape_Destroy(innerHandle)
            }
        }
        var settings = JPH_CharacterVirtualSettings()
        JPH_CharacterVirtualSettings_Init(&settings)
        settings.base.shape = shapeHandle
        settings.base.maxSlopeAngle = maxSlopeAngle
        settings.mass = mass
        settings.maxStrength = maxStrength
        settings.innerBodyShape = innerHandle
        settings.innerBodyLayer = ObjectLayer.moving.rawValue
        var jphPosition = JPH_RVec3(position)
        var jphRotation = JPH_Quat(rotation)
        let handle = JPH_CharacterVirtual_Create(&settings, &jphPosition, &jphRotation, 0, system)!
        let character = VirtualCharacter(handle: handle, world: self)
        virtualCharacters[handle] = WeakVirtualCharacter(character)
        return character
    }
}
