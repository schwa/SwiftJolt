import CJolt
import simd

public enum GroundState: Sendable {
    case onGround
    /// Touching ground that is steeper than the max slope angle.
    case onSteepGround
    /// Touching something, but it does not hold the character up.
    case notSupported
    case inAir

    init(_ state: JPH_GroundState) {
        switch state {
        case JPH_GroundState_OnGround: self = .onGround
        case JPH_GroundState_OnSteepGround: self = .onSteepGround
        case JPH_GroundState_NotSupported: self = .notSupported
        default: self = .inAir
        }
    }
}

/// Shared state of `Character` and `VirtualCharacter`: slope limit, up vector and ground contact.
public class CharacterBase {
    let handle: OpaquePointer
    // Keeps the physics system alive for as long as the character exists.
    let world: PhysicsWorld

    init(handle: OpaquePointer, world: PhysicsWorld) {
        self.handle = handle
        self.world = world
    }

    deinit {
        JPH_CharacterBase_Destroy(handle)
    }

    /// Radians. Ground steeper than this counts as `.onSteepGround`.
    public var maxSlopeAngle: Float {
        get { acos(JPH_CharacterBase_GetCosMaxSlopeAngle(handle)) }
        set { JPH_CharacterBase_SetMaxSlopeAngle(handle, newValue) }
    }

    public var up: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_CharacterBase_GetUp(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_CharacterBase_SetUp(handle, &value)
        }
    }

    public var groundState: GroundState {
        GroundState(JPH_CharacterBase_GetGroundState(handle))
    }

    /// True when standing on ground that is not too steep.
    public var isSupported: Bool {
        JPH_CharacterBase_IsSupported(handle)
    }

    /// The body the character stands on, or nil when in the air.
    public var groundBody: BodyID? {
        groundState == .inAir ? nil : BodyID(rawValue: JPH_CharacterBase_GetGroundBodyId(handle))
    }

    public var groundPosition: SIMD3<Float> {
        var result = JPH_RVec3()
        JPH_CharacterBase_GetGroundPosition(handle, &result)
        return result.simd
    }

    public var groundNormal: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_CharacterBase_GetGroundNormal(handle, &result)
        return result.simd
    }

    /// Velocity of the ground under the character, e.g. a moving platform.
    public var groundVelocity: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_CharacterBase_GetGroundVelocity(handle, &result)
        return result.simd
    }

    /// Which part of the ground body's shape the character stands on.
    public var groundSubShape: SubShapeID {
        SubShapeID(rawValue: JPH_CharacterBase_GetGroundSubShapeId(handle))
    }

    /// User data of the ground body.
    public var groundUserData: UInt64 {
        JPH_CharacterBase_GetGroundUserData(handle)
    }

    /// Material of the ground under the character, or nil when in the air.
    public var groundMaterial: PhysicsMaterial? {
        JPH_CharacterBase_GetGroundMaterial(handle).map(PhysicsMaterial.init)
    }

    /// True if ground with this surface normal is steeper than `maxSlopeAngle`.
    public func isSlopeTooSteep(normal: SIMD3<Float>) -> Bool {
        var value = JPH_Vec3(normal)
        return JPH_CharacterBase_IsSlopeTooSteep(handle, &value)
    }
}

/// A character controller backed by a rigid body. It collides like a normal dynamic body, never
/// rotates, and tracks the ground it stands on. Create one with `PhysicsWorld.addCharacter`.
/// The world updates ground contact after every `update`. The character is removed from the
/// world when this object is released.
public final class Character: CharacterBase {
    deinit {
        JPH_Character_RemoveFromPhysicsSystem(handle, true)
    }

    /// The character's body. Use it with queries and contact events.
    public var body: BodyID {
        BodyID(rawValue: JPH_Character_GetBodyID(handle))
    }

    public var position: SIMD3<Float> {
        get {
            var result = JPH_RVec3()
            JPH_Character_GetPosition(handle, &result, true)
            return result.simd
        }
        set {
            var value = JPH_RVec3(newValue)
            JPH_Character_SetPosition(handle, &value, JPH_Activation_Activate, true)
        }
    }

    public var rotation: simd_quatf {
        get {
            var result = JPH_Quat()
            JPH_Character_GetRotation(handle, &result, true)
            return result.simd
        }
        set {
            var value = JPH_Quat(newValue)
            JPH_Character_SetRotation(handle, &value, JPH_Activation_Activate, true)
        }
    }

    public var linearVelocity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_Character_GetLinearVelocity(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_Character_SetLinearVelocity(handle, &value, true)
        }
    }

    public func addImpulse(_ impulse: SIMD3<Float>) {
        var value = JPH_Vec3(impulse)
        JPH_Character_AddImpulse(handle, &value, true)
    }

    public func addLinearVelocity(_ velocity: SIMD3<Float>) {
        var value = JPH_Vec3(velocity)
        JPH_Character_AddLinearVelocity(handle, &value, true)
    }

    public func setVelocity(linear: SIMD3<Float>, angular: SIMD3<Float>) {
        var jphLinear = JPH_Vec3(linear)
        var jphAngular = JPH_Vec3(angular)
        JPH_Character_SetLinearAndAngularVelocity(handle, &jphLinear, &jphAngular, true)
    }

    /// Wakes the character's body.
    public func activate() {
        JPH_Character_Activate(handle, true)
    }

    public var positionAndRotation: (position: SIMD3<Float>, rotation: simd_quatf) {
        var jphPosition = JPH_RVec3()
        var jphRotation = JPH_Quat()
        JPH_Character_GetPositionAndRotation(handle, &jphPosition, &jphRotation, true)
        return (jphPosition.simd, jphRotation.simd)
    }

    public func setPosition(_ position: SIMD3<Float>, rotation: simd_quatf) {
        var jphPosition = JPH_RVec3(position)
        var jphRotation = JPH_Quat(rotation)
        JPH_Character_SetPositionAndRotation(handle, &jphPosition, &jphRotation, JPH_Activation_Activate, true)
    }

    public var centerOfMassPosition: SIMD3<Float> {
        var result = JPH_RVec3()
        JPH_Character_GetCenterOfMassPosition(handle, &result, true)
        return result.simd
    }

    public var worldTransform: simd_float4x4 {
        var result = JPH_RMat4()
        JPH_Character_GetWorldTransform(handle, &result, true)
        return result.simd
    }

    public var layer: ObjectLayer {
        get { ObjectLayer(rawValue: JPH_Character_GetLayer(handle)) }
        set { JPH_Character_SetLayer(handle, newValue.rawValue, true) }
    }

    /// Replaces the collision shape, e.g. to crouch. With `maxPenetrationDepth` below infinity, Jolt
    /// checks the new shape against the world first and keeps the old one if it would penetrate
    /// deeper than that.
    public func setShape(_ shape: Shape, maxPenetrationDepth: Float = .infinity) {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        JPH_Character_SetShape(handle, shapeHandle, maxPenetrationDepth, true)
    }

    func postSimulation() {
        JPH_Character_PostSimulation(handle, 0.05, true)
    }
}

final class WeakCharacter {
    weak var character: Character?

    init(_ character: Character) {
        self.character = character
    }
}

extension PhysicsWorld {
    /// Adds a character at `position`. A capsule is the usual shape.
    public func addCharacter(
        shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        mass: Float = 80,
        friction: Float = 0.2,
        maxSlopeAngle: Float = .pi / 4
    ) -> Character {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        var settings = JPH_CharacterSettings()
        JPH_CharacterSettings_Init(&settings)
        settings.base.shape = shapeHandle
        settings.base.maxSlopeAngle = maxSlopeAngle
        settings.layer = ObjectLayer.moving.rawValue
        settings.mass = mass
        settings.friction = friction
        var jphPosition = JPH_RVec3(position)
        var jphRotation = JPH_Quat(rotation)
        let handle = JPH_Character_Create(&settings, &jphPosition, &jphRotation, 0, system)!
        JPH_Character_AddToPhysicsSystem(handle, JPH_Activation_Activate, true)
        let character = Character(handle: handle, world: self)
        characters.removeAll { $0.character == nil }
        characters.append(WeakCharacter(character))
        return character
    }

    func postSimulateCharacters() {
        for entry in characters {
            entry.character?.postSimulation()
        }
    }
}
