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
