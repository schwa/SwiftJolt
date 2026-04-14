import CJolt
import simd

/// A character that is not a body in the simulation. It moves only when you call `update`,
/// slides along walls, and can walk up stairs and stick to the floor. Gravity is not applied
/// for you: add it to `linearVelocity` yourself each frame. Create one with
/// `PhysicsWorld.addVirtualCharacter`.
public final class VirtualCharacter: CharacterBase {
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

    /// Moves the character by `linearVelocity * deltaTime`, sliding along anything it hits.
    /// With `stickToFloor`, it stays on the ground when walking down slopes and steps.
    /// With `walkStairs`, it climbs steps up to 0.4m high.
    public func update(deltaTime: Float, stickToFloor: Bool = true, walkStairs: Bool = true) {
        let layer = ObjectLayer.moving.rawValue
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

extension PhysicsWorld {
    public func addVirtualCharacter(
        shape: Shape,
        position: SIMD3<Float>,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        mass: Float = 70,
        maxStrength: Float = 100,
        maxSlopeAngle: Float = .pi / 4
    ) -> VirtualCharacter {
        let shapeHandle = shape.createHandle()
        defer { JPH_Shape_Destroy(shapeHandle) }
        var settings = JPH_CharacterVirtualSettings()
        JPH_CharacterVirtualSettings_Init(&settings)
        settings.base.shape = shapeHandle
        settings.base.maxSlopeAngle = maxSlopeAngle
        settings.mass = mass
        settings.maxStrength = maxStrength
        var jphPosition = JPH_RVec3(position)
        var jphRotation = JPH_Quat(rotation)
        let handle = JPH_CharacterVirtual_Create(&settings, &jphPosition, &jphRotation, 0, system)!
        return VirtualCharacter(handle: handle, world: self)
    }
}
