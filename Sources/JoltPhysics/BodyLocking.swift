import CJolt
import simd

/// A body held under a Jolt body lock. Only valid inside `PhysicsWorld.readBody` / `writeBody`.
/// A borrowed `LockedBody` (from `readBody`) can only be read; an `inout` one (from `writeBody`) can also be changed.
public struct LockedBody: ~Copyable {
    let pointer: OpaquePointer

    public var id: BodyID {
        BodyID(rawValue: JPH_Body_GetID(pointer))
    }

    public var position: SIMD3<Float> {
        var result = JPH_RVec3()
        JPH_Body_GetPosition(pointer, &result)
        return result.simd
    }

    public var rotation: simd_quatf {
        var result = JPH_Quat()
        JPH_Body_GetRotation(pointer, &result)
        return result.simd
    }

    public var worldTransform: simd_float4x4 {
        var result = JPH_RMat4()
        JPH_Body_GetWorldTransform(pointer, &result)
        return result.simd
    }

    public var centerOfMassPosition: SIMD3<Float> {
        var result = JPH_RVec3()
        JPH_Body_GetCenterOfMassPosition(pointer, &result)
        return result.simd
    }

    public var worldSpaceBounds: (min: SIMD3<Float>, max: SIMD3<Float>) {
        var result = JPH_AABox()
        JPH_Body_GetWorldSpaceBounds(pointer, &result)
        return (result.min.simd, result.max.simd)
    }

    public var motionType: MotionType {
        switch JPH_Body_GetMotionType(pointer) {
        case JPH_MotionType_Static: return .static
        case JPH_MotionType_Kinematic: return .kinematic
        default: return .dynamic
        }
    }

    public var isActive: Bool {
        JPH_Body_IsActive(pointer)
    }

    public var linearVelocity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_Body_GetLinearVelocity(pointer, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_Body_SetLinearVelocity(pointer, &value)
        }
    }

    public var angularVelocity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_Body_GetAngularVelocity(pointer, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_Body_SetAngularVelocity(pointer, &value)
        }
    }

    public var friction: Float {
        get { JPH_Body_GetFriction(pointer) }
        set { JPH_Body_SetFriction(pointer, newValue) }
    }

    public var restitution: Float {
        get { JPH_Body_GetRestitution(pointer) }
        set { JPH_Body_SetRestitution(pointer, newValue) }
    }

    /// Sensors report contacts but do not collide.
    public var isSensor: Bool {
        get { JPH_Body_IsSensor(pointer) }
        set { JPH_Body_SetIsSensor(pointer, newValue) }
    }

    /// Velocity of a world-space point fixed to the body.
    public func velocity(at point: SIMD3<Float>) -> SIMD3<Float> {
        var p = JPH_RVec3(point)
        var result = JPH_Vec3()
        JPH_Body_GetPointVelocity(pointer, &p, &result)
        return result.simd
    }

    public mutating func addForce(_ force: SIMD3<Float>, at point: SIMD3<Float>? = nil) {
        var f = JPH_Vec3(force)
        if let point {
            var p = JPH_RVec3(point)
            JPH_Body_AddForceAtPosition(pointer, &f, &p)
        } else {
            JPH_Body_AddForce(pointer, &f)
        }
    }

    public mutating func addTorque(_ torque: SIMD3<Float>) {
        var t = JPH_Vec3(torque)
        JPH_Body_AddTorque(pointer, &t)
    }

    public mutating func addImpulse(_ impulse: SIMD3<Float>, at point: SIMD3<Float>? = nil) {
        var i = JPH_Vec3(impulse)
        if let point {
            var p = JPH_RVec3(point)
            JPH_Body_AddImpulseAtPosition(pointer, &i, &p)
        } else {
            JPH_Body_AddImpulse(pointer, &i)
        }
    }

    public mutating func addAngularImpulse(_ impulse: SIMD3<Float>) {
        var i = JPH_Vec3(impulse)
        JPH_Body_AddAngularImpulse(pointer, &i)
    }
}

extension PhysicsWorld {
    /// Runs `action` with a read lock on `body`. Returns nil if the body does not exist.
    public func readBody<Result>(_ body: BodyID, _ action: (borrowing LockedBody) throws -> Result) rethrows -> Result? {
        let lockInterface = JPH_PhysicsSystem_GetBodyLockInterface(system)
        var lock = JPH_BodyLockRead()
        JPH_BodyLockInterface_LockRead(lockInterface, body.rawValue, &lock)
        defer { JPH_BodyLockInterface_UnlockRead(lockInterface, &lock) }
        guard let pointer = lock.body else {
            return nil
        }
        return try action(LockedBody(pointer: pointer))
    }

    /// Runs `action` with a write lock on `body`. Returns nil if the body does not exist.
    public func writeBody<Result>(_ body: BodyID, _ action: (inout LockedBody) throws -> Result) rethrows -> Result? {
        let lockInterface = JPH_PhysicsSystem_GetBodyLockInterface(system)
        var lock = JPH_BodyLockWrite()
        JPH_BodyLockInterface_LockWrite(lockInterface, body.rawValue, &lock)
        defer { JPH_BodyLockInterface_UnlockWrite(lockInterface, &lock) }
        guard let pointer = lock.body else {
            return nil
        }
        var lockedBody = LockedBody(pointer: pointer)
        return try action(&lockedBody)
    }

    /// World-space surface normal at a ray hit.
    public func surfaceNormal(at hit: RayHit) -> SIMD3<Float>? {
        readBody(hit.body) { body in
            var position = JPH_RVec3(hit.position)
            var result = JPH_Vec3()
            JPH_Body_GetWorldSpaceSurfaceNormal(body.pointer, hit.subShapeID, &position, &result)
            return result.simd
        }
    }
}
