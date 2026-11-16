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
        MotionType(JPH_Body_GetMotionType(pointer))
    }

    public var isSoftBody: Bool {
        JPH_Body_IsSoftBody(pointer)
    }

    public var isInBroadPhase: Bool {
        JPH_Body_IsInBroadPhase(pointer)
    }

    /// True when the body was created with `BodyOptions.allowDynamicOrKinematic` or is not static.
    public var canBeKinematicOrDynamic: Bool {
        JPH_Body_CanBeKinematicOrDynamic(pointer)
    }

    public var allowSleeping: Bool {
        get { JPH_Body_GetAllowSleeping(pointer) }
        set { JPH_Body_SetAllowSleeping(pointer, newValue) }
    }

    public var applyGyroscopicForce: Bool {
        get { JPH_Body_GetApplyGyroscopicForce(pointer) }
        set { JPH_Body_SetApplyGyroscopicForce(pointer, newValue) }
    }

    public var collideKinematicVsNonDynamic: Bool {
        get { JPH_Body_GetCollideKinematicVsNonDynamic(pointer) }
        set { JPH_Body_SetCollideKinematicVsNonDynamic(pointer, newValue) }
    }

    public var useManifoldReduction: Bool {
        get { JPH_Body_GetUseManifoldReduction(pointer) }
        set { JPH_Body_SetUseManifoldReduction(pointer, newValue) }
    }

    public var enhancedInternalEdgeRemoval: Bool {
        get { JPH_Body_GetEnhancedInternalEdgeRemoval(pointer) }
        set { JPH_Body_SetEnhancedInternalEdgeRemoval(pointer, newValue) }
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

    public var userData: UInt64 {
        get { JPH_Body_GetUserData(pointer) }
        set { JPH_Body_SetUserData(pointer, newValue) }
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

    /// Applies buoyancy and drag from a fluid whose surface is the plane through `surfacePosition` with `surfaceNormal`.
    /// Returns false if the body is not in the fluid.
    @discardableResult
    public mutating func applyBuoyancyImpulse(
        surfacePosition: SIMD3<Float>,
        surfaceNormal: SIMD3<Float>,
        buoyancy: Float,
        linearDrag: Float,
        angularDrag: Float,
        fluidVelocity: SIMD3<Float> = .zero,
        gravity: SIMD3<Float>,
        deltaTime: Float
    ) -> Bool {
        var surfacePoint = JPH_RVec3(surfacePosition)
        var normal = JPH_Vec3(surfaceNormal)
        var velocity = JPH_Vec3(fluidVelocity)
        var g = JPH_Vec3(gravity)
        return JPH_Body_ApplyBuoyancyImpulse(
            pointer, &surfacePoint, &normal, buoyancy, linearDrag, angularDrag, &velocity, &g, deltaTime
        )
    }

    /// Force added since the last step, not yet applied.
    public var accumulatedForce: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_Body_GetAccumulatedForce(pointer, &result)
        return result.simd
    }

    /// Torque added since the last step, not yet applied.
    public var accumulatedTorque: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_Body_GetAccumulatedTorque(pointer, &result)
        return result.simd
    }

    public mutating func resetForce() {
        JPH_Body_ResetForce(pointer)
    }

    public mutating func resetTorque() {
        JPH_Body_ResetTorque(pointer)
    }

    /// Clears velocities and accumulated force and torque.
    public mutating func resetMotion() {
        JPH_Body_ResetMotion(pointer)
    }

    /// Sets the linear velocity, clamped to the body's max linear velocity.
    public mutating func setLinearVelocityClamped(_ velocity: SIMD3<Float>) {
        var v = JPH_Vec3(velocity)
        JPH_Body_SetLinearVelocityClamped(pointer, &v)
    }

    /// Sets the angular velocity, clamped to the body's max angular velocity.
    public mutating func setAngularVelocityClamped(_ velocity: SIMD3<Float>) {
        var v = JPH_Vec3(velocity)
        JPH_Body_SetAngularVelocityClamped(pointer, &v)
    }

    public mutating func resetSleepTimer() {
        JPH_Body_ResetSleepTimer(pointer)
    }

    public var centerOfMassTransform: simd_float4x4 {
        var result = JPH_RMat4()
        JPH_Body_GetCenterOfMassTransform(pointer, &result)
        return result.simd
    }
}

extension PhysicsWorld {
    /// Runs `action` with a read lock on `body`. Returns nil if the body does not exist.
    public func readBody<Result>(_ body: BodyID, _ action: (borrowing LockedBody) throws -> Result) rethrows -> Result? {
        let lockInterface = bodyLockInterface
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
        let lockInterface = bodyLockInterface
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
