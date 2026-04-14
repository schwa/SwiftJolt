import CJolt
import simd

// MARK: - Constraint

/// Base class for constraints. Create constraints with the `PhysicsWorld.add…Constraint` methods.
public class Constraint {
    let handle: OpaquePointer

    init(handle: OpaquePointer) {
        self.handle = handle
    }

    deinit {
        JPH_Constraint_Destroy(handle)
    }

    public var isEnabled: Bool {
        get { JPH_Constraint_GetEnabled(handle) }
        set { JPH_Constraint_SetEnabled(handle, newValue) }
    }

    /// Called just before the constraint is removed from `system`. Vehicles unregister their step listener here.
    func willRemove(from _: OpaquePointer) {}
}

public enum MotorState: Sendable {
    case off
    case velocity
    case position

    init(_ state: JPH_MotorState) {
        switch state {
        case JPH_MotorState_Velocity: self = .velocity
        case JPH_MotorState_Position: self = .position
        default: self = .off
        }
    }

    var jph: JPH_MotorState {
        switch self {
        case .off: return JPH_MotorState_Off
        case .velocity: return JPH_MotorState_Velocity
        case .position: return JPH_MotorState_Position
        }
    }
}

// MARK: - SpringSettings

/// How soft a constraint limit or motor is. A frequency or stiffness of 0 means rigid.
public struct SpringSettings: Sendable, Equatable {
    public enum Mode: Sendable, Equatable {
        /// Oscillation frequency in Hz. Independent of the bodies' masses.
        case frequency(Float)
        /// Spring constant in N/m (or N·m/rad for rotation).
        case stiffness(Float)
    }

    public var mode: Mode
    /// 0 = no damping, 1 = critical damping (when using `.frequency`). With `.stiffness`, the damping coefficient.
    public var damping: Float

    public init(_ mode: Mode, damping: Float = 0) {
        self.mode = mode
        self.damping = damping
    }

    public static let rigid = Self(.frequency(0))

    init(_ settings: JPH_SpringSettings) {
        mode = settings.mode == JPH_SpringMode_StiffnessAndDamping
            ? .stiffness(settings.frequencyOrStiffness)
            : .frequency(settings.frequencyOrStiffness)
        damping = settings.damping
    }

    var jph: JPH_SpringSettings {
        switch mode {
        case .frequency(let frequency):
            return JPH_SpringSettings(mode: JPH_SpringMode_FrequencyAndDamping, frequencyOrStiffness: frequency, damping: damping)
        case .stiffness(let stiffness):
            return JPH_SpringSettings(mode: JPH_SpringMode_StiffnessAndDamping, frequencyOrStiffness: stiffness, damping: damping)
        }
    }
}

/// How a constraint motor drives toward its target.
public struct MotorSettings: Sendable, Equatable {
    /// Used by position motors.
    public var spring: SpringSettings
    /// Force limits for linear motors (newtons).
    public var forceLimits: ClosedRange<Float>
    /// Torque limits for angular motors (newton-meters).
    public var torqueLimits: ClosedRange<Float>

    public init(
        spring: SpringSettings = SpringSettings(.frequency(2), damping: 1),
        forceLimits: ClosedRange<Float> = -.greatestFiniteMagnitude ... .greatestFiniteMagnitude,
        torqueLimits: ClosedRange<Float> = -.greatestFiniteMagnitude ... .greatestFiniteMagnitude
    ) {
        self.spring = spring
        self.forceLimits = forceLimits
        self.torqueLimits = torqueLimits
    }

    init(_ settings: JPH_MotorSettings) {
        spring = SpringSettings(settings.springSettings)
        forceLimits = settings.minForceLimit...settings.maxForceLimit
        torqueLimits = settings.minTorqueLimit...settings.maxTorqueLimit
    }

    var jph: JPH_MotorSettings {
        JPH_MotorSettings(
            springSettings: spring.jph,
            minForceLimit: forceLimits.lowerBound,
            maxForceLimit: forceLimits.upperBound,
            minTorqueLimit: torqueLimits.lowerBound,
            maxTorqueLimit: torqueLimits.upperBound
        )
    }
}

// MARK: - PhysicsWorld + Constraints

extension PhysicsWorld {
    /// Removes a constraint from the world. The constraint can not be added again.
    public func removeConstraint(_ constraint: Constraint) {
        guard constraints.removeValue(forKey: ObjectIdentifier(constraint)) != nil else {
            return
        }
        constraint.willRemove(from: system)
        JPH_PhysicsSystem_RemoveConstraint(system, constraint.handle)
    }

    public var constraintCount: Int {
        constraints.count
    }

    func add<C: Constraint>(_ constraint: C) -> C {
        JPH_PhysicsSystem_AddConstraint(system, constraint.handle)
        constraints[ObjectIdentifier(constraint)] = constraint
        return constraint
    }

    /// Calls `body` with Jolt body pointers for `body1` and `body2`. A nil `body1` means the static world.
    func withBodies<Result>(
        _ body1: BodyID?,
        _ body2: BodyID,
        _ action: (OpaquePointer, OpaquePointer) -> Result
    ) -> Result {
        let ids = [body1, body2].compactMap(\.self).map(\.rawValue)
        let lockInterface = JPH_PhysicsSystem_GetBodyLockInterface(system)
        let lock = JPH_BodyLockInterface_LockMultiWrite(lockInterface, ids, UInt32(ids.count))!
        defer { JPH_BodyLockMultiWrite_Destroy(lock) }
        let bodies = (0..<UInt32(ids.count)).map { index in
            guard let pointer = JPH_BodyLockMultiWrite_GetBody(lock, index) else {
                preconditionFailure("Body \(ids[Int(index)]) does not exist")
            }
            return pointer
        }
        if body1 == nil {
            return action(JPH_Body_GetFixedToWorldBody()!, bodies[0])
        }
        return action(bodies[0], bodies[1])
    }
}

/// Returns a unit vector perpendicular to `v`.
func perpendicular(to v: SIMD3<Float>) -> SIMD3<Float> {
    let reference: SIMD3<Float> = abs(v.x) < 0.9 ? [1, 0, 0] : [0, 1, 0]
    return simd_normalize(simd_cross(v, reference))
}

// MARK: - FixedConstraint

/// Welds two bodies together so they move as one.
public final class FixedConstraint: Constraint {}

extension PhysicsWorld {
    /// Welds `body2` to `body1` (or to the world when `body1` is nil) in their current relative pose.
    @discardableResult
    public func addFixedConstraint(between body1: BodyID?, and body2: BodyID) -> FixedConstraint {
        var settings = JPH_FixedConstraintSettings()
        JPH_FixedConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.autoDetectPoint = true
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_FixedConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(FixedConstraint(handle: handle))
    }
}

// MARK: - DistanceConstraint

/// Keeps two attachment points within a distance range, like a rope (`0...length`) or a rod (`length...length`).
public final class DistanceConstraint: Constraint {
    public var distance: ClosedRange<Float> {
        get { JPH_DistanceConstraint_GetMinDistance(handle)...JPH_DistanceConstraint_GetMaxDistance(handle) }
        set { JPH_DistanceConstraint_SetDistance(handle, newValue.lowerBound, newValue.upperBound) }
    }

    /// Makes the distance limits springy, e.g. a bungee cord.
    public var limitsSpring: SpringSettings {
        get {
            var result = JPH_SpringSettings()
            JPH_DistanceConstraint_GetLimitsSpringSettings(handle, &result)
            return SpringSettings(result)
        }
        set {
            var value = newValue.jph
            JPH_DistanceConstraint_SetLimitsSpringSettings(handle, &value)
        }
    }
}

extension PhysicsWorld {
    /// Connects world-space `point1` on `body1` (or the world when `body1` is nil) to `point2` on `body2`.
    /// A nil `distance` fixes it at the current distance between the points.
    @discardableResult
    public func addDistanceConstraint(
        between body1: BodyID?,
        and body2: BodyID,
        point1: SIMD3<Float>,
        point2: SIMD3<Float>,
        distance: ClosedRange<Float>? = nil
    ) -> DistanceConstraint {
        let range = distance ?? simd_distance(point1, point2)...simd_distance(point1, point2)
        var settings = JPH_DistanceConstraintSettings()
        JPH_DistanceConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.point1 = JPH_RVec3(point1)
        settings.point2 = JPH_RVec3(point2)
        settings.minDistance = range.lowerBound
        settings.maxDistance = range.upperBound
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_DistanceConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(DistanceConstraint(handle: handle))
    }
}

// MARK: - SliderConstraint

/// Lets two bodies translate along one axis relative to each other, with no rotation.
public final class SliderConstraint: Constraint {
    /// Offset along the slider axis, relative to the pose when the constraint was created.
    public var currentPosition: Float {
        JPH_SliderConstraint_GetCurrentPosition(handle)
    }

    /// Position limits along the axis. `nil` means unlimited.
    public var limits: ClosedRange<Float>? {
        get {
            JPH_SliderConstraint_HasLimits(handle)
                ? JPH_SliderConstraint_GetLimitsMin(handle)...JPH_SliderConstraint_GetLimitsMax(handle)
                : nil
        }
        set {
            let range = newValue ?? -.greatestFiniteMagnitude ... .greatestFiniteMagnitude
            JPH_SliderConstraint_SetLimits(handle, range.lowerBound, range.upperBound)
        }
    }

    public var maxFrictionForce: Float {
        get { JPH_SliderConstraint_GetMaxFrictionForce(handle) }
        set { JPH_SliderConstraint_SetMaxFrictionForce(handle, newValue) }
    }

    public var limitsSpring: SpringSettings {
        get {
            var result = JPH_SpringSettings()
            JPH_SliderConstraint_GetLimitsSpringSettings(handle, &result)
            return SpringSettings(result)
        }
        set {
            var value = newValue.jph
            JPH_SliderConstraint_SetLimitsSpringSettings(handle, &value)
        }
    }

    public var motorSettings: MotorSettings {
        get {
            var result = JPH_MotorSettings()
            JPH_SliderConstraint_GetMotorSettings(handle, &result)
            return MotorSettings(result)
        }
        set {
            var value = newValue.jph
            JPH_SliderConstraint_SetMotorSettings(handle, &value)
        }
    }

    public var motorState: MotorState {
        get { MotorState(JPH_SliderConstraint_GetMotorState(handle)) }
        set { JPH_SliderConstraint_SetMotorState(handle, newValue.jph) }
    }

    /// Used when `motorState` is `.velocity`. Meters per second.
    public var targetVelocity: Float {
        get { JPH_SliderConstraint_GetTargetVelocity(handle) }
        set { JPH_SliderConstraint_SetTargetVelocity(handle, newValue) }
    }

    /// Used when `motorState` is `.position`. Meters.
    public var targetPosition: Float {
        get { JPH_SliderConstraint_GetTargetPosition(handle) }
        set { JPH_SliderConstraint_SetTargetPosition(handle, newValue) }
    }
}

extension PhysicsWorld {
    /// Lets `body2` slide relative to `body1` (or the world when `body1` is nil) along a world-space axis.
    @discardableResult
    public func addSliderConstraint(
        between body1: BodyID?,
        and body2: BodyID,
        axis: SIMD3<Float>,
        limits: ClosedRange<Float>? = nil
    ) -> SliderConstraint {
        var settings = JPH_SliderConstraintSettings()
        JPH_SliderConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.autoDetectPoint = true
        var sliderAxis = JPH_Vec3(simd_normalize(axis))
        JPH_SliderConstraintSettings_SetSliderAxis(&settings, &sliderAxis)
        if let limits {
            settings.limitsMin = limits.lowerBound
            settings.limitsMax = limits.upperBound
        }
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_SliderConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(SliderConstraint(handle: handle))
    }
}

// MARK: - ConeConstraint

/// A ball joint whose twist axis may swing at most `halfConeAngle` away from its rest direction.
/// Twisting around the axis is free.
public final class ConeConstraint: Constraint {
    /// Radians.
    public var halfConeAngle: Float {
        get { acos(min(max(JPH_ConeConstraint_GetCosHalfConeAngle(handle), -1), 1)) }
        set { JPH_ConeConstraint_SetHalfConeAngle(handle, newValue) }
    }
}

extension PhysicsWorld {
    /// Joins `body2` to `body1` (or the world when `body1` is nil) at a world-space point. `twistAxis`
    /// is the rest direction of the cone, in world space.
    @discardableResult
    public func addConeConstraint(
        between body1: BodyID?,
        and body2: BodyID,
        point: SIMD3<Float>,
        twistAxis: SIMD3<Float>,
        halfConeAngle: Float
    ) -> ConeConstraint {
        let axis = JPH_Vec3(simd_normalize(twistAxis))
        var settings = JPH_ConeConstraintSettings()
        JPH_ConeConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.point1 = JPH_RVec3(point)
        settings.point2 = JPH_RVec3(point)
        settings.twistAxis1 = axis
        settings.twistAxis2 = axis
        settings.halfConeAngle = halfConeAngle
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_ConeConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(ConeConstraint(handle: handle))
    }
}

// MARK: - SwingTwistConstraint

/// Limits for a swing-twist joint, like a shoulder or hip.
public struct SwingTwistLimits: Sendable {
    public enum SwingShape: Sendable {
        /// Elliptical cone.
        case cone
        /// Box-shaped limits on the two swing angles.
        case pyramid
    }

    /// Max swing (radians) toward `cross(twistAxis, planeAxis)`.
    public var normalHalfConeAngle: Float
    /// Max swing (radians) toward `planeAxis`.
    public var planeHalfConeAngle: Float
    /// Allowed twist around the twist axis, radians.
    public var twist: ClosedRange<Float>
    public var swingShape: SwingShape
    public var maxFrictionTorque: Float

    public init(
        normalHalfConeAngle: Float,
        planeHalfConeAngle: Float,
        twist: ClosedRange<Float>,
        swingShape: SwingShape = .cone,
        maxFrictionTorque: Float = 0
    ) {
        self.normalHalfConeAngle = normalHalfConeAngle
        self.planeHalfConeAngle = planeHalfConeAngle
        self.twist = twist
        self.swingShape = swingShape
        self.maxFrictionTorque = maxFrictionTorque
    }
}

/// A ball joint with separate limits for swing (bending away from the twist axis) and twist
/// (rotating around it). joltc only allows setting the limits when the constraint is created,
/// and does not expose its motors.
public final class SwingTwistConstraint: Constraint {
    public var normalHalfConeAngle: Float {
        JPH_SwingTwistConstraint_GetNormalHalfConeAngle(handle)
    }
}

extension PhysicsWorld {
    /// Joins `body2` to `body1` (or the world when `body1` is nil) at a world-space point.
    /// `twistAxis` is the rest direction of the joint; `planeAxis` must be perpendicular to it and
    /// picks which way `planeHalfConeAngle` applies.
    @discardableResult
    public func addSwingTwistConstraint(
        between body1: BodyID?,
        and body2: BodyID,
        point: SIMD3<Float>,
        twistAxis: SIMD3<Float>,
        planeAxis: SIMD3<Float>? = nil,
        limits: SwingTwistLimits
    ) -> SwingTwistConstraint {
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
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_SwingTwistConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(SwingTwistConstraint(handle: handle))
    }
}

// MARK: - SixDOFConstraint

/// One of the six degrees of freedom of a `SixDOFConstraint`, in constraint space.
public enum SixDOFAxis: Int, CaseIterable, Sendable {
    case translationX, translationY, translationZ
    case rotationX, rotationY, rotationZ

    var jph: JPH_SixDOFConstraintAxis {
        JPH_SixDOFConstraintAxis(rawValue: UInt32(rawValue))
    }

    var isTranslation: Bool {
        rawValue < 3
    }
}

public enum AxisLimit: Sendable, Equatable {
    case free
    case fixed
    /// Meters for translation axes, radians for rotation axes.
    case limited(ClosedRange<Float>)
}

/// Constrains each of the six degrees of freedom independently: free, fixed or limited.
/// Axes are in constraint space, built from `axisX` and `axisY` at creation.
public final class SixDOFConstraint: Constraint {
    public func limit(for axis: SixDOFAxis) -> AxisLimit {
        if JPH_SixDOFConstraint_IsFreeAxis(handle, axis.jph) {
            return .free
        }
        if JPH_SixDOFConstraint_IsFixedAxis(handle, axis.jph) {
            return .fixed
        }
        return .limited(JPH_SixDOFConstraint_GetLimitsMin(handle, axis.jph)...JPH_SixDOFConstraint_GetLimitsMax(handle, axis.jph))
    }

    public func maxFriction(for axis: SixDOFAxis) -> Float {
        JPH_SixDOFConstraint_GetMaxFriction(handle, axis.jph)
    }

    /// Newtons for translation axes, newton-meters for rotation axes.
    public func setMaxFriction(_ friction: Float, for axis: SixDOFAxis) {
        JPH_SixDOFConstraint_SetMaxFriction(handle, axis.jph, friction)
    }

    /// Springs only apply to translation limits.
    public func limitsSpring(for axis: SixDOFAxis) -> SpringSettings {
        precondition(axis.isTranslation, "Limit springs only apply to translation axes")
        var result = JPH_SpringSettings()
        JPH_SixDOFConstraint_GetLimitsSpringSettings(handle, &result, axis.jph)
        return SpringSettings(result)
    }

    public func setLimitsSpring(_ spring: SpringSettings, for axis: SixDOFAxis) {
        precondition(axis.isTranslation, "Limit springs only apply to translation axes")
        var value = spring.jph
        JPH_SixDOFConstraint_SetLimitsSpringSettings(handle, &value, axis.jph)
    }

    public func motorSettings(for axis: SixDOFAxis) -> MotorSettings {
        var result = JPH_MotorSettings()
        JPH_SixDOFConstraint_GetMotorSettings(handle, axis.jph, &result)
        return MotorSettings(result)
    }

    public func motorState(for axis: SixDOFAxis) -> MotorState {
        MotorState(JPH_SixDOFConstraint_GetMotorState(handle, axis.jph))
    }

    public func setMotorState(_ state: MotorState, for axis: SixDOFAxis) {
        JPH_SixDOFConstraint_SetMotorState(handle, axis.jph, state.jph)
    }

    /// Constraint space. Used by translation axes with a `.velocity` motor.
    public var targetVelocity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_SixDOFConstraint_GetTargetVelocityCS(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_SixDOFConstraint_SetTargetVelocityCS(handle, &value)
        }
    }

    /// Constraint space. Used by rotation axes with a `.velocity` motor.
    public var targetAngularVelocity: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_SixDOFConstraint_GetTargetAngularVelocityCS(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_SixDOFConstraint_SetTargetAngularVelocityCS(handle, &value)
        }
    }

    /// Constraint space. Used by translation axes with a `.position` motor.
    public var targetPosition: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_SixDOFConstraint_GetTargetPositionCS(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Vec3(newValue)
            JPH_SixDOFConstraint_SetTargetPositionCS(handle, &value)
        }
    }

    /// Constraint space. Used by rotation axes with a `.position` motor.
    public var targetOrientation: simd_quatf {
        get {
            var result = JPH_Quat()
            JPH_SixDOFConstraint_GetTargetOrientationCS(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_Quat(newValue)
            JPH_SixDOFConstraint_SetTargetOrientationCS(handle, &value)
        }
    }

    /// Current rotation of body2 relative to body1, in constraint space.
    public var rotationInConstraintSpace: simd_quatf {
        var result = JPH_Quat()
        JPH_SixDOFConstraint_GetRotationInConstraintSpace(handle, &result)
        return result.simd
    }
}

extension PhysicsWorld {
    /// Joins `body2` to `body1` (or the world when `body1` is nil) at a world-space point.
    /// `axisX` and `axisY` (world space, perpendicular) define the constraint axes; Z is their cross
    /// product. Axes missing from `limits` are free.
    @discardableResult
    public func addSixDOFConstraint(
        between body1: BodyID?,
        and body2: BodyID,
        point: SIMD3<Float>,
        axisX: SIMD3<Float> = [1, 0, 0],
        axisY: SIMD3<Float> = [0, 1, 0],
        limits: [SixDOFAxis: AxisLimit],
        motorSettings: [SixDOFAxis: MotorSettings] = [:]
    ) -> SixDOFConstraint {
        let x = simd_normalize(axisX)
        let y = simd_normalize(axisY)
        precondition(abs(simd_dot(x, y)) < 1e-3, "axisX and axisY must be perpendicular")
        var settings = JPH_SixDOFConstraintSettings()
        JPH_SixDOFConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.position1 = JPH_RVec3(point)
        settings.position2 = JPH_RVec3(point)
        settings.axisX1 = JPH_Vec3(x)
        settings.axisX2 = JPH_Vec3(x)
        settings.axisY1 = JPH_Vec3(y)
        settings.axisY2 = JPH_Vec3(y)
        for axis in SixDOFAxis.allCases {
            switch limits[axis] ?? .free {
            case .free:
                JPH_SixDOFConstraintSettings_MakeFreeAxis(&settings, axis.jph)
            case .fixed:
                JPH_SixDOFConstraintSettings_MakeFixedAxis(&settings, axis.jph)
            case .limited(let range):
                JPH_SixDOFConstraintSettings_SetLimitedAxis(&settings, axis.jph, range.lowerBound, range.upperBound)
            }
        }
        withUnsafeMutableBytes(of: &settings.motorSettings) { buffer in
            let motors = buffer.bindMemory(to: JPH_MotorSettings.self)
            for (axis, motor) in motorSettings {
                motors[axis.rawValue] = motor.jph
            }
        }
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_SixDOFConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(SixDOFConstraint(handle: handle))
    }
}

// MARK: - GearConstraint

/// Couples the rotation of two bodies around their hinge axes, like meshing gears.
/// Each body should also be held by a hinge (usually to the world).
public final class GearConstraint: Constraint {}

extension PhysicsWorld {
    /// Gears `body1` and `body2` together. `axis1` and `axis2` are their world-space hinge axes.
    /// `ratio` is teeth on gear 2 / teeth on gear 1, so gear 2 turns `1 / ratio` times as fast,
    /// in the opposite direction, like real meshing gears. Pass the bodies' hinges to let Jolt
    /// correct angle drift over time.
    @discardableResult
    public func addGearConstraint(
        between body1: BodyID,
        and body2: BodyID,
        axis1: SIMD3<Float>,
        axis2: SIMD3<Float>,
        ratio: Float,
        hinges: (HingeConstraint, HingeConstraint)? = nil
    ) -> GearConstraint {
        var settings = JPH_GearConstraintSettings()
        JPH_GearConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.hingeAxis1 = JPH_Vec3(simd_normalize(axis1))
        settings.hingeAxis2 = JPH_Vec3(simd_normalize(axis2))
        settings.ratio = ratio
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_GearConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        if let hinges {
            JPH_GearConstraint_SetConstraints(handle, hinges.0.handle, hinges.1.handle)
        }
        return add(GearConstraint(handle: handle))
    }
}

// MARK: - PointConstraint

/// Pins two bodies together at a point, leaving rotation free, like a ball joint.
public final class PointConstraint: Constraint {
    /// Attachment point in body1's local space, relative to its center of mass.
    public var localPoint1: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_PointConstraint_GetLocalSpacePoint1(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_RVec3(newValue)
            JPH_PointConstraint_SetPoint1(handle, JPH_ConstraintSpace_LocalToBodyCOM, &value)
        }
    }

    /// Attachment point in body2's local space, relative to its center of mass.
    public var localPoint2: SIMD3<Float> {
        get {
            var result = JPH_Vec3()
            JPH_PointConstraint_GetLocalSpacePoint2(handle, &result)
            return result.simd
        }
        set {
            var value = JPH_RVec3(newValue)
            JPH_PointConstraint_SetPoint2(handle, JPH_ConstraintSpace_LocalToBodyCOM, &value)
        }
    }
}

extension PhysicsWorld {
    /// Pins `body2` to `body1` (or to the world when `body1` is nil) at a world-space point.
    @discardableResult
    public func addPointConstraint(between body1: BodyID?, and body2: BodyID, point: SIMD3<Float>) -> PointConstraint {
        var settings = JPH_PointConstraintSettings()
        JPH_PointConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.point1 = JPH_RVec3(point)
        settings.point2 = JPH_RVec3(point)
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_PointConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(PointConstraint(handle: handle))
    }
}

// MARK: - HingeConstraint

/// Lets two bodies rotate around a shared axis, like a door hinge.
public final class HingeConstraint: Constraint {
    /// Angle around the hinge axis in radians, relative to the pose when the constraint was created.
    public var currentAngle: Float {
        JPH_HingeConstraint_GetCurrentAngle(handle)
    }

    /// Angle limits in radians. `nil` means the hinge rotates freely.
    public var limits: ClosedRange<Float>? {
        get {
            let range = JPH_HingeConstraint_GetLimitsMin(handle)...JPH_HingeConstraint_GetLimitsMax(handle)
            // Jolt stores ±π with rounding, so HasLimits can report true for a full circle.
            let tolerance: Float = 1e-4
            return range.lowerBound <= -.pi + tolerance && range.upperBound >= .pi - tolerance ? nil : range
        }
        set {
            let range = newValue ?? -.pi ... .pi
            JPH_HingeConstraint_SetLimits(handle, range.lowerBound, range.upperBound)
        }
    }

    public var maxFrictionTorque: Float {
        get { JPH_HingeConstraint_GetMaxFrictionTorque(handle) }
        set { JPH_HingeConstraint_SetMaxFrictionTorque(handle, newValue) }
    }

    public var limitsSpring: SpringSettings {
        get {
            var result = JPH_SpringSettings()
            JPH_HingeConstraint_GetLimitsSpringSettings(handle, &result)
            return SpringSettings(result)
        }
        set {
            var value = newValue.jph
            JPH_HingeConstraint_SetLimitsSpringSettings(handle, &value)
        }
    }

    public var motorSettings: MotorSettings {
        get {
            var result = JPH_MotorSettings()
            JPH_HingeConstraint_GetMotorSettings(handle, &result)
            return MotorSettings(result)
        }
        set {
            var value = newValue.jph
            JPH_HingeConstraint_SetMotorSettings(handle, &value)
        }
    }

    public var motorState: MotorState {
        get { MotorState(JPH_HingeConstraint_GetMotorState(handle)) }
        set { JPH_HingeConstraint_SetMotorState(handle, newValue.jph) }
    }

    /// Used when `motorState` is `.velocity`. Radians per second.
    public var targetAngularVelocity: Float {
        get { JPH_HingeConstraint_GetTargetAngularVelocity(handle) }
        set { JPH_HingeConstraint_SetTargetAngularVelocity(handle, newValue) }
    }

    /// Used when `motorState` is `.position`. Radians.
    public var targetAngle: Float {
        get { JPH_HingeConstraint_GetTargetAngle(handle) }
        set { JPH_HingeConstraint_SetTargetAngle(handle, newValue) }
    }
}

extension PhysicsWorld {
    /// Hinges `body2` to `body1` (or to the world when `body1` is nil) at a world-space point and axis.
    @discardableResult
    public func addHingeConstraint(
        between body1: BodyID?,
        and body2: BodyID,
        point: SIMD3<Float>,
        axis: SIMD3<Float>,
        limits: ClosedRange<Float>? = nil
    ) -> HingeConstraint {
        let hingeAxis = simd_normalize(axis)
        let normalAxis = perpendicular(to: hingeAxis)
        var settings = JPH_HingeConstraintSettings()
        JPH_HingeConstraintSettings_Init(&settings)
        settings.space = JPH_ConstraintSpace_WorldSpace
        settings.point1 = JPH_RVec3(point)
        settings.point2 = JPH_RVec3(point)
        settings.hingeAxis1 = JPH_Vec3(hingeAxis)
        settings.hingeAxis2 = JPH_Vec3(hingeAxis)
        settings.normalAxis1 = JPH_Vec3(normalAxis)
        settings.normalAxis2 = JPH_Vec3(normalAxis)
        if let limits {
            settings.limitsMin = limits.lowerBound
            settings.limitsMax = limits.upperBound
        }
        let handle = withBodies(body1, body2) { jphBody1, jphBody2 in
            JPH_HingeConstraint_Create(&settings, jphBody1, jphBody2)!
        }
        return add(HingeConstraint(handle: handle))
    }
}
