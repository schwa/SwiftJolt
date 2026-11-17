import CJolt
import simd

// MARK: - Shared settings

/// Suspension and size of one wheel. `position` is where the suspension attaches, in the vehicle
/// body's local space. Defaults match Jolt's.
public struct WheelSettings: Sendable {
    public var position: SIMD3<Float>
    public var radius: Float = 0.3
    public var width: Float = 0.1
    public var suspensionLength: ClosedRange<Float> = 0.3...0.5
    public var suspensionSpring = SpringSettings(.frequency(1.5), damping: 0.5)
    /// Extra compression at rest, in meters, so the spring already pushes when fully extended.
    public var suspensionPreloadLength: Float = 0
    /// Direction the suspension extends in, in body space.
    public var suspensionDirection: SIMD3<Float> = [0, -1, 0]
    /// Axis the wheel steers around, in body space.
    public var steeringAxis: SIMD3<Float> = [0, 1, 0]
    /// Wheel up and forward when not steering, in body space.
    public var wheelUp: SIMD3<Float> = [0, 1, 0]
    public var wheelForward: SIMD3<Float> = [0, 0, 1]
    /// Where the suspension force is applied, in body space. nil applies it at the contact point.
    public var suspensionForcePoint: SIMD3<Float>?

    public init(position: SIMD3<Float>) {
        self.position = position
    }

    func apply(to settings: OpaquePointer) {
        var p = JPH_Vec3(position)
        JPH_WheelSettings_SetPosition(settings, &p)
        JPH_WheelSettings_SetRadius(settings, radius)
        JPH_WheelSettings_SetWidth(settings, width)
        JPH_WheelSettings_SetSuspensionMinLength(settings, suspensionLength.lowerBound)
        JPH_WheelSettings_SetSuspensionMaxLength(settings, suspensionLength.upperBound)
        var spring = suspensionSpring.jph
        JPH_WheelSettings_SetSuspensionSpring(settings, &spring)
        JPH_WheelSettings_SetSuspensionPreloadLength(settings, suspensionPreloadLength)
        var direction = JPH_Vec3(simd_normalize(suspensionDirection))
        JPH_WheelSettings_SetSuspensionDirection(settings, &direction)
        var steering = JPH_Vec3(simd_normalize(steeringAxis))
        JPH_WheelSettings_SetSteeringAxis(settings, &steering)
        var up = JPH_Vec3(simd_normalize(wheelUp))
        JPH_WheelSettings_SetWheelUp(settings, &up)
        var forward = JPH_Vec3(simd_normalize(wheelForward))
        JPH_WheelSettings_SetWheelForward(settings, &forward)
        JPH_WheelSettings_SetEnableSuspensionForcePoint(settings, suspensionForcePoint != nil)
        if let suspensionForcePoint {
            var point = JPH_Vec3(suspensionForcePoint)
            JPH_WheelSettings_SetSuspensionForcePoint(settings, &point)
        }
    }
}

public struct EngineSettings: Sendable {
    /// Newton-meters.
    public var maxTorque: Float = 500
    public var rpm: ClosedRange<Float> = 1000...6000
    /// Fraction of `maxTorque` available across the RPM range. `rpm` 0 is `rpm.lowerBound`, 1 is
    /// `rpm.upperBound`. Empty keeps Jolt's curve: 0.8 at 0, 1.0 at 0.66, 0.8 at 1.
    public var torqueCurve: [(rpm: Float, torque: Float)]
    /// kg m².
    public var inertia: Float = 0.5
    public var angularDamping: Float = 0.2

    public init(
        maxTorque: Float = 500,
        rpm: ClosedRange<Float> = 1000...6000,
        torqueCurve: [(rpm: Float, torque: Float)] = [],
        inertia: Float = 0.5,
        angularDamping: Float = 0.2
    ) {
        self.maxTorque = maxTorque
        self.rpm = rpm
        self.torqueCurve = torqueCurve
        self.inertia = inertia
        self.angularDamping = angularDamping
    }
}

/// State of a wheel after the last update.
public struct WheelState: Sendable {
    /// Radians per second around the wheel axle.
    public var angularVelocity: Float
    public var rotationAngle: Float
    public var steerAngle: Float
    public var suspensionLength: Float
    /// The body under the wheel, or nil when the wheel is in the air.
    public var contactBody: BodyID?
    public var contactPosition: SIMD3<Float>
    public var contactNormal: SIMD3<Float>
    /// Which part of the contact body's shape the wheel touches.
    public var contactSubShape: SubShapeID
    /// Velocity of the ground at the contact point.
    public var contactPointVelocity: SIMD3<Float>
    /// Rolling direction and sideways direction at the contact.
    public var contactLongitudinal: SIMD3<Float>
    public var contactLateral: SIMD3<Float>
    /// Impulses applied in the last step by the suspension and the tire's friction.
    public var suspensionLambda: Float
    public var longitudinalLambda: Float
    public var lateralLambda: Float
    /// True when the suspension bottomed out.
    public var hasHitHardPoint: Bool
}

/// Runs `body` with a temporary Jolt curve. Jolt copies curves when they are set.
func withLinearCurve(_ points: [(Float, Float)], _ body: (OpaquePointer) -> Void) {
    let curve = JPH_LinearCurve_Create()!
    defer { JPH_LinearCurve_Destroy(curve) }
    for (x, y) in points {
        JPH_LinearCurve_AddPoint(curve, x, y)
    }
    JPH_LinearCurve_Sort(curve)
    body(curve)
}

// MARK: - Wheel collision

/// How a vehicle's wheels find the ground.
public struct WheelCollisionTester: Sendable, Equatable {
    public enum Kind: Sendable, Equatable {
        /// A ray per wheel. Cheapest; wheels can drop into gaps narrower than the wheel.
        /// Ground steeper than `maxSlopeAngle` (radians) is ignored.
        case ray(maxSlopeAngle: Float)
        /// A swept sphere per wheel.
        case castSphere(radius: Float, maxSlopeAngle: Float)
        /// A swept cylinder matching the wheel. Most accurate. `convexRadiusFraction` rounds its edges, in 0...1.
        case castCylinder(convexRadiusFraction: Float)
    }

    public var kind: Kind
    /// Wheels only collide with bodies that this layer collides with.
    public var objectLayer: ObjectLayer
    /// World up, used by the ray and sphere testers to measure slopes.
    public var up: SIMD3<Float>

    public init(_ kind: Kind, objectLayer: ObjectLayer = .moving, up: SIMD3<Float> = [0, 1, 0]) {
        self.kind = kind
        self.objectLayer = objectLayer
        self.up = up
    }

    public static func ray(maxSlopeAngle: Float = 80 * .pi / 180, objectLayer: ObjectLayer = .moving) -> Self {
        Self(.ray(maxSlopeAngle: maxSlopeAngle), objectLayer: objectLayer)
    }

    public static func castSphere(radius: Float, maxSlopeAngle: Float = 80 * .pi / 180, objectLayer: ObjectLayer = .moving) -> Self {
        Self(.castSphere(radius: radius, maxSlopeAngle: maxSlopeAngle), objectLayer: objectLayer)
    }

    public static func castCylinder(convexRadiusFraction: Float = 0.1, objectLayer: ObjectLayer = .moving) -> Self {
        Self(.castCylinder(convexRadiusFraction: convexRadiusFraction), objectLayer: objectLayer)
    }

    public static let `default` = castCylinder()

    /// Returns an owned reference; release it with `JPH_VehicleCollisionTester_Destroy`.
    func createHandle() -> OpaquePointer {
        var jphUp = JPH_Vec3(simd_normalize(up))
        switch kind {
        case .ray(let maxSlopeAngle):
            return JPH_VehicleCollisionTesterRay_Create(objectLayer.rawValue, &jphUp, maxSlopeAngle)!
        case .castSphere(let radius, let maxSlopeAngle):
            return JPH_VehicleCollisionTesterCastSphere_Create(objectLayer.rawValue, radius, &jphUp, maxSlopeAngle)!
        case .castCylinder(let convexRadiusFraction):
            return JPH_VehicleCollisionTesterCastCylinder_Create(objectLayer.rawValue, convexRadiusFraction)!
        }
    }
}

/// A spring between a left and right wheel that resists body roll in corners.
public struct AntiRollBar: Sendable, Equatable {
    public var leftWheel: Int
    public var rightWheel: Int
    /// Newton-meters per radian of roll.
    public var stiffness: Float

    public init(leftWheel: Int, rightWheel: Int, stiffness: Float = 1000) {
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        self.stiffness = stiffness
    }
}

// MARK: - Vehicle

/// Base class for vehicles. A vehicle is a constraint on its body that also runs every physics
/// step. Remove it with `PhysicsWorld.removeConstraint`.
public class Vehicle: Constraint {
    let stepListener: OpaquePointer
    /// The vehicle's body.
    public let body: BodyID
    // Weak: the world holds its constraints strongly.
    weak var world: PhysicsWorld?

    required init(handle: OpaquePointer, world: PhysicsWorld, body: BodyID) {
        stepListener = JPH_VehicleConstraint_AsPhysicsStepListener(handle)!
        self.body = body
        self.world = world
        super.init(handle: handle)
        JPH_PhysicsSystem_AddStepListener(world.system, stepListener)
    }

    // Jolt's vehicle samples wake the body whenever driver input changes; do the same.
    func wake() {
        guard let world else {
            return
        }
        JPH_BodyInterface_ActivateBody(world.bodyInterface, body.rawValue)
    }

    override func willRemove(from system: OpaquePointer) {
        JPH_PhysicsSystem_RemoveStepListener(system, stepListener)
    }

    var controller: OpaquePointer {
        JPH_VehicleConstraint_GetController(handle)!
    }

    public var wheelCount: Int {
        Int(JPH_VehicleConstraint_GetWheelsCount(handle))
    }

    public func wheelState(at index: Int) -> WheelState {
        precondition(index >= 0 && index < wheelCount, "Wheel index out of range")
        let wheel = JPH_VehicleConstraint_GetWheel(handle, UInt32(index))!
        var contactPosition = JPH_RVec3()
        JPH_Wheel_GetContactPosition(wheel, &contactPosition)
        func vector(_ read: (OpaquePointer, UnsafeMutablePointer<JPH_Vec3>) -> Void) -> SIMD3<Float> {
            var result = JPH_Vec3()
            read(wheel, &result)
            return result.simd
        }
        return WheelState(
            angularVelocity: JPH_Wheel_GetAngularVelocity(wheel),
            rotationAngle: JPH_Wheel_GetRotationAngle(wheel),
            steerAngle: JPH_Wheel_GetSteerAngle(wheel),
            suspensionLength: JPH_Wheel_GetSuspensionLength(wheel),
            contactBody: JPH_Wheel_HasContact(wheel) ? BodyID(rawValue: JPH_Wheel_GetContactBodyID(wheel)) : nil,
            contactPosition: contactPosition.simd,
            contactNormal: vector { JPH_Wheel_GetContactNormal($0, $1) },
            contactSubShape: SubShapeID(rawValue: JPH_Wheel_GetContactSubShapeID(wheel)),
            contactPointVelocity: vector { JPH_Wheel_GetContactPointVelocity($0, $1) },
            contactLongitudinal: vector { JPH_Wheel_GetContactLongitudinal($0, $1) },
            contactLateral: vector { JPH_Wheel_GetContactLateral($0, $1) },
            suspensionLambda: JPH_Wheel_GetSuspensionLambda(wheel),
            longitudinalLambda: JPH_Wheel_GetLongitudinalLambda(wheel),
            lateralLambda: JPH_Wheel_GetLateralLambda(wheel),
            hasHitHardPoint: JPH_Wheel_HasHitHardPoint(wheel)
        )
    }

    private func wheel(at index: Int) -> OpaquePointer {
        precondition(index >= 0 && index < wheelCount, "Wheel index out of range")
        return JPH_VehicleConstraint_GetWheel(handle, UInt32(index))!
    }

    /// Overrides a wheel's spin, e.g. after teleporting the vehicle. Radians per second.
    public func setWheelAngularVelocity(_ velocity: Float, at index: Int) {
        JPH_Wheel_SetAngularVelocity(wheel(at: index), velocity)
    }

    public func setWheelRotationAngle(_ angle: Float, at index: Int) {
        JPH_Wheel_SetRotationAngle(wheel(at: index), angle)
    }

    /// The controller sets steering from driver input every step, so this only lasts until then.
    public func setWheelSteerAngle(_ angle: Float, at index: Int) {
        JPH_Wheel_SetSteerAngle(wheel(at: index), angle)
    }

    /// A wheel's transform relative to the vehicle body. Axle along local X, Y up.
    public func wheelLocalTransform(at index: Int) -> simd_float4x4 {
        precondition(index >= 0 && index < wheelCount, "Wheel index out of range")
        var right = JPH_Vec3([1, 0, 0])
        var up = JPH_Vec3([0, 1, 0])
        var result = JPH_Mat4()
        JPH_VehicleConstraint_GetWheelLocalTransform(handle, UInt32(index), &right, &up, &result)
        return result.simd
    }

    /// The wheel's current forward, up and right directions in body space, including steering.
    public func wheelLocalBasis(at index: Int) -> (forward: SIMD3<Float>, up: SIMD3<Float>, right: SIMD3<Float>) {
        var forward = JPH_Vec3()
        var up = JPH_Vec3()
        var right = JPH_Vec3()
        JPH_VehicleConstraint_GetWheelLocalBasis(handle, wheel(at: index), &forward, &up, &right)
        return (forward.simd, up.simd, right.simd)
    }

    /// Vehicle forward and up in body space, as given at creation.
    public var localForward: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_VehicleConstraint_GetLocalForward(handle, &result)
        return result.simd
    }

    public var localUp: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_VehicleConstraint_GetLocalUp(handle, &result)
        return result.simd
    }

    /// The vehicle's up in world space after the last step (opposite gravity, or `localUp` rotated).
    public var worldUp: SIMD3<Float> {
        var result = JPH_Vec3()
        JPH_VehicleConstraint_GetWorldUp(handle, &result)
        return result.simd
    }

    /// Gravity used for this vehicle instead of the world's, e.g. to drive on walls. nil uses the world's.
    public var gravityOverride: SIMD3<Float>? {
        get {
            guard JPH_VehicleConstraint_IsGravityOverridden(handle) else {
                return nil
            }
            var result = JPH_Vec3()
            JPH_VehicleConstraint_GetGravityOverride(handle, &result)
            return result.simd
        }
        set {
            if let newValue {
                var value = JPH_Vec3(newValue)
                JPH_VehicleConstraint_OverrideGravity(handle, &value)
            } else {
                JPH_VehicleConstraint_ResetGravityOverride(handle)
            }
            wake()
        }
    }

    /// World transform of a wheel, for drawing. The wheel's axle is its local X axis and it rolls
    /// around it; Y points up.
    public func wheelWorldTransform(at index: Int) -> simd_float4x4 {
        precondition(index >= 0 && index < wheelCount, "Wheel index out of range")
        var right = JPH_Vec3([1, 0, 0])
        var up = JPH_Vec3([0, 1, 0])
        var result = JPH_RMat4()
        JPH_VehicleConstraint_GetWheelWorldTransform(handle, UInt32(index), &right, &up, &result)
        return result.simd
    }

    /// Replaces how the wheels find the ground.
    public func setWheelCollisionTester(_ tester: WheelCollisionTester) {
        let testerHandle = tester.createHandle()
        // The constraint keeps its own reference.
        defer { JPH_VehicleCollisionTester_Destroy(testerHandle) }
        JPH_VehicleConstraint_SetVehicleCollisionTester(handle, testerHandle)
    }

    /// Radians. The vehicle constraint stops the body from pitching or rolling further than this.
    public func setMaxPitchRollAngle(_ angle: Float) {
        JPH_VehicleConstraint_SetMaxPitchRollAngle(handle, angle)
    }
}

extension PhysicsWorld {
    /// Builds and adds a vehicle constraint. `makeWheel` returns an owned JPH_WheelSettings for a
    /// wheel index; `controllerSettings` is consumed.
    func addVehicle<V: Vehicle>(
        body: BodyID,
        wheelCount: Int,
        makeWheel: (Int) -> OpaquePointer,
        controllerSettings: OpaquePointer,
        up: SIMD3<Float>,
        forward: SIMD3<Float>,
        maxPitchRollAngle: Float,
        collisionTester: WheelCollisionTester,
        antiRollBars: [AntiRollBar] = []
    ) -> V {
        precondition(
            antiRollBars.allSatisfy { (0..<wheelCount).contains($0.leftWheel) && (0..<wheelCount).contains($0.rightWheel) },
            "Anti-roll bar wheel index out of range"
        )
        let testerHandle = collisionTester.createHandle()
        let wheels = (0..<wheelCount).map(makeWheel)
        defer {
            for wheel in wheels {
                JPH_WheelSettings_Destroy(wheel)
            }
            JPH_VehicleControllerSettings_Destroy(controllerSettings)
            JPH_VehicleCollisionTester_Destroy(testerHandle)
        }
        var settings = JPH_VehicleConstraintSettings()
        JPH_VehicleConstraintSettings_Init(&settings)
        settings.up = JPH_Vec3(simd_normalize(up))
        settings.forward = JPH_Vec3(simd_normalize(forward))
        settings.maxPitchRollAngle = maxPitchRollAngle
        settings.controller = controllerSettings
        var wheelPointers: [OpaquePointer?] = wheels
        let jphAntiRollBars = antiRollBars.map { bar in
            var jph = JPH_VehicleAntiRollBar()
            JPH_VehicleAntiRollBar_Init(&jph)
            jph.leftWheel = Int32(bar.leftWheel)
            jph.rightWheel = Int32(bar.rightWheel)
            jph.stiffness = bar.stiffness
            return jph
        }
        let handle = wheelPointers.withUnsafeMutableBufferPointer { buffer in
            jphAntiRollBars.withUnsafeBufferPointer { bars in
                settings.wheelsCount = UInt32(buffer.count)
                settings.wheels = buffer.baseAddress
                settings.antiRollBarsCount = UInt32(bars.count)
                settings.antiRollBars = bars.baseAddress
                return withBodies(nil, body) { _, jphBody in
                    JPH_VehicleConstraint_Create(jphBody, &settings)!
                }
            }
        }
        JPH_VehicleConstraint_SetVehicleCollisionTester(handle, testerHandle)
        return add(V(handle: handle, world: self, body: body))
    }
}

// MARK: - WheeledVehicle

/// Settings for one wheel of a wheeled vehicle.
public struct WheeledVehicleWheel: Sendable {
    public var wheel: WheelSettings
    /// Radians. 0 for wheels that do not steer.
    public var maxSteerAngle: Float
    /// Newton-meters.
    public var maxBrakeTorque: Float = 1500
    public var maxHandBrakeTorque: Float = 4000
    /// kg m².
    public var inertia: Float = 0.9
    public var angularDamping: Float = 0.2
    /// Friction against longitudinal slip ratio. Empty keeps Jolt's curve.
    public var longitudinalFriction: [(slip: Float, friction: Float)] = []
    /// Friction against lateral slip angle in degrees. Empty keeps Jolt's curve.
    public var lateralFriction: [(slipAngle: Float, friction: Float)] = []

    public init(_ wheel: WheelSettings, maxSteerAngle: Float = 0) {
        self.wheel = wheel
        self.maxSteerAngle = maxSteerAngle
    }

    func makeSettings() -> OpaquePointer {
        let settings = JPH_WheelSettingsWV_Create()!
        wheel.apply(to: settings)
        JPH_WheelSettingsWV_SetMaxSteerAngle(settings, maxSteerAngle)
        JPH_WheelSettingsWV_SetMaxBrakeTorque(settings, maxBrakeTorque)
        JPH_WheelSettingsWV_SetMaxHandBrakeTorque(settings, maxHandBrakeTorque)
        JPH_WheelSettingsWV_SetInertia(settings, inertia)
        JPH_WheelSettingsWV_SetAngularDamping(settings, angularDamping)
        if !longitudinalFriction.isEmpty {
            withLinearCurve(longitudinalFriction.map { ($0.slip, $0.friction) }) { JPH_WheelSettingsWV_SetLongitudinalFriction(settings, $0) }
        }
        if !lateralFriction.isEmpty {
            withLinearCurve(lateralFriction.map { ($0.slipAngle, $0.friction) }) { JPH_WheelSettingsWV_SetLateralFriction(settings, $0) }
        }
        return settings
    }
}

/// A car-like vehicle with an engine, gearbox and differentials.
public class WheeledVehicle: Vehicle {
    /// Sets all driver inputs. `forward` and `right` are in -1...1, `brake` and `handBrake` in 0...1.
    /// Setting any input wakes the vehicle's body.
    public func setDriverInput(forward: Float, right: Float = 0, brake: Float = 0, handBrake: Float = 0) {
        JPH_WheeledVehicleController_SetDriverInput(controller, forward, right, brake, handBrake)
        wake()
    }

    public var forwardInput: Float {
        get { JPH_WheeledVehicleController_GetForwardInput(controller) }
        set {
            JPH_WheeledVehicleController_SetForwardInput(controller, newValue)
            wake()
        }
    }

    public var rightInput: Float {
        get { JPH_WheeledVehicleController_GetRightInput(controller) }
        set {
            JPH_WheeledVehicleController_SetRightInput(controller, newValue)
            wake()
        }
    }

    public var brakeInput: Float {
        get { JPH_WheeledVehicleController_GetBrakeInput(controller) }
        set {
            JPH_WheeledVehicleController_SetBrakeInput(controller, newValue)
            wake()
        }
    }

    public var handBrakeInput: Float {
        get { JPH_WheeledVehicleController_GetHandBrakeInput(controller) }
        set {
            JPH_WheeledVehicleController_SetHandBrakeInput(controller, newValue)
            wake()
        }
    }

    public var engineRPM: Float {
        JPH_VehicleEngine_GetCurrentRPM(JPH_WheeledVehicleController_GetEngine(controller))
    }

    public var transmission: Transmission {
        Transmission(handle: JPH_WheeledVehicleController_GetTransmission(controller)!, vehicle: self)
    }

    /// 0 is neutral, negative is reverse.
    public var currentGear: Int {
        transmission.currentGear
    }

    /// Spins a wheel by `torque` (Newton-meters) over `deltaTime`, outside the drivetrain.
    public func applyWheelTorque(_ torque: Float, deltaTime: Float, at index: Int) {
        precondition(index >= 0 && index < wheelCount, "Wheel index out of range")
        JPH_WheelWV_ApplyTorque(JPH_VehicleConstraint_GetWheel(handle, UInt32(index)), torque, deltaTime)
        wake()
    }

    /// Average rotation speed of the driven wheels as seen from the clutch, in RPM.
    public var wheelSpeedAtClutch: Float {
        JPH_WheeledVehicleController_GetWheelSpeedAtClutch(controller)
    }
}

extension PhysicsWorld {
    /// Turns `body` (a dynamic body) into a wheeled vehicle. `drivenWheels` lists left/right wheel
    /// index pairs connected to the engine through a differential. The engine torque is split
    /// evenly between them.
    @discardableResult
    public func addWheeledVehicle(
        body: BodyID,
        wheels: [WheeledVehicleWheel],
        drivenWheels: [(left: Int, right: Int)],
        engine: EngineSettings = EngineSettings(),
        transmission: TransmissionSettings = TransmissionSettings(),
        up: SIMD3<Float> = [0, 1, 0],
        forward: SIMD3<Float> = [0, 0, 1],
        maxPitchRollAngle: Float = .pi,
        antiRollBars: [AntiRollBar] = [],
        collisionTester: WheelCollisionTester = .default
    ) -> WheeledVehicle {
        addWheeledVehicle(
            body: body,
            wheels: wheels,
            differentials: drivenWheels.map { DifferentialSettings(leftWheel: $0.left, rightWheel: $0.right) },
            engine: engine,
            transmission: transmission,
            up: up,
            forward: forward,
            maxPitchRollAngle: maxPitchRollAngle,
            antiRollBars: antiRollBars,
            collisionTester: collisionTester
        )
    }

    /// Turns `body` into a wheeled vehicle with full control over the differentials. Differentials
    /// without an `engineTorqueRatio` share what the others leave evenly. `differentialLimitedSlipRatio`
    /// limits the speed difference between differentials (`.infinity` for none).
    @discardableResult
    public func addWheeledVehicle(
        body: BodyID,
        wheels: [WheeledVehicleWheel],
        differentials: [DifferentialSettings],
        differentialLimitedSlipRatio: Float = 1.4,
        engine: EngineSettings = EngineSettings(),
        transmission: TransmissionSettings = TransmissionSettings(),
        up: SIMD3<Float> = [0, 1, 0],
        forward: SIMD3<Float> = [0, 0, 1],
        maxPitchRollAngle: Float = .pi,
        antiRollBars: [AntiRollBar] = [],
        collisionTester: WheelCollisionTester = .default
    ) -> WheeledVehicle {
        precondition(
            differentials.allSatisfy { wheels.indices.contains($0.leftWheel) && wheels.indices.contains($0.rightWheel) },
            "Differential wheel index out of range"
        )
        let controllerSettings = makeWheeledControllerSettings(differentials: differentials, engine: engine, transmission: transmission)
        JPH_WheeledVehicleControllerSettings_SetDifferentialLimitedSlipRatio(controllerSettings, differentialLimitedSlipRatio)
        return addVehicle(
            body: body,
            wheelCount: wheels.count,
            makeWheel: { wheels[$0].makeSettings() },
            controllerSettings: controllerSettings,
            up: up,
            forward: forward,
            maxPitchRollAngle: maxPitchRollAngle,
            collisionTester: collisionTester,
            antiRollBars: antiRollBars
        )
    }

    /// A wheel index of -1 in a differential means "no wheel" (used for motorcycles).
    func makeWheeledControllerSettings(
        differentials: [DifferentialSettings],
        engine: EngineSettings,
        transmission: TransmissionSettings,
        create: () -> OpaquePointer = { JPH_WheeledVehicleControllerSettings_Create()! }
    ) -> OpaquePointer {
        let settings = create()
        engine.apply(to: settings)
        transmission.apply { JPH_WheeledVehicleControllerSettings_SetTransmission(settings, $0) }
        let jph = DifferentialSettings.joltSettings(for: differentials)
        JPH_WheeledVehicleControllerSettings_SetDifferentials(settings, jph, UInt32(jph.count))
        return settings
    }
}

extension EngineSettings {
    /// Applies to wheeled (and motorcycle) controller settings.
    func apply(to controllerSettings: OpaquePointer) {
        var engine = JPH_VehicleEngineSettings()
        JPH_WheeledVehicleControllerSettings_GetEngine(controllerSettings, &engine)
        withJoltSettings(startingFrom: engine) { JPH_WheeledVehicleControllerSettings_SetEngine(controllerSettings, $0) }
    }

    /// `base` comes from the controller settings; its curve pointer points into them and is kept
    /// when `torqueCurve` is empty. Jolt copies the curve when the settings are set.
    func withJoltSettings(startingFrom base: JPH_VehicleEngineSettings, _ body: (UnsafePointer<JPH_VehicleEngineSettings>) -> Void) {
        var engine = base
        engine.maxTorque = maxTorque
        engine.minRPM = rpm.lowerBound
        engine.maxRPM = rpm.upperBound
        engine.inertia = inertia
        engine.angularDamping = angularDamping
        guard !torqueCurve.isEmpty else {
            body(&engine)
            return
        }
        let curve = JPH_LinearCurve_Create()!
        defer { JPH_LinearCurve_Destroy(curve) }
        for point in torqueCurve {
            JPH_LinearCurve_AddPoint(curve, point.rpm, point.torque)
        }
        JPH_LinearCurve_Sort(curve)
        engine.normalizedTorque = curve
        body(&engine)
    }
}
