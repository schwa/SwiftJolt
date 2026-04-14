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
    }
}

public struct EngineSettings: Sendable {
    /// Newton-meters.
    public var maxTorque: Float = 500
    public var rpm: ClosedRange<Float> = 1000...6000

    public init(maxTorque: Float = 500, rpm: ClosedRange<Float> = 1000...6000) {
        self.maxTorque = maxTorque
        self.rpm = rpm
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
        let contactID = JPH_Wheel_GetContactBodyID(wheel)
        var contactPosition = JPH_RVec3()
        var contactNormal = JPH_Vec3()
        JPH_Wheel_GetContactPosition(wheel, &contactPosition)
        JPH_Wheel_GetContactNormal(wheel, &contactNormal)
        return WheelState(
            angularVelocity: JPH_Wheel_GetAngularVelocity(wheel),
            rotationAngle: JPH_Wheel_GetRotationAngle(wheel),
            steerAngle: JPH_Wheel_GetSteerAngle(wheel),
            suspensionLength: JPH_Wheel_GetSuspensionLength(wheel),
            // Jolt marks "no contact" with an all-ones body ID.
            contactBody: contactID == .max ? nil : BodyID(rawValue: contactID),
            contactPosition: contactPosition.simd,
            contactNormal: contactNormal.simd
        )
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
        collisionTester: OpaquePointer
    ) -> V {
        let wheels = (0..<wheelCount).map(makeWheel)
        defer {
            for wheel in wheels {
                JPH_WheelSettings_Destroy(wheel)
            }
            JPH_VehicleControllerSettings_Destroy(controllerSettings)
            JPH_VehicleCollisionTester_Destroy(collisionTester)
        }
        var settings = JPH_VehicleConstraintSettings()
        JPH_VehicleConstraintSettings_Init(&settings)
        settings.up = JPH_Vec3(simd_normalize(up))
        settings.forward = JPH_Vec3(simd_normalize(forward))
        settings.maxPitchRollAngle = maxPitchRollAngle
        settings.controller = controllerSettings
        var wheelPointers: [OpaquePointer?] = wheels
        let handle = wheelPointers.withUnsafeMutableBufferPointer { buffer in
            settings.wheelsCount = UInt32(buffer.count)
            settings.wheels = buffer.baseAddress
            return withBodies(nil, body) { _, jphBody in
                JPH_VehicleConstraint_Create(jphBody, &settings)!
            }
        }
        JPH_VehicleConstraint_SetVehicleCollisionTester(handle, collisionTester)
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

    /// 0 is neutral, negative is reverse.
    public var currentGear: Int {
        Int(JPH_VehicleTransmission_GetCurrentGear(JPH_WheeledVehicleController_GetTransmission(controller)))
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
        up: SIMD3<Float> = [0, 1, 0],
        forward: SIMD3<Float> = [0, 0, 1],
        maxPitchRollAngle: Float = .pi
    ) -> WheeledVehicle {
        precondition(drivenWheels.allSatisfy { wheels.indices.contains($0.left) && wheels.indices.contains($0.right) }, "Driven wheel index out of range")
        let controllerSettings = makeWheeledControllerSettings(drivenWheels: drivenWheels, engine: engine)
        return addVehicle(
            body: body,
            wheelCount: wheels.count,
            makeWheel: { wheels[$0].makeSettings() },
            controllerSettings: controllerSettings,
            up: up,
            forward: forward,
            maxPitchRollAngle: maxPitchRollAngle,
            collisionTester: JPH_VehicleCollisionTesterCastCylinder_Create(ObjectLayer.moving.rawValue, 0.1)!
        )
    }

    /// A wheel index of -1 in `drivenWheels` means "no wheel" (used for motorcycles).
    func makeWheeledControllerSettings(
        drivenWheels: [(left: Int, right: Int)],
        engine: EngineSettings,
        create: () -> OpaquePointer = { JPH_WheeledVehicleControllerSettings_Create()! }
    ) -> OpaquePointer {
        let settings = create()
        engine.apply(to: settings)
        JPH_WheeledVehicleControllerSettings_SetDifferentialsCount(settings, 0)
        for pair in drivenWheels {
            JPH_WheeledVehicleControllerSettings_AddDifferential(settings, Int32(pair.left), Int32(pair.right))
        }
        // AddDifferential leaves each at the default share; split the engine torque evenly.
        for index in 0..<drivenWheels.count {
            var differential = JPH_VehicleDifferentialSettings()
            JPH_WheeledVehicleControllerSettings_GetDifferential(settings, UInt32(index), &differential)
            differential.engineTorqueRatio = 1 / Float(drivenWheels.count)
            JPH_WheeledVehicleControllerSettings_SetDifferential(settings, UInt32(index), &differential)
        }
        return settings
    }
}

extension EngineSettings {
    /// Applies to wheeled (and motorcycle) controller settings, keeping Jolt's default torque curve.
    func apply(to controllerSettings: OpaquePointer) {
        var engine = JPH_VehicleEngineSettings()
        JPH_WheeledVehicleControllerSettings_GetEngine(controllerSettings, &engine)
        update(&engine)
        JPH_WheeledVehicleControllerSettings_SetEngine(controllerSettings, &engine)
    }

    func update(_ engine: inout JPH_VehicleEngineSettings) {
        engine.maxTorque = maxTorque
        engine.minRPM = rpm.lowerBound
        engine.maxRPM = rpm.upperBound
    }
}
