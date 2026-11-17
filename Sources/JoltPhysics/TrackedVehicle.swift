import CJolt
import simd

/// Settings for one road wheel of a tracked vehicle.
public struct TrackedVehicleWheel: Sendable {
    public var wheel: WheelSettings
    /// Friction along the track. Jolt's default is 4.
    public var longitudinalFriction: Float = 4
    /// Friction across the track. Jolt's default is 2.
    public var lateralFriction: Float = 2

    public init(_ wheel: WheelSettings) {
        self.wheel = wheel
    }

    func makeSettings() -> OpaquePointer {
        let settings = JPH_WheelSettingsTV_Create()!
        wheel.apply(to: settings)
        JPH_WheelSettingsTV_SetLongitudinalFriction(settings, longitudinalFriction)
        JPH_WheelSettingsTV_SetLateralFriction(settings, lateralFriction)
        return settings
    }
}

/// One track: the wheels it runs over (indices into the vehicle's wheel list) and which of them
/// the engine drives.
public struct Track: Sendable {
    public var wheels: [Int]
    public var drivenWheel: Int

    public init(wheels: [Int], drivenWheel: Int) {
        precondition(wheels.contains(drivenWheel), "The driven wheel must be one of the track's wheels")
        self.wheels = wheels
        self.drivenWheel = drivenWheel
    }
}

public enum TrackSide: Sendable {
    case left
    case right

    var jph: JPH_TrackSide {
        self == .left ? JPH_TrackSide_Left : JPH_TrackSide_Right
    }
}

public struct TrackDetails: Sendable, Equatable {
    public var drivenWheel: Int
    /// kg m².
    public var inertia: Float
    public var angularDamping: Float
    /// Newton-meters.
    public var maxBrakeTorque: Float
    /// Gearbox-to-track rotation ratio.
    public var differentialRatio: Float
}

/// A tank-like vehicle. It steers by running its two tracks at different speeds.
public final class TrackedVehicle: Vehicle {
    /// `forward` in -1...1. `leftRatio` and `rightRatio` (-1...1) scale each track's speed: equal
    /// values drive straight, opposite values turn in place. `brake` in 0...1. Wakes the body.
    public func setDriverInput(forward: Float, leftRatio: Float = 1, rightRatio: Float = 1, brake: Float = 0) {
        JPH_TrackedVehicleController_SetDriverInput(controller, forward, leftRatio, rightRatio, brake)
        wake()
    }

    public var forwardInput: Float {
        get { JPH_TrackedVehicleController_GetForwardInput(controller) }
        set {
            JPH_TrackedVehicleController_SetForwardInput(controller, newValue)
            wake()
        }
    }

    public var leftRatio: Float {
        get { JPH_TrackedVehicleController_GetLeftRatio(controller) }
        set {
            JPH_TrackedVehicleController_SetLeftRatio(controller, newValue)
            wake()
        }
    }

    public var rightRatio: Float {
        get { JPH_TrackedVehicleController_GetRightRatio(controller) }
        set {
            JPH_TrackedVehicleController_SetRightRatio(controller, newValue)
            wake()
        }
    }

    public var brakeInput: Float {
        get { JPH_TrackedVehicleController_GetBrakeInput(controller) }
        set {
            JPH_TrackedVehicleController_SetBrakeInput(controller, newValue)
            wake()
        }
    }

    public var engineRPM: Float {
        JPH_VehicleEngine_GetCurrentRPM(JPH_TrackedVehicleController_GetEngine(controller))
    }

    public var transmission: Transmission {
        Transmission(handle: JPH_TrackedVehicleController_GetTransmission(controller)!, vehicle: self)
    }

    /// 0 is neutral, negative is reverse.
    public var currentGear: Int {
        transmission.currentGear
    }

    /// Radians per second of the track's driven wheel.
    public func trackAngularVelocity(_ side: TrackSide) -> Float {
        JPH_VehicleTrack_GetAngularVelocity(JPH_TrackedVehicleController_GetTrack(controller, side.jph))
    }

    public func setTrackAngularVelocity(_ velocity: Float, side: TrackSide) {
        JPH_VehicleTrack_SetAngularVelocity(JPH_TrackedVehicleController_GetTrack(controller, side.jph), velocity)
        wake()
    }

    /// Settings Jolt uses for a track.
    public func trackDetails(_ side: TrackSide) -> TrackDetails {
        let track = JPH_TrackedVehicleController_GetTrack(controller, side.jph)
        return TrackDetails(
            drivenWheel: Int(JPH_VehicleTrack_GetDrivenWheel(track)),
            inertia: JPH_VehicleTrack_GetInertia(track),
            angularDamping: JPH_VehicleTrack_GetAngularDamping(track),
            maxBrakeTorque: JPH_VehicleTrack_GetMaxBrakeTorque(track),
            differentialRatio: JPH_VehicleTrack_GetDifferentialRatio(track)
        )
    }
}

extension PhysicsWorld {
    /// Turns `body` (a dynamic body) into a tracked vehicle.
    @discardableResult
    public func addTrackedVehicle(
        body: BodyID,
        wheels: [TrackedVehicleWheel],
        leftTrack: Track,
        rightTrack: Track,
        engine: EngineSettings = EngineSettings(),
        transmission: TransmissionSettings = .tracked,
        up: SIMD3<Float> = [0, 1, 0],
        forward: SIMD3<Float> = [0, 0, 1],
        maxPitchRollAngle: Float = .pi,
        antiRollBars: [AntiRollBar] = [],
        collisionTester: WheelCollisionTester = .default
    ) -> TrackedVehicle {
        for track in [leftTrack, rightTrack] {
            precondition(track.wheels.allSatisfy(wheels.indices.contains), "Track wheel index out of range")
        }
        let controllerSettings = JPH_TrackedVehicleControllerSettings_Create()!
        var engineSettings = JPH_VehicleEngineSettings()
        JPH_TrackedVehicleControllerSettings_GetEngine(controllerSettings, &engineSettings)
        engine.withJoltSettings(startingFrom: engineSettings) { JPH_TrackedVehicleControllerSettings_SetEngine(controllerSettings, $0) }
        transmission.apply { JPH_TrackedVehicleControllerSettings_SetTransmission(controllerSettings, $0) }
        for (side, track) in [(JPH_TrackSide_Left, leftTrack), (JPH_TrackSide_Right, rightTrack)] {
            var settings = JPH_VehicleTrackSettings()
            JPH_VehicleTrackSettings_Init(&settings)
            let wheelIndices = track.wheels.map(UInt32.init)
            wheelIndices.withUnsafeBufferPointer { buffer in
                settings.wheels = buffer.baseAddress
                settings.wheelsCount = UInt32(buffer.count)
                settings.drivenWheel = UInt32(track.drivenWheel)
                JPH_TrackedVehicleControllerSettings_SetTrack(controllerSettings, side.rawValue, &settings)
            }
        }
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
}
