import CJolt

/// Gearbox settings for wheeled, tracked and motorcycle vehicles. Defaults are Jolt's.
public struct TransmissionSettings: Sendable {
    public enum Mode: Sendable {
        /// Shifts by itself using `shiftUpRPM` and `shiftDownRPM`.
        case automatic
        /// Stays in the gear set with `Transmission.setGear`.
        case manual

        var jph: JPH_TransmissionMode {
            self == .automatic ? JPH_TransmissionMode_Auto : JPH_TransmissionMode_Manual
        }
    }

    public var mode: Mode = .automatic
    /// Engine-to-gearbox rotation ratios, first gear first.
    public var gearRatios: [Float] = [2.66, 1.78, 1.3, 1.0, 0.74]
    /// Ratios for reverse gears. Negative.
    public var reverseGearRatios: [Float] = [-2.9]
    /// Seconds to change gear (automatic only).
    public var switchTime: Float = 0.5
    /// Seconds to fully engage the clutch after a change (automatic only).
    public var clutchReleaseTime: Float = 0.3
    /// Seconds to wait after a change before the next one (automatic only).
    public var switchLatency: Float = 0.5
    public var shiftUpRPM: Float = 4000
    public var shiftDownRPM: Float = 2000
    public var clutchStrength: Float = 10

    public init(
        mode: Mode = .automatic,
        gearRatios: [Float] = [2.66, 1.78, 1.3, 1.0, 0.74],
        reverseGearRatios: [Float] = [-2.9],
        switchTime: Float = 0.5,
        clutchReleaseTime: Float = 0.3,
        switchLatency: Float = 0.5,
        shiftUpRPM: Float = 4000,
        shiftDownRPM: Float = 2000,
        clutchStrength: Float = 10
    ) {
        precondition(!gearRatios.isEmpty, "A transmission needs at least one forward gear")
        self.mode = mode
        self.gearRatios = gearRatios
        self.reverseGearRatios = reverseGearRatios
        self.switchTime = switchTime
        self.clutchReleaseTime = clutchReleaseTime
        self.switchLatency = switchLatency
        self.shiftUpRPM = shiftUpRPM
        self.shiftDownRPM = shiftDownRPM
        self.clutchStrength = clutchStrength
    }

    /// Jolt's defaults for tracked vehicles: four low gears and two reverse gears.
    public static let tracked = Self(gearRatios: [4, 3, 2, 1], reverseGearRatios: [-4, -3], shiftUpRPM: 3500, shiftDownRPM: 1000)

    /// Builds Jolt settings, passes them to `body` (which copies them), then frees them.
    func apply(_ body: (OpaquePointer) -> Void) {
        let settings = JPH_VehicleTransmissionSettings_Create()!
        defer { JPH_VehicleTransmissionSettings_Destroy(settings) }
        JPH_VehicleTransmissionSettings_SetMode(settings, mode.jph)
        JPH_VehicleTransmissionSettings_SetGearRatios(settings, gearRatios, UInt32(gearRatios.count))
        JPH_VehicleTransmissionSettings_SetReverseGearRatios(settings, reverseGearRatios, UInt32(reverseGearRatios.count))
        JPH_VehicleTransmissionSettings_SetSwitchTime(settings, switchTime)
        JPH_VehicleTransmissionSettings_SetClutchReleaseTime(settings, clutchReleaseTime)
        JPH_VehicleTransmissionSettings_SetSwitchLatency(settings, switchLatency)
        JPH_VehicleTransmissionSettings_SetShiftUpRPM(settings, shiftUpRPM)
        JPH_VehicleTransmissionSettings_SetShiftDownRPM(settings, shiftDownRPM)
        JPH_VehicleTransmissionSettings_SetClutchStrength(settings, clutchStrength)
        body(settings)
    }
}

/// Connects a left/right wheel pair to the gearbox. Defaults are Jolt's.
public struct DifferentialSettings: Sendable {
    /// Wheel index, or -1 for no wheel.
    public var leftWheel: Int
    public var rightWheel: Int
    /// Gearbox-to-wheel rotation ratio.
    public var ratio: Float = 3.42
    /// Torque split: 0 all left, 0.5 even, 1 all right.
    public var leftRightSplit: Float = 0.5
    /// Max/min wheel speed ratio before torque goes to the slower wheel. `.infinity` for an open differential.
    public var limitedSlipRatio: Float = 1.4
    /// Share of engine torque for this differential. nil shares what the others leave evenly.
    public var engineTorqueRatio: Float?

    public init(
        leftWheel: Int,
        rightWheel: Int,
        ratio: Float = 3.42,
        leftRightSplit: Float = 0.5,
        limitedSlipRatio: Float = 1.4,
        engineTorqueRatio: Float? = nil
    ) {
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        self.ratio = ratio
        self.leftRightSplit = leftRightSplit
        self.limitedSlipRatio = limitedSlipRatio
        self.engineTorqueRatio = engineTorqueRatio
    }

    static func joltSettings(for differentials: [Self]) -> [JPH_VehicleDifferentialSettings] {
        let assigned = differentials.compactMap(\.engineTorqueRatio).reduce(0, +)
        let unassignedCount = differentials.filter { $0.engineTorqueRatio == nil }.count
        let share = unassignedCount == 0 ? 0 : max(0, 1 - assigned) / Float(unassignedCount)
        return differentials.map { differential in
            JPH_VehicleDifferentialSettings(
                leftWheel: Int32(differential.leftWheel),
                rightWheel: Int32(differential.rightWheel),
                differentialRatio: differential.ratio,
                leftRightSplit: differential.leftRightSplit,
                limitedSlipRatio: differential.limitedSlipRatio == .infinity ? .greatestFiniteMagnitude : differential.limitedSlipRatio,
                engineTorqueRatio: differential.engineTorqueRatio ?? share
            )
        }
    }
}

/// A vehicle's live gearbox. Get one from `WheeledVehicle.transmission` or `TrackedVehicle.transmission`.
public struct Transmission {
    let handle: OpaquePointer
    // Keeps the vehicle, and so the gearbox, alive.
    let vehicle: Vehicle

    /// 0 is neutral, negative is reverse.
    public var currentGear: Int {
        Int(JPH_VehicleTransmission_GetCurrentGear(handle))
    }

    /// 0 is disengaged, 1 is fully engaged.
    public var clutchFriction: Float {
        JPH_VehicleTransmission_GetClutchFriction(handle)
    }

    public var isSwitchingGear: Bool {
        JPH_VehicleTransmission_IsSwitchingGear(handle)
    }

    /// Ratio of the current gear, 0 in neutral.
    public var currentRatio: Float {
        JPH_VehicleTransmission_GetCurrentRatio(handle)
    }

    public func setMode(_ mode: TransmissionSettings.Mode) {
        JPH_VehicleTransmission_SetMode(handle, mode.jph)
        vehicle.wake()
    }

    /// Selects `gear` (0 neutral, negative reverse) and sets the clutch. Use with `.manual` mode;
    /// automatic mode changes gear again by itself.
    public func setGear(_ gear: Int, clutchFriction: Float = 1) {
        JPH_VehicleTransmission_Set(handle, Int32(gear), clutchFriction)
        vehicle.wake()
    }
}
