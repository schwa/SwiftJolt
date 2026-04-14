import CJolt
import simd

/// A two-wheeled vehicle that leans into turns. A lean controller keeps it upright; turn it off
/// and the bike falls over like an unridden one.
public final class Motorcycle: WheeledVehicle {
    /// Distance between the two wheels' contact points.
    public var wheelBase: Float {
        JPH_MotorcycleController_GetWheelBase(controller)
    }

    public var isLeanControllerEnabled: Bool {
        get { JPH_MotorcycleController_IsLeanControllerEnabled(controller) }
        set { JPH_MotorcycleController_EnableLeanController(controller, newValue) }
    }

    /// Limits steering at speed so the required lean stays within the max lean angle.
    public var isLeanSteeringLimitEnabled: Bool {
        get { JPH_MotorcycleController_IsLeanSteeringLimitEnabled(controller) }
        set { JPH_MotorcycleController_EnableLeanSteeringLimit(controller, newValue) }
    }

    /// Strength of the spring that pulls the bike toward the target lean angle. Jolt's default (5000)
    /// made a 240 kg test bike with a 0.4 m wide body fly apart; 500 with damping 100 was stable.
    /// Tune it to the bike's roll inertia.
    public var leanSpringConstant: Float {
        get { JPH_MotorcycleController_GetLeanSpringConstant(controller) }
        set { JPH_MotorcycleController_SetLeanSpringConstant(controller, newValue) }
    }

    public var leanSpringDamping: Float {
        get { JPH_MotorcycleController_GetLeanSpringDamping(controller) }
        set { JPH_MotorcycleController_SetLeanSpringDamping(controller, newValue) }
    }

    /// 0...1. Higher values smooth the lean angle more.
    public var leanSmoothingFactor: Float {
        get { JPH_MotorcycleController_GetLeanSmoothingFactor(controller) }
        set { JPH_MotorcycleController_SetLeanSmoothingFactor(controller, newValue) }
    }
}

extension PhysicsWorld {
    /// Turns `body` (a dynamic body) into a motorcycle. The front wheel steers; the rear wheel is
    /// driven. Wheel 0 is the front wheel, wheel 1 the rear.
    @discardableResult
    public func addMotorcycle(
        body: BodyID,
        front: WheeledVehicleWheel,
        rear: WheeledVehicleWheel,
        engine: EngineSettings = EngineSettings(maxTorque: 150, rpm: 1000...10000),
        maxLeanAngle: Float = .pi / 4,
        up: SIMD3<Float> = [0, 1, 0],
        forward: SIMD3<Float> = [0, 0, 1]
    ) -> Motorcycle {
        let wheels = [front, rear]
        let controllerSettings = makeWheeledControllerSettings(drivenWheels: [(left: -1, right: 1)], engine: engine) {
            let settings = JPH_MotorcycleControllerSettings_Create()!
            JPH_MotorcycleControllerSettings_SetMaxLeanAngle(settings, maxLeanAngle)
            return settings
        }
        return addVehicle(
            body: body,
            wheelCount: 2,
            makeWheel: { wheels[$0].makeSettings() },
            controllerSettings: controllerSettings,
            up: up,
            forward: forward,
            maxPitchRollAngle: .pi,
            collisionTester: JPH_VehicleCollisionTesterCastCylinder_Create(ObjectLayer.moving.rawValue, 0.1)!
        )
    }
}
