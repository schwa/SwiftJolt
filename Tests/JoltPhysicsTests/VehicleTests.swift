import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct VehicleTests {
        /// A 4-wheel car on a large floor, front-wheel drive and steering. Wheel order: FL, FR, RL, RR.
        func makeCar(
            differentials: [DifferentialSettings] = [DifferentialSettings(leftWheel: 0, rightWheel: 1)],
            engine: EngineSettings = EngineSettings(),
            transmission: TransmissionSettings = TransmissionSettings(),
            antiRollBars: [AntiRollBar] = [],
            collisionTester: WheelCollisionTester = .default,
            adjustWheel: (inout WheeledVehicleWheel) -> Void = { _ in }
        ) throws -> (PhysicsWorld, body: BodyID, car: WheeledVehicle) {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [100, 0.5, 100]), position: [0, -0.5, 0], motionType: .static, friction: 1)
            let body = world.addBody(.box(halfExtents: [0.9, 0.2, 2]), position: [0, 1, 0])
            world.setMass(of: body, to: 1500)
            let wheels = [
                WheeledVehicleWheel(WheelSettings(position: [-0.9, 0, 1.4]), maxSteerAngle: .pi / 6),
                WheeledVehicleWheel(WheelSettings(position: [0.9, 0, 1.4]), maxSteerAngle: .pi / 6),
                WheeledVehicleWheel(WheelSettings(position: [-0.9, 0, -1.4])),
                WheeledVehicleWheel(WheelSettings(position: [0.9, 0, -1.4]))
            ].map { wheel in
                var wheel = wheel
                adjustWheel(&wheel)
                return wheel
            }
            let car = world.addWheeledVehicle(
                body: body, wheels: wheels, differentials: differentials, engine: engine, transmission: transmission,
                antiRollBars: antiRollBars, collisionTester: collisionTester
            )
            return (world, body, car)
        }

        func driveDistance(_ car: (PhysicsWorld, body: BodyID, car: WheeledVehicle), seconds: Float = 3) -> Float {
            simulate(car.0, seconds: 1)
            car.car.setDriverInput(forward: 1)
            simulate(car.0, seconds: seconds)
            return car.0.position(of: car.body).z
        }

        @Test func vehicleOutlivesWorld() throws {
            let car = try makeCar().car
            car.gravityOverride = [0, 0, -9.81]
            _ = car.wheelState(at: 0)
        }

        @Test func manualTransmissionHoldsGear() throws {
            let car = try makeCar(transmission: TransmissionSettings(mode: .manual))
            car.car.transmission.setGear(2)
            _ = driveDistance(car)
            #expect(car.car.currentGear == 2)
            #expect(car.car.transmission.currentRatio == 1.78)
            #expect(car.car.transmission.clutchFriction == 1)
            #expect(!car.car.transmission.isSwitchingGear)
            #expect(car.car.wheelSpeedAtClutch > 0)
            car.car.transmission.setGear(-1)
            simulate(car.0, seconds: 2)
            #expect(car.car.currentGear == -1)
            #expect(car.car.transmission.currentRatio == -2.9)
            car.car.transmission.setMode(.automatic)
        }

        @Test func singleGearNeverShifts() throws {
            let car = try makeCar(transmission: TransmissionSettings(gearRatios: [3]))
            #expect(driveDistance(car) > 2)
            #expect(car.car.currentGear == 1)
        }

        @Test func flatZeroTorqueCurveDoesNotMove() throws {
            // Jolt keeps the engine at least at min RPM, which drives the car, so allow 0 RPM.
            let stalled = driveDistance(try makeCar(engine: EngineSettings(rpm: 0...6000, torqueCurve: [(0, 0), (1, 0)])))
            let normal = driveDistance(try makeCar(engine: EngineSettings(rpm: 0...6000)))
            #expect(stalled < normal / 4, "stalled \(stalled), normal \(normal)")
        }

        @Test func moreTorqueGoesFurther() throws {
            let weak = driveDistance(try makeCar(engine: EngineSettings(maxTorque: 100)))
            let strong = driveDistance(try makeCar(engine: EngineSettings(maxTorque: 800, inertia: 0.2)))
            #expect(strong > weak + 1)
        }

        @Test func rearWheelDriveViaDifferential() throws {
            let car = try makeCar(differentials: [DifferentialSettings(leftWheel: 2, rightWheel: 3, ratio: 4, limitedSlipRatio: .infinity)])
            #expect(driveDistance(car) > 3)
            #expect(car.car.wheelState(at: 2).angularVelocity > 1)
        }

        @Test func differentialTorqueShares() throws {
            // All torque to the front; the rear differential gets none.
            let car = try makeCar(differentials: [
                DifferentialSettings(leftWheel: 0, rightWheel: 1, engineTorqueRatio: 1),
                DifferentialSettings(leftWheel: 2, rightWheel: 3)
            ])
            #expect(driveDistance(car) > 3)
        }

        @Test func carSettlesOnItsWheels() throws {
            let (world, body, car) = try makeCar()
            #expect(car.wheelCount == 4)
            #expect(world.constraintCount == 1)
            simulate(world, seconds: 2)
            for index in 0..<4 {
                let wheel = car.wheelState(at: index)
                #expect(wheel.contactBody != nil, "wheel \(index) not touching")
                #expect(wheel.suspensionLength > 0.29 && wheel.suspensionLength < 0.51)
            }
            // Body held above the ground by the suspension, not resting on it.
            #expect(world.position(of: body).y > 0.6)
        }

        @Test(arguments: [
            WheelCollisionTester.ray(),
            .castSphere(radius: 0.3),
            .castCylinder(convexRadiusFraction: 0.5)
        ])
        func carSettlesWithEachCollisionTester(tester: WheelCollisionTester) throws {
            let (world, body, car) = try makeCar(collisionTester: tester)
            simulate(world, seconds: 2)
            #expect((0..<4).allSatisfy { car.wheelState(at: $0).contactBody != nil })
            #expect(world.position(of: body).y > 0.6)
        }

        @Test func collisionTesterLayerFiltersGround() throws {
            // The floor is on .nonMoving, which does not collide with .nonMoving.
            let (world, _, car) = try makeCar(collisionTester: .ray(objectLayer: .nonMoving))
            simulate(world, seconds: 1)
            #expect((0..<4).allSatisfy { car.wheelState(at: $0).contactBody == nil })
            car.setWheelCollisionTester(.ray())
            simulate(world, seconds: 1)
            #expect((0..<4).allSatisfy { car.wheelState(at: $0).contactBody != nil })
        }

        @Test func antiRollBarsReduceRoll() throws {
            func roll(antiRollBars: [AntiRollBar]) throws -> Float {
                let (world, body, car) = try makeCar(antiRollBars: antiRollBars)
                simulate(world, seconds: 1)
                car.setDriverInput(forward: 1, right: 1)
                var maxRoll: Float = 0
                for _ in 0..<180 {
                    world.update(deltaTime: 1.0 / 60.0)
                    let right = world.rotation(of: body).act([1, 0, 0])
                    maxRoll = max(maxRoll, abs(right.y))
                }
                return maxRoll
            }
            let soft = try roll(antiRollBars: [])
            let stiff = try roll(antiRollBars: [AntiRollBar(leftWheel: 0, rightWheel: 1, stiffness: 5_000), AntiRollBar(leftWheel: 2, rightWheel: 3, stiffness: 5_000)])
            #expect(stiff < soft * 0.8, "soft \(soft), stiff \(stiff)")
        }

        @Test func wheeledInputPropertiesDriveCar() throws {
            let (world, body, car) = try makeCar()
            simulate(world, seconds: 1)
            car.forwardInput = 1
            car.rightInput = 0.5
            car.brakeInput = 0
            car.handBrakeInput = 0
            #expect(car.forwardInput == 1)
            #expect(car.rightInput == 0.5)
            #expect(car.brakeInput == 0)
            #expect(car.handBrakeInput == 0)
            simulate(world, seconds: 2)
            #expect(world.position(of: body).z > 1)
            car.brakeInput = 1
            car.handBrakeInput = 1
            #expect(car.brakeInput == 1 && car.handBrakeInput == 1)
        }

        @Test func wheelWorldTransformFollowsBody() throws {
            let (world, body, car) = try makeCar()
            simulate(world, seconds: 1)
            let wheel = car.wheelWorldTransform(at: 3).columns.3
            let bodyPosition = world.position(of: body)
            // Rear right wheel attaches at (0.9, 0, -1.4) and hangs down by its suspension.
            #expect(abs(wheel.x - (bodyPosition.x + 0.9)) < 0.05)
            #expect(abs(wheel.z - (bodyPosition.z - 1.4)) < 0.05)
            #expect(wheel.y < bodyPosition.y)
            let local = car.wheelLocalTransform(at: 3).columns.3
            #expect(abs(local.y - (wheel.y - bodyPosition.y)) < 0.05)
        }

        @Test func maxPitchRollAngleLimitsTilt() throws {
            func tilt(limit: Float?) throws -> Float {
                let (world, body, car) = try makeCar()
                if let limit {
                    car.setMaxPitchRollAngle(limit)
                }
                simulate(world, seconds: 1)
                world.addAngularImpulse(to: body, impulse: [0, 0, 4000])
                var maxTilt: Float = 0
                for _ in 0..<60 {
                    world.update(deltaTime: 1.0 / 60.0)
                    maxTilt = max(maxTilt, acos(min(1, world.rotation(of: body).act([0, 1, 0]).y)))
                }
                return maxTilt
            }
            let free = try tilt(limit: nil)
            let limited = try tilt(limit: 0.1)
            #expect(limited < 0.2, "limited \(limited)")
            #expect(limited < free, "free \(free), limited \(limited)")
        }

        @Test func wheelStateReportsContactDetails() throws {
            let (world, _, car) = try makeCar()
            simulate(world, seconds: 2)
            let wheel = car.wheelState(at: 0)
            #expect(wheel.contactBody != nil)
            #expect(wheel.suspensionLambda > 0)
            #expect(abs(abs(wheel.contactLongitudinal.z) - 1) < 0.01)
            #expect(abs(abs(wheel.contactLateral.x) - 1) < 0.01)
            #expect(simd_length(wheel.contactPointVelocity) < 0.01)
            #expect(!wheel.hasHitHardPoint)
        }

        @Test func vehicleFramesAndWheelTransforms() throws {
            let (world, _, car) = try makeCar()
            simulate(world, seconds: 1)
            #expect(car.localForward == [0, 0, 1])
            #expect(car.localUp == [0, 1, 0])
            #expect(simd_distance(car.worldUp, [0, 1, 0]) < 0.01)
            let local = car.wheelLocalTransform(at: 0).columns.3
            #expect(abs(local.x + 0.9) < 1e-3 && abs(local.z - 1.4) < 1e-3)
            let basis = car.wheelLocalBasis(at: 0)
            #expect(simd_distance(basis.forward, [0, 0, 1]) < 1e-3)
            #expect(simd_distance(basis.up, [0, 1, 0]) < 1e-3)
        }

        @Test func gravityOverrideRoundTrips() throws {
            let (world, _, car) = try makeCar()
            defer { withExtendedLifetime(world) {} }
            #expect(car.gravityOverride == nil)
            car.gravityOverride = [0, 0, -9.81]
            #expect(car.gravityOverride == [0, 0, -9.81])
            car.gravityOverride = nil
            #expect(car.gravityOverride == nil)
        }

        @Test func wheelSpinCanBeSetAndDriven() throws {
            let (world, _, car) = try makeCar()
            car.setWheelAngularVelocity(3, at: 1)
            car.setWheelRotationAngle(0.5, at: 1)
            car.setWheelSteerAngle(0.2, at: 0)
            #expect(car.wheelState(at: 1).angularVelocity == 3)
            #expect(car.wheelState(at: 1).rotationAngle == 0.5)
            #expect(car.wheelState(at: 0).steerAngle == 0.2)
            car.setWheelAngularVelocity(0, at: 2)
            car.applyWheelTorque(100, deltaTime: 1.0 / 60.0, at: 2)
            #expect(car.wheelState(at: 2).angularVelocity > 0)
            _ = world
        }

        @Test func noLateralFrictionStopsTurning() throws {
            // Rear-wheel drive, so steering relies only on the front tires' lateral grip.
            func sideways(_ adjust: @escaping (inout WheeledVehicleWheel) -> Void) throws -> Float {
                let (world, body, car) = try makeCar(differentials: [DifferentialSettings(leftWheel: 2, rightWheel: 3)], adjustWheel: adjust)
                simulate(world, seconds: 1)
                car.setDriverInput(forward: 1, right: 1)
                simulate(world, seconds: 3)
                return abs(world.position(of: body).x)
            }
            let normal = try sideways { _ in }
            let slippery = try sideways { $0.lateralFriction = [(0, 0), (90, 0)] }
            #expect(slippery < normal * 0.5, "normal \(normal), slippery \(slippery)")
        }

        @Test func wheelGeometrySettingsApply() throws {
            let (world, body, car) = try makeCar { wheel in
                wheel.wheel.suspensionPreloadLength = 0.1
                wheel.wheel.suspensionForcePoint = wheel.wheel.position
                wheel.inertia = 2
                wheel.angularDamping = 0.5
                wheel.longitudinalFriction = [(0, 0), (0.06, 1.2), (0.2, 1)]
            }
            simulate(world, seconds: 2)
            #expect((0..<4).allSatisfy { car.wheelState(at: $0).contactBody != nil })
            #expect(world.position(of: body).y > 0.6)
        }

        @Test func carDrivesForwardAndBrakes() throws {
            let (world, body, car) = try makeCar()
            simulate(world, seconds: 1)
            car.setDriverInput(forward: 1)
            simulate(world, seconds: 3)
            let position = world.position(of: body)
            #expect(position.z > 5, "z \(position.z)")
            #expect(abs(position.x) < 0.5)
            #expect(car.engineRPM > 1000)
            #expect(car.currentGear >= 1)
            #expect(car.wheelState(at: 0).angularVelocity > 1)

            car.setDriverInput(forward: 0, brake: 1)
            simulate(world, seconds: 3)
            #expect(simd_length(world.linearVelocity(of: body)) < 0.5)
        }

        @Test func driverInputWakesSleepingCar() throws {
            let (world, body, car) = try makeCar()
            simulate(world, seconds: 5)
            #expect(world.readBody(body) { $0.isActive } == false)
            car.setDriverInput(forward: 1)
            simulate(world, seconds: 2)
            #expect(world.position(of: body).z > 1)
        }

        @Test func carSteers() throws {
            let (world, body, car) = try makeCar()
            simulate(world, seconds: 1)
            car.setDriverInput(forward: 1, right: 1)
            simulate(world, seconds: 3)
            // Facing +z with +y up, the driver's right is -x.
            let x = world.position(of: body).x
            #expect(x < -1, "x = \(x)")
            #expect(car.wheelState(at: 0).steerAngle != 0)
            #expect(car.wheelState(at: 2).steerAngle == 0)
        }

        func makeMotorcycle(leanController: Bool = true) throws -> (PhysicsWorld, body: BodyID, bike: Motorcycle) {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [100, 0.5, 100]), position: [0, -0.5, 0], motionType: .static, friction: 1)
            let body = world.addBody(.box(halfExtents: [0.2, 0.3, 0.8]), position: [0, 0.9, 0])
            world.setMass(of: body, to: 240)
            var frontWheel = WheelSettings(position: [0, -0.3, 0.75])
            frontWheel.radius = 0.31
            frontWheel.width = 0.05
            var rearWheel = frontWheel
            rearWheel.position = [0, -0.3, -0.75]
            let bike = world.addMotorcycle(
                body: body,
                front: WheeledVehicleWheel(frontWheel, maxSteerAngle: .pi / 6),
                rear: WheeledVehicleWheel(rearWheel)
            )
            bike.isLeanControllerEnabled = leanController
            // Jolt's default lean spring (5000) flings this bike sideways; see Motorcycle.leanSpringConstant.
            bike.leanSpringConstant = 500
            bike.leanSpringDamping = 100
            return (world, body, bike)
        }

        /// Angle between the body's up axis and world up.
        func tilt(of world: PhysicsWorld, _ body: BodyID) -> Float {
            acos(min(1, simd_dot(world.rotation(of: body).act([0, 1, 0]), [0, 1, 0])))
        }

        @Test func motorcycleDrivesUpright() throws {
            let (world, body, bike) = try makeMotorcycle()
            #expect(bike.wheelCount == 2)
            #expect(abs(bike.wheelBase - 1.5) < 0.05)
            #expect(bike.isLeanControllerEnabled)
            simulate(world, seconds: 1)
            bike.setDriverInput(forward: 1)
            simulate(world, seconds: 3)
            #expect(world.position(of: body).z > 5)
            #expect(tilt(of: world, body) < 0.2)
        }

        @Test func motorcycleLeansIntoTurn() throws {
            let (world, body, bike) = try makeMotorcycle()
            simulate(world, seconds: 1)
            bike.setDriverInput(forward: 1)
            simulate(world, seconds: 2)
            bike.setDriverInput(forward: 0.5, right: 1)
            var maxTilt: Float = 0
            for _ in 0..<120 {
                world.update(deltaTime: 1.0 / 60.0)
                maxTilt = max(maxTilt, tilt(of: world, body))
            }
            // Leans into the turn but does not fall over.
            #expect(maxTilt > 0.1, "tilt \(maxTilt)")
            #expect(maxTilt < .pi / 4 + 0.1, "tilt \(maxTilt)")
            #expect(bike.wheelState(at: 0).contactBody != nil)
            #expect(bike.wheelState(at: 1).contactBody != nil)
        }

        @Test func motorcycleLeanSettingsRoundTrip() throws {
            let (_, _, bike) = try makeMotorcycle()
            bike.leanSmoothingFactor = 0.5
            bike.isLeanSteeringLimitEnabled = false
            #expect(bike.leanSpringConstant == 500)
            #expect(bike.leanSpringDamping == 100)
            #expect(bike.leanSmoothingFactor == 0.5)
            #expect(!bike.isLeanSteeringLimitEnabled)
            bike.isLeanControllerEnabled = false
            #expect(!bike.isLeanControllerEnabled)
        }

        /// Four road wheels per side; wheels 0-3 left, 4-7 right. The rearmost wheel of each side is driven.
        func makeTank() throws -> (PhysicsWorld, body: BodyID, tank: TrackedVehicle) {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [100, 0.5, 100]), position: [0, -0.5, 0], motionType: .static, friction: 1)
            let body = world.addBody(.box(halfExtents: [1.2, 0.4, 2.5]), position: [0, 1.2, 0])
            world.setMass(of: body, to: 4000)
            var wheels: [TrackedVehicleWheel] = []
            for x: Float in [-1.4, 1.4] {
                for z: Float in [1.8, 0.6, -0.6, -1.8] {
                    var wheel = WheelSettings(position: [x, -0.3, z])
                    wheel.radius = 0.4
                    wheel.width = 0.4
                    wheels.append(TrackedVehicleWheel(wheel))
                }
            }
            let tank = world.addTrackedVehicle(
                body: body,
                wheels: wheels,
                leftTrack: Track(wheels: [0, 1, 2, 3], drivenWheel: 3),
                rightTrack: Track(wheels: [4, 5, 6, 7], drivenWheel: 7),
                engine: EngineSettings(maxTorque: 2000)
            )
            return (world, body, tank)
        }

        func yaw(of rotation: simd_quatf) -> Float {
            let forward = rotation.act([0, 0, 1])
            return atan2(forward.x, forward.z)
        }

        @Test func tankDrivesForward() throws {
            let (world, body, tank) = try makeTank()
            #expect(tank.wheelCount == 8)
            simulate(world, seconds: 1)
            #expect((0..<8).allSatisfy { tank.wheelState(at: $0).contactBody != nil })
            tank.setDriverInput(forward: 1)
            simulate(world, seconds: 3)
            #expect(world.position(of: body).z > 3, "z \(world.position(of: body).z)")
            #expect(abs(yaw(of: world.rotation(of: body))) < 0.1)
            #expect(tank.trackAngularVelocity(.left) > 1)
            #expect(tank.currentGear >= 1)
        }

        @Test func tankTurnsInPlace() throws {
            let (world, body, tank) = try makeTank()
            simulate(world, seconds: 1)
            tank.setDriverInput(forward: 1, leftRatio: 1, rightRatio: -1)
            #expect(tank.leftRatio == 1 && tank.rightRatio == -1)
            simulate(world, seconds: 3)
            let position = world.position(of: body)
            #expect(abs(yaw(of: world.rotation(of: body))) > 0.5, "yaw \(yaw(of: world.rotation(of: body)))")

            let (straightWorld, straightBody, straightTank) = try makeTank()
            simulate(straightWorld, seconds: 1)
            straightTank.setDriverInput(forward: 1)
            simulate(straightWorld, seconds: 3)
            // Turning on the spot moves it far less than driving straight for the same time.
            let turnDrift = simd_length(SIMD2(position.x, position.z))
            let straightDistance = straightWorld.position(of: straightBody).z
            #expect(turnDrift < straightDistance / 3, "drift \(turnDrift), straight \(straightDistance)")
        }

        @Test func removingVehicleStopsDrive() throws {
            let (world, body, car) = try makeCar()
            simulate(world, seconds: 1)
            world.removeConstraint(car)
            #expect(world.constraintCount == 0)
            car.setDriverInput(forward: 1)
            simulate(world, seconds: 1)
            // Without the vehicle the box just sits on the floor.
            #expect(abs(world.position(of: body).z) < 0.1)
        }

        @Test func tankInputPropertiesAndTrackDetails() throws {
            let (world, body, tank) = try makeTank()
            simulate(world, seconds: 1)
            tank.forwardInput = 1
            tank.leftRatio = 1
            tank.rightRatio = 1
            tank.brakeInput = 0
            #expect(tank.forwardInput == 1)
            #expect(tank.leftRatio == 1 && tank.rightRatio == 1 && tank.brakeInput == 0)
            simulate(world, seconds: 2)
            #expect(world.position(of: body).z > 1)
            #expect(tank.engineRPM > 0)
            tank.brakeInput = 1
            #expect(tank.brakeInput == 1)

            #expect(tank.trackDetails(.left).drivenWheel == 3)
            #expect(tank.trackDetails(.right).drivenWheel == 7)
            #expect(tank.trackDetails(.left).inertia > 0)
            tank.setTrackAngularVelocity(5, side: .left)
            #expect(tank.trackAngularVelocity(.left) == 5)
        }

        @Test func motorcycleLeanIntegrationRoundTrips() throws {
            let (world, _, bike) = try makeMotorcycle()
            defer { withExtendedLifetime(world) {} }
            bike.leanSpringIntegrationCoefficient = 0.5
            bike.leanSpringIntegrationCoefficientDecay = 2
            #expect(bike.leanSpringIntegrationCoefficient == 0.5)
            #expect(bike.leanSpringIntegrationCoefficientDecay == 2)
        }

        @Test func tankUsesTrackedTransmissionDefaults() throws {
            let (world, _, tank) = try makeTank()
            tank.setDriverInput(forward: 1)
            simulate(world, seconds: 1)
            #expect([4, 3, 2, 1].contains(tank.transmission.currentRatio))
        }

        @Test func vehicleIsNotATwoBodyConstraint() throws {
            let (_, _, car) = try makeCar()
            #expect(car.body1 == nil && car.body2 == nil)
            #expect(car.constraintToBody1Transform == nil)
            #expect(car.constraintToBody2Transform == nil)
        }
    }
}
