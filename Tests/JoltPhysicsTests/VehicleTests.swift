import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct VehicleTests {
        /// A 4-wheel car on a large floor, front-wheel drive and steering. Wheel order: FL, FR, RL, RR.
        func makeCar() throws -> (PhysicsWorld, body: BodyID, car: WheeledVehicle) {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [100, 0.5, 100]), position: [0, -0.5, 0], motionType: .static, friction: 1)
            let body = world.addBody(.box(halfExtents: [0.9, 0.2, 2]), position: [0, 1, 0])
            world.setMass(of: body, to: 1500)
            let wheels = [
                WheeledVehicleWheel(WheelSettings(position: [-0.9, 0, 1.4]), maxSteerAngle: .pi / 6),
                WheeledVehicleWheel(WheelSettings(position: [0.9, 0, 1.4]), maxSteerAngle: .pi / 6),
                WheeledVehicleWheel(WheelSettings(position: [-0.9, 0, -1.4])),
                WheeledVehicleWheel(WheelSettings(position: [0.9, 0, -1.4]))
            ]
            let car = world.addWheeledVehicle(body: body, wheels: wheels, drivenWheels: [(0, 1)])
            return (world, body, car)
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
    }
}
