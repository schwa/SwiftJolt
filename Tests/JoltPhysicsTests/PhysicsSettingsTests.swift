import JoltPhysics
import Testing

extension JoltTests {
    @Suite
    struct PhysicsSettingsTests {
        @Test func defaultsAreJolts() throws {
            let settings = try makeWorld().settings
            #expect(settings.velocitySteps == 10)
            #expect(settings.positionSteps == 2)
            #expect(settings.baumgarte == 0.2)
            #expect(settings.allowSleeping)
        }

        @Test func settingsRoundTrip() throws {
            let world = try makeWorld()
            var settings = world.settings
            settings.velocitySteps = 4
            settings.positionSteps = 1
            settings.baumgarte = 0.5
            settings.timeBeforeSleep = 2
            settings.deterministicSimulation = false
            world.settings = settings
            #expect(world.settings == settings)
        }

        @Test func disablingSleepKeepsBodiesAwake() throws {
            let world = try makeWorld()
            world.settings.allowSleeping = false
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0.5, 0])
            simulate(world, seconds: 3)
            #expect(world.isActive(body))
        }

        @Test func activatesAndDeactivatesBodies() throws {
            let world = try makeWorld(gravity: .zero)
            let a = world.addBody(.sphere(radius: 0.5), position: [0, 0, 0])
            let b = world.addBody(.sphere(radius: 0.5), position: [5, 0, 0])
            let c = world.addBody(.sphere(radius: 0.5), position: [10, 0, 0])
            world.deactivate(a)
            #expect(!world.isActive(a))
            #expect(world.isActive(b))
            world.deactivate([b, c])
            #expect(!world.isActive(b) && !world.isActive(c))
            world.activate(a)
            #expect(world.isActive(a))
            world.activate([b, c])
            #expect(world.isActive(b) && world.isActive(c))
        }

        @Test func enumeratesBodiesAndConstraints() throws {
            let world = try makeWorld()
            #expect(world.maxBodies == 10240)
            let floor = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 3, 0])
            #expect(Set(world.bodies) == [floor, ball])
            #expect(world.activeBodies == [ball])
            let constraint = world.addPointConstraint(between: nil, and: ball, point: [0, 4, 0])
            #expect(world.allConstraints.map(ObjectIdentifier.init) == [ObjectIdentifier(constraint)])
            world.removeConstraint(constraint)
            #expect(world.allConstraints.isEmpty)
        }

        @Test func contactAndActivationQueries() throws {
            let world = try makeWorld()
            let floor = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 0.5, 0])
            let far = world.addBody(.sphere(radius: 0.5), position: [20, 10, 0])
            world.update(deltaTime: 1.0 / 60.0)
            #expect(world.wereBodiesInContact(floor, ball))
            #expect(!world.wereBodiesInContact(floor, far))

            simulate(world, seconds: 3)
            #expect(!world.isActive(ball))
            world.activateBodies(inBoxFrom: [-1, -1, -1], to: [1, 2, 1])
            #expect(world.isActive(ball))
        }
    }
}
