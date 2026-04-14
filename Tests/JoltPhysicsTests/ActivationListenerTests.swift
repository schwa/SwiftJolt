import JoltPhysics
import Synchronization
import Testing

final class RecordingActivationListener: ActivationListener {
    enum Event: Equatable {
        case activated(BodyID)
        case deactivated(BodyID)
    }

    let events = Mutex<[Event]>([])

    func bodyActivated(_ body: BodyID) {
        events.withLock { $0.append(.activated(body)) }
    }

    func bodyDeactivated(_ body: BodyID) {
        events.withLock { $0.append(.deactivated(body)) }
    }
}

extension JoltTests {
    @Suite
    struct ActivationListenerTests {
        @Test func reportsActivationSleepAndWake() throws {
            let world = try makeWorld()
            let listener = RecordingActivationListener()
            world.activationListener = listener
            world.addBody(.box(halfExtents: [10, 0.5, 10]), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0.5, 0])
            #expect(listener.events.withLock { $0 } == [.activated(ball)])

            // A resting body goes to sleep after a short while.
            simulate(world, seconds: 3)
            #expect(listener.events.withLock { $0 } == [.activated(ball), .deactivated(ball)])
            #expect(world.activeBodyCount == 0)

            world.setLinearVelocity(of: ball, to: [0, 3, 0])
            #expect(listener.events.withLock { $0.last } == .activated(ball))
        }

        @Test func clearingListenerStopsEvents() throws {
            let world = try makeWorld()
            let listener = RecordingActivationListener()
            world.activationListener = listener
            world.activationListener = nil
            world.addBody(.sphere(radius: 1))
            #expect(listener.events.withLock { $0.isEmpty })
        }
    }
}
