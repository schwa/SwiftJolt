import JoltPhysics
import Synchronization
import Testing

final class RecordingListener: ContactListener {
    struct Events {
        var validated = 0
        var added: [Contact] = []
        var persisted = 0
        var removed: [(BodyID, BodyID)] = []
    }

    let events = Mutex(Events())
    let reject: Bool
    let makeSensor: Bool

    init(reject: Bool = false, makeSensor: Bool = false) {
        self.reject = reject
        self.makeSensor = makeSensor
    }

    func contactValidate(body1 _: BodyID, body2 _: BodyID) -> ContactValidation {
        events.withLock { $0.validated += 1 }
        return reject ? .rejectAllContactsForThisBodyPair : .acceptAllContactsForThisBodyPair
    }

    func contactAdded(_ contact: Contact, settings: inout ContactSettings) {
        events.withLock { $0.added.append(contact) }
        if makeSensor {
            settings.isSensor = true
        }
    }

    func contactPersisted(_: Contact, settings: inout ContactSettings) {
        events.withLock { $0.persisted += 1 }
        if makeSensor {
            settings.isSensor = true
        }
    }

    func contactRemoved(body1: BodyID, body2: BodyID) {
        events.withLock { $0.removed.append((body1, body2)) }
    }
}

extension JoltTests {
    @Suite
    struct ContactListenerTests {
        func makeScene(listener: RecordingListener) throws -> (PhysicsWorld, floor: BodyID, ball: BodyID) {
            let world = try makeWorld()
            world.contactListener = listener
            let floor = world.addBody(.box(halfExtents: [10, 0.5, 10]), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 1, 0], restitution: 0)
            return (world, floor, ball)
        }

        @Test func reportsAddedAndPersistedContacts() throws {
            let listener = RecordingListener()
            let (world, floor, ball) = try makeScene(listener: listener)
            simulate(world, seconds: 1)
            let events = listener.events.withLock { $0 }
            #expect(events.validated > 0)
            let contact = try #require(events.added.first)
            #expect(Set([contact.body1, contact.body2]) == [floor, ball])
            #expect(!contact.pointsOn1.isEmpty)
            #expect(contact.pointsOn1.count == contact.pointsOn2.count)
            #expect(abs(abs(contact.normal.y) - 1) < 0.01)
            #expect(events.persisted > 0)
        }

        @Test func reportsRemovedContacts() throws {
            let listener = RecordingListener()
            let (world, _, ball) = try makeScene(listener: listener)
            simulate(world, seconds: 0.5)
            world.setPosition(of: ball, to: [0, 20, 0])
            simulate(world, seconds: 0.1)
            #expect(listener.events.withLock { $0.removed.count } > 0)
        }

        @Test func rejectingValidationLetsBodiesPassThrough() throws {
            let (world, _, ball) = try makeScene(listener: RecordingListener(reject: true))
            simulate(world, seconds: 1)
            #expect(world.position(of: ball).y < -1)
        }

        @Test func sensorSettingLetsBodiesPassThrough() throws {
            let listener = RecordingListener(makeSensor: true)
            let (world, _, ball) = try makeScene(listener: listener)
            simulate(world, seconds: 1)
            #expect(!listener.events.withLock { $0.added.isEmpty })
            #expect(world.position(of: ball).y < -1)
        }

        @Test func clearingListenerStopsEvents() throws {
            let listener = RecordingListener()
            let (world, _, _) = try makeScene(listener: listener)
            world.contactListener = nil
            simulate(world, seconds: 1)
            #expect(listener.events.withLock { $0.validated } == 0)
        }
    }
}
