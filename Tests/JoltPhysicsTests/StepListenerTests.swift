import JoltPhysics
import Synchronization
import Testing

final class RecordingStepListener: StepListener {
    let contexts = Mutex<[StepContext]>([])
    let action: @Sendable (StepContext) -> Void

    init(action: @escaping @Sendable (StepContext) -> Void = { _ in }) {
        self.action = action
    }

    func step(_ context: StepContext) {
        contexts.withLock { $0.append(context) }
        action(context)
    }
}

extension JoltTests {
    @Suite
    struct StepListenerTests {
        @Test func calledOncePerCollisionStep() throws {
            let world = try makeWorld()
            let listener = RecordingStepListener()
            world.addStepListener(listener)
            world.addStepListener(listener)
            world.update(deltaTime: 0.1, collisionSteps: 2)
            let contexts = listener.contexts.withLock { $0 }
            try #require(contexts.count == 2)
            #expect(abs(contexts[0].deltaTime - 0.05) < 1e-6)
            #expect(contexts[0].isFirstStep && !contexts[0].isLastStep)
            #expect(!contexts[1].isFirstStep && contexts[1].isLastStep)
        }

        @Test func removedListenerIsNotCalled() throws {
            let world = try makeWorld()
            let listener = RecordingStepListener()
            world.addStepListener(listener)
            world.removeStepListener(listener)
            world.removeStepListener(listener)
            world.update(deltaTime: 1.0 / 60.0)
            #expect(listener.contexts.withLock { $0.isEmpty })
        }

        @Test func listenerCanApplyForces() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5), options: BodyOptions(linearDamping: 0, mass: 1))
            // Cancel gravity every step, so the body hovers.
            // PhysicsWorld is not Sendable; step listeners are the documented exception.
            nonisolated(unsafe) unowned let unownedWorld = world
            let listener = RecordingStepListener { _ in
                unownedWorld.addForce(to: body, force: [0, 9.81, 0])
            }
            world.addStepListener(listener)
            simulate(world, seconds: 1)
            #expect(abs(world.position(of: body).y) < 1e-3)
        }

        @Test func listenerCanLockBodiesAndQuery() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [10, 0.5, 10]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(.sphere(radius: 0.5), position: [0, 5, 0], options: BodyOptions(mass: 1))
            nonisolated(unsafe) unowned let unownedWorld = world
            let hits = Mutex(0)
            let listener = RecordingStepListener { _ in
                unownedWorld.writeBody(body) { $0.addForce([0, 9.81, 0]) }
                if unownedWorld.castRay(origin: [0, 10, 0], direction: [0, -20, 0]) != nil {
                    hits.withLock { $0 += 1 }
                }
                _ = unownedWorld.mass(of: body)
            }
            world.addStepListener(listener)
            simulate(world, seconds: 0.5)
            #expect(hits.withLock { $0 } == 30)
            #expect(world.position(of: body).y > 4.9)
        }

        @Test func worldReleasesListenersOnDeinit() throws {
            weak var weakListener: RecordingStepListener?
            do {
                let world = try makeWorld()
                let listener = RecordingStepListener()
                weakListener = listener
                world.addStepListener(listener)
                world.update(deltaTime: 1.0 / 60.0)
            }
            #expect(weakListener == nil)
        }
    }
}
