import Foundation
@testable import JoltPhysics
import Synchronization
import Testing

extension JoltTests {
    @Suite
    struct JobThreadPoolTests {
        private func makeJobs(_ count: Int, recorder: Recorder) -> [PooledJob] {
            let arg = Unmanaged.passUnretained(recorder).toOpaque()
            return (0..<count).map { _ in
                PooledJob(job: { context in
                    let jobRecorder = Unmanaged<Recorder>.fromOpaque(context!).takeUnretainedValue()
                    jobRecorder.qosClasses.withLock { $0.append(qos_class_self()) }
                    // Make jobs pile up so shutdown has queued work to finish.
                    usleep(100)
                    jobRecorder.runCount.withLock { $0 += 1 }
                }, arg: arg)
            }
        }

        @Test func runsJobsAtUserInteractiveQoS() {
            let recorder = Recorder()
            let pool = JobThreadPool()
            pool.enqueue(makeJobs(20, recorder: recorder))
            pool.shutdown()
            let classes = recorder.qosClasses.withLock { $0 }
            #expect(classes.count == 20)
            #expect(classes.allSatisfy { $0 == QOS_CLASS_USER_INTERACTIVE })
        }

        @Test func shutdownFinishesQueuedJobs() {
            let recorder = Recorder()
            let pool = JobThreadPool()
            pool.enqueue(makeJobs(1_000, recorder: recorder))
            pool.shutdown()
            #expect(recorder.runCount.withLock { $0 } == 1_000)
        }

        @Test func poolUsesRequestedThreadCount() {
            let pool = JobThreadPool(threadCount: 2)
            #expect(pool.threadCount == 2)
            let recorder = Recorder()
            pool.enqueue(makeJobs(10, recorder: recorder))
            pool.shutdown()
            #expect(recorder.runCount.withLock { $0 } == 10)
        }

        @Test(arguments: [
            JobSystemKind.userInteractiveThreadPool(threadCount: 1),
            .userInteractiveThreadPool(threadCount: 3, maxBarriers: 4),
            .joltThreadPool(),
            .joltThreadPool(threadCount: 0),
            .joltThreadPool(threadCount: 2, maxJobs: 512)
        ])
        func worldsSimulateWithEachJobSystem(kind: JobSystemKind) throws {
            let world = PhysicsWorld(engine: try JoltEngine(), jobSystem: kind)
            world.addBody(.box(halfExtents: [10, 0.5, 10]), position: [0, -0.5, 0], motionType: .static)
            // A row of separate balls, enough bodies for the solver to split work into jobs.
            let balls = (0..<20).map { world.addBody(.sphere(radius: 0.4), position: [Float($0) * 0.9 - 9, 2, 0]) }
            simulate(world, seconds: 2)
            #expect(balls.allSatisfy { abs(world.position(of: $0).y - 0.4) < 0.05 })
        }

        @Test func repeatedlyCreatingAndDestroyingWorldsIsSafe() throws {
            for _ in 0..<50 {
                let world = try makeWorld()
                world.addBody(.box(halfExtents: [10, 0.5, 10]), position: [0, -0.5, 0], motionType: .static)
                for index in 0..<10 {
                    world.addBody(.sphere(radius: 0.4), position: [Float(index), 1, 0])
                }
                for _ in 0..<5 {
                    world.update(deltaTime: 1.0 / 60.0)
                }
            }
        }
    }
}

private final class Recorder: Sendable {
    let runCount = Mutex(0)
    let qosClasses = Mutex<[qos_class_t]>([])
}
