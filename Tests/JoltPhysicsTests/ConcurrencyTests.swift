import Dispatch
import JoltPhysics
import Testing

extension JoltTests {
    @Suite
    struct ConcurrencyTests {
        /// A world with enough contacts to make Jolt use its temp allocator heavily.
        func makeBusyWorld() throws -> PhysicsWorld {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            for index in 0..<50 {
                world.addBody(.sphere(radius: 0.4), position: [Float(index % 5), 1 + Float(index / 5), 0])
            }
            return world
        }

        @Test func separateWorldsCanUpdateConcurrently() throws {
            let worlds = try (0..<4).map { _ in try makeBusyWorld() }
            let characters = worlds.map { $0.addVirtualCharacter(shape: .capsule(halfHeightOfCylinder: 0.5, radius: 0.4), position: [-3, 0.9, 0]) }
            nonisolated(unsafe) let unsafeWorlds = worlds
            nonisolated(unsafe) let unsafeCharacters = characters
            DispatchQueue.concurrentPerform(iterations: worlds.count) { index in
                for _ in 0..<200 {
                    unsafeCharacters[index].linearVelocity = [1, 0, 0]
                    unsafeCharacters[index].update(deltaTime: 1.0 / 60.0)
                    unsafeWorlds[index].update(deltaTime: 1.0 / 60.0)
                }
            }
            #expect(worlds.allSatisfy { $0.bodyCount == 51 })
        }

        @Test func worldsCanBeCreatedAndDestroyedConcurrently() throws {
            let engine = try JoltEngine()
            // Keeping worlds alive grows joltc's global world map, so inserts rehash it.
            DispatchQueue.concurrentPerform(iterations: 8) { _ in
                for _ in 0..<4 {
                    let worlds = (0..<32).map { _ in PhysicsWorld(engine: engine, maxBodies: 16, jobSystem: .joltThreadPool(threadCount: 0)) }
                    for world in worlds {
                        world.update(deltaTime: 1.0 / 60.0)
                    }
                }
            }
        }
    }
}
