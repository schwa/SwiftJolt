import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct CharacterTests {
        // 1.8m tall: cylinder part 1m, radius 0.4. Centre is 0.9m above the feet.
        let capsule = Shape.capsule(halfHeightOfCylinder: 0.5, radius: 0.4)

        func makeWorldWithFloor() throws -> (PhysicsWorld, floor: BodyID) {
            let world = try makeWorld()
            let floor = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            return (world, floor)
        }

        @Test func characterFallsAndLands() throws {
            let (world, floor) = try makeWorldWithFloor()
            let character = world.addCharacter(shape: capsule, position: [0, 3, 0])
            world.update(deltaTime: 1.0 / 60.0)
            #expect(character.groundState == .inAir)
            #expect(character.groundBody == nil)
            simulate(world, seconds: 2)
            #expect(character.groundState == .onGround)
            #expect(character.isSupported)
            #expect(character.groundBody == floor)
            #expect(abs(character.position.y - 0.9) < 0.05)
            #expect(simd_distance(character.groundNormal, [0, 1, 0]) < 0.01)
        }

        @Test func characterMovesWhenVelocitySet() throws {
            let (world, _) = try makeWorldWithFloor()
            let character = world.addCharacter(shape: capsule, position: [0, 0.9, 0])
            simulate(world, seconds: 0.5)
            for _ in 0..<60 {
                // Typical game loop: keep the horizontal velocity, let gravity own vertical.
                character.linearVelocity = [2, character.linearVelocity.y, 0]
                world.update(deltaTime: 1.0 / 60.0)
            }
            #expect(abs(character.position.x - 2) < 0.1)
            // Does not tip over.
            #expect(simd_distance(character.rotation.vector, [0, 0, 0, 1]) < 1e-3)
        }

        @Test func steepSlopeIsDetected() throws {
            let world = try makeWorld()
            // 60 degree ramp.
            let ramp = simd_quatf(angle: .pi / 3, axis: [0, 0, 1])
            world.addBody(.box(halfExtents: [10, 0.5, 10]), rotation: ramp, motionType: .static)
            let character = world.addCharacter(shape: capsule, position: [0, 3, 0], maxSlopeAngle: .pi / 4)
            var sawSteepGround = false
            for _ in 0..<120 {
                world.update(deltaTime: 1.0 / 60.0)
                sawSteepGround = sawSteepGround || character.groundState == .onSteepGround
            }
            #expect(sawSteepGround)
            #expect(abs(character.maxSlopeAngle - .pi / 4) < 1e-4)
        }

        @Test func characterBodyIsRemovedOnRelease() throws {
            let (world, _) = try makeWorldWithFloor()
            var character: Character? = world.addCharacter(shape: capsule, position: [0, 0.9, 0])
            #expect(world.bodyCount == 2)
            let body = character!.body
            #expect(world.readBody(body) { _ in true } == true)
            character = nil
            #expect(world.bodyCount == 1)
            world.update(deltaTime: 1.0 / 60.0)
        }

        /// Runs a virtual character for `seconds` with a horizontal velocity, applying gravity by hand.
        func walk(_ character: VirtualCharacter, velocity: SIMD3<Float>, seconds: Float, world: PhysicsWorld) {
            let deltaTime: Float = 1.0 / 60.0
            for _ in 0..<Int((seconds / deltaTime).rounded()) {
                var newVelocity = velocity
                newVelocity.y = character.isSupported ? 0 : character.linearVelocity.y + world.gravity.y * deltaTime
                character.linearVelocity = newVelocity
                character.update(deltaTime: deltaTime)
                world.update(deltaTime: deltaTime)
            }
        }

        @Test func virtualCharacterFallsAndLands() throws {
            let (world, floor) = try makeWorldWithFloor()
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 3, 0])
            // Not a body in the simulation.
            #expect(world.bodyCount == 1)
            walk(character, velocity: .zero, seconds: 2, world: world)
            #expect(character.groundState == .onGround)
            #expect(character.groundBody == floor)
            #expect(abs(character.position.y - 0.9) < 0.05)
        }

        @Test func virtualCharacterStopsAtWall() throws {
            let (world, _) = try makeWorldWithFloor()
            let wall = world.addBody(.box(halfExtents: [0.5, 2, 5]), position: [3, 2, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            walk(character, velocity: [3, 0, 0], seconds: 2, world: world)
            // Wall face at x = 2.5, capsule radius 0.4.
            #expect(abs(character.position.x - 2.1) < 0.05)
            #expect(character.hasCollided(with: wall))
        }

        @Test func virtualCharacterSlidesAlongWall() throws {
            let (world, _) = try makeWorldWithFloor()
            world.addBody(.box(halfExtents: [0.5, 2, 50]), position: [3, 2, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            // Walking diagonally into the wall keeps the motion along it.
            walk(character, velocity: [3, 0, 3], seconds: 2, world: world)
            #expect(character.position.x < 2.2)
            #expect(character.position.z > 5)
        }

        @Test func virtualCharacterWalksUpStep() throws {
            let (world, _) = try makeWorldWithFloor()
            // 0.3m step starting at x = 2.
            world.addBody(.box(halfExtents: [5, 0.15, 5]), position: [7, 0.15, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            walk(character, velocity: [2, 0, 0], seconds: 2, world: world)
            #expect(character.position.x > 3.5)
            #expect(abs(character.position.y - 1.2) < 0.05)
        }

        @Test func virtualCharacterWithoutStairsIsBlockedByStep() throws {
            let (world, _) = try makeWorldWithFloor()
            world.addBody(.box(halfExtents: [5, 0.15, 5]), position: [7, 0.15, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            let deltaTime: Float = 1.0 / 60.0
            for _ in 0..<120 {
                character.linearVelocity = [2, character.isSupported ? 0 : character.linearVelocity.y - 9.81 * deltaTime, 0]
                character.update(deltaTime: deltaTime, stickToFloor: true, walkStairs: false)
            }
            #expect(character.position.x < 2)
        }

        @Test func characterCanOutliveWorldReference() throws {
            var character: Character?
            do {
                let (world, _) = try makeWorldWithFloor()
                character = world.addCharacter(shape: capsule, position: [0, 0.9, 0])
            }
            // The character keeps its world alive.
            #expect(character!.position.y > 0)
            character = nil
        }
    }
}
