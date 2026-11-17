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

        @Test func groundDetailsDescribeFloor() throws {
            let world = try makeWorld()
            let stone = PhysicsMaterial(name: "stone")
            let floor = world.addBody(.plane(material: stone), motionType: .static)
            world.setUserData(of: floor, to: 42)
            let character = world.addCharacter(shape: capsule, position: [0, 1, 0])
            simulate(world, seconds: 1)
            #expect(character.groundBody == floor)
            #expect(character.groundUserData == 42)
            #expect(character.groundMaterial == stone)
            #expect(world.physicsMaterial(of: floor, at: character.groundSubShape) == stone)
            #expect(!character.isSlopeTooSteep(normal: [0, 1, 0]))
            #expect(character.isSlopeTooSteep(normal: simd_normalize([1, 0.2, 0])))
        }

        @Test func transformAndVelocityAccessors() throws {
            let world = try makeWorld(gravity: .zero)
            let character = world.addCharacter(shape: capsule, position: [1, 2, 3])
            let turn = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
            character.setPosition([4, 5, 6], rotation: turn)
            let pose = character.positionAndRotation
            #expect(simd_distance(pose.position, [4, 5, 6]) < 1e-4)
            #expect(abs(simd_dot(pose.rotation.vector, turn.vector)) > 0.9999)
            #expect(simd_distance(SIMD3(character.worldTransform.columns.3.x, character.worldTransform.columns.3.y, character.worldTransform.columns.3.z), [4, 5, 6]) < 1e-4)
            #expect(simd_distance(character.centerOfMassPosition, [4, 5, 6]) < 1e-4)

            character.setVelocity(linear: [1, 0, 0], angular: .zero)
            character.addLinearVelocity([0, 0, 2])
            #expect(simd_distance(character.linearVelocity, [1, 0, 2]) < 1e-4)
            character.activate()
            #expect(world.isActive(character.body))

            character.layer = .nonMoving
            #expect(character.layer == .nonMoving)
        }

        @Test func setShapeChangesHeight() throws {
            let (world, _) = try makeWorldWithFloor()
            let character = world.addCharacter(shape: capsule, position: [0, 0.9, 0])
            simulate(world, seconds: 0.5)
            character.setShape(.sphere(radius: 0.4))
            #expect(world.castRay(origin: [0, 1.7, 0], direction: [0, -0.2, 0])?.body != character.body)
            #expect(world.castRay(origin: [0, 1.5, 0], direction: [1, 0, 0]) == nil)
            #expect(world.castRay(origin: [-2, 0.9, 0], direction: [4, 0, 0])?.body == character.body)
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

        @Test func characterBaseSettingsAndGroundDetails() throws {
            let world = try makeWorld()
            // A kinematic platform moving along x.
            let platform = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .kinematic)
            world.setLinearVelocity(of: platform, to: [1, 0, 0])
            let character = world.addCharacter(shape: capsule, position: [0, 1, 0])
            character.maxSlopeAngle = .pi / 6
            #expect(abs(character.maxSlopeAngle - .pi / 6) < 1e-4)
            character.up = [0, 1, 0]
            #expect(character.up == [0, 1, 0])
            simulate(world, seconds: 1)
            #expect(abs(character.groundPosition.y) < 0.05)
            #expect(simd_distance(character.groundVelocity, [1, 0, 0]) < 0.01)
        }

        @Test func characterSettersAndImpulse() throws {
            let world = try makeWorld(gravity: .zero)
            let character = world.addCharacter(shape: capsule, position: .zero, mass: 80)
            character.position = [1, 2, 3]
            let turn = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
            character.rotation = turn
            #expect(simd_distance(character.position, [1, 2, 3]) < 1e-4)
            #expect(abs(simd_dot(character.rotation.vector, turn.vector)) > 0.9999)
            character.addImpulse([80, 0, 0])
            #expect(simd_distance(character.linearVelocity, [1, 0, 0]) < 0.01)
        }

        @Test func virtualCharacterProperties() throws {
            let (world, _) = try makeWorldWithFloor()
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 3, 0])
            character.position = [1, 4, 0]
            #expect(simd_distance(character.position, [1, 4, 0]) < 1e-4)
            let turn = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
            character.rotation = turn
            #expect(abs(simd_dot(character.rotation.vector, turn.vector)) > 0.9999)
            character.mass = 90
            #expect(character.mass == 90)
            character.maxStrength = 250
            #expect(character.maxStrength == 250)

            // Plain update, without stick-to-floor or stairs, still moves the character.
            character.linearVelocity = [0, -5, 0]
            character.update(deltaTime: 0.1, stickToFloor: false, walkStairs: false)
            #expect(abs(character.position.y - 3.5) < 0.01)
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

        @Test func virtualCharacterReportsActiveContacts() throws {
            let (world, floor) = try makeWorldWithFloor()
            let wall = world.addBody(.box(halfExtents: [0.5, 2, 5]), position: [3, 2, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            walk(character, velocity: [3, 0, 0], seconds: 2, world: world)
            let contacts = character.activeContacts
            #expect(Set(contacts.compactMap(\.body)) == [floor, wall])
            let wallContact = try #require(contacts.first { $0.body == wall })
            #expect(wallContact.motionType == .static)
            #expect(abs(wallContact.surfaceNormal.x + 1) < 0.01)
            #expect(!character.maxHitsExceeded)
            character.refreshContacts()
            #expect(character.activeContacts.count == contacts.count)
        }

        @Test func virtualCharacterTransforms() throws {
            let (world, _) = try makeWorldWithFloor()
            let character = world.addVirtualCharacter(shape: capsule, position: [1, 2, 3])
            #expect(character.worldTransform.columns.3 == [1, 2, 3, 1])
            character.shapeOffset = [0, 0.5, 0]
            #expect(character.shapeOffset == [0, 0.5, 0])
            // Jolt also lifts the shape by the character padding.
            #expect(abs(character.centerOfMassTransform.columns.3.y - (2.5 + character.characterPadding)) < 1e-4)
        }

        @Test func virtualCharacterTuning() throws {
            let (world, _) = try makeWorldWithFloor()
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            character.maxHitCount = 64
            character.hitReductionCosMaxAngle = 0.9
            character.penetrationRecoverySpeed = 0.5
            character.enhancedInternalEdgeRemoval = true
            #expect(character.maxHitCount == 64)
            #expect(character.hitReductionCosMaxAngle == 0.9)
            #expect(character.penetrationRecoverySpeed == 0.5)
            #expect(character.enhancedInternalEdgeRemoval)
            #expect(character.characterPadding > 0)
        }

        @Test func virtualCharacterSetShapeRespectsSpace() throws {
            let (world, _) = try makeWorldWithFloor()
            let character = world.addVirtualCharacter(shape: .sphere(radius: 0.4), position: [0, 0.4, 0])
            // Centered at y = 0.4, this capsule reaches down to y = -0.7, deep into the floor.
            #expect(!character.setShape(.capsule(halfHeightOfCylinder: 0.7, radius: 0.4)))
            #expect(character.setShape(.sphere(radius: 0.3)))
            #expect(character.setShape(.capsule(halfHeightOfCylinder: 0.7, radius: 0.4), maxPenetrationDepth: .infinity))
        }

        @Test func virtualCharacterStepwiseStairs() throws {
            let (world, _) = try makeWorldWithFloor()
            world.addBody(.box(halfExtents: [5, 0.15, 5]), position: [7, 0.15, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            let deltaTime: Float = 1.0 / 60.0
            let velocity: SIMD3<Float> = [2, 0, 0]
            for _ in 0..<120 {
                character.linearVelocity = velocity
                character.update(deltaTime: deltaTime, stickToFloor: false, walkStairs: false)
                if character.canWalkStairs(velocity: velocity) {
                    character.walkStairs(deltaTime: deltaTime, stepForward: velocity * deltaTime, stepForwardTest: [0.15, 0, 0])
                }
                character.stickToFloor()
            }
            #expect(character.position.x > 3.5)
            #expect(abs(character.position.y - 1.2) < 0.05)
        }

        @Test func virtualCharacterCancelsVelocityUpSteepSlope() throws {
            let world = try makeWorld()
            // 60° ramp rising toward +X.
            world.addBody(.box(halfExtents: [5, 0.5, 5]), rotation: simd_quatf(angle: .pi / 3, axis: [0, 0, 1]), motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [-1.5, 0, 0])
            character.linearVelocity = [0, -1, 0]
            for _ in 0..<30 {
                character.update(deltaTime: 1.0 / 60.0)
            }
            #expect(character.groundState == .onSteepGround)
            let cancelled = character.cancelVelocityTowardsSteepSlopes([3, 0, 0])
            #expect(cancelled.x < 3)
            character.updateGroundVelocity()
            #expect(character.groundVelocity == .zero)
        }

        @Test func virtualCharacterInnerBody() throws {
            let (world, _) = try makeWorldWithFloor()
            let plain = world.addVirtualCharacter(shape: capsule, position: [5, 0.9, 0])
            #expect(plain.innerBody == nil)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0], innerBodyShape: capsule)
            let inner = try #require(character.innerBody)
            #expect(world.bodies.contains(inner))
            walk(character, velocity: [1, 0, 0], seconds: 1, world: world)
            #expect(abs(world.position(of: inner).x - character.position.x) < 0.05)
            character.setInnerBodyShape(.sphere(radius: 0.4))
            let hit = try #require(world.castRay(origin: [character.position.x, 5, 0], direction: [0, -10, 0]))
            #expect(hit.body == inner)
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
