import JoltPhysics
import simd
import Testing

func makeWorld(gravity: SIMD3<Float> = [0, -9.81, 0]) throws -> PhysicsWorld {
    PhysicsWorld(engine: try JoltEngine(), gravity: gravity)
}

func simulate(_ world: PhysicsWorld, seconds: Float, deltaTime: Float = 1.0 / 60.0) {
    for _ in 0..<Int((seconds / deltaTime).rounded()) {
        world.update(deltaTime: deltaTime)
    }
}

// Groups all suites. Tests run in parallel; JoltEngine serializes joltc's shared temp allocator.
@Suite
enum JoltTests {}

extension JoltTests {
    @Suite
    struct PhysicsWorldTests {
        @Test func engineCanBeRecreated() throws {
            do {
                let world = try makeWorld()
                world.addBody(.sphere(radius: 1))
            }
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1))
            #expect(world.bodyCount == 1)
        }

        @Test func multipleEnginesCoexist() throws {
            let a = try JoltEngine()
            let b = try JoltEngine()
            let worldA = PhysicsWorld(engine: a)
            let worldB = PhysicsWorld(engine: b)
            worldA.addBody(.sphere(radius: 1))
            #expect(worldA.bodyCount == 1)
            #expect(worldB.bodyCount == 0)
        }

        @Test func gravityRoundTrips() throws {
            let world = try makeWorld(gravity: [0, -9.81, 0])
            #expect(world.gravity == [0, -9.81, 0])
            world.gravity = [1, 2, 3]
            #expect(world.gravity == [1, 2, 3])
        }

        @Test func addAndRemoveBodiesUpdatesCount() throws {
            let world = try makeWorld()
            #expect(world.bodyCount == 0)
            let a = world.addBody(.sphere(radius: 1), position: [0, 5, 0])
            world.addBody(.box(halfExtents: [1, 1, 1]), position: [5, 5, 0])
            #expect(world.bodyCount == 2)
            world.removeBody(a)
            #expect(world.bodyCount == 1)
        }

        @Test func bodyStartsAtGivenPosition() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1), position: [1, 2, 3])
            #expect(world.position(of: body) == [1, 2, 3])
        }

        @Test func dynamicBodyFallsUnderGravity() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5), position: [0, 10, 0])
            simulate(world, seconds: 1)
            let y = world.position(of: body).y
            // Free fall for 1s: ~4.9m.
            #expect(y < 6 && y > 4)
            #expect(world.linearVelocity(of: body).y < -9)
        }

        @Test func staticBodyDoesNotMove() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 1, 1]), position: [0, 3, 0], motionType: .static)
            simulate(world, seconds: 1)
            #expect(world.position(of: body) == [0, 3, 0])
        }

        @Test func dynamicBodyRestsOnStaticFloor() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 3, 0], restitution: 0)
            simulate(world, seconds: 3)
            #expect(abs(world.position(of: ball).y - 0.5) < 0.05)
        }

        @Test func setLinearVelocityRoundTrips() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.setLinearVelocity(of: body, to: [3, 0, 0])
            #expect(world.linearVelocity(of: body) == [3, 0, 0])
            simulate(world, seconds: 1)
            // Slightly under 3 because of Jolt's default linear damping.
            let x = world.position(of: body).x
            #expect(x > 2.8 && x <= 3, "x = \(x)")
        }

        @Test func impulseChangesVelocity() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.addImpulse(to: body, impulse: [0, 0, 10])
            #expect(world.linearVelocity(of: body).z > 0)
        }

        @Test func forceChangesVelocityAfterStep() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.addForce(to: body, force: [100, 0, 0])
            world.update(deltaTime: 1.0 / 60.0)
            #expect(world.linearVelocity(of: body).x > 0)
        }

        @Test func setPositionAndRotation() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 1, 1]))
            let rotation = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
            world.setPosition(of: body, to: [4, 5, 6])
            #expect(world.position(of: body) == [4, 5, 6])
            world.setRotation(of: body, to: rotation)
            #expect(simd_distance(world.rotation(of: body).vector, rotation.vector) < 1e-5)
            world.setPositionAndRotation(of: body, position: [1, 1, 1], rotation: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1))
            #expect(world.position(of: body) == [1, 1, 1])
            #expect(world.rotation(of: body).vector == [0, 0, 0, 1])
        }

        @Test func angularVelocityRoundTrips() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.setAngularVelocity(of: body, to: [0, 2, 0])
            #expect(world.angularVelocity(of: body) == [0, 2, 0])
        }

        @Test func moveKinematicReachesTargetAndPushesDynamicBody() throws {
            let world = try makeWorld(gravity: .zero)
            let pusher = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0, 0], motionType: .kinematic)
            let ball = world.addBody(.sphere(radius: 0.5), position: [1.5, 0, 0])
            let deltaTime: Float = 1.0 / 60.0
            let identity = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            for step in 1...60 {
                world.moveKinematic(pusher, to: [Float(step) * 0.05, 0, 0], rotation: identity, deltaTime: deltaTime)
                world.update(deltaTime: deltaTime)
            }
            #expect(abs(world.position(of: pusher).x - 3) < 0.01)
            #expect(world.position(of: ball).x > 3.5)
        }

        @Test func castRayFindsClosestHit() throws {
            let world = try makeWorld()
            let near = world.addBody(.sphere(radius: 1), position: [5, 0, 0], motionType: .static)
            world.addBody(.sphere(radius: 1), position: [10, 0, 0], motionType: .static)
            let hit = try #require(world.castRay(origin: .zero, direction: [20, 0, 0]))
            #expect(hit.body == near)
            #expect(abs(hit.fraction - 0.2) < 1e-4)
            #expect(simd_distance(hit.position, [4, 0, 0]) < 1e-3)
        }

        @Test func castRayMissesReturnsNil() throws {
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1), position: [5, 0, 0], motionType: .static)
            #expect(world.castRay(origin: .zero, direction: [0, 20, 0]) == nil)
            // Too short to reach.
            #expect(world.castRay(origin: .zero, direction: [3, 0, 0]) == nil)
        }

        @Test func castRayAllReturnsSortedHits() throws {
            let world = try makeWorld()
            let far = world.addBody(.sphere(radius: 1), position: [10, 0, 0], motionType: .static)
            let near = world.addBody(.sphere(radius: 1), position: [5, 0, 0], motionType: .static)
            let hits = world.castRayAll(origin: .zero, direction: [20, 0, 0])
            #expect(hits.map(\.body) == [near, far])
        }

        @Test func dampingRoundTripsAndAffectsMotion() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.setLinearDamping(of: body, to: 0)
            world.setAngularDamping(of: body, to: 0.7)
            #expect(world.linearDamping(of: body) == 0)
            #expect(world.angularDamping(of: body) == 0.7)
            world.setLinearVelocity(of: body, to: [3, 0, 0])
            simulate(world, seconds: 1)
            #expect(abs(world.position(of: body).x - 3) < 1e-3)
        }

        @Test func setMassScalesMassAndInertia() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            let before = world.inverseInertia(of: body).diagonal
            let originalMass = world.mass(of: body)
            world.setMass(of: body, to: originalMass * 2)
            #expect(abs(world.mass(of: body) - originalMass * 2) < 1e-3)
            #expect(simd_distance(world.inverseInertia(of: body).diagonal, before / 2) < 1e-5)
            world.setMass(of: body, to: 4)
            world.addImpulse(to: body, impulse: [8, 0, 0])
            #expect(abs(world.linearVelocity(of: body).x - 2) < 1e-4)
        }

        @Test func setInverseInertiaRoundTrips() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 1, 1]))
            let identity = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            world.setInverseInertia(of: body, diagonal: [1, 2, 3], rotation: identity)
            let inertia = world.inverseInertia(of: body)
            #expect(inertia.diagonal == [1, 2, 3])
            #expect(inertia.rotation.vector == identity.vector)
        }

        @Test func gravityFactorAndMotionQuality() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5), position: [0, 10, 0])
            world.setGravityFactor(of: body, to: 0)
            #expect(world.gravityFactor(of: body) == 0)
            simulate(world, seconds: 1)
            #expect(world.position(of: body).y == 10)
            #expect(world.motionQuality(of: body) == .discrete)
            world.setMotionQuality(of: body, to: .linearCast)
            #expect(world.motionQuality(of: body) == .linearCast)
        }

        @Test func rotationDefaultsToIdentity() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 1, 1]))
            #expect(world.rotation(of: body).vector == [0, 0, 0, 1])
        }
    }
}
