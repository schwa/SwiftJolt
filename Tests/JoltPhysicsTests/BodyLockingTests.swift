import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct BodyLockingTests {
        @Test func readBodyReportsState() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 2, 3]), position: [1, 2, 3], motionType: .dynamic, restitution: 0.4, friction: 0.6)
            let state = try #require(world.readBody(body) { body in
                (body.id, body.position, body.motionType, body.friction, body.restitution, body.isActive, body.worldSpaceBounds)
            })
            #expect(state.0 == body)
            #expect(state.1 == [1, 2, 3])
            #expect(state.2 == .dynamic)
            #expect(state.3 == 0.6)
            #expect(state.4 == 0.4)
            #expect(state.5)
            #expect(simd_distance(state.6.min, [0, 0, 0]) < 1e-4)
            #expect(simd_distance(state.6.max, [2, 4, 6]) < 1e-4)
        }

        @Test func readBodyReturnsNilForRemovedBody() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1))
            world.removeBody(body)
            #expect(world.readBody(body) { $0.position } == nil)
            #expect(world.writeBody(body) { $0.linearVelocity = .zero } == nil)
        }

        @Test func writeBodyChangesState() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.writeBody(body) { body in
                body.linearVelocity = [1, 0, 0]
                body.angularVelocity = [0, 2, 0]
                body.friction = 0.9
                body.isSensor = true
            }
            #expect(world.linearVelocity(of: body) == [1, 0, 0])
            #expect(world.angularVelocity(of: body) == [0, 2, 0])
            #expect(world.readBody(body) { ($0.friction, $0.isSensor) }! == (0.9, true))
        }

        @Test func writeBodyImpulseAtPointSpinsBody() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            let mass = world.mass(of: body)
            // Push the top of the sphere along +x: it moves along +x and spins around -z.
            world.writeBody(body) { $0.addImpulse([mass, 0, 0], at: [0, 1, 0]) }
            #expect(abs(world.linearVelocity(of: body).x - 1) < 1e-4)
            #expect(world.angularVelocity(of: body).z < 0)
            let topVelocity = try #require(world.readBody(body) { $0.velocity(at: [0, 1, 0]) })
            #expect(topVelocity.x > 1)
        }

        @Test func writeBodyForceActsOnNextStep() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1))
            world.writeBody(body) { body in
                body.addForce([100, 0, 0])
                body.addTorque([0, 100, 0])
            }
            world.update(deltaTime: 1.0 / 60.0)
            #expect(world.linearVelocity(of: body).x > 0)
            #expect(world.angularVelocity(of: body).y > 0)
        }

        @Test func surfaceNormalAtRayHit() throws {
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1), motionType: .static)
            let hit = try #require(world.castRay(origin: [5, 5, 0], direction: [-10, -10, 0]))
            let normal = try #require(world.surfaceNormal(at: hit))
            #expect(simd_distance(normal, simd_normalize([1, 1, 0])) < 1e-3)
        }
    }
}
