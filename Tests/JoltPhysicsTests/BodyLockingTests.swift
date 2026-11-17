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

        @Test func readBodiesLocksSeveralAtOnce() throws {
            let world = try makeWorld()
            let a = world.addBody(.sphere(radius: 1), position: [1, 0, 0])
            let b = world.addBody(.sphere(radius: 1), position: [0, 2, 0])
            let removed = world.addBody(.sphere(radius: 1))
            world.removeBody(removed)
            let result = world.readBodies([a, b, removed]) { bodies in
                (
                    bodies.count,
                    bodies.withBody(at: 0) { $0.position },
                    bodies.withBody(at: 1) { $0.id },
                    bodies.withBody(at: 2) { $0.id }
                )
            }
            #expect(result.0 == 3)
            #expect(result.1 == [1, 0, 0])
            #expect(result.2 == b)
            #expect(result.3 == nil)
        }

        @Test func lockedBodyTransformAccessors() throws {
            let world = try makeWorld(gravity: .zero)
            let turn = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
            let body = world.addBody(
                .offsetCenterOfMass(.box(halfExtents: [1, 1, 1]), offset: [1, 0, 0]),
                position: [1, 2, 3],
                rotation: turn
            )
            let state = try #require(world.readBody(body) { body in
                (body.rotation, body.worldTransform, body.centerOfMassPosition)
            })
            #expect(abs(simd_dot(state.0.vector, turn.vector)) > 0.9999)
            #expect(simd_distance(SIMD3(state.1.columns.3.x, state.1.columns.3.y, state.1.columns.3.z), [1, 2, 3]) < 1e-4)
            // The local +x offset turns to world -z.
            #expect(simd_distance(state.2, [1, 2, 2]) < 1e-4)
        }

        @Test func lockedBodyVelocitiesAndImpulses() throws {
            let world = try makeWorld(gravity: .zero)
            let options = BodyOptions(linearDamping: 0, angularDamping: 0, mass: 1)
            let body = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), options: options)
            world.writeBody(body) { body in
                body.addImpulse([1, 0, 0], at: [0, 0.5, 0])
                body.addAngularImpulse([0, 1, 0])
            }
            let velocities = try #require(world.readBody(body) { ($0.linearVelocity, $0.angularVelocity) })
            #expect(simd_distance(velocities.0, [1, 0, 0]) < 1e-4)
            // The off-center impulse spins around -z; the angular impulse spins around +y.
            #expect(velocities.1.z < 0)
            #expect(velocities.1.y > 0)
        }

        @Test func lockedBodyFlagsAndProperties() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1))
            let floor = world.addBody(.box(halfExtents: [1, 1, 1]), position: [0, -10, 0], motionType: .static)
            world.writeBody(body) { body in
                body.restitution = 0.7
                body.userData = 99
                body.allowSleeping = false
                body.applyGyroscopicForce = true
                body.collideKinematicVsNonDynamic = true
                body.useManifoldReduction = false
                body.enhancedInternalEdgeRemoval = true
            }
            let state = try #require(world.readBody(body) { body in
                (
                    body.restitution, body.userData, body.allowSleeping, body.applyGyroscopicForce,
                    body.collideKinematicVsNonDynamic, body.useManifoldReduction, body.enhancedInternalEdgeRemoval,
                    body.canBeKinematicOrDynamic, body.isActive
                )
            })
            #expect(state.0 == 0.7)
            #expect(state.1 == 99)
            #expect(!state.2)
            #expect(state.3)
            #expect(state.4)
            #expect(!state.5)
            #expect(state.6)
            #expect(state.7)
            #expect(state.8)
            #expect(world.restitution(of: body) == 0.7)
            #expect(world.readBody(floor) { $0.canBeKinematicOrDynamic } == false)
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
