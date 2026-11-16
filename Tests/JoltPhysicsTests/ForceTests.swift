import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct ForceTests {
        func near(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ tolerance: Float = 1e-3) -> Bool {
            simd_reduce_max(simd_abs(a - b)) < tolerance
        }

        /// A zero-gravity world with one undamped unit-mass box.
        func makeBox() throws -> (PhysicsWorld, BodyID) {
            let world = try makeWorld(gravity: .zero)
            let options = BodyOptions(linearDamping: 0, angularDamping: 0, mass: 1)
            return (world, world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), options: options))
        }

        @Test func setAndAddVelocity() throws {
            let (world, body) = try makeBox()
            world.setVelocity(of: body, linear: [1, 0, 0], angular: [0, 1, 0])
            #expect(world.linearVelocity(of: body) == [1, 0, 0])
            #expect(world.angularVelocity(of: body) == [0, 1, 0])
            world.addVelocity(to: body, linear: [0, 2, 0])
            world.addVelocity(to: body, angular: [0, 0, 3])
            #expect(world.linearVelocity(of: body) == [1, 2, 0])
            #expect(world.angularVelocity(of: body) == [0, 1, 3])
        }

        @Test func pointVelocityIncludesSpin() throws {
            let (world, body) = try makeBox()
            world.setVelocity(of: body, linear: [1, 0, 0], angular: [0, 0, 2])
            // v = linear + angular × r = (1,0,0) + (0,0,2) × (0,1,0) = (-1,0,0)
            #expect(near(world.velocity(of: body, at: [0, 1, 0]), [-1, 0, 0]))
            #expect(world.readBody(body) { $0.velocity(at: [0, 1, 0]) }.map { near($0, [-1, 0, 0]) } == true)
        }

        @Test func centerOfMassFollowsOffset() throws {
            let world = try makeWorld()
            let body = world.addBody(.offsetCenterOfMass(.sphere(radius: 1), offset: [0.5, 0, 0]), position: [1, 2, 3], motionType: .kinematic)
            #expect(near(world.centerOfMassPosition(of: body), [1.5, 2, 3]))
            #expect(near(world.centerOfMassTransform(of: body).columns.3.xyz, [1.5, 2, 3]))
            #expect(world.readBody(body) { near($0.centerOfMassTransform.columns.3.xyz, [1.5, 2, 3]) } == true)
        }

        @Test func offCenterImpulseSpins() throws {
            let (world, body) = try makeBox()
            world.addImpulse(to: body, impulse: [1, 0, 0], at: [0, 0.5, 0])
            #expect(near(world.linearVelocity(of: body), [1, 0, 0]))
            #expect(world.angularVelocity(of: body).z < 0)
        }

        @Test func angularImpulseSpinsWithoutMoving() throws {
            let (world, body) = try makeBox()
            world.addAngularImpulse(to: body, impulse: [0, 1, 0])
            #expect(world.linearVelocity(of: body) == .zero)
            #expect(world.angularVelocity(of: body).y > 0)
        }

        @Test func forceAndTorqueApplyNextStep() throws {
            let (world, body) = try makeBox()
            world.addForce(to: body, force: [60, 0, 0], at: [0, 0.5, 0])
            world.addTorque(to: body, torque: [0, 6, 0])
            world.update(deltaTime: 1.0 / 60.0)
            // 60 N on 1 kg for 1/60 s gives 1 m/s.
            #expect(near(world.linearVelocity(of: body), [1, 0, 0], 1e-2))
            let spin = world.angularVelocity(of: body)
            #expect(spin.y > 0 && spin.z < 0)
        }

        @Test func accumulatedForceAndReset() throws {
            let (world, body) = try makeBox()
            world.writeBody(body) { locked in
                locked.addForce([1, 2, 3])
                locked.addTorque([4, 5, 6])
                #expect(locked.accumulatedForce == [1, 2, 3])
                #expect(locked.accumulatedTorque == [4, 5, 6])
                locked.resetForce()
                locked.resetTorque()
                #expect(locked.accumulatedForce == .zero)
                #expect(locked.accumulatedTorque == .zero)
            }
            world.update(deltaTime: 1.0 / 60.0)
            #expect(world.linearVelocity(of: body) == .zero)
        }

        @Test func resetMotionClearsVelocity() throws {
            let (world, body) = try makeBox()
            world.setVelocity(of: body, linear: [1, 2, 3], angular: [1, 1, 1])
            world.writeBody(body) { $0.resetMotion() }
            #expect(world.linearVelocity(of: body) == .zero)
            #expect(world.angularVelocity(of: body) == .zero)
        }

        @Test func clampedSettersRespectLimits() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1), options: BodyOptions(maxLinearVelocity: 2, maxAngularVelocity: 1))
            world.writeBody(body) { locked in
                locked.setLinearVelocityClamped([10, 0, 0])
                locked.setAngularVelocityClamped([0, 10, 0])
            }
            #expect(near(world.linearVelocity(of: body), [2, 0, 0]))
            #expect(near(world.angularVelocity(of: body), [0, 1, 0]))
        }

        @Test func resetSleepTimerKeepsBodyAwake() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0.5, 0])
            for _ in 0..<180 {
                world.resetSleepTimer(of: body)
                world.writeBody(body) { $0.resetSleepTimer() }
                world.update(deltaTime: 1.0 / 60.0)
            }
            #expect(world.isActive(body))
        }
    }
}

private extension SIMD4<Float> {
    var xyz: SIMD3<Float> { SIMD3(x, y, z) }
}
