import CJolt
@testable import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct BodyOptionsTests {
        @Test func defaultsMatchJolt() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1))
            let options = BodyOptions.default
            #expect(world.linearDamping(of: body) == options.linearDamping)
            #expect(world.angularDamping(of: body) == options.angularDamping)
            #expect(world.gravityFactor(of: body) == options.gravityFactor)
            #expect(world.motionQuality(of: body) == options.motionQuality)
        }

        @Test func motionSettingsAreApplied() throws {
            let world = try makeWorld()
            let options = BodyOptions(
                linearVelocity: [1, 2, 3],
                angularVelocity: [0, 1, 0],
                linearDamping: 0.5,
                angularDamping: 0.7,
                gravityFactor: 0.25,
                motionQuality: .linearCast
            )
            let body = world.addBody(.sphere(radius: 1), options: options)
            #expect(world.linearVelocity(of: body) == [1, 2, 3])
            #expect(world.angularVelocity(of: body) == [0, 1, 0])
            #expect(world.linearDamping(of: body) == 0.5)
            #expect(world.angularDamping(of: body) == 0.7)
            #expect(world.gravityFactor(of: body) == 0.25)
            #expect(world.motionQuality(of: body) == .linearCast)
        }

        @Test func maxLinearVelocityClampsSpeed() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.sphere(radius: 1), options: BodyOptions(linearVelocity: [100, 0, 0], maxLinearVelocity: 2))
            simulate(world, seconds: 0.1)
            #expect(simd_length(world.linearVelocity(of: body)) <= 2.001)
        }

        @Test func maxAngularVelocityClampsSpin() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.box(halfExtents: [1, 1, 1]), options: BodyOptions(angularVelocity: [0, 50, 0], maxAngularVelocity: 1))
            simulate(world, seconds: 0.1)
            #expect(simd_length(world.angularVelocity(of: body)) <= 1.001)
        }

        @Test func massOverrideScalesInertia() throws {
            let world = try makeWorld()
            let plain = world.addBody(.box(halfExtents: [1, 1, 1]))
            let heavy = world.addBody(.box(halfExtents: [1, 1, 1]), options: BodyOptions(mass: 2 * world.mass(of: plain)))
            #expect(abs(world.mass(of: heavy) - 2 * world.mass(of: plain)) < 1e-2)
            let ratio = world.inverseInertia(of: plain).diagonal / world.inverseInertia(of: heavy).diagonal
            #expect(simd_reduce_max(simd_abs(ratio - 2)) < 1e-3)
        }

        @Test func massOverrideWinsOverMaterialDensity() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 1, 1]), material: .steel, options: BodyOptions(mass: 5))
            #expect(abs(world.mass(of: body) - 5) < 1e-3)
        }

        @Test func inertiaMultiplierScalesInertia() throws {
            let world = try makeWorld()
            let plain = world.addBody(.box(halfExtents: [1, 1, 1]))
            let stiff = world.addBody(.box(halfExtents: [1, 1, 1]), options: BodyOptions(inertiaMultiplier: 4))
            let ratio = world.inverseInertia(of: plain).diagonal / world.inverseInertia(of: stiff).diagonal
            #expect(simd_reduce_max(simd_abs(ratio - 4)) < 1e-3)
        }

        @Test func disallowingSleepKeepsBodyActive() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let sleeper = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [-3, 0.5, 0])
            let awake = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [3, 0.5, 0], options: BodyOptions(allowSleeping: false))
            simulate(world, seconds: 3)
            #expect(!JPH_BodyInterface_IsActive(world.bodyInterface, sleeper.rawValue))
            #expect(JPH_BodyInterface_IsActive(world.bodyInterface, awake.rawValue))
        }

        @Test func plane2DBodyStaysInPlane() throws {
            let world = try makeWorld()
            let body = world.addBody(
                .box(halfExtents: [0.5, 0.5, 0.5]),
                options: BodyOptions(linearVelocity: [1, 0, 3], angularVelocity: [2, 2, 2], allowedDegreesOfFreedom: .plane2D)
            )
            #expect(world.allowedDegreesOfFreedom(of: body) == .plane2D)
            simulate(world, seconds: 1)
            #expect(world.position(of: body).z == 0)
            #expect(world.position(of: body).y < -1)
            #expect(world.position(of: body).x > 0.5)
            let spin = world.angularVelocity(of: body)
            #expect(spin.x == 0 && spin.y == 0 && spin.z != 0)
        }

        @Test func defaultAllowsAllDegreesOfFreedom() throws {
            let world = try makeWorld()
            #expect(world.allowedDegreesOfFreedom(of: world.addBody(.sphere(radius: 1))) == .all)
        }

        @Test func flagsAreApplied() throws {
            let world = try makeWorld()
            let options = BodyOptions(
                applyGyroscopicForce: true,
                allowDynamicOrKinematic: true,
                collideKinematicVsNonDynamic: true,
                useManifoldReduction: false,
                enhancedInternalEdgeRemoval: true
            )
            let body = world.addBody(.sphere(radius: 1), motionType: .static, options: options)
            let flags = world.readBody(body) { locked in
                let pointer = locked.pointer
                return (
                    gyroscopic: JPH_Body_GetApplyGyroscopicForce(pointer),
                    kinematicVsNonDynamic: JPH_Body_GetCollideKinematicVsNonDynamic(pointer),
                    manifoldReduction: JPH_Body_GetUseManifoldReduction(pointer),
                    edgeRemoval: JPH_Body_GetEnhancedInternalEdgeRemoval(pointer),
                    canChange: JPH_Body_CanBeKinematicOrDynamic(pointer)
                )
            }
            let result = try #require(flags)
            #expect(result.gyroscopic)
            #expect(result.kinematicVsNonDynamic)
            #expect(!result.manifoldReduction)
            #expect(result.edgeRemoval)
            #expect(result.canChange)
        }
    }
}
