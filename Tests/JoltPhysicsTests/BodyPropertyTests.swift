import CJolt
@testable import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct BodyPropertyTests {
        @Test func motionTypeCanChange() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5), position: [0, 5, 0])
            #expect(world.motionType(of: body) == .dynamic)
            world.setMotionType(of: body, to: .kinematic)
            #expect(world.motionType(of: body) == .kinematic)
            simulate(world, seconds: 0.5)
            #expect(world.position(of: body).y == 5)
            world.setMotionType(of: body, to: .dynamic)
            simulate(world, seconds: 0.5)
            #expect(world.position(of: body).y < 4)
        }

        @Test func staticBodyWithAllowDynamicCanBecomeDynamic() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5), position: [0, 5, 0], motionType: .static, options: BodyOptions(allowDynamicOrKinematic: true))
            #expect(world.motionType(of: body) == .static)
            world.setMotionType(of: body, to: .dynamic)
            simulate(world, seconds: 0.5)
            #expect(world.position(of: body).y < 4)
        }

        @Test func setShapeChangesGeometryAndMass() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5), motionType: .kinematic)
            world.setShape(of: body, to: .box(halfExtents: [2, 1, 1], convexRadius: 0))
            let hit = try #require(world.castRay(origin: [10, 0, 0], direction: [-20, 0, 0]))
            #expect(abs(hit.position.x - 2) < 1e-3)

            let dynamic = world.addBody(.sphere(radius: 0.5), position: [10, 0, 0])
            let smallMass = world.mass(of: dynamic)
            world.setShape(of: dynamic, to: .sphere(radius: 1))
            #expect(abs(world.mass(of: dynamic) / smallMass - 8) < 0.01)
            world.setShape(of: dynamic, to: .sphere(radius: 0.5), updateMassProperties: false)
            #expect(abs(world.mass(of: dynamic) / smallMass - 8) < 0.01)
        }

        @Test func setShapeLeavesBodyAsOnlyOwner() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5))
            world.setShape(of: body, to: .box(halfExtents: [1, 1, 1]))
            let shape = JPH_BodyInterface_GetShape(world.bodyInterface, body.rawValue)!
            #expect(referenceCount(of: shape) == 1)
        }

        @Test func bodyKind() throws {
            let world = try makeWorld()
            let rigid = world.addBody(.sphere(radius: 0.5))
            #expect(!world.isSoftBody(rigid))
            let cloth = try makeCloth(engine: world.engine)
            cloth.createConstraints()
            let soft = world.addSoftBody(cloth)
            #expect(world.isSoftBody(soft))
            #expect(world.readBody(soft) { $0.isSoftBody } == true)
            #expect(world.readBody(rigid) { $0.isInBroadPhase } == true)
        }

        @Test func lockedBodyFlagsRoundTrip() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5))
            world.writeBody(body) { locked in
                locked.allowSleeping = false
                locked.applyGyroscopicForce = true
                locked.collideKinematicVsNonDynamic = true
                locked.useManifoldReduction = false
                locked.enhancedInternalEdgeRemoval = true
            }
            let flags = world.readBody(body) { locked in
                [locked.allowSleeping, locked.applyGyroscopicForce, locked.collideKinematicVsNonDynamic, locked.useManifoldReduction, locked.enhancedInternalEdgeRemoval, locked.canBeKinematicOrDynamic]
            }
            #expect(flags == [false, true, true, false, true, true])
            world.invalidateContactCache(of: body)
        }
    }
}
