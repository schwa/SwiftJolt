import JoltPhysics
import Testing

extension JoltTests {
    @Suite
    struct MaterialTests {
        @Test func materialSetsFrictionAndRestitution() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1), material: .rubber)
            #expect(world.friction(of: body) == Material.rubber.friction)
            #expect(world.restitution(of: body) == Material.rubber.restitution)
        }

        @Test func densityScalesMass() throws {
            let world = try makeWorld()
            let shape = Shape.box(halfExtents: [0.5, 0.5, 0.5])
            let water = world.addBody(shape, material: .default)
            let steel = world.addBody(shape, material: .steel)
            #expect(abs(world.mass(of: water) - 1000) < 1)
            #expect(abs(world.mass(of: steel) - 7850) < 1)
        }

        @Test func staticBodyIgnoresDensity() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [1, 1, 1]), motionType: .static, material: .steel)
            #expect(world.friction(of: body) == Material.steel.friction)
        }

        @Test func settersUpdateBody() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1))
            world.setFriction(of: body, to: 0.7)
            world.setRestitution(of: body, to: 0.1)
            #expect(world.friction(of: body) == 0.7)
            #expect(world.restitution(of: body) == 0.1)
        }
    }
}
