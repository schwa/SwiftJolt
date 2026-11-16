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

        @Test func surfaceMaterialIsReportedAtHit() throws {
            let world = try makeWorld()
            let grass = PhysicsMaterial(name: "grass", debugColor: DebugColor(red: 10, green: 200, blue: 30))
            let ground = world.addBody(.plane(material: grass), motionType: .static)
            let post = world.addBody(
                .taperedCylinder(halfHeight: 1, topRadius: 0.2, bottomRadius: 0.4, material: PhysicsMaterial(name: "wood")),
                position: [5, 1, 0],
                motionType: .static
            )
            let groundHit = try #require(world.castRay(origin: [0, 5, 0], direction: [0, -10, 0]))
            #expect(groundHit.body == ground)
            #expect(world.physicsMaterial(of: ground, at: groundHit.subShape) == grass)
            let postHit = try #require(world.castRay(origin: [5, 5, 0], direction: [0, -10, 0]))
            #expect(postHit.body == post)
            #expect(world.physicsMaterial(of: post, at: postHit.subShape)?.name == "wood")
        }

        @Test func shapesWithoutMaterialReportDefault() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1), motionType: .static)
            let hit = try #require(world.castRay(origin: [0, 5, 0], direction: [0, -10, 0]))
            #expect(world.physicsMaterial(of: body, at: hit.subShape) != nil)
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
