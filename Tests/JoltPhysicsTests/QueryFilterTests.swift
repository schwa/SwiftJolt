import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct QueryFilterTests {
        static let debris = ObjectLayer(rawValue: 2)

        /// Three static boxes stacked along -Y: `top` at y = 4 (debris layer), `middle` at y = 2, `bottom` at y = 0.
        func makeStack() throws -> (PhysicsWorld, top: BodyID, middle: BodyID, bottom: BodyID) {
            var layers = CollisionLayers(count: 3)
            layers.enableCollision(.nonMoving, .moving)
            let world = PhysicsWorld(engine: try JoltEngine(), collisionLayers: layers)
            let box = Shape.box(halfExtents: [0.5, 0.5, 0.5], convexRadius: 0)
            let top = world.addBody(box, position: [0, 4, 0], motionType: .static, options: BodyOptions(objectLayer: Self.debris))
            let middle = world.addBody(box, position: [0, 2, 0], motionType: .static)
            let bottom = world.addBody(box, position: [0, 0, 0], motionType: .static)
            return (world, top, middle, bottom)
        }

        let origin: SIMD3<Float> = [0, 10, 0]
        let down: SIMD3<Float> = [0, -20, 0]

        @Test func castRayExcludesBodies() throws {
            let (world, top, middle, bottom) = try makeStack()
            #expect(world.castRay(origin: origin, direction: down)?.body == top)
            #expect(world.castRay(origin: origin, direction: down, filter: QueryFilter(excludedBodies: [top]))?.body == middle)
            #expect(world.castRay(origin: origin, direction: down, filter: QueryFilter(excludedBodies: [top, middle]))?.body == bottom)
        }

        @Test func castRayFiltersByLayer() throws {
            let (world, top, middle, _) = try makeStack()
            #expect(world.castRay(origin: origin, direction: down, filter: QueryFilter(layers: [.nonMoving]))?.body == middle)
            #expect(world.castRay(origin: origin, direction: down, filter: QueryFilter(layers: [Self.debris]))?.body == top)
            #expect(world.castRay(origin: origin, direction: down, filter: QueryFilter(layers: [.moving])) == nil)
        }

        @Test func predicateFilters() throws {
            let (world, _, _, bottom) = try makeStack()
            let filter = QueryFilter { $0 == bottom }
            #expect(world.castRayAll(origin: origin, direction: down, filter: filter).map(\.body) == [bottom])
        }

        @Test func castRayAllAndShapeQueriesUseFilter() throws {
            let (world, top, middle, bottom) = try makeStack()
            let filter = QueryFilter(excludedBodies: [middle])
            #expect(world.castRayAll(origin: origin, direction: down, filter: filter).map(\.body) == [top, bottom])
            let sphere = Shape.sphere(radius: 0.2)
            #expect(world.castShapeAll(sphere, position: origin, direction: down, filter: filter).map(\.body) == [top, bottom])
            #expect(world.castShape(sphere, position: origin, direction: down, filter: QueryFilter(excludedBodies: [top]))?.body == middle)
            let big = Shape.box(halfExtents: [1, 3, 1])
            #expect(Set(world.collideShape(big, position: [0, 2, 0], filter: filter).map(\.body)) == [top, bottom])
            #expect(world.bodies(containing: [0, 2, 0], filter: filter).isEmpty)
            #expect(world.bodies(containing: [0, 2, 0]) == [middle])
        }
    }
}
