import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct CollisionFilteringTests {
        static let debris = ObjectLayer(rawValue: 2)

        /// Debris hits static bodies but passes through moving ones.
        static let debrisLayers: CollisionLayers = {
            var layers = CollisionLayers(count: 3)
            layers.enableCollision(.nonMoving, .moving)
            layers.enableCollision(.moving, .moving)
            layers.enableCollision(.nonMoving, debris)
            return layers
        }()

        /// Rests box `lower` on a floor and drops `upper` on it. Returns `upper`'s resting height:
        /// about 1.5 when it lands on `lower`, about 0.5 when it falls through to the floor.
        func stackHeight(world: PhysicsWorld, lower: BodyOptions = .default, upper: BodyOptions = .default) -> Float {
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0.5, 0], restitution: 0, options: lower)
            let top = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 3, 0], restitution: 0, options: upper)
            simulate(world, seconds: 3)
            return world.position(of: top).y
        }

        @Test func defaultLayers() {
            let layers = CollisionLayers.default
            #expect(layers.count == 2)
            #expect(layers.collides(.nonMoving, .moving))
            #expect(layers.collides(.moving, .nonMoving))
            #expect(layers.collides(.moving, .moving))
            #expect(!layers.collides(.nonMoving, .nonMoving))
            #expect(layers.staticLayers == [.nonMoving])
        }

        @Test func enableAndDisableAreSymmetric() {
            var layers = CollisionLayers(count: 3)
            layers.enableCollision(Self.debris, .moving)
            #expect(layers.collides(.moving, Self.debris))
            layers.disableCollision(.moving, Self.debris)
            #expect(!layers.collides(Self.debris, .moving))
        }

        @Test func movingBodiesStackByDefault() throws {
            let height = stackHeight(world: try makeWorld())
            #expect(abs(height - 1.5) < 0.05)
        }

        @Test func customLayerPassesThroughMovingBodies() throws {
            let world = PhysicsWorld(engine: try JoltEngine(), collisionLayers: Self.debrisLayers)
            let height = stackHeight(world: world, upper: BodyOptions(objectLayer: Self.debris))
            #expect(abs(height - 0.5) < 0.05)
        }

        @Test func disablingMovingVsMovingLetsBodiesPass() throws {
            var layers = CollisionLayers.default
            layers.disableCollision(.moving, .moving)
            let world = PhysicsWorld(engine: try JoltEngine(), collisionLayers: layers)
            let height = stackHeight(world: world)
            #expect(abs(height - 0.5) < 0.05)
        }

        @Test func objectLayerCanChange() throws {
            let world = PhysicsWorld(engine: try JoltEngine(), collisionLayers: Self.debrisLayers)
            let body = world.addBody(.sphere(radius: 1))
            #expect(world.objectLayer(of: body) == .moving)
            world.setObjectLayer(of: body, to: Self.debris)
            #expect(world.objectLayer(of: body) == Self.debris)
            let floor = world.addBody(.box(halfExtents: [1, 1, 1]), motionType: .static)
            #expect(world.objectLayer(of: floor) == .nonMoving)
        }

        @Test func sameGroupWithDisabledSubGroupsPassesThrough() throws {
            let filter = CollisionGroupFilter(subGroupCount: 2)
            filter.disableCollision(0, 1)
            #expect(!filter.isCollisionEnabled(1, 0))
            let height = stackHeight(
                world: try makeWorld(),
                lower: BodyOptions(collisionGroup: CollisionGroup(filter: filter, groupID: 7, subGroupID: 0)),
                upper: BodyOptions(collisionGroup: CollisionGroup(filter: filter, groupID: 7, subGroupID: 1))
            )
            #expect(abs(height - 0.5) < 0.05)
        }

        @Test func differentGroupsStillCollide() throws {
            let filter = CollisionGroupFilter(subGroupCount: 2)
            filter.disableCollision(0, 1)
            let height = stackHeight(
                world: try makeWorld(),
                lower: BodyOptions(collisionGroup: CollisionGroup(filter: filter, groupID: 7, subGroupID: 0)),
                upper: BodyOptions(collisionGroup: CollisionGroup(filter: filter, groupID: 8, subGroupID: 1))
            )
            #expect(abs(height - 1.5) < 0.05)
        }

        @Test func collisionGroupRoundTrips() throws {
            let world = try makeWorld()
            let filter = CollisionGroupFilter(subGroupCount: 4)
            let body = world.addBody(.sphere(radius: 1))
            #expect(world.collisionGroup(of: body) == .ungrouped)
            let group = CollisionGroup(filter: filter, groupID: 3, subGroupID: 2)
            world.setCollisionGroup(of: body, to: group)
            #expect(world.collisionGroup(of: body) == group)
            filter.disableCollision(1, 2)
            filter.enableCollision(1, 2)
            #expect(filter.isCollisionEnabled(2, 1))
        }
    }
}
