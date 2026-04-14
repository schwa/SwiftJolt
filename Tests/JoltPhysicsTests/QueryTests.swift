import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct QueryTests {
        @Test func bodiesContainingPoint() throws {
            let world = try makeWorld()
            let sphere = world.addBody(.sphere(radius: 1), position: [0, 0, 0], motionType: .static)
            let box = world.addBody(.box(halfExtents: [1, 1, 1]), position: [1.5, 0, 0], motionType: .static)
            let capsule = world.addBody(.capsule(halfHeightOfCylinder: 1, radius: 0.5), position: [10, 0, 0])

            #expect(world.bodies(containing: [0, 0, 0]) == [sphere])
            #expect(Set(world.bodies(containing: [0.75, 0, 0])) == [sphere, box])
            #expect(world.bodies(containing: [2, 0.9, 0]) == [box])
            #expect(world.bodies(containing: [10, 1.4, 0]) == [capsule])
            #expect(world.bodies(containing: [5, 5, 5]).isEmpty)
        }

        @Test func castShapeHitsFloor() throws {
            let world = try makeWorld()
            let floor = world.addBody(.box(halfExtents: [10, 0.5, 10], convexRadius: 0), position: [0, -0.5, 0], motionType: .static)
            // A unit sphere swept down 10m from y = 5 touches the floor top (y = 0) after 4m.
            let hit = try #require(world.castShape(.sphere(radius: 1), position: [0, 5, 0], direction: [0, -10, 0]))
            #expect(hit.body == floor)
            #expect(abs(hit.fraction - 0.4) < 1e-3)
            #expect(abs(hit.contactPoint.y) < 1e-3)
            #expect(abs(hit.normal.y - 1) < 1e-3)
            // Too short to reach.
            #expect(world.castShape(.sphere(radius: 1), position: [0, 5, 0], direction: [0, -3, 0]) == nil)
        }

        @Test func castShapeUsesShapeSize() throws {
            let world = try makeWorld()
            world.addBody(.sphere(radius: 0.5), position: [0, 0, 0], motionType: .static)
            // A ray along y = 1.2 misses the sphere, but a wide box swept along the same line hits it.
            #expect(world.castRay(origin: [-5, 1.2, 0], direction: [10, 0, 0]) == nil)
            let rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
            #expect(world.castShape(.box(halfExtents: [1, 0.1, 0.1]), position: [-5, 1.2, 0], rotation: rotation, direction: [10, 0, 0]) != nil)
            #expect(world.castShape(.box(halfExtents: [1, 0.1, 0.1]), position: [-5, 1.2, 0], direction: [10, 0, 0]) == nil)
        }

        @Test func collideShapeFindsOverlaps() throws {
            let world = try makeWorld()
            let floor = world.addBody(.box(halfExtents: [10, 0.5, 10], convexRadius: 0), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.sphere(radius: 1), position: [5, 1, 0], motionType: .static)

            // Sphere sunk 0.25 into the floor.
            let hits = world.collideShape(.sphere(radius: 1), position: [0, 0.75, 0])
            let hit = try #require(hits.first)
            #expect(hits.map(\.body) == [floor])
            #expect(abs(hit.penetrationDepth - 0.25) < 1e-3)
            #expect(abs(hit.normal.y - 1) < 1e-3)

            // Overlaps floor and ball.
            #expect(Set(world.collideShape(.box(halfExtents: [1, 1, 1]), position: [4, 0.5, 0]).map(\.body)) == [floor, ball])

            // Clear of everything.
            #expect(world.collideShape(.sphere(radius: 0.5), position: [0, 3, 0]).isEmpty)
        }

        @Test func collideShapeMaxSeparationFindsNearbyBodies() throws {
            let world = try makeWorld()
            let ball = world.addBody(.sphere(radius: 1), motionType: .static)
            // 0.5m gap between surfaces.
            #expect(world.collideShape(.sphere(radius: 0.5), position: [2, 0, 0]).isEmpty)
            let near = world.collideShape(.sphere(radius: 0.5), position: [2, 0, 0], maxSeparationDistance: 1)
            #expect(near.map(\.body) == [ball])
            #expect(abs(near[0].penetrationDepth + 0.5) < 1e-3)
        }

        @Test func collideShapeAccountsForCenterOfMass() throws {
            let world = try makeWorld()
            let ball = world.addBody(.sphere(radius: 0.1), position: [2.5, 0, 0], motionType: .static)
            // Hull from x = 2 to 3, whose center of mass is far from its origin.
            let hull = Shape.convexHull(points: [[2, -1, -1], [3, -1, -1], [2, 1, -1], [3, 1, -1], [2, -1, 1], [3, -1, 1], [2, 1, 1], [3, 1, 1]], convexRadius: 0)
            #expect(world.collideShape(hull, position: .zero).map(\.body) == [ball])
            #expect(world.collideShape(hull, position: [5, 0, 0]).isEmpty)
            // castShape takes the shape origin too. Hull top starts at y = -9 and meets the ball bottom (y = -0.1) after 8.9m.
            let hit = try #require(world.castShape(hull, position: [0, -10, 0], direction: [0, 20, 0]))
            #expect(hit.body == ball)
            #expect(abs(hit.fraction * 20 - 8.9) < 1e-2)
        }

        @Test func broadPhaseUsesBoundingBoxes() throws {
            let world = try makeWorld()
            let sphere = world.addBody(.sphere(radius: 1), position: [0, 0, 0], motionType: .static)
            let box = world.addBody(.box(halfExtents: [1, 1, 1]), position: [5, 0, 0], motionType: .static)
            // Inside the sphere's bounding box corner but outside the sphere itself.
            let corner: SIMD3<Float> = [0.9, 0.9, 0.9]
            #expect(world.bodies(containing: corner).isEmpty)
            #expect(world.broadPhase.bodies(containing: corner) == [sphere])

            #expect(Set(world.broadPhase.bodies(overlappingBoxFrom: [-2, -2, -2], to: [10, 2, 2])) == [sphere, box])
            #expect(world.broadPhase.bodies(overlappingBoxFrom: [3, -2, -2], to: [4.5, 2, 2]) == [box])
            // Reaches z = 0.8, inside the box's z = 1 face.
            #expect(world.broadPhase.bodies(overlappingSphereAt: [5, 0, 1.8], radius: 1) == [box])
            #expect(world.broadPhase.bodies(overlappingSphereAt: [5, 0, 2.5], radius: 1).isEmpty)
            #expect(world.broadPhase.bodies(overlappingSphereAt: [0, 10, 0], radius: 1).isEmpty)
        }

        @Test func broadPhaseRayReturnsSortedBoxes() throws {
            let world = try makeWorld()
            let near = world.addBody(.sphere(radius: 1), position: [5, 0, 0], motionType: .static)
            let far = world.addBody(.sphere(radius: 1), position: [10, 0, 0], motionType: .static)
            let hits = world.broadPhase.castRay(origin: .zero, direction: [20, 0, 0])
            #expect(hits.map(\.body) == [near, far])
            #expect(hits[0].fraction <= 0.2)
        }

        @Test func broadPhaseCollectsManyBodies() throws {
            let world = try makeWorld()
            var expected: Set<BodyID> = []
            for index in 0..<100 {
                expected.insert(world.addBody(.sphere(radius: 0.2), position: [Float(index % 10), 0, Float(index / 10)], motionType: .static))
            }
            #expect(Set(world.broadPhase.bodies(overlappingBoxFrom: [-1, -1, -1], to: [10, 1, 10])) == expected)
        }

        @Test func castShapeAllReturnsSortedHits() throws {
            let world = try makeWorld()
            let far = world.addBody(.sphere(radius: 1), position: [10, 0, 0], motionType: .static)
            let near = world.addBody(.sphere(radius: 1), position: [5, 0, 0], motionType: .static)
            let hits = world.castShapeAll(.sphere(radius: 0.5), position: .zero, direction: [20, 0, 0])
            #expect(hits.map(\.body) == [near, far])
        }
    }
}
