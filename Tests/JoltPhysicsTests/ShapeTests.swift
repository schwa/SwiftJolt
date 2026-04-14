import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct ShapeTests {
        /// Drops `shape` onto a floor at y = 0 and returns where it settles.
        func restingHeight(of shape: Shape) throws -> Float {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(shape, position: [0, 5, 0], restitution: 0)
            simulate(world, seconds: 3)
            return world.position(of: body).y
        }

        @Test func capsuleRestsOnFloor() throws {
            // Upright: center at halfHeight + radius. Tipped over: center at radius.
            let y = try restingHeight(of: .capsule(halfHeightOfCylinder: 1, radius: 0.5))
            #expect(abs(y - 1.5) < 0.05 || abs(y - 0.5) < 0.05)
        }

        @Test func capsuleRayHitMatchesDimensions() throws {
            let world = try makeWorld()
            let body = world.addBody(.capsule(halfHeightOfCylinder: 1, radius: 0.5), motionType: .static)
            let top = try #require(world.castRay(origin: [0, 10, 0], direction: [0, -20, 0]))
            #expect(top.body == body)
            #expect(abs(top.position.y - 1.5) < 1e-3)
            let side = try #require(world.castRay(origin: [10, 0, 0], direction: [-20, 0, 0]))
            #expect(abs(side.position.x - 0.5) < 1e-3)
        }

        @Test func taperedCapsuleRayHitsMatchRadii() throws {
            let world = try makeWorld()
            let body = world.addBody(.taperedCapsule(halfHeight: 1, topRadius: 0.2, bottomRadius: 0.6), motionType: .static)
            let top = try #require(world.castRay(origin: [0, 10, 0], direction: [0, -20, 0]))
            #expect(top.body == body)
            #expect(abs(top.position.y - 1.2) < 1e-3)
            let bottom = try #require(world.castRay(origin: [0, -10, 0], direction: [0, 20, 0]))
            #expect(abs(bottom.position.y + 1.6) < 1e-3)
            // Wider at the bottom than the top.
            #expect(!world.bodies(containing: [0.5, -1, 0]).isEmpty)
            #expect(world.bodies(containing: [0.5, 1, 0]).isEmpty)
        }

        @Test func taperedCapsuleWithEqualRadiiIsCapsule() throws {
            let world = try makeWorld()
            world.addBody(.taperedCapsule(halfHeight: 1, topRadius: 0.5, bottomRadius: 0.5), motionType: .static)
            let top = try #require(world.castRay(origin: [0, 10, 0], direction: [0, -20, 0]))
            #expect(abs(top.position.y - 1.5) < 1e-3)
        }

        @Test func planeIsSolidBelowSurface() throws {
            let world = try makeWorld()
            let body = world.addBody(.plane(offset: 2), motionType: .static)
            let hit = try #require(world.castRay(origin: [30, 10, -40], direction: [0, -20, 0]))
            #expect(hit.body == body)
            #expect(abs(hit.position.y - 2) < 1e-3)
            #expect(world.bodies(containing: [0, 1, 0]) == [body])
            #expect(world.bodies(containing: [0, 3, 0]).isEmpty)
        }

        @Test func verticalPlaneStopsBall() throws {
            let world = try makeWorld()
            // A wall facing +x at x = -1.
            world.addBody(.plane(normal: [1, 0, 0], offset: -1), motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [3, 0, 0])
            world.setLinearVelocity(of: ball, to: [-10, 0, 0])
            world.gravity = .zero
            simulate(world, seconds: 1)
            #expect(world.position(of: ball).x > -0.6)
        }

        @Test func triangleRayHits() throws {
            let world = try makeWorld()
            // Right triangle in the y = 1 plane, front face up.
            let body = world.addBody(.triangle([0, 1, 0], [0, 1, 2], [2, 1, 0]), motionType: .static)
            let hit = try #require(world.castRay(origin: [0.5, 5, 0.5], direction: [0, -10, 0]))
            #expect(hit.body == body)
            #expect(abs(hit.position.y - 1) < 1e-4)
            // Outside the hypotenuse.
            #expect(world.castRay(origin: [1.5, 5, 1.5], direction: [0, -10, 0]) == nil)
        }

        @Test func ballLandsOnTriangle() throws {
            let world = try makeWorld()
            world.addBody(.triangle([-5, 0, -5], [-5, 0, 10], [10, 0, -5]), motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 3, 0], restitution: 0)
            simulate(world, seconds: 3)
            #expect(abs(world.position(of: ball).y - 0.5) < 0.05)
        }

        @Test func cylinderRayHitMatchesDimensions() throws {
            let world = try makeWorld()
            let body = world.addBody(.cylinder(halfHeight: 1, radius: 0.5), motionType: .static)
            let top = try #require(world.castRay(origin: [0, 10, 0], direction: [0, -20, 0]))
            #expect(top.body == body)
            // Flat top, unlike a capsule.
            #expect(abs(top.position.y - 1) < 1e-3)
            let edge = try #require(world.castRay(origin: [0.4, 10, 0], direction: [0, -20, 0]))
            #expect(abs(edge.position.y - 1) < 1e-3)
            let side = try #require(world.castRay(origin: [10, 0, 0], direction: [-20, 0, 0]))
            #expect(abs(side.position.x - 0.5) < 1e-3)
        }

        // A 10x10 quad at y = 0, front face pointing up.
        let floorMesh = Shape.mesh(
            vertices: [[-5, 0, -5], [-5, 0, 5], [5, 0, 5], [5, 0, -5]],
            indices: [0, 1, 2, 0, 2, 3]
        )

        @Test func meshRayHits() throws {
            let world = try makeWorld()
            let body = world.addBody(floorMesh, motionType: .static)
            let hit = try #require(world.castRay(origin: [1, 5, 2], direction: [0, -10, 0]))
            #expect(hit.body == body)
            #expect(abs(hit.position.y) < 1e-4)
            // Outside the quad.
            #expect(world.castRay(origin: [6, 5, 0], direction: [0, -10, 0]) == nil)
        }

        @Test func dynamicBodyLandsOnMesh() throws {
            let world = try makeWorld()
            world.addBody(floorMesh, motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 3, 0], restitution: 0)
            simulate(world, seconds: 3)
            #expect(abs(world.position(of: ball).y - 0.5) < 0.05)
        }

        // 8x8 samples spaced 1m apart, sloping up along x: height = x / 2.
        let rampHeightField: Shape = {
            let count = 8
            let samples = (0..<count * count).map { Float($0 % count) / 2 }
            return .heightField(samples: samples, sampleCount: count, offset: [-4, 0, -4])
        }()

        @Test func heightFieldRayHitsSurface() throws {
            let world = try makeWorld()
            let body = world.addBody(rampHeightField, motionType: .static)
            // x = 0 is sample column 4 -> height 2. x = -3 is column 1 -> height 0.5.
            let middle = try #require(world.castRay(origin: [0, 10, 0], direction: [0, -20, 0]))
            #expect(middle.body == body)
            #expect(abs(middle.position.y - 2) < 0.01)
            let low = try #require(world.castRay(origin: [-3, 10, 1], direction: [0, -20, 0]))
            #expect(abs(low.position.y - 0.5) < 0.01)
            // Between samples the surface is interpolated.
            let between = try #require(world.castRay(origin: [0.5, 10, 0], direction: [0, -20, 0]))
            #expect(abs(between.position.y - 2.25) < 0.01)
            // Outside the grid.
            #expect(world.castRay(origin: [10, 10, 0], direction: [0, -20, 0]) == nil)
        }

        @Test func ballRollsDownHeightField() throws {
            let world = try makeWorld()
            world.addBody(rampHeightField, motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.3), position: [1, 4, 0])
            simulate(world, seconds: 1)
            let position = world.position(of: ball)
            // It landed on the slope rather than falling through, and rolls toward -x.
            #expect(position.y > 0)
            #expect(position.x < 1)
        }

        @Test func convexHullIgnoresInteriorPoints() throws {
            let world = try makeWorld()
            // Corners of a 2x2x2 cube plus interior points that must not affect the hull.
            var points: [SIMD3<Float>] = []
            for x: Float in [-1, 1] {
                for y: Float in [-1, 1] {
                    for z: Float in [-1, 1] {
                        points.append([x, y, z])
                    }
                }
            }
            points += [[0, 0, 0], [0.5, 0.2, -0.3]]
            let body = world.addBody(.convexHull(points: points, convexRadius: 0), motionType: .static)
            let top = try #require(world.castRay(origin: [0.3, 10, 0.3], direction: [0, -20, 0]))
            #expect(top.body == body)
            #expect(abs(top.position.y - 1) < 1e-3)
            let side = try #require(world.castRay(origin: [10, 0.5, 0], direction: [-20, 0, 0]))
            #expect(abs(side.position.x - 1) < 1e-3)
            #expect(world.bodies(containing: [0.9, 0.9, 0.9]) == [body])
        }

        @Test func tetrahedronHullFallsAndRests() throws {
            let tetrahedron = Shape.convexHull(points: [[0, 0, 0], [1, 0, 0], [0, 0, 1], [0, 1, 0]])
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(tetrahedron, position: [0, 3, 0])
            simulate(world, seconds: 3)
            #expect(world.position(of: body).y > -0.1)
            #expect(simd_length(world.linearVelocity(of: body)) < 0.05)
        }

        // Two unit spheres at x = -2 and x = 2.
        let dumbbellChildren = [
            CompoundChild(.sphere(radius: 1), position: [-2, 0, 0]),
            CompoundChild(.sphere(radius: 1), position: [2, 0, 0])
        ]

        @Test func staticCompoundHasAllChildren() throws {
            let world = try makeWorld()
            let body = world.addBody(.compound(dumbbellChildren), motionType: .static)
            #expect(world.compoundChildCount(of: body) == 2)
            #expect(world.bodies(containing: [-2, 0, 0]) == [body])
            #expect(world.bodies(containing: [2, 0, 0]) == [body])
            #expect(world.bodies(containing: [0, 0, 0]).isEmpty)
            let rotated = CompoundChild(.box(halfExtents: [2, 0.1, 0.1]), rotation: simd_quatf(angle: .pi / 2, axis: [0, 0, 1]))
            let cross = world.addBody(.compound([rotated]), position: [10, 0, 0], motionType: .static)
            #expect(world.bodies(containing: [10, 1.5, 0]) == [cross])
        }

        @Test func dynamicCompoundRestsOnBothChildren() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(.compound(dumbbellChildren), position: [0, 3, 0])
            simulate(world, seconds: 3)
            // Symmetric, so it lands level on both spheres.
            #expect(abs(world.position(of: body).y - 1) < 0.05)
        }

        @Test func mutableCompoundChildrenCanChange() throws {
            let world = try makeWorld()
            let body = world.addBody(.mutableCompound(dumbbellChildren), motionType: .static)
            #expect(world.compoundChildCount(of: body) == 2)

            let index = world.addCompoundChild(CompoundChild(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 5, 0]), to: body)
            #expect(index == 2)
            #expect(world.compoundChildCount(of: body) == 3)
            #expect(world.bodies(containing: [0, 5, 0]) == [body])

            world.moveCompoundChild(at: index, of: body, to: [0, -5, 0], rotation: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1))
            #expect(world.bodies(containing: [0, 5, 0]).isEmpty)
            #expect(world.bodies(containing: [0, -5, 0]) == [body])

            world.removeCompoundChild(at: 0, from: body)
            #expect(world.compoundChildCount(of: body) == 2)
            #expect(world.bodies(containing: [-2, 0, 0]).isEmpty)
            #expect(world.bodies(containing: [2, 0, 0]) == [body])
        }

        @Test func mutableCompoundUpdatesMass() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.mutableCompound([CompoundChild(.sphere(radius: 1))]))
            let massBefore = world.mass(of: body)
            world.addCompoundChild(CompoundChild(.sphere(radius: 1), position: [3, 0, 0]), to: body)
            #expect(abs(world.mass(of: body) / massBefore - 2) < 0.01)
        }

        @Test func cylinderRestsOnFlatEnd() throws {
            let y = try restingHeight(of: .cylinder(halfHeight: 1, radius: 0.5))
            #expect(abs(y - 1) < 0.05)
        }
    }
}
