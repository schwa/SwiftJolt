import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct ShapeInspectionTests {
        func near(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ tolerance: Float = 1e-3) -> Bool {
            simd_reduce_max(simd_abs(a - b)) < tolerance
        }

        @Test func boxProperties() {
            let properties = Shape.box(halfExtents: [1, 2, 3], convexRadius: 0).properties
            #expect(abs(properties.volume - 48) < 1e-3)
            #expect(abs(properties.mass - 48_000) < 1)
            #expect(near(properties.centerOfMass, .zero))
            #expect(near(properties.boundsMin, [-1, -2, -3]))
            #expect(near(properties.boundsMax, [1, 2, 3]))
            #expect(abs(properties.innerRadius - 1) < 1e-3)
            // Box inertia about X: m (h² + d²) / 12 with full extents 4 and 6.
            #expect(abs(properties.inertia[0][0] - 48_000 * (16 + 36) / 12) < 1)
            #expect(!properties.mustBeStatic)
        }

        @Test func offsetShapeBoundsAreInShapeSpace() {
            let shape = Shape.rotatedTranslated(.sphere(radius: 1), position: [5, 0, 0])
            let properties = shape.properties
            #expect(near(properties.centerOfMass, [5, 0, 0]))
            #expect(near(properties.boundsMin, [4, -1, -1]))
            #expect(near(properties.boundsMax, [6, 1, 1]))
        }

        @Test func meshMustBeStatic() {
            let mesh = Shape.mesh(vertices: [[0, 0, 0], [1, 0, 0], [0, 0, 1]], indices: [0, 2, 1])
            #expect(mesh.properties.mustBeStatic)
        }

        @Test func castRayAgainstShape() throws {
            let shape = Shape.rotatedTranslated(.box(halfExtents: [1, 1, 1], convexRadius: 0), position: [5, 0, 0])
            let hit = try #require(shape.castRay(origin: [5, 10, 0], direction: [0, -20, 0]))
            #expect(abs(hit.fraction - 0.45) < 1e-4)
            #expect(near(hit.position, [5, 1, 0]))
            #expect(near(hit.normal, [0, 1, 0]))
            #expect(shape.castRay(origin: [0, 10, 0], direction: [0, -20, 0]) == nil)
        }

        @Test func containsPoint() {
            let shape = Shape.offsetCenterOfMass(.sphere(radius: 1), offset: [0.5, 0, 0])
            #expect(shape.contains([0.9, 0, 0]))
            #expect(!shape.contains([1.1, 0, 0]))
        }

        @Test func scaleValidity() {
            let sphere = Shape.sphere(radius: 1)
            #expect(sphere.isValidScale([2, 2, 2]))
            #expect(!sphere.isValidScale([1, 2, 3]))
            let valid = sphere.validScale(closestTo: [1, 2, 3])
            #expect(valid.x == valid.y && valid.y == valid.z)
            #expect(Shape.box(halfExtents: [1, 1, 1]).isValidScale([1, 2, 3]))
        }
    }
}
