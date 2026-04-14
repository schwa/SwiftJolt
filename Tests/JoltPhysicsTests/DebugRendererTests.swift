import JoltPhysics
import simd
import Testing

final class RecordingRenderer: DebugRenderer {
    var lines: [(SIMD3<Float>, SIMD3<Float>, DebugColor)] = []
    var triangles: [(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>, DebugColor)] = []

    func drawLine(from: SIMD3<Float>, to: SIMD3<Float>, color: DebugColor) {
        lines.append((from, to, color))
    }

    func drawTriangle(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, color: DebugColor) {
        triangles.append((a, b, c, color))
    }

    var allPoints: [SIMD3<Float>] {
        lines.flatMap { [$0.0, $0.1] } + triangles.flatMap { [$0.0, $0.1, $0.2] }
    }
}

extension JoltTests {
    @Suite
    struct DebugRendererTests {
        @Test func drawsSolidShapesAsTrianglesAroundBody() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [1, 2, 3], convexRadius: 0), position: [10, 0, 0], motionType: .static)
            let renderer = RecordingRenderer()
            world.debugDraw(to: renderer)
            #expect(!renderer.triangles.isEmpty)
            // Every vertex lies on the box.
            for point in renderer.allPoints {
                let local = abs(point - [10, 0, 0])
                #expect(local.x <= 1.001 && local.y <= 2.001 && local.z <= 3.001, "\(point)")
            }
            #expect(renderer.allPoints.contains { simd_distance($0, [11, 2, 3]) < 1e-3 })
        }

        @Test func wireframeAndExtrasDrawLines() throws {
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1), position: [0, 5, 0])
            let renderer = RecordingRenderer()
            var settings = DebugDrawSettings()
            settings.wireframe = true
            settings.boundingBoxes = true
            world.debugDraw(to: renderer, settings: settings)
            #expect(!renderer.lines.isEmpty)
            // The bounding box corner shows up as a line endpoint.
            #expect(renderer.lines.contains { simd_distance($0.0, [1, 6, 1]) < 0.01 || simd_distance($0.1, [1, 6, 1]) < 0.01 })
        }

        @Test func nothingDrawnWhenDisabled() throws {
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1))
            let renderer = RecordingRenderer()
            var settings = DebugDrawSettings()
            settings.shapes = false
            world.debugDraw(to: renderer, settings: settings)
            #expect(renderer.lines.isEmpty && renderer.triangles.isEmpty)
        }

        @Test func drawsConstraints() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            world.addHingeConstraint(between: nil, and: body, point: .zero, axis: [0, 0, 1])
            var settings = DebugDrawSettings()
            settings.shapes = false
            let without = RecordingRenderer()
            world.debugDraw(to: without, settings: settings)
            settings.constraints = true
            let with = RecordingRenderer()
            world.debugDraw(to: with, settings: settings)
            #expect(with.lines.count + with.triangles.count > without.lines.count + without.triangles.count)
        }

        @Test func colorsDecodeAsOpaque() throws {
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1))
            let renderer = RecordingRenderer()
            world.debugDraw(to: renderer)
            // Jolt sends this body as (254, 0, 0, 254). A wrong byte order would show up as alpha 0
            // or a different channel.
            #expect(renderer.triangles.allSatisfy { $0.3.alpha > 250 }, "\(Set(renderer.triangles.map(\.3)))")
        }
    }
}
