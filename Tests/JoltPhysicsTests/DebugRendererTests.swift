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

        @Test func drawsConstraintLimitsAndFrames() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            world.addHingeConstraint(between: nil, and: body, point: .zero, axis: [0, 0, 1], limits: -1...1)
            var settings = DebugDrawSettings()
            settings.shapes = false
            settings.constraintLimits = true
            let limits = RecordingRenderer()
            world.debugDraw(to: limits, settings: settings)
            #expect(!limits.allPoints.isEmpty)
            settings.constraintLimits = false
            settings.constraintReferenceFrames = true
            let frames = RecordingRenderer()
            world.debugDraw(to: frames, settings: settings)
            #expect(!frames.allPoints.isEmpty)
        }

        @Test func bodyFilterSkipsBodies() throws {
            let world = try makeWorld()
            let near = world.addBody(.sphere(radius: 1), position: [0, 0, 0], motionType: .static)
            world.addBody(.sphere(radius: 1), position: [50, 0, 0], motionType: .static)
            let renderer = RecordingRenderer()
            world.debugDraw(to: renderer) { $0 == near }
            #expect(!renderer.allPoints.isEmpty)
            #expect(renderer.allPoints.allSatisfy { simd_length($0) < 1.01 })
        }

        @Test func drawerHelpersEmitGeometry() throws {
            let world = try makeWorld()
            let red = DebugColor(red: 255, green: 0, blue: 0)
            let renderer = RecordingRenderer()
            world.withDebugDrawer(renderer, cameraPosition: [0, 0, 10]) { drawer in
                drawer.drawLine(from: .zero, to: [1, 0, 0], color: red)
            }
            #expect(renderer.lines.count == 1)
            #expect(renderer.lines.first.map { $0.1 == [1, 0, 0] && $0.2 == red } == true)

            let identity = matrix_identity_float4x4
            let calls: [(String, (borrowing DebugDrawer) -> Void)] = [
                ("triangle", { $0.drawTriangle(.zero, [1, 0, 0], [0, 1, 0], color: red) }),
                ("wireTriangle", { $0.drawTriangle(.zero, [1, 0, 0], [0, 1, 0], color: red, mode: .wireframe) }),
                ("marker", { $0.drawMarker(at: .zero, color: red) }),
                ("arrow", { $0.drawArrow(from: .zero, to: [0, 1, 0], color: red) }),
                ("axes", { $0.drawCoordinateSystem(identity) }),
                ("plane", { $0.drawPlane(point: .zero, normal: [0, 1, 0], color: red) }),
                ("wireBox", { $0.drawBox(min: [-1, -1, -1], max: [1, 1, 1], color: red) }),
                ("solidBox", { $0.drawBox(min: [-1, -1, -1], max: [1, 1, 1], transform: identity, color: red, mode: .solid) }),
                ("wireSphere", { $0.drawSphere(center: .zero, radius: 1, color: red) }),
                ("solidSphere", { $0.drawSphere(center: .zero, radius: 1, color: red, mode: .solid) }),
                ("unitSphere", { $0.drawUnitSphere(identity, color: red, mode: .solid) }),
                ("capsule", { $0.drawCapsule(identity, halfHeightOfCylinder: 1, radius: 0.5, color: red) }),
                ("cylinder", { $0.drawCylinder(identity, halfHeight: 1, radius: 0.5, color: red) }),
                ("taperedCylinder", { $0.drawTaperedCylinder(identity, top: 1, bottom: -1, topRadius: 0.2, bottomRadius: 0.5, color: red) }),
                ("cone", { $0.drawOpenCone(top: .zero, axis: [0, 1, 0], perpendicular: [1, 0, 0], halfAngle: 0.5, length: 1, color: red) }),
                ("pie", { $0.drawPie(center: .zero, radius: 1, normal: [0, 1, 0], axis: [1, 0, 0], angles: 0...1, color: red) }),
                ("swingCone", { $0.drawSwingConeLimits(identity, swingYHalfAngle: 0.5, swingZHalfAngle: 0.3, edgeLength: 1, color: red) }),
                ("swingPyramid", { $0.drawSwingPyramidLimits(identity, swingY: -0.5...0.5, swingZ: -0.3...0.3, edgeLength: 1, color: red) })
            ]
            for (name, call) in calls {
                let recorder = RecordingRenderer()
                world.withDebugDrawer(recorder) { call($0) }
                #expect(!recorder.allPoints.isEmpty, "\(name) drew nothing")
            }
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
