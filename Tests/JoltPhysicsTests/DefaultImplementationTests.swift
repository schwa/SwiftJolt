import JoltPhysics
import Testing

/// Implements nothing, so every callback runs the protocol's default.
private final class SilentActivationListener: ActivationListener {}

private final class SilentContactListener: ContactListener {}

/// Only implements the required methods; `drawText` uses the default.
private final class LinesOnlyRenderer: DebugRenderer {
    var primitiveCount = 0

    func drawLine(from _: SIMD3<Float>, to _: SIMD3<Float>, color _: DebugColor) {
        primitiveCount += 1
    }

    func drawTriangle(_: SIMD3<Float>, _: SIMD3<Float>, _: SIMD3<Float>, color _: DebugColor) {
        primitiveCount += 1
    }
}

private final class TextRenderer: DebugRenderer {
    var texts: [String] = []

    func drawLine(from _: SIMD3<Float>, to _: SIMD3<Float>, color _: DebugColor) {}
    func drawTriangle(_: SIMD3<Float>, _: SIMD3<Float>, _: SIMD3<Float>, color _: DebugColor) {}

    func drawText(_ text: String, at _: SIMD3<Float>, color _: DebugColor, height _: Float) {
        texts.append(text)
    }
}

extension JoltTests {
    @Suite
    struct DefaultImplementationTests {
        @Test func defaultListenersAcceptContactsAndActivation() throws {
            let world = try makeWorld()
            world.activationListener = SilentActivationListener()
            world.contactListener = SilentContactListener()
            world.addBody(.box(halfExtents: [10, 0.5, 10]), position: [0, -0.5, 0], motionType: .static)
            let ball = world.addBody(.sphere(radius: 0.5), position: [0, 1, 0], restitution: 0)
            // Lands (validate, added, persisted), sleeps, then leaves the floor (removed).
            simulate(world, seconds: 3)
            #expect(abs(world.position(of: ball).y - 0.5) < 0.05)
            world.setPosition(of: ball, to: [0, 20, 0])
            simulate(world, seconds: 0.1)
            #expect(world.position(of: ball).y > 10)
        }

        @Test func debugColorComponents() {
            let color = DebugColor(red: 255, green: 0, blue: 51)
            #expect(color.alpha == 255)
            #expect(color.simd == [1, 0, 0.2, 1])
            #expect(DebugColor(red: 0, green: 0, blue: 0, alpha: 0).simd == .zero)
        }

        // A wheeled vehicle draws a status text when constraints are drawn.
        func drawVehicle(to renderer: any DebugRenderer) throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [100, 0.5, 100]), position: [0, -0.5, 0], motionType: .static)
            let body = world.addBody(.box(halfExtents: [0.9, 0.2, 2]), position: [0, 1, 0])
            world.addWheeledVehicle(
                body: body,
                wheels: [[-0.9, 0, 1.4], [0.9, 0, 1.4], [-0.9, 0, -1.4], [0.9, 0, -1.4]].map { WheeledVehicleWheel(WheelSettings(position: $0)) },
                drivenWheels: [(0, 1)]
            )
            simulate(world, seconds: 0.5)
            var settings = DebugDrawSettings()
            settings.constraints = true
            world.debugDraw(to: renderer, settings: settings)
        }

        @Test func textReachesRendererThatDrawsIt() throws {
            let renderer = TextRenderer()
            try drawVehicle(to: renderer)
            #expect(!renderer.texts.isEmpty)
        }

        @Test func defaultDrawTextIgnoresText() throws {
            let renderer = LinesOnlyRenderer()
            try drawVehicle(to: renderer)
            #expect(renderer.primitiveCount > 0)
        }
    }
}
