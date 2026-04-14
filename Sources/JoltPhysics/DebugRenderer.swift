import CJolt
import simd

public struct DebugColor: Sendable, Hashable {
    public var red: UInt8
    public var green: UInt8
    public var blue: UInt8
    public var alpha: UInt8

    public init(red: UInt8, green: UInt8, blue: UInt8, alpha: UInt8 = 255) {
        self.red = red
        self.green = green
        self.blue = blue
        self.alpha = alpha
    }

    // Jolt's Color packs red in the lowest byte.
    init(_ color: JPH_Color) {
        red = UInt8(truncatingIfNeeded: color)
        green = UInt8(truncatingIfNeeded: color >> 8)
        blue = UInt8(truncatingIfNeeded: color >> 16)
        alpha = UInt8(truncatingIfNeeded: color >> 24)
    }

    /// Components in 0...1.
    public var simd: SIMD4<Float> {
        SIMD4(Float(red), Float(green), Float(blue), Float(alpha)) / 255
    }
}

/// Receives debug geometry from `PhysicsWorld.debugDraw`. Called on the calling thread.
public protocol DebugRenderer: AnyObject {
    func drawLine(from: SIMD3<Float>, to: SIMD3<Float>, color: DebugColor)
    func drawTriangle(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>, color: DebugColor)
    func drawText(_ text: String, at position: SIMD3<Float>, color: DebugColor, height: Float)
}

public extension DebugRenderer {
    func drawText(_: String, at _: SIMD3<Float>, color _: DebugColor, height _: Float) {}
}

public struct DebugDrawSettings: Sendable {
    public var shapes = true
    /// Draw shapes as wireframes (lines) instead of triangles.
    public var wireframe = false
    public var boundingBoxes = false
    public var centerOfMass = false
    public var worldTransforms = false
    public var velocities = false
    /// Also draw constraints.
    public var constraints = false

    public init() {}
}

private final class DebugRendererBox {
    let renderer: any DebugRenderer

    init(_ renderer: any DebugRenderer) {
        self.renderer = renderer
    }

    static func from(_ userData: UnsafeMutableRawPointer?) -> any DebugRenderer {
        Unmanaged<DebugRendererBox>.fromOpaque(userData!).takeUnretainedValue().renderer
    }
}

// joltc keeps the pointer it is given, so the procs struct is allocated once and never freed.
private let installDebugRendererProcs: Void = {
    let procs = UnsafeMutablePointer<JPH_DebugRenderer_Procs>.allocate(capacity: 1)
    procs.initialize(to: JPH_DebugRenderer_Procs(
        DrawLine: { userData, from, to, color in
            DebugRendererBox.from(userData).drawLine(from: from!.pointee.simd, to: to!.pointee.simd, color: DebugColor(color))
        },
        DrawTriangle: { userData, a, b, c, color, _ in
            DebugRendererBox.from(userData).drawTriangle(a!.pointee.simd, b!.pointee.simd, c!.pointee.simd, color: DebugColor(color))
        },
        DrawText3D: { userData, position, text, color, height in
            DebugRendererBox.from(userData).drawText(String(cString: text!), at: position!.pointee.simd, color: DebugColor(color), height: height)
        }
    ))
    JPH_DebugRenderer_SetProcs(procs)
}()

extension PhysicsWorld {
    /// Sends debug geometry for all bodies (and optionally constraints) to `renderer`.
    public func debugDraw(to renderer: any DebugRenderer, settings: DebugDrawSettings = DebugDrawSettings()) {
        _ = installDebugRendererProcs
        let box = DebugRendererBox(renderer)
        withExtendedLifetime(box) {
            let handle = JPH_DebugRenderer_Create(Unmanaged.passUnretained(box).toOpaque())!
            defer { JPH_DebugRenderer_Destroy(handle) }
            var drawSettings = JPH_DrawSettings()
            JPH_DrawSettings_InitDefault(&drawSettings)
            drawSettings.drawShape = settings.shapes
            drawSettings.drawShapeWireframe = settings.wireframe
            drawSettings.drawBoundingBox = settings.boundingBoxes
            drawSettings.drawCenterOfMassTransform = settings.centerOfMass
            drawSettings.drawWorldTransform = settings.worldTransforms
            drawSettings.drawVelocity = settings.velocities
            JPH_PhysicsSystem_DrawBodies(system, &drawSettings, handle, nil)
            if settings.constraints {
                JPH_PhysicsSystem_DrawConstraints(system, handle)
            }
        }
    }
}
