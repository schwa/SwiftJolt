import CJolt
import simd

public enum DebugDrawMode: Sendable {
    case solid
    case wireframe

    var jph: JPH_DebugRenderer_DrawMode {
        self == .solid ? JPH_DebugRenderer_DrawMode_Solid : JPH_DebugRenderer_DrawMode_Wireframe
    }
}

/// Jolt's debug drawing helpers. Only valid inside `PhysicsWorld.withDebugDrawer`. Transforms
/// place the shape in world space; shapes are centered on the transform's origin.
public struct DebugDrawer: ~Copyable {
    let handle: OpaquePointer
    private let noShadow = JPH_DebugRenderer_CastShadow_Off

    public func drawLine(from: SIMD3<Float>, to: SIMD3<Float>, color: DebugColor) {
        var a = JPH_RVec3(from)
        var b = JPH_RVec3(to)
        JPH_DebugRenderer_DrawLine(handle, &a, &b, color.jph)
    }

    public func drawTriangle(_ v1: SIMD3<Float>, _ v2: SIMD3<Float>, _ v3: SIMD3<Float>, color: DebugColor, mode: DebugDrawMode = .solid) {
        var a = JPH_RVec3(v1)
        var b = JPH_RVec3(v2)
        var c = JPH_RVec3(v3)
        switch mode {
        case .solid: JPH_DebugRenderer_DrawTriangle(handle, &a, &b, &c, color.jph, noShadow)
        case .wireframe: JPH_DebugRenderer_DrawWireTriangle(handle, &a, &b, &c, color.jph)
        }
    }

    /// A small 3D cross.
    public func drawMarker(at position: SIMD3<Float>, color: DebugColor, size: Float = 0.1) {
        var p = JPH_RVec3(position)
        JPH_DebugRenderer_DrawMarker(handle, &p, color.jph, size)
    }

    /// `size` is the arrow head size.
    public func drawArrow(from: SIMD3<Float>, to: SIMD3<Float>, color: DebugColor, size: Float = 0.1) {
        var a = JPH_RVec3(from)
        var b = JPH_RVec3(to)
        JPH_DebugRenderer_DrawArrow(handle, &a, &b, color.jph, size)
    }

    /// Red, green and blue arrows along the transform's X, Y and Z axes.
    public func drawCoordinateSystem(_ transform: simd_float4x4, size: Float = 1) {
        var matrix = JPH_RMat4(transform)
        JPH_DebugRenderer_DrawCoordinateSystem(handle, &matrix, size)
    }

    public func drawPlane(point: SIMD3<Float>, normal: SIMD3<Float>, color: DebugColor, size: Float = 1) {
        var p = JPH_RVec3(point)
        var n = JPH_Vec3(simd_normalize(normal))
        JPH_DebugRenderer_DrawPlane(handle, &p, &n, color.jph, size)
    }

    /// An axis-aligned box from `min` to `max`, moved by `transform` if given.
    public func drawBox(min: SIMD3<Float>, max: SIMD3<Float>, transform: simd_float4x4? = nil, color: DebugColor, mode: DebugDrawMode = .wireframe) {
        var box = JPH_AABox(min: min, max: max)
        if let transform {
            var matrix = JPH_RMat4(transform)
            switch mode {
            case .solid: JPH_DebugRenderer_DrawBox2(handle, &matrix, &box, color.jph, noShadow, mode.jph)
            case .wireframe: JPH_DebugRenderer_DrawWireBox2(handle, &matrix, &box, color.jph)
            }
        } else {
            switch mode {
            case .solid: JPH_DebugRenderer_DrawBox(handle, &box, color.jph, noShadow, mode.jph)
            case .wireframe: JPH_DebugRenderer_DrawWireBox(handle, &box, color.jph)
            }
        }
    }

    /// `level` controls wireframe tessellation.
    public func drawSphere(center: SIMD3<Float>, radius: Float, color: DebugColor, mode: DebugDrawMode = .wireframe, level: Int = 3) {
        var c = JPH_RVec3(center)
        switch mode {
        case .solid: JPH_DebugRenderer_DrawSphere(handle, &c, radius, color.jph, noShadow, mode.jph)
        case .wireframe: JPH_DebugRenderer_DrawWireSphere(handle, &c, radius, color.jph, Int32(level))
        }
    }

    /// A unit sphere shaped by `transform`, e.g. an ellipsoid.
    public func drawUnitSphere(_ transform: simd_float4x4, color: DebugColor, mode: DebugDrawMode = .wireframe, level: Int = 3) {
        var matrix = JPH_RMat4(transform)
        switch mode {
        case .solid: JPH_DebugRenderer_DrawUnitSphere(handle, matrix, color.jph, noShadow, mode.jph)
        case .wireframe: JPH_DebugRenderer_DrawWireUnitSphere(handle, &matrix, color.jph, Int32(level))
        }
    }

    /// Along the transform's Y axis.
    public func drawCapsule(_ transform: simd_float4x4, halfHeightOfCylinder: Float, radius: Float, color: DebugColor, mode: DebugDrawMode = .wireframe) {
        var matrix = JPH_RMat4(transform)
        JPH_DebugRenderer_DrawCapsule(handle, &matrix, halfHeightOfCylinder, radius, color.jph, noShadow, mode.jph)
    }

    /// Along the transform's Y axis.
    public func drawCylinder(_ transform: simd_float4x4, halfHeight: Float, radius: Float, color: DebugColor, mode: DebugDrawMode = .wireframe) {
        var matrix = JPH_RMat4(transform)
        JPH_DebugRenderer_DrawCylinder(handle, &matrix, halfHeight, radius, color.jph, noShadow, mode.jph)
    }

    /// Along the transform's Y axis, from `bottom` to `top`.
    public func drawTaperedCylinder(
        _ transform: simd_float4x4,
        top: Float,
        bottom: Float,
        topRadius: Float,
        bottomRadius: Float,
        color: DebugColor,
        mode: DebugDrawMode = .wireframe
    ) {
        var matrix = JPH_RMat4(transform)
        JPH_DebugRenderer_DrawTaperedCylinder(handle, &matrix, top, bottom, topRadius, bottomRadius, color.jph, noShadow, mode.jph)
    }

    /// A cone with its tip at `top`, opening along `axis`. `perpendicular` must be perpendicular to `axis`.
    public func drawOpenCone(
        top: SIMD3<Float>,
        axis: SIMD3<Float>,
        perpendicular: SIMD3<Float>,
        halfAngle: Float,
        length: Float,
        color: DebugColor,
        mode: DebugDrawMode = .wireframe
    ) {
        var t = JPH_RVec3(top)
        var a = JPH_Vec3(simd_normalize(axis))
        var p = JPH_Vec3(simd_normalize(perpendicular))
        JPH_DebugRenderer_DrawOpenCone(handle, &t, &a, &p, halfAngle, length, color.jph, noShadow, mode.jph)
    }

    /// A pie slice around `normal`, from `minAngle` to `maxAngle` measured from `axis`.
    public func drawPie(
        center: SIMD3<Float>,
        radius: Float,
        normal: SIMD3<Float>,
        axis: SIMD3<Float>,
        angles: ClosedRange<Float>,
        color: DebugColor,
        mode: DebugDrawMode = .wireframe
    ) {
        var c = JPH_RVec3(center)
        var n = JPH_Vec3(simd_normalize(normal))
        var a = JPH_Vec3(simd_normalize(axis))
        JPH_DebugRenderer_DrawPie(handle, &c, radius, &n, &a, angles.lowerBound, angles.upperBound, color.jph, noShadow, mode.jph)
    }

    /// Elliptical swing limits of a swing-twist joint. The twist axis is the transform's X axis.
    public func drawSwingConeLimits(
        _ transform: simd_float4x4,
        swingYHalfAngle: Float,
        swingZHalfAngle: Float,
        edgeLength: Float,
        color: DebugColor,
        mode: DebugDrawMode = .wireframe
    ) {
        var matrix = JPH_RMat4(transform)
        JPH_DebugRenderer_DrawSwingConeLimits(handle, &matrix, swingYHalfAngle, swingZHalfAngle, edgeLength, color.jph, noShadow, mode.jph)
    }

    /// Box-shaped swing limits of a swing-twist joint. The twist axis is the transform's X axis.
    public func drawSwingPyramidLimits(
        _ transform: simd_float4x4,
        swingY: ClosedRange<Float>,
        swingZ: ClosedRange<Float>,
        edgeLength: Float,
        color: DebugColor,
        mode: DebugDrawMode = .wireframe
    ) {
        var matrix = JPH_RMat4(transform)
        JPH_DebugRenderer_DrawSwingPyramidLimits(
            handle, &matrix, swingY.lowerBound, swingY.upperBound, swingZ.lowerBound, swingZ.upperBound,
            edgeLength, color.jph, noShadow, mode.jph
        )
    }
}
