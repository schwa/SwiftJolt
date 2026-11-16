import CJolt
import simd

/// Computed properties of a `Shape`. Positions are in the shape's own space (its origin, not its center of mass).
public struct ShapeProperties: Sendable {
    public var volume: Float
    public var centerOfMass: SIMD3<Float>
    public var boundsMin: SIMD3<Float>
    public var boundsMax: SIMD3<Float>
    /// Radius of the largest sphere around the center of mass that fits inside the shape.
    public var innerRadius: Float
    /// Mass at the shape's density (1000 kg/m³ unless set by a material).
    public var mass: Float
    /// Inertia tensor about the center of mass.
    public var inertia: simd_float3x3
    /// Mesh, height field, plane and triangle-based shapes can only be used on static or kinematic bodies.
    public var mustBeStatic: Bool
}

/// A ray hit on a lone shape.
public struct ShapeRayHit: Sendable {
    /// Distance along the ray as a fraction of `direction`, in 0...1.
    public var fraction: Float
    public var position: SIMD3<Float>
    public var normal: SIMD3<Float>
}

extension Shape {
    // Each call builds a Jolt shape, which is slow for meshes and convex hulls, and starts Jolt if
    // no JoltEngine is alive. Keep the results if you need them often.
    func withHandle<Result>(_ body: (OpaquePointer, _ centerOfMass: SIMD3<Float>) -> Result) -> Result {
        guard let engine = try? JoltEngine() else {
            preconditionFailure("Could not initialize Jolt")
        }
        defer { withExtendedLifetime(engine) {} }
        let handle = createHandle()
        defer { JPH_Shape_Destroy(handle) }
        var centerOfMass = JPH_Vec3()
        JPH_Shape_GetCenterOfMass(handle, &centerOfMass)
        return body(handle, centerOfMass.simd)
    }

    public var properties: ShapeProperties {
        withHandle { handle, centerOfMass in
            var bounds = JPH_AABox()
            JPH_Shape_GetLocalBounds(handle, &bounds)
            var massProperties = JPH_MassProperties()
            JPH_Shape_GetMassProperties(handle, &massProperties)
            let inertia = massProperties.inertia.simd
            return ShapeProperties(
                volume: JPH_Shape_GetVolume(handle),
                centerOfMass: centerOfMass,
                boundsMin: bounds.min.simd + centerOfMass,
                boundsMax: bounds.max.simd + centerOfMass,
                innerRadius: JPH_Shape_GetInnerRadius(handle),
                mass: massProperties.mass,
                inertia: simd_float3x3(inertia.columns.0.xyz, inertia.columns.1.xyz, inertia.columns.2.xyz),
                mustBeStatic: JPH_Shape_MustBeStatic(handle)
            )
        }
    }

    /// Casts a ray in the shape's own space. Returns the closest hit, or nil.
    public func castRay(origin: SIMD3<Float>, direction: SIMD3<Float>) -> ShapeRayHit? {
        withHandle { handle, centerOfMass in
            // Jolt's shape queries are relative to the center of mass.
            var localOrigin = JPH_Vec3(origin - centerOfMass)
            var jphDirection = JPH_Vec3(direction)
            var result = JPH_RayCastResult()
            guard JPH_Shape_CastRay(handle, &localOrigin, &jphDirection, &result) else {
                return nil
            }
            let position = origin + direction * result.fraction
            var localPosition = JPH_Vec3(position - centerOfMass)
            var normal = JPH_Vec3()
            JPH_Shape_GetSurfaceNormal(handle, result.subShapeID2, &localPosition, &normal)
            return ShapeRayHit(fraction: result.fraction, position: position, normal: normal.simd)
        }
    }

    /// True if `point`, in the shape's own space, is inside the shape.
    public func contains(_ point: SIMD3<Float>) -> Bool {
        withHandle { handle, centerOfMass in
            var localPoint = JPH_Vec3(point - centerOfMass)
            return JPH_Shape_CollidePoint(handle, &localPoint, nil)
        }
    }

    /// False when Jolt can't apply `scale` to this shape, e.g. non-uniform scale on a sphere.
    public func isValidScale(_ scale: SIMD3<Float>) -> Bool {
        withHandle { handle, _ in
            var jphScale = JPH_Vec3(scale)
            return JPH_Shape_IsValidScale(handle, &jphScale)
        }
    }

    /// The closest scale to `scale` that Jolt can apply to this shape.
    public func validScale(closestTo scale: SIMD3<Float>) -> SIMD3<Float> {
        withHandle { handle, _ in
            var jphScale = JPH_Vec3(scale)
            var result = JPH_Vec3()
            JPH_Shape_MakeScaleValid(handle, &jphScale, &result)
            return result.simd
        }
    }
}

extension SIMD4<Float> {
    var xyz: SIMD3<Float> { SIMD3(x, y, z) }
}
