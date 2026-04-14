import CJolt
import simd

// JPH_RVec3 is a typedef of JPH_Vec3 in single-precision builds, so it needs no separate bridge.

extension JPH_Vec3 {
    init(_ v: SIMD3<Float>) {
        self.init(x: v.x, y: v.y, z: v.z)
    }
    var simd: SIMD3<Float> { .init(x, y, z) }
}

extension JPH_Vec4 {
    init(_ v: SIMD4<Float>) {
        self.init(x: v.x, y: v.y, z: v.z, w: v.w)
    }
    var simd: SIMD4<Float> { .init(x, y, z, w) }
}

extension JPH_Quat {
    init(_ q: simd_quatf) {
        self.init(x: q.vector.x, y: q.vector.y, z: q.vector.z, w: q.vector.w)
    }
    var simd: simd_quatf { .init(vector: .init(x, y, z, w)) }
}

// Both JPH_Mat4 and simd_float4x4 are column-major.
extension JPH_Mat4 {
    init(_ m: simd_float4x4) {
        self.init(column: (JPH_Vec4(m.columns.0), JPH_Vec4(m.columns.1), JPH_Vec4(m.columns.2), JPH_Vec4(m.columns.3)))
    }
    var simd: simd_float4x4 {
        .init(column.0.simd, column.1.simd, column.2.simd, column.3.simd)
    }
}

extension JPH_AABox {
    init(min: SIMD3<Float>, max: SIMD3<Float>) {
        self.init(min: JPH_Vec3(min), max: JPH_Vec3(max))
    }
}

extension JPH_Plane {
    init(normal: SIMD3<Float>, distance: Float) {
        self.init(normal: JPH_Vec3(normal), distance: distance)
    }
}
