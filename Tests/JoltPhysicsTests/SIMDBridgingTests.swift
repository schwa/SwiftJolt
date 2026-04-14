import CJolt
@testable import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct SIMDBridgingTests {
        @Test func vectorsAndQuaternionsRoundTrip() {
            #expect(JPH_Vec3([1, 2, 3]).simd == [1, 2, 3])
            #expect(JPH_Vec4([1, 2, 3, 4]).simd == [1, 2, 3, 4])
            let q = simd_quatf(angle: 0.5, axis: simd_normalize([1, 2, 3]))
            #expect(JPH_Quat(q).simd.vector == q.vector)
        }

        @Test func matrixRoundTrips() {
            let m = simd_float4x4(rows: [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]])
            #expect(JPH_Mat4(m).simd == m)
        }

        @Test func matrixLayoutMatchesJolt() throws {
            _ = try JoltEngine()
            let m = simd_float4x4(rows: [[1, 2, 3, 10], [4, 5, 6, 20], [7, 8, 9, 30], [0, 0, 0, 1]])
            var jphMatrix = JPH_Mat4(m)
            var v = JPH_Vec3([1, 2, 3])
            var result = JPH_Vec3()
            JPH_Vec3_MultiplyMatrix(&jphMatrix, &v, &result)
            let expected = m * SIMD4<Float>(1, 2, 3, 1)
            #expect(result.simd == SIMD3(expected.x, expected.y, expected.z))
        }

        @Test func worldTransformMatchesPositionAndRotation() throws {
            let world = try makeWorld()
            let rotation = simd_quatf(angle: .pi / 3, axis: [0, 0, 1])
            let body = world.addBody(.box(halfExtents: [1, 1, 1]), position: [1, 2, 3], rotation: rotation, motionType: .static)
            var expected = simd_float4x4(rotation)
            expected.columns.3 = [1, 2, 3, 1]
            let transform = world.worldTransform(of: body)
            for column in 0..<4 {
                #expect(simd_distance(transform[column], expected[column]) < 1e-5)
            }
        }
    }
}
