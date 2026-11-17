import JoltPhysics
import simd
import Testing

private func isClose(_ a: SIMD3<Float>, _ b: SIMD3<Float>, tolerance: Float = 1e-4) -> Bool {
    simd_distance(a, b) < tolerance
}

extension JoltTests {
    @Suite
    struct SkeletonPoseTests {
        @Test func sizedToSkeleton() throws {
            let pose = SkeletonPose(skeleton: try makeSkeleton())
            #expect(pose.jointCount == 4)
            #expect(pose.jointMatrices.count == 4)
        }

        @Test func jointMatricesSetterUpdatesStates() throws {
            let pose = SkeletonPose(skeleton: try makeSkeleton())
            var matrices = [simd_float4x4](repeating: matrix_identity_float4x4, count: 4)
            matrices[0].columns.3 = [0, 1, 0, 1]
            matrices[1].columns.3 = [0, 1.5, 0, 1]
            pose.jointMatrices = matrices
            #expect(pose.jointMatrices[1].columns.3 == [0, 1.5, 0, 1])
            pose.calculateJointStates()
            // The spine is 0.5 above its parent.
            #expect(isClose(pose.jointState(at: 1).translation, [0, 0.5, 0]))
        }

        @Test func statesRoundTripThroughMatrices() throws {
            let pose = SkeletonPose(skeleton: try makeSkeleton())
            let turn = simd_quatf(angle: .pi / 2, axis: [0, 1, 0])
            pose.setJointState(.init(translation: [0, 1, 0]), at: 0)
            pose.setJointState(.init(translation: [0, 0.5, 0], rotation: turn), at: 1)
            pose.setJointState(.init(translation: [1, 0, 0]), at: 2)
            pose.setJointState(.init(translation: [0.2, -0.5, 0]), at: 3)
            pose.calculateJointMatrices()

            // head = pelvis(0,1,0) + spine(0,0.5,0) + turn * (1,0,0)
            let head = pose.jointMatrix(at: 2).columns.3
            #expect(isClose([head.x, head.y, head.z], [0, 1.5, -1]))

            let state = pose.jointState(at: 1)
            pose.setJointState(.init(), at: 1)
            pose.calculateJointStates()
            #expect(isClose(pose.jointState(at: 1).translation, state.translation))
            #expect(abs(simd_dot(pose.jointState(at: 1).rotation, turn)) > 0.9999)
        }

        @Test func localSpaceMatrices() throws {
            let pose = SkeletonPose(skeleton: try makeSkeleton())
            pose.setJointState(.init(translation: [0, 1, 0]), at: 0)
            pose.setJointState(.init(translation: [0, 0.5, 0]), at: 1)
            pose.calculateJointMatrices()
            let local = pose.localSpaceJointMatrices()[1].columns.3
            #expect(isClose([local.x, local.y, local.z], [0, 0.5, 0]))
        }

        @Test func rootOffsetAndMatrixSetters() throws {
            let pose = SkeletonPose(skeleton: try makeSkeleton())
            pose.rootOffset = [1, 2, 3]
            #expect(pose.rootOffset == [1, 2, 3])
            let matrix = simd_float4x4(translation: [4, 5, 6])
            pose.setJointMatrix(matrix, at: 0)
            #expect(pose.jointMatrix(at: 0) == matrix)
        }
    }
}

private extension simd_float4x4 {
    init(translation: SIMD3<Float>) {
        self = matrix_identity_float4x4
        columns.3 = SIMD4(translation, 1)
    }
}
