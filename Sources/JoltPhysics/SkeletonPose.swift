import CJolt
import simd

/// Joint transforms for a skeleton. States are local to the parent; matrices are model space.
public final class SkeletonPose {
    public struct JointState: Sendable, Equatable {
        public var translation: SIMD3<Float>
        public var rotation: simd_quatf

        public init(translation: SIMD3<Float> = .zero, rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)) {
            self.translation = translation
            self.rotation = rotation
        }
    }

    /// Jolt holds a raw pointer to the skeleton, so the pose keeps it alive.
    public let skeleton: Skeleton
    let handle: OpaquePointer

    /// Sized to the skeleton's joints at creation. Add all joints first.
    public init(skeleton: Skeleton) {
        self.skeleton = skeleton
        handle = JPH_SkeletonPose_Create()
        JPH_SkeletonPose_SetSkeleton(handle, skeleton.handle)
    }

    deinit {
        JPH_SkeletonPose_Destroy(handle)
    }

    public var jointCount: Int {
        Int(JPH_SkeletonPose_GetJointCount(handle))
    }

    public var rootOffset: SIMD3<Float> {
        get {
            var offset = JPH_RVec3()
            JPH_SkeletonPose_GetRootOffset(handle, &offset)
            return offset.simd
        }
        set {
            var offset = JPH_RVec3(newValue)
            JPH_SkeletonPose_SetRootOffset(handle, &offset)
        }
    }

    public func jointState(at index: Int) -> JointState {
        checkIndex(index)
        var translation = JPH_Vec3()
        var rotation = JPH_Quat()
        JPH_SkeletonPose_GetJointState(handle, Int32(index), &translation, &rotation)
        return JointState(translation: translation.simd, rotation: rotation.simd)
    }

    public func setJointState(_ state: JointState, at index: Int) {
        checkIndex(index)
        var translation = JPH_Vec3(state.translation)
        var rotation = JPH_Quat(state.rotation)
        JPH_SkeletonPose_SetJointState(handle, Int32(index), &translation, &rotation)
    }

    public func jointMatrix(at index: Int) -> simd_float4x4 {
        checkIndex(index)
        var matrix = JPH_Mat4()
        JPH_SkeletonPose_GetJointMatrix(handle, Int32(index), &matrix)
        return matrix.simd
    }

    public func setJointMatrix(_ matrix: simd_float4x4, at index: Int) {
        checkIndex(index)
        var jphMatrix = JPH_Mat4(matrix)
        JPH_SkeletonPose_SetJointMatrix(handle, Int32(index), &jphMatrix)
    }

    public var jointMatrices: [simd_float4x4] {
        get {
            var matrices = [JPH_Mat4](repeating: JPH_Mat4(), count: jointCount)
            JPH_SkeletonPose_GetJointMatrices(handle, &matrices, Int32(matrices.count))
            return matrices.map(\.simd)
        }
        set {
            precondition(newValue.count == jointCount, "Expected \(jointCount) matrices")
            let matrices = newValue.map(JPH_Mat4.init)
            JPH_SkeletonPose_SetJointMatrices(handle, matrices, Int32(matrices.count))
        }
    }

    /// Updates model-space matrices from joint states.
    public func calculateJointMatrices() {
        JPH_SkeletonPose_CalculateJointMatrices(handle)
    }

    /// Updates joint states from model-space matrices.
    public func calculateJointStates() {
        JPH_SkeletonPose_CalculateJointStates(handle)
    }

    /// Joint matrices relative to each parent.
    public func localSpaceJointMatrices() -> [simd_float4x4] {
        var matrices = [JPH_Mat4](repeating: JPH_Mat4(), count: jointCount)
        JPH_SkeletonPose_CalculateLocalSpaceJointMatrices(handle, &matrices)
        return matrices.map(\.simd)
    }

    private func checkIndex(_ index: Int) {
        precondition((0..<jointCount).contains(index), "Joint index out of range")
    }
}
