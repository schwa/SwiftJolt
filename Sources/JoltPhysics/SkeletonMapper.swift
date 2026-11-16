import CJolt
import simd

/// Maps poses between a detailed skeleton (e.g. an animation rig) and a simpler one (e.g. a
/// ragdoll). Joints with the same name map directly; chains between mapped joints are
/// interpolated. Poses are arrays of model-space matrices, one per joint.
public final class SkeletonMapper {
    public let skeleton1: Skeleton
    public let skeleton2: Skeleton
    let handle: OpaquePointer

    public init(skeleton1: Skeleton, neutralPose1: [simd_float4x4], skeleton2: Skeleton, neutralPose2: [simd_float4x4]) {
        precondition(neutralPose1.count == skeleton1.jointCount, "neutralPose1 needs one matrix per joint")
        precondition(neutralPose2.count == skeleton2.jointCount, "neutralPose2 needs one matrix per joint")
        self.skeleton1 = skeleton1
        self.skeleton2 = skeleton2
        handle = JPH_SkeletonMapper_Create()
        let pose1 = neutralPose1.map(JPH_Mat4.init)
        let pose2 = neutralPose2.map(JPH_Mat4.init)
        JPH_SkeletonMapper_Initialize(handle, skeleton1.handle, pose1, skeleton2.handle, pose2)
    }

    deinit {
        JPH_SkeletonMapper_Destroy(handle)
    }

    /// Keeps every skeleton2 joint at its neutral distance from its parent when mapping.
    public func lockAllTranslations(neutralPose2: [simd_float4x4]) {
        checkPose2(neutralPose2)
        JPH_SkeletonMapper_LockAllTranslations(handle, skeleton2.handle, neutralPose2.map(JPH_Mat4.init))
    }

    /// Like `lockAllTranslations`, but only for skeleton2 joints where `locked` is true.
    public func lockTranslations(_ locked: [Bool], neutralPose2: [simd_float4x4]) {
        precondition(locked.count == skeleton2.jointCount, "Need one flag per skeleton2 joint")
        checkPose2(neutralPose2)
        JPH_SkeletonMapper_LockTranslations(handle, skeleton2.handle, locked, neutralPose2.map(JPH_Mat4.init))
    }

    /// Maps a skeleton1 model-space pose to skeleton2. `localPose2` supplies parent-relative
    /// matrices for skeleton2 joints that have no counterpart.
    public func map(_ pose1: [simd_float4x4], localPose2: [simd_float4x4]) -> [simd_float4x4] {
        precondition(pose1.count == skeleton1.jointCount, "pose1 needs one matrix per joint")
        checkPose2(localPose2)
        var result = [JPH_Mat4](repeating: JPH_Mat4(), count: skeleton2.jointCount)
        JPH_SkeletonMapper_Map(handle, pose1.map(JPH_Mat4.init), localPose2.map(JPH_Mat4.init), &result)
        return result.map(\.simd)
    }

    /// Maps a skeleton2 model-space pose back onto `pose1`. Joints with no counterpart keep their
    /// matrices from `pose1`.
    public func mapReverse(_ pose2: [simd_float4x4], onto pose1: [simd_float4x4]) -> [simd_float4x4] {
        checkPose2(pose2)
        precondition(pose1.count == skeleton1.jointCount, "pose1 needs one matrix per joint")
        var result = pose1.map(JPH_Mat4.init)
        JPH_SkeletonMapper_MapReverse(handle, pose2.map(JPH_Mat4.init), &result)
        return result.map(\.simd)
    }

    /// The skeleton2 joint mapped directly to a skeleton1 joint, if any.
    public func mappedJointIndex(forJoint1 index: Int) -> Int? {
        precondition((0..<skeleton1.jointCount).contains(index), "Joint index out of range")
        let mapped = JPH_SkeletonMapper_GetMappedJointIndex(handle, Int32(index))
        return mapped >= 0 ? Int(mapped) : nil
    }

    public func isJointTranslationLocked(joint2 index: Int) -> Bool {
        precondition((0..<skeleton2.jointCount).contains(index), "Joint index out of range")
        return JPH_SkeletonMapper_IsJointTranslationLocked(handle, Int32(index))
    }

    private func checkPose2(_ pose: [simd_float4x4]) {
        precondition(pose.count == skeleton2.jointCount, "Pose needs one matrix per skeleton2 joint")
    }
}
