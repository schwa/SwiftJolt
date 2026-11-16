import CJolt
import simd

/// Keyframed joint animation. Sample it into a `SkeletonPose`, for example to drive a ragdoll
/// with `Ragdoll.driveToPoseUsingMotors`.
public final class SkeletalAnimation {
    public let engine: JoltEngine
    let handle: OpaquePointer
    private var lastKeyframeTimes: [Float?] = []
    private var jointNames: [String] = []

    public init(engine: JoltEngine) {
        self.engine = engine
        handle = JPH_SkeletalAnimation_Create()
    }

    deinit {
        JPH_SkeletalAnimation_Destroy(handle)
    }

    /// Adds a track for the skeleton joint named `jointName` and returns its track index.
    /// Joints without a track keep their pose state when sampling.
    @discardableResult
    public func addJoint(_ jointName: String) -> Int {
        JPH_SkeletalAnimation_AddAnimatedJoint(handle, jointName)
        jointNames.append(jointName)
        lastKeyframeTimes.append(nil)
        return lastKeyframeTimes.count - 1
    }

    public var jointCount: Int {
        Int(JPH_SkeletalAnimation_GetAnimatedJointCount(handle))
    }

    /// Adds a keyframe to a track. `state` is relative to the joint's parent. Keyframes of a track
    /// must be added in increasing time order.
    public func addKeyframe(track: Int, time: Float, state: SkeletonPose.JointState) {
        precondition(lastKeyframeTimes.indices.contains(track), "Track index out of range")
        if let last = lastKeyframeTimes[track] {
            precondition(time > last, "Keyframes must be added in increasing time order")
        }
        lastKeyframeTimes[track] = time
        var translation = JPH_Vec3(state.translation)
        var rotation = JPH_Quat(state.rotation)
        JPH_SkeletalAnimation_AddKeyframe(handle, Int32(track), time, &translation, &rotation)
    }

    /// Seconds. Jolt takes this from the last keyframe of the first track only, so give that
    /// track a keyframe at the end of the animation.
    public var duration: Float {
        JPH_SkeletalAnimation_GetDuration(handle)
    }

    /// When true (Jolt's default), sampling past `duration` wraps around. When false, each track
    /// holds its last keyframe.
    public var isLooping: Bool {
        get { JPH_SkeletalAnimation_IsLooping(handle) }
        set { JPH_SkeletalAnimation_SetIsLooping(handle, newValue) }
    }

    /// Multiplies all keyframe translations by `scale`, e.g. to fit a smaller character.
    public func scaleJoints(by scale: Float) {
        JPH_SkeletalAnimation_ScaleJoints(handle, scale)
    }

    /// Writes the joint states at `time` into `pose` and updates its joint matrices.
    public func sample(at time: Float, into pose: SkeletonPose) {
        precondition(time >= 0, "Sample time must not be negative")
        for name in jointNames where pose.skeleton.jointIndex(named: name) == nil {
            preconditionFailure("Animated joint \(name) is not in the pose's skeleton")
        }
        JPH_SkeletalAnimation_Sample(handle, time, pose.handle)
        pose.calculateJointMatrices()
    }
}
