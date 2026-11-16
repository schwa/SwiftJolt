import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct SkeletalAnimationTests {
        /// The spine rises from y 0.5 to 1.5 over 2 seconds; the pelvis stays put.
        func makeAnimation(engine: JoltEngine) -> SkeletalAnimation {
            let animation = SkeletalAnimation(engine: engine)
            let pelvis = animation.addJoint("pelvis")
            let spine = animation.addJoint("spine")
            animation.addKeyframe(track: pelvis, time: 0, state: .init(translation: [0, 1, 0]))
            animation.addKeyframe(track: pelvis, time: 2, state: .init(translation: [0, 1, 0]))
            animation.addKeyframe(track: spine, time: 0, state: .init(translation: [0, 0.5, 0]))
            animation.addKeyframe(track: spine, time: 2, state: .init(translation: [0, 1.5, 0]))
            return animation
        }

        @Test func reportsDurationAndJoints() throws {
            let animation = makeAnimation(engine: try JoltEngine())
            #expect(animation.jointCount == 2)
            #expect(animation.duration == 2)
            #expect(animation.isLooping)
            animation.isLooping = false
            #expect(!animation.isLooping)
        }

        @Test func sampleInterpolatesKeyframes() throws {
            let skeleton = try makeSkeleton()
            let animation = makeAnimation(engine: skeleton.engine)
            animation.isLooping = false
            let pose = SkeletonPose(skeleton: skeleton)
            animation.sample(at: 1, into: pose)
            #expect(simd_distance(pose.jointState(at: 1).translation, [0, 1, 0]) < 1e-4)
            // Model space: pelvis (0,1,0) + spine (0,1,0).
            let spine = pose.jointMatrix(at: 1).columns.3
            #expect(simd_distance(SIMD3(spine.x, spine.y, spine.z), [0, 2, 0]) < 1e-4)
            // Past the end, a non-looping animation holds the last keyframe.
            animation.sample(at: 5, into: pose)
            #expect(simd_distance(pose.jointState(at: 1).translation, [0, 1.5, 0]) < 1e-4)
        }

        @Test func loopingWrapsTime() throws {
            let skeleton = try makeSkeleton()
            let animation = makeAnimation(engine: skeleton.engine)
            let pose = SkeletonPose(skeleton: skeleton)
            animation.sample(at: 3, into: pose)
            #expect(simd_distance(pose.jointState(at: 1).translation, [0, 1, 0]) < 1e-4)
        }

        @Test func scaleJointsScalesTranslations() throws {
            let skeleton = try makeSkeleton()
            let animation = makeAnimation(engine: skeleton.engine)
            animation.scaleJoints(by: 2)
            let pose = SkeletonPose(skeleton: skeleton)
            animation.sample(at: 0, into: pose)
            #expect(simd_distance(pose.jointState(at: 1).translation, [0, 1, 0]) < 1e-4)
        }
    }
}
