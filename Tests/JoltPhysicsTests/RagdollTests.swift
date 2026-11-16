import JoltPhysics
import simd
import Testing

/// Three stacked boxes: pelvis at y=1, spine above it, head on top.
func makeRagdollSettings(engine: JoltEngine) -> RagdollSettings {
    let skeleton = Skeleton(engine: engine)
    let pelvis = skeleton.addJoint("pelvis")
    let spine = skeleton.addJoint("spine", parent: pelvis)
    skeleton.addJoint("head", parent: spine)

    let settings = RagdollSettings(skeleton: skeleton)
    let box = Shape.box(halfExtents: [0.2, 0.2, 0.2])
    let limits = SwingTwistLimits(normalHalfConeAngle: .pi / 4, planeHalfConeAngle: .pi / 4, twist: -0.3...0.3)
    for index in 0..<3 {
        settings.setPart(RagdollPart(shape: box, position: [0, 1 + Float(index) * 0.5, 0]), at: index)
    }
    settings.setPartToParent(1, point: [0, 1.25, 0], twistAxis: [0, 1, 0], limits: limits)
    settings.setPartToParent(2, point: [0, 1.75, 0], twistAxis: [0, 1, 0], limits: limits)
    return settings
}

extension JoltTests {
    @Suite
    struct RagdollSettingsTests {
        @Test func partsMatchSkeleton() throws {
            let settings = makeRagdollSettings(engine: try JoltEngine())
            #expect(settings.partCount == 3)
            #expect(settings.skeleton.jointCount == 3)
        }

        @Test func stabilizeAndCollisionFiltering() throws {
            let settings = makeRagdollSettings(engine: try JoltEngine())
            settings.setPart(RagdollPart(shape: .sphere(radius: 0.2), position: [0, 2, 0], mass: 50), at: 2)
            #expect(settings.stabilize())
            settings.disableParentChildCollisions()
        }

        @Test func constraintIndices() throws {
            let settings = makeRagdollSettings(engine: try JoltEngine())
            settings.calculateBodyIndexToConstraintIndex()
            settings.calculateConstraintIndexToBodyIndexPair()
            #expect(settings.constraintIndex(forBodyIndex: 0) == nil)
            #expect(settings.constraintIndex(forBodyIndex: 1) == 0)
            #expect(settings.constraintIndex(forBodyIndex: 2) == 1)
        }

        @Test func disableCollisionsWithPose() throws {
            let settings = makeRagdollSettings(engine: try JoltEngine())
            let pose = SkeletonPose(skeleton: settings.skeleton)
            for index in 0..<3 {
                pose.setJointState(.init(translation: [0, index == 0 ? 1 : 0.5, 0]), at: index)
            }
            pose.calculateJointMatrices()
            settings.disableParentChildCollisions(jointMatrices: pose.jointMatrices, minSeparationDistance: 0.1)
        }
    }
}
