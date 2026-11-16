import JoltPhysics
import Testing

func makeSkeleton(engine: JoltEngine? = nil) throws -> Skeleton {
    let skeleton = Skeleton(engine: try engine ?? JoltEngine())
    let pelvis = skeleton.addJoint("pelvis")
    let spine = skeleton.addJoint("spine", parent: pelvis)
    skeleton.addJoint("head", parent: spine)
    skeleton.addJoint("leftLeg", parentName: "pelvis")
    return skeleton
}

extension JoltTests {
    @Suite
    struct SkeletonTests {
        @Test func jointsAndParents() throws {
            let skeleton = try makeSkeleton()
            #expect(skeleton.jointCount == 4)
            #expect(skeleton.joint(at: 0) == .init(name: "pelvis", parentName: nil, parentIndex: nil))
            #expect(skeleton.joint(at: 2).parentName == "spine")
            #expect(skeleton.joint(at: 2).parentIndex == 1)
            #expect(skeleton.joint(at: 3).parentIndex == 0)
            #expect(skeleton.joints.map(\.name) == ["pelvis", "spine", "head", "leftLeg"])
        }

        @Test func lookupByName() throws {
            let skeleton = try makeSkeleton()
            #expect(skeleton.jointIndex(named: "head") == 2)
            #expect(skeleton.jointIndex(named: "tail") == nil)
        }

        @Test func ordering() throws {
            let skeleton = try makeSkeleton()
            skeleton.calculateParentJointIndices()
            #expect(skeleton.areJointsCorrectlyOrdered)
        }
    }
}
