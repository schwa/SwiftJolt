import JoltPhysics
import simd
import Testing

private func translation(_ v: SIMD3<Float>) -> simd_float4x4 {
    var matrix = matrix_identity_float4x4
    matrix.columns.3 = SIMD4(v, 1)
    return matrix
}

private func origin(_ m: simd_float4x4) -> SIMD3<Float> {
    [m.columns.3.x, m.columns.3.y, m.columns.3.z]
}

private struct Rigs {
    let mapper: SkeletonMapper
    let neutral1: [simd_float4x4]
    let neutral2: [simd_float4x4]
}

/// Detailed: pelvis, spine, chest, head. Simple: pelvis, chest, head.
private func makeRigs() throws -> Rigs {
    let engine = try JoltEngine()
    let detailed = Skeleton(engine: engine)
    detailed.addJoint("pelvis")
    detailed.addJoint("spine", parentName: "pelvis")
    detailed.addJoint("chest", parentName: "spine")
    detailed.addJoint("head", parentName: "chest")
    let simple = Skeleton(engine: engine)
    simple.addJoint("pelvis")
    simple.addJoint("chest", parentName: "pelvis")
    simple.addJoint("head", parentName: "chest")
    let neutral1 = [0, 0.5, 1, 1.5].map { translation([0, 1 + Float($0), 0]) }
    let neutral2 = [0, 1, 1.5].map { translation([0, 1 + Float($0), 0]) }
    let mapper = SkeletonMapper(skeleton1: detailed, neutralPose1: neutral1, skeleton2: simple, neutralPose2: neutral2)
    return Rigs(mapper: mapper, neutral1: neutral1, neutral2: neutral2)
}

extension JoltTests {
    @Suite
    struct SkeletonMapperTests {
        @Test func jointsMapByName() throws {
            let mapper = try makeRigs().mapper
            #expect(mapper.mappedJointIndex(forJoint1: 0) == 0)
            #expect(mapper.mappedJointIndex(forJoint1: 1) == nil)
            #expect(mapper.mappedJointIndex(forJoint1: 2) == 1)
            #expect(mapper.mappedJointIndex(forJoint1: 3) == 2)
        }

        @Test func mapDetailedToSimple() throws {
            let rigs = try makeRigs()
            var pose1 = rigs.neutral1
            pose1[2] = translation([0.3, 2, 0])
            let local2 = [matrix_identity_float4x4, translation([0, 1, 0]), translation([0, 0.5, 0])]
            let pose2 = rigs.mapper.map(pose1, localPose2: local2)
            #expect(simd_distance(origin(pose2[1]), [0.3, 2, 0]) < 1e-4)
        }

        @Test func mapSimpleBackToDetailed() throws {
            let rigs = try makeRigs()
            var pose2 = rigs.neutral2
            pose2[2] = translation([0.2, 2.5, 0])
            let pose1 = rigs.mapper.mapReverse(pose2, onto: rigs.neutral1)
            #expect(simd_distance(origin(pose1[3]), [0.2, 2.5, 0]) < 1e-4)
        }

        @Test func lockTranslations() throws {
            let rigs = try makeRigs()
            #expect(!rigs.mapper.isJointTranslationLocked(joint2: 1))
            rigs.mapper.lockTranslations([false, true, false], neutralPose2: rigs.neutral2)
            #expect(rigs.mapper.isJointTranslationLocked(joint2: 1))
            rigs.mapper.lockAllTranslations(neutralPose2: rigs.neutral2)
            #expect(rigs.mapper.isJointTranslationLocked(joint2: 2))
        }
    }
}
