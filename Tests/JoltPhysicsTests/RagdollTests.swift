import JoltPhysics
import simd
import Testing

/// Three stacked boxes: pelvis at y=1, spine above it, head on top. `lean` shifts each level in x.
func makeRagdollSettings(engine: JoltEngine, base: SIMD3<Float> = [0, 1, 0], lean: Float = 0) -> RagdollSettings {
    let skeleton = Skeleton(engine: engine)
    let pelvis = skeleton.addJoint("pelvis")
    let spine = skeleton.addJoint("spine", parent: pelvis)
    skeleton.addJoint("head", parent: spine)

    let settings = RagdollSettings(skeleton: skeleton)
    let box = Shape.box(halfExtents: [0.2, 0.2, 0.2])
    let limits = SwingTwistLimits(normalHalfConeAngle: .pi / 4, planeHalfConeAngle: .pi / 4, twist: -0.3...0.3)
    func level(_ step: Float) -> SIMD3<Float> { base + [lean * step, 0.5 * step, 0] }
    for index in 0..<3 {
        settings.setPart(RagdollPart(shape: box, position: level(Float(index))), at: index)
    }
    settings.setPartToParent(1, point: level(0.5), twistAxis: [0, 1, 0], limits: limits)
    settings.setPartToParent(2, point: level(1.5), twistAxis: [0, 1, 0], limits: limits)
    settings.stabilize()
    settings.disableParentChildCollisions()
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

        @Test func ragdollFalls() throws {
            let world = try makeWorld()
            let ragdoll = world.createRagdoll(makeRagdollSettings(engine: world.engine, base: [0, 10, 0]))
            #expect(ragdoll.bodyCount == 3)
            #expect(ragdoll.constraintCount == 2)
            #expect(ragdoll.bodyIDs.allSatisfy { !world.isAdded($0) })
            ragdoll.addToWorld()
            #expect(ragdoll.bodyIDs.allSatisfy { world.isAdded($0) })
            #expect(ragdoll.isActive)
            simulate(world, seconds: 1)
            #expect(ragdoll.rootTransform.position.y < 7)
        }

        @Test func ragdollCollapsesOnFloor() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [20, 0.5, 20]), position: [0, -0.5, 0], motionType: .static)
            let ragdoll = world.createRagdoll(makeRagdollSettings(engine: world.engine, base: [0, 0.3, 0], lean: 0.15))
            ragdoll.addToWorld()
            let head = ragdoll.bodyIDs[2]
            let startY = world.position(of: head).y
            simulate(world, seconds: 3)
            #expect(world.position(of: head).y < startY - 0.3)
        }

        @Test func poseRoundTrip() throws {
            let world = try makeWorld(gravity: .zero)
            let ragdoll = world.createRagdoll(makeRagdollSettings(engine: world.engine))
            ragdoll.addToWorld()
            let pose = ragdoll.currentPose()
            // Joint matrices are relative to the root offset (the pelvis position).
            #expect(abs(pose.rootOffset.y - 1) < 1e-3)
            #expect(abs(pose.jointMatrix(at: 2).columns.3.y - 1) < 1e-3)

            pose.rootOffset += [5, 0, 0]
            ragdoll.setPose(pose)
            #expect(abs(world.position(of: ragdoll.bodyIDs[0]).x - 5) < 1e-3)

            var matrices = pose.jointMatrices
            for index in matrices.indices {
                matrices[index].columns.3.y += 1
            }
            ragdoll.setPose(rootOffset: pose.rootOffset, jointMatrices: matrices)
            #expect(abs(world.position(of: ragdoll.bodyIDs[0]).y - 2) < 1e-3)
        }

        @Test func driveToPose() throws {
            let world = try makeWorld()
            let ragdoll = world.createRagdoll(makeRagdollSettings(engine: world.engine))
            ragdoll.addToWorld()
            let target = ragdoll.currentPose()
            ragdoll.driveToPoseUsingMotors(target)
            ragdoll.driveToPoseUsingKinematics(target, deltaTime: 1.0 / 60.0)
            ragdoll.resetWarmStart()
            world.update(deltaTime: 1.0 / 60.0)
        }

        @Test func removeAndRelease() throws {
            let world = try makeWorld()
            do {
                let ragdoll = world.createRagdoll(makeRagdollSettings(engine: world.engine))
                ragdoll.addToWorld()
                #expect(world.bodyCount == 3)
                ragdoll.removeFromWorld()
                #expect(ragdoll.bodyIDs.allSatisfy { !world.isAdded($0) })
                ragdoll.addToWorld()
                #expect(ragdoll.bodyIDs.allSatisfy { world.isAdded($0) })
            }
            #expect(world.bodyCount == 0)
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
