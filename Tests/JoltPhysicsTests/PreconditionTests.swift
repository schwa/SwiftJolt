import JoltPhysics
import simd
import Testing

/// Checks that a child process stopped with `message` on stderr, so it failed for the expected reason.
private func expectMessage(_ result: ExitTest.Result?, _ message: String, sourceLocation: SourceLocation = #_sourceLocation) {
    let output = String(bytes: result?.standardErrorContent ?? [], encoding: .utf8) ?? ""
    #expect(output.contains(message), "stderr: \(output)", sourceLocation: sourceLocation)
}

// Each exit test runs its body in a child process and expects it to stop on a precondition.
// Exit test bodies can't capture state, so each one builds what it needs.
extension JoltTests {
    @Suite
    struct PreconditionTests {
        // MARK: - Shapes

        @Test func invalidTaperedCapsuleStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                world.addBody(.taperedCapsule(halfHeight: 1, topRadius: -1, bottomRadius: 0.5))
            }
            expectMessage(result, "Invalid tapered capsule")
        }

        @Test func degenerateConvexHullStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                world.addBody(.convexHull(points: [[0, 0, 0], [0, 0, 0]]))
            }
            expectMessage(result, "Could not build a convex hull")
        }

        @Test func meshWithBadIndexCountStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                world.addBody(.mesh(vertices: [[0, 0, 0], [1, 0, 0], [0, 0, 1]], indices: [0, 1]), motionType: .static)
            }
            expectMessage(result, "Mesh indices must come in groups of three")
        }

        @Test func emptyCompoundStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                world.addBody(.compound([]))
            }
            expectMessage(result, "A compound shape needs at least one child")
        }

        @Test func removingMissingCompoundChildStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.mutableCompound([CompoundChild(.sphere(radius: 1))]))
                world.removeCompoundChild(at: 5, from: body)
            }
            expectMessage(result, "Compound child index out of range")
        }

        // MARK: - Bodies and materials

        @Test func staticBodyHasNoMotionProperties() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.sphere(radius: 1), motionType: .static)
                _ = world.linearDamping(of: body)
            }
            expectMessage(result, "has no motion properties")
        }

        @Test func nonPositiveDensityStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                _ = Material(density: 0)
            }
            expectMessage(result, "Density must be positive")
        }

        @Test func addingBodyTwiceStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.sphere(radius: 1))
                world.addToWorld(body)
            }
            expectMessage(result, "Body is already in the world")
        }

        // MARK: - Constraints

        @Test func negativeStepOverrideStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.sphere(radius: 1))
                let constraint = world.addFixedConstraint(between: nil, and: body)
                constraint.velocityStepsOverride = -1
            }
            expectMessage(result, "Step override must not be negative")
        }

        @Test func constraintOnMissingBodyStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.sphere(radius: 1))
                world.removeBody(body)
                world.addFixedConstraint(between: nil, and: body)
            }
            expectMessage(result, "does not exist")
        }

        @Test func nonPerpendicularSixDOFAxesStop() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.sphere(radius: 1))
                world.addSixDOFConstraint(between: nil, and: body, point: .zero, axisX: [1, 0, 0], axisY: [1, 1, 0], limits: [:])
            }
            expectMessage(result, "axisX and axisY must be perpendicular")
        }

        // MARK: - Skeletons and ragdolls

        @Test func unknownParentJointStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let skeleton = Skeleton(engine: try JoltEngine())
                skeleton.addJoint("arm", parentName: "missing")
            }
            expectMessage(result, "Unknown parent joint missing")
        }

        @Test func poseJointIndexOutOfRangeStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let pose = SkeletonPose(skeleton: try makeSkeleton())
                _ = pose.jointState(at: 10)
            }
            expectMessage(result, "Joint index out of range")
        }

        @Test func skeletonMapperNeedsMatchingPose() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let skeleton = try makeSkeleton()
                let identity = [simd_float4x4](repeating: matrix_identity_float4x4, count: skeleton.jointCount)
                _ = SkeletonMapper(skeleton1: skeleton, neutralPose1: identity, skeleton2: skeleton, neutralPose2: [])
            }
            expectMessage(result, "neutralPose2 needs one matrix per joint")
        }

        @Test func ragdollRootPartHasNoParent() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let settings = makeRagdollSettings(engine: try JoltEngine())
                let limits = SwingTwistLimits(normalHalfConeAngle: 0.5, planeHalfConeAngle: 0.5, twist: -0.1...0.1)
                settings.setPartToParent(0, point: .zero, twistAxis: [0, 1, 0], limits: limits)
            }
            expectMessage(result, "Root part has no parent")
        }

        @Test func ragdollFromOtherEngineStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                _ = world.createRagdoll(makeRagdollSettings(engine: try JoltEngine()))
            }
            expectMessage(result, "Ragdoll settings belong to a different engine")
        }

        // MARK: - Soft bodies and vehicles

        @Test func softBodySettingsLockedOnceInUse() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let cloth = try makeCloth(engine: world.engine)
                cloth.createConstraints()
                world.addSoftBody(cloth)
                cloth.addVertex([0, 0, 0])
            }
            expectMessage(result, "Soft body settings can't change")
        }

        @Test func trackDrivenWheelMustBeOnTrack() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                _ = Track(wheels: [0, 1], drivenWheel: 5)
            }
            expectMessage(result, "The driven wheel must be one of the track's wheels")
        }

        @Test func vehicleWheelIndexOutOfRangeStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                let body = world.addBody(.box(halfExtents: [1, 0.2, 2]))
                let car = world.addWheeledVehicle(
                    body: body,
                    wheels: [WheeledVehicleWheel(WheelSettings(position: [0, 0, 1]))],
                    drivenWheels: []
                )
                _ = car.wheelState(at: 3)
            }
            expectMessage(result, "Wheel index out of range")
        }

        // MARK: - Engine and collision filtering

        @Test func jobPoolNeedsThreads() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                _ = PhysicsWorld(engine: try JoltEngine(), jobSystem: .userInteractiveThreadPool(threadCount: 0))
            }
            expectMessage(result, "The job thread pool needs at least one thread")
        }

        @Test func objectLayerOutOfRangeStops() async {
            let result = await #expect(processExitsWith: .failure, observing: [\.standardErrorContent]) {
                let world = try makeWorld()
                world.addBody(.sphere(radius: 1), options: BodyOptions(objectLayer: ObjectLayer(rawValue: 99)))
            }
            expectMessage(result, "Object layer 99 is out of range")
        }
    }
}
