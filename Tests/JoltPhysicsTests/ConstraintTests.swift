import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct ConstraintTests {
        @Test func hingePendulumKeepsDistanceToPivot() throws {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            let hinge = world.addHingeConstraint(between: nil, and: bob, point: .zero, axis: [0, 0, 1])
            #expect(world.constraintCount == 1)
            simulate(world, seconds: 0.5)
            let position = world.position(of: bob)
            #expect(abs(simd_length(position) - 1) < 0.01)
            #expect(position.y < -0.5)
            #expect(abs(position.z) < 0.01)
            #expect(abs(hinge.currentAngle) > 0.5)
        }

        @Test func hingeLimitsStopRotation() throws {
            let world = try makeWorld()
            let bob = world.addBody(.box(halfExtents: [0.5, 0.1, 0.1]), position: [0.5, 0, 0])
            let hinge = world.addHingeConstraint(between: nil, and: bob, point: .zero, axis: [0, 0, 1], limits: -0.2...0.2)
            #expect(hinge.limits == -0.2...0.2)
            var maxAngle: Float = 0
            for _ in 0..<120 {
                world.update(deltaTime: 1.0 / 60.0)
                maxAngle = max(maxAngle, abs(hinge.currentAngle))
            }
            // Jolt's limits are soft enough to overshoot a little on impact; unlimited it swings past 1.5.
            #expect(maxAngle < 0.3, "max \(maxAngle)")
            #expect(abs(abs(hinge.currentAngle) - 0.2) < 0.02, "final \(hinge.currentAngle)")
            hinge.limits = nil
            #expect(hinge.limits == nil)
        }

        @Test func hingeVelocityMotorSpinsBody() throws {
            let world = try makeWorld(gravity: .zero)
            let wheel = world.addBody(.cylinder(halfHeight: 0.1, radius: 1))
            let hinge = world.addHingeConstraint(between: nil, and: wheel, point: .zero, axis: [0, 1, 0])
            hinge.motorState = .velocity
            hinge.targetAngularVelocity = 2
            #expect(hinge.motorState == .velocity)
            simulate(world, seconds: 0.25)
            #expect(abs(world.angularVelocity(of: wheel).y - 2) < 0.05)
        }

        @Test func hingePositionMotorReachesTargetAngle() throws {
            let world = try makeWorld(gravity: .zero)
            let door = world.addBody(.box(halfExtents: [0.5, 1, 0.05]), position: [0.5, 0, 0])
            let hinge = world.addHingeConstraint(between: nil, and: door, point: .zero, axis: [0, 1, 0])
            hinge.motorState = .position
            hinge.targetAngle = 1
            simulate(world, seconds: 3)
            #expect(abs(hinge.currentAngle - 1) < 0.05)
        }

        @Test func hingeBetweenTwoDynamicBodies() throws {
            let world = try makeWorld(gravity: .zero)
            let a = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0, 0])
            let b = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [1, 0, 0])
            world.addHingeConstraint(between: a, and: b, point: [0.5, 0, 0], axis: [0, 0, 1])
            world.setLinearVelocity(of: a, to: [0, 0, 3])
            simulate(world, seconds: 1)
            // b is dragged along; the pair keeps its spacing.
            #expect(world.position(of: b).z > 0.5)
            #expect(abs(simd_distance(world.position(of: a), world.position(of: b)) - 1) < 0.02)
        }

        @Test func removedAndDisabledConstraintsStopActing() throws {
            let world = try makeWorld()
            let first = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            let second = world.addBody(.sphere(radius: 0.1), position: [-1, 0, 0])
            let removed = world.addHingeConstraint(between: nil, and: first, point: .zero, axis: [0, 0, 1])
            let disabled = world.addHingeConstraint(between: nil, and: second, point: .zero, axis: [0, 0, 1])
            world.removeConstraint(removed)
            disabled.isEnabled = false
            #expect(world.constraintCount == 1)
            #expect(!disabled.isEnabled)
            simulate(world, seconds: 1)
            // Free fall drops ~4.9m; a pendulum can't drop below -1.
            #expect(world.position(of: first).y < -3)
            #expect(world.position(of: second).y < -3)
        }

        @Test func pointConstraintSwingsInAnyDirection() throws {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            let constraint = world.addPointConstraint(between: nil, and: bob, point: .zero)
            // Push sideways so it leaves the XY plane, which a hinge would forbid.
            world.setLinearVelocity(of: bob, to: [0, 0, 2])
            simulate(world, seconds: 0.5)
            let position = world.position(of: bob)
            #expect(abs(simd_length(position) - 1) < 0.01)
            #expect(abs(position.z) > 0.2)
            #expect(simd_distance(constraint.localPoint1, .zero) < 1e-5)
            #expect(simd_distance(constraint.localPoint2, [-1, 0, 0]) < 1e-5)
        }

        @Test func pointConstraintLocalPointCanBeMoved() throws {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            let constraint = world.addPointConstraint(between: nil, and: bob, point: .zero)
            // Hang from [0, 2, 0] instead.
            constraint.localPoint1 = [0, 2, 0]
            #expect(simd_distance(constraint.localPoint1, [0, 2, 0]) < 1e-5)
            simulate(world, seconds: 3)
            #expect(abs(simd_distance(world.position(of: bob), [0, 2, 0]) - 1) < 0.02)
        }

        @Test func fixedConstraintToWorldHoldsBodyInPlace() throws {
            let world = try makeWorld()
            let rotation = simd_quatf(angle: 0.3, axis: [0, 0, 1])
            let body = world.addBody(.box(halfExtents: [1, 0.2, 0.2]), position: [2, 3, 0], rotation: rotation)
            world.addFixedConstraint(between: nil, and: body)
            simulate(world, seconds: 1)
            #expect(simd_distance(world.position(of: body), [2, 3, 0]) < 0.01)
            #expect(simd_distance(world.rotation(of: body).vector, rotation.vector) < 0.01)
        }

        @Test func fixedConstraintMovesBodiesTogether() throws {
            let world = try makeWorld(gravity: .zero)
            let a = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [0, 0, 0])
            let b = world.addBody(.box(halfExtents: [0.5, 0.5, 0.5]), position: [1, 0, 0])
            world.addFixedConstraint(between: a, and: b)
            // Kicking only a drags b along; the weld keeps their relative pose.
            world.setLinearVelocity(of: a, to: [0, 0, 3])
            simulate(world, seconds: 1)
            let offset = world.position(of: b) - world.position(of: a)
            let expectedOffset = world.rotation(of: a).act([1, 0, 0])
            #expect(simd_distance(offset, expectedOffset) < 0.02)
            #expect(simd_distance(world.rotation(of: a).vector, world.rotation(of: b).vector) < 0.01)
            #expect(world.position(of: b).z > 0.5)
        }

        @Test func distanceConstraintActsAsRod() throws {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.1), position: [2, 0, 0])
            let rod = world.addDistanceConstraint(between: nil, and: bob, point1: .zero, point2: [2, 0, 0])
            #expect(abs(rod.distance.lowerBound - 2) < 1e-5)
            #expect(abs(rod.distance.upperBound - 2) < 1e-5)
            simulate(world, seconds: 0.5)
            #expect(abs(simd_length(world.position(of: bob)) - 2) < 0.02)
            #expect(world.position(of: bob).y < -1)
        }

        @Test func distanceConstraintActsAsRope() throws {
            let world = try makeWorld()
            // Starts 1m below the anchor with a 3m rope: falls freely until the rope goes taut.
            let bob = world.addBody(.sphere(radius: 0.1), position: [0, -1, 0])
            world.addDistanceConstraint(between: nil, and: bob, point1: .zero, point2: [0, -1, 0], distance: 0...3)
            world.update(deltaTime: 1.0 / 60.0)
            #expect(world.position(of: bob).y < -1)
            simulate(world, seconds: 3)
            #expect(abs(world.position(of: bob).y + 3) < 0.05)
        }

        @Test func distanceRangeCanChange() throws {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.1), position: [0, -1, 0])
            let rope = world.addDistanceConstraint(between: nil, and: bob, point1: .zero, point2: [0, -1, 0])
            rope.distance = 0...2
            #expect(rope.distance == 0...2)
            simulate(world, seconds: 2)
            #expect(abs(world.position(of: bob).y + 2) < 0.05)
        }

        @Test func sliderOnlyMovesAlongAxis() throws {
            let world = try makeWorld()
            // A 45 degree ramp: gravity pulls the body down and along +x.
            let axis = simd_normalize(SIMD3<Float>(1, -1, 0))
            let body = world.addBody(.box(halfExtents: [0.2, 0.2, 0.2]), position: [0, 5, 0])
            let slider = world.addSliderConstraint(between: nil, and: body, axis: axis)
            #expect(slider.limits == nil)
            world.setLinearVelocity(of: body, to: [0, 0, 3])
            simulate(world, seconds: 1)
            let offset = world.position(of: body) - [0, 5, 0]
            #expect(simd_length(offset) > 1)
            #expect(simd_length(simd_cross(offset, axis)) < 0.02)
            #expect(abs(slider.currentPosition - simd_dot(offset, axis)) < 0.02)
            #expect(world.rotation(of: body).vector == [0, 0, 0, 1])
        }

        @Test func sliderLimitsStopMotion() throws {
            let world = try makeWorld()
            let body = world.addBody(.box(halfExtents: [0.2, 0.2, 0.2]), position: [0, 5, 0])
            let slider = world.addSliderConstraint(between: nil, and: body, axis: [0, 1, 0], limits: -1...1)
            #expect(slider.limits == -1...1)
            simulate(world, seconds: 2)
            #expect(abs(slider.currentPosition + 1) < 0.02)
            slider.limits = nil
            #expect(slider.limits == nil)
        }

        @Test func sliderMotors() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.box(halfExtents: [0.2, 0.2, 0.2]))
            let slider = world.addSliderConstraint(between: nil, and: body, axis: [1, 0, 0])
            slider.motorState = .velocity
            slider.targetVelocity = 2
            simulate(world, seconds: 0.5)
            #expect(abs(world.linearVelocity(of: body).x - 2) < 0.05)
            slider.motorState = .position
            slider.targetPosition = -1
            simulate(world, seconds: 4)
            #expect(abs(slider.currentPosition + 1) < 0.05)
        }

        /// Angle between the pendulum arm (pivot to bob) and straight down.
        func angleFromDown(_ position: SIMD3<Float>) -> Float {
            acos(simd_dot(simd_normalize(position), [0, -1, 0]))
        }

        @Test func coneConstraintLimitsSwing() throws {
            let world = try makeWorld()
            // Bar hanging from the origin; twist axis points down along the bar.
            let bob = world.addBody(.box(halfExtents: [0.1, 0.5, 0.1]), position: [0, -0.5, 0])
            let cone = world.addConeConstraint(between: nil, and: bob, point: .zero, twistAxis: [0, -1, 0], halfConeAngle: 0.3)
            #expect(abs(cone.halfConeAngle - 0.3) < 1e-4)
            // Kick it sideways; unconstrained it would swing to roughly 60 degrees. Much harder kicks
            // overshoot the limit briefly, as Jolt's limits are solved iteratively.
            world.setLinearVelocity(of: bob, to: [2, 0, 1])
            var maxAngle: Float = 0
            for _ in 0..<120 {
                world.update(deltaTime: 1.0 / 60.0)
                maxAngle = max(maxAngle, angleFromDown(world.position(of: bob)))
            }
            #expect(maxAngle > 0.2)
            #expect(maxAngle < 0.35, "max \(maxAngle)")
            #expect(abs(simd_length(world.position(of: bob)) - 0.5) < 0.02)
        }

        @Test func coneAngleCanChange() throws {
            let world = try makeWorld()
            let bob = world.addBody(.box(halfExtents: [0.1, 0.5, 0.1]), position: [0, -0.5, 0])
            let cone = world.addConeConstraint(between: nil, and: bob, point: .zero, twistAxis: [0, -1, 0], halfConeAngle: 0.1)
            cone.halfConeAngle = 1
            world.setLinearVelocity(of: bob, to: [5, 0, 0])
            var maxAngle: Float = 0
            for _ in 0..<60 {
                world.update(deltaTime: 1.0 / 60.0)
                maxAngle = max(maxAngle, angleFromDown(world.position(of: bob)))
            }
            #expect(maxAngle > 0.8)
        }

        /// Rotation of `body` around the y axis, from its x axis.
        func twistAngle(of rotation: simd_quatf) -> Float {
            let x = rotation.act([1, 0, 0])
            return atan2(-x.z, x.x)
        }

        @Test func swingTwistLimitsSwingAndTwistSeparately() throws {
            let world = try makeWorld(gravity: .zero)
            let limits = SwingTwistLimits(normalHalfConeAngle: 0.3, planeHalfConeAngle: 0.3, twist: -0.2...0.2)
            // Bar hanging from the origin; twist axis points down along the bar.
            let swinging = world.addBody(.box(halfExtents: [0.1, 0.5, 0.1]), position: [0, -0.5, 0])
            let constraint = world.addSwingTwistConstraint(between: nil, and: swinging, point: .zero, twistAxis: [0, -1, 0], limits: limits)
            #expect(abs(constraint.normalHalfConeAngle - 0.3) < 1e-4)
            let twisting = world.addBody(.box(halfExtents: [0.1, 0.5, 0.1]), position: [5, -0.5, 0])
            world.addSwingTwistConstraint(between: nil, and: twisting, point: [5, 0, 0], twistAxis: [0, -1, 0], limits: limits)

            world.setLinearVelocity(of: swinging, to: [1, 0, 0])
            world.setAngularVelocity(of: twisting, to: [0, 2, 0])
            var maxSwing: Float = 0
            var maxTwist: Float = 0
            for _ in 0..<120 {
                world.update(deltaTime: 1.0 / 60.0)
                maxSwing = max(maxSwing, angleFromDown(world.position(of: swinging)))
                maxTwist = max(maxTwist, abs(twistAngle(of: world.rotation(of: twisting))))
            }
            #expect(maxSwing > 0.2 && maxSwing < 0.35, "swing \(maxSwing)")
            #expect(maxTwist > 0.1 && maxTwist < 0.25, "twist \(maxTwist)")
            // The twisting bar stays hanging straight.
            #expect(simd_distance(world.position(of: twisting), [5, -0.5, 0]) < 0.02)
        }

        @Test func swingTwistConeAnglesApplyToTheirAxes() throws {
            func maxSwing(pushing push: SIMD3<Float>) throws -> Float {
                let world = try makeWorld(gravity: .zero)
                let bar = world.addBody(.box(halfExtents: [0.1, 0.5, 0.1]), position: [0, -0.5, 0])
                let limits = SwingTwistLimits(normalHalfConeAngle: 0.1, planeHalfConeAngle: 0.6, twist: -0.1...0.1)
                world.addSwingTwistConstraint(between: nil, and: bar, point: .zero, twistAxis: [0, -1, 0], planeAxis: [0, 0, 1], limits: limits)
                world.setLinearVelocity(of: bar, to: push)
                var result: Float = 0
                for _ in 0..<120 {
                    world.update(deltaTime: 1.0 / 60.0)
                    result = max(result, angleFromDown(world.position(of: bar)))
                }
                return result
            }
            // planeAxis is +z; cross(twistAxis, planeAxis) is along x.
            #expect(abs(try maxSwing(pushing: [0, 0, 1]) - 0.6) < 0.05)
            #expect(abs(try maxSwing(pushing: [1, 0, 0]) - 0.1) < 0.05)
        }

        @Test func sixDOFReportsLimits() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 0.5))
            let motor = MotorSettings(spring: SpringSettings(.frequency(3), damping: 0.7), forceLimits: -5...5, torqueLimits: -6...6)
            let constraint = world.addSixDOFConstraint(between: nil, and: body, point: .zero, limits: [
                .translationX: .limited(-1...2),
                .translationY: .fixed,
                .rotationZ: .limited(-0.5...0.5)
            ], motorSettings: [.rotationY: motor])
            #expect(constraint.motorSettings(for: .rotationY) == motor)
            #expect(constraint.motorSettings(for: .rotationX) != motor)
            #expect(constraint.limit(for: .translationX) == .limited(-1...2))
            #expect(constraint.limit(for: .translationY) == .fixed)
            #expect(constraint.limit(for: .translationZ) == .free)
            #expect(constraint.limit(for: .rotationX) == .free)
            #expect(constraint.limit(for: .rotationZ) == .limited(-0.5...0.5))
            constraint.setMaxFriction(3, for: .translationZ)
            #expect(constraint.maxFriction(for: .translationZ) == 3)
            constraint.setLimitsSpring(SpringSettings(.frequency(4), damping: 0.5), for: .translationX)
            #expect(constraint.limitsSpring(for: .translationX) == SpringSettings(.frequency(4), damping: 0.5))
        }

        @Test func sixDOFActsAsSliderWithLimits() throws {
            let world = try makeWorld()
            // Only X translation allowed, between -1 and 1. Gravity pulls along -y, a push along x.
            let body = world.addBody(.box(halfExtents: [0.2, 0.2, 0.2]))
            world.addSixDOFConstraint(between: nil, and: body, point: .zero, limits: [
                .translationX: .limited(-1...1),
                .translationY: .fixed, .translationZ: .fixed,
                .rotationX: .fixed, .rotationY: .fixed, .rotationZ: .fixed
            ])
            world.setLinearVelocity(of: body, to: [3, 0, 2])
            simulate(world, seconds: 1)
            let position = world.position(of: body)
            #expect(abs(position.x - 1) < 0.02)
            #expect(abs(position.y) < 0.02)
            #expect(abs(position.z) < 0.02)
        }

        @Test func sixDOFFreeRotationAxisActsAsHinge() throws {
            let world = try makeWorld()
            let bob = world.addBody(.box(halfExtents: [0.5, 0.1, 0.1]), position: [0.5, 0, 0])
            world.addSixDOFConstraint(between: nil, and: bob, point: .zero, limits: [
                .translationX: .fixed, .translationY: .fixed, .translationZ: .fixed,
                .rotationX: .fixed, .rotationY: .fixed, .rotationZ: .free
            ])
            simulate(world, seconds: 0.5)
            let position = world.position(of: bob)
            #expect(position.y < -0.2)
            #expect(abs(simd_length(position) - 0.5) < 0.02)
            #expect(abs(position.z) < 0.01)
        }

        @Test func sixDOFVelocityMotor() throws {
            let world = try makeWorld(gravity: .zero)
            let body = world.addBody(.box(halfExtents: [0.2, 0.2, 0.2]))
            let constraint = world.addSixDOFConstraint(between: nil, and: body, point: .zero, limits: [
                .translationY: .fixed, .translationZ: .fixed,
                .rotationX: .fixed, .rotationY: .fixed, .rotationZ: .fixed
            ])
            constraint.setMotorState(.velocity, for: .translationX)
            #expect(constraint.motorState(for: .translationX) == .velocity)
            constraint.targetVelocity = [1.5, 0, 0]
            simulate(world, seconds: 0.5)
            #expect(abs(world.linearVelocity(of: body).x - 1.5) < 0.05)
        }

        @Test(arguments: [false, true])
        func gearCouplesWheels(passHinges: Bool) throws {
            let world = try makeWorld(gravity: .zero)
            let small = world.addBody(.cylinder(halfHeight: 0.1, radius: 0.5), position: [0, 0, 0])
            let large = world.addBody(.cylinder(halfHeight: 0.1, radius: 1), position: [1.5, 0, 0])
            let driver = world.addHingeConstraint(between: nil, and: small, point: [0, 0, 0], axis: [0, 1, 0])
            let driven = world.addHingeConstraint(between: nil, and: large, point: [1.5, 0, 0], axis: [0, 1, 0])
            // Large gear has twice the teeth.
            world.addGearConstraint(between: small, and: large, axis1: [0, 1, 0], axis2: [0, 1, 0], ratio: 2, hinges: passHinges ? (driver, driven) : nil)
            driver.motorState = .velocity
            driver.targetAngularVelocity = 2
            simulate(world, seconds: 1)
            let smallSpeed = world.angularVelocity(of: small).y
            let largeSpeed = world.angularVelocity(of: large).y
            #expect(abs(smallSpeed - 2) < 0.1, "small \(smallSpeed)")
            #expect(abs(largeSpeed + 1) < 0.1, "large \(largeSpeed)")
        }

        @Test func springAndMotorSettingsRoundTrip() throws {
            let world = try makeWorld()
            let a = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
            let b = world.addBody(.sphere(radius: 0.1), position: [2, 0, 0])
            let c = world.addBody(.sphere(radius: 0.1), position: [3, 0, 0])
            let hinge = world.addHingeConstraint(between: nil, and: a, point: .zero, axis: [0, 0, 1])
            let slider = world.addSliderConstraint(between: nil, and: b, axis: [1, 0, 0])
            let rope = world.addDistanceConstraint(between: nil, and: c, point1: .zero, point2: [3, 0, 0])

            let spring = SpringSettings(.stiffness(500), damping: 3)
            let motor = MotorSettings(spring: SpringSettings(.frequency(5), damping: 0.5), forceLimits: -10...10, torqueLimits: -20...20)
            hinge.limitsSpring = spring
            hinge.motorSettings = motor
            slider.limitsSpring = spring
            slider.motorSettings = motor
            rope.limitsSpring = SpringSettings(.frequency(2), damping: 0.1)
            #expect(hinge.limitsSpring == spring)
            #expect(hinge.motorSettings == motor)
            #expect(slider.limitsSpring == spring)
            #expect(slider.motorSettings == motor)
            #expect(rope.limitsSpring == SpringSettings(.frequency(2), damping: 0.1))
        }

        func maxDrop(limitsSpring: SpringSettings?) throws -> Float {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.1), position: [0, -1, 0])
            let rope = world.addDistanceConstraint(between: nil, and: bob, point1: .zero, point2: [0, -1, 0], distance: 0...1)
            if let limitsSpring {
                rope.limitsSpring = limitsSpring
            }
            world.setLinearVelocity(of: bob, to: [0, -5, 0])
            var lowest: Float = 0
            for _ in 0..<60 {
                world.update(deltaTime: 1.0 / 60.0)
                lowest = min(lowest, world.position(of: bob).y)
            }
            return -lowest
        }

        @Test func springyDistanceLimitStretches() throws {
            let rigid = try maxDrop(limitsSpring: nil)
            let bungee = try maxDrop(limitsSpring: SpringSettings(.frequency(1), damping: 0.1))
            #expect(rigid < 1.05)
            #expect(bungee > 1.3)
        }

        @Test func motorTorqueLimitCapsAcceleration() throws {
            func spinUp(torqueLimit: Float) throws -> Float {
                let world = try makeWorld(gravity: .zero)
                let wheel = world.addBody(.cylinder(halfHeight: 0.1, radius: 1))
                let hinge = world.addHingeConstraint(between: nil, and: wheel, point: .zero, axis: [0, 1, 0])
                hinge.motorSettings = MotorSettings(torqueLimits: -torqueLimit...torqueLimit)
                hinge.motorState = .velocity
                hinge.targetAngularVelocity = 10
                simulate(world, seconds: 0.25)
                return world.angularVelocity(of: wheel).y
            }
            #expect(try spinUp(torqueLimit: .greatestFiniteMagnitude) > 9.5)
            #expect(try spinUp(torqueLimit: 10) < 1)
        }

        @Test func constraintCanOutliveWorld() throws {
            var hinge: HingeConstraint?
            do {
                let world = try makeWorld()
                let bob = world.addBody(.sphere(radius: 0.1), position: [1, 0, 0])
                hinge = world.addHingeConstraint(between: nil, and: bob, point: .zero, axis: [0, 0, 1])
                world.update(deltaTime: 1.0 / 60.0)
            }
            #expect(hinge != nil)
            hinge = nil
        }
    }
}
