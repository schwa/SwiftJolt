import CJolt
import simd

// Total lambdas are the impulses the solver applied in the last step. Divide by the step's
// delta time for a force or torque, e.g. to break a joint under too much load.

private func vector(_ read: (UnsafeMutablePointer<JPH_Vec3>) -> Void) -> SIMD3<Float> {
    var result = JPH_Vec3()
    read(&result)
    return result.simd
}

private func pair(_ read: (UnsafeMutablePointer<Float>) -> Void) -> SIMD2<Float> {
    var values: [Float] = [0, 0]
    values.withUnsafeMutableBufferPointer { read($0.baseAddress!) }
    return SIMD2(values[0], values[1])
}

extension FixedConstraint {
    public var totalLambdaPosition: SIMD3<Float> { vector { JPH_FixedConstraint_GetTotalLambdaPosition(handle, $0) } }
    public var totalLambdaRotation: SIMD3<Float> { vector { JPH_FixedConstraint_GetTotalLambdaRotation(handle, $0) } }
}

extension DistanceConstraint {
    public var totalLambdaPosition: Float { JPH_DistanceConstraint_GetTotalLambdaPosition(handle) }
}

extension PointConstraint {
    public var totalLambdaPosition: SIMD3<Float> { vector { JPH_PointConstraint_GetTotalLambdaPosition(handle, $0) } }
}

extension HingeConstraint {
    public var totalLambdaPosition: SIMD3<Float> { vector { JPH_HingeConstraint_GetTotalLambdaPosition(handle, $0) } }
    /// Impulses keeping the two bodies' hinge axes aligned.
    public var totalLambdaRotation: SIMD2<Float> { pair { JPH_HingeConstraint_GetTotalLambdaRotation(handle, $0) } }
    public var totalLambdaRotationLimits: Float { JPH_HingeConstraint_GetTotalLambdaRotationLimits(handle) }
    public var totalLambdaMotor: Float { JPH_HingeConstraint_GetTotalLambdaMotor(handle) }

    public var hasLimits: Bool { JPH_HingeConstraint_HasLimits(handle) }

    /// Attachment point relative to body 1's center of mass.
    public var localSpacePoint1: SIMD3<Float> { vector { JPH_HingeConstraint_GetLocalSpacePoint1(handle, $0) } }
    public var localSpacePoint2: SIMD3<Float> { vector { JPH_HingeConstraint_GetLocalSpacePoint2(handle, $0) } }
    public var localSpaceHingeAxis1: SIMD3<Float> { vector { JPH_HingeConstraint_GetLocalSpaceHingeAxis1(handle, $0) } }
    public var localSpaceHingeAxis2: SIMD3<Float> { vector { JPH_HingeConstraint_GetLocalSpaceHingeAxis2(handle, $0) } }
    public var localSpaceNormalAxis1: SIMD3<Float> { vector { JPH_HingeConstraint_GetLocalSpaceNormalAxis1(handle, $0) } }
    public var localSpaceNormalAxis2: SIMD3<Float> { vector { JPH_HingeConstraint_GetLocalSpaceNormalAxis2(handle, $0) } }
}

extension SliderConstraint {
    /// Impulses keeping body 2 on the slider axis.
    public var totalLambdaPosition: SIMD2<Float> { pair { JPH_SliderConstraint_GetTotalLambdaPosition(handle, $0) } }
    public var totalLambdaPositionLimits: Float { JPH_SliderConstraint_GetTotalLambdaPositionLimits(handle) }
    public var totalLambdaRotation: SIMD3<Float> { vector { JPH_SliderConstraint_GetTotalLambdaRotation(handle, $0) } }
    public var totalLambdaMotor: Float { JPH_SliderConstraint_GetTotalLambdaMotor(handle) }
}

extension ConeConstraint {
    public var totalLambdaPosition: SIMD3<Float> { vector { JPH_ConeConstraint_GetTotalLambdaPosition(handle, $0) } }
    public var totalLambdaRotation: Float { JPH_ConeConstraint_GetTotalLambdaRotation(handle) }
}

extension SwingTwistConstraint {
    public var totalLambdaPosition: SIMD3<Float> { vector { JPH_SwingTwistConstraint_GetTotalLambdaPosition(handle, $0) } }
    public var totalLambdaTwist: Float { JPH_SwingTwistConstraint_GetTotalLambdaTwist(handle) }
    public var totalLambdaSwingY: Float { JPH_SwingTwistConstraint_GetTotalLambdaSwingY(handle) }
    public var totalLambdaSwingZ: Float { JPH_SwingTwistConstraint_GetTotalLambdaSwingZ(handle) }
    public var totalLambdaMotor: SIMD3<Float> { vector { JPH_SwingTwistConstraint_GetTotalLambdaMotor(handle, $0) } }
}

extension SixDOFConstraint {
    public var totalLambdaPosition: SIMD3<Float> { vector { JPH_SixDOFConstraint_GetTotalLambdaPosition(handle, $0) } }
    public var totalLambdaRotation: SIMD3<Float> { vector { JPH_SixDOFConstraint_GetTotalLambdaRotation(handle, $0) } }
    public var totalLambdaMotorTranslation: SIMD3<Float> { vector { JPH_SixDOFConstraint_GetTotalLambdaMotorTranslation(handle, $0) } }
    public var totalLambdaMotorRotation: SIMD3<Float> { vector { JPH_SixDOFConstraint_GetTotalLambdaMotorRotation(handle, $0) } }

    /// Per-axis translation limits in constraint space. Free axes report ±Float.greatestFiniteMagnitude.
    public var translationLimits: (min: SIMD3<Float>, max: SIMD3<Float>) {
        (vector { JPH_SixDOFConstraint_GetTranslationLimitsMin(handle, $0) }, vector { JPH_SixDOFConstraint_GetTranslationLimitsMax(handle, $0) })
    }

    /// Per-axis rotation limits in radians, in constraint space.
    public var rotationLimits: (min: SIMD3<Float>, max: SIMD3<Float>) {
        (vector { JPH_SixDOFConstraint_GetRotationLimitsMin(handle, $0) }, vector { JPH_SixDOFConstraint_GetRotationLimitsMax(handle, $0) })
    }

    /// Sets `targetOrientation` from a target rotation of body 2 relative to body 1 (body space).
    public func setTargetOrientationInBodySpace(_ orientation: simd_quatf) {
        var value = JPH_Quat(orientation)
        JPH_SixDOFConstraint_SetTargetOrientationBS(handle, &value)
    }
}

extension GearConstraint {
    public var totalLambda: Float { JPH_GearConstraint_GetTotalLambda(handle) }
}
