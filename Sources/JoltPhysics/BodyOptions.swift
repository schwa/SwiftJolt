import CJolt
import simd

/// Which axes a body can move and rotate along. Use `.plane2D` for 2D physics in the XY plane.
public struct AllowedDegreesOfFreedom: OptionSet, Sendable, Hashable {
    public let rawValue: UInt32

    public init(rawValue: UInt32) {
        self.rawValue = rawValue
    }

    public static let translationX = Self(rawValue: JPH_AllowedDOFs_TranslationX.rawValue)
    public static let translationY = Self(rawValue: JPH_AllowedDOFs_TranslationY.rawValue)
    public static let translationZ = Self(rawValue: JPH_AllowedDOFs_TranslationZ.rawValue)
    public static let rotationX = Self(rawValue: JPH_AllowedDOFs_RotationX.rawValue)
    public static let rotationY = Self(rawValue: JPH_AllowedDOFs_RotationY.rawValue)
    public static let rotationZ = Self(rawValue: JPH_AllowedDOFs_RotationZ.rawValue)
    public static let all: Self = [.translationX, .translationY, .translationZ, .rotationX, .rotationY, .rotationZ]
    public static let plane2D: Self = [.translationX, .translationY, .rotationZ]
}

/// Less common body creation settings. Defaults match Jolt's.
public struct BodyOptions: Sendable, Equatable {
    public var linearVelocity: SIMD3<Float> = .zero
    public var angularVelocity: SIMD3<Float> = .zero
    public var linearDamping: Float = 0.05
    public var angularDamping: Float = 0.05
    public var maxLinearVelocity: Float = 500
    public var maxAngularVelocity: Float = 0.25 * .pi * 60
    public var gravityFactor: Float = 1
    public var motionQuality: MotionQuality = .discrete
    public var allowSleeping = true
    public var applyGyroscopicForce = false
    /// Scales the inertia computed from the shape. Values above 1 make the body harder to spin.
    public var inertiaMultiplier: Float = 1
    /// Overrides the mass computed from the shape. Inertia is scaled to match.
    public var mass: Float?
    /// Lets a static body later change to kinematic or dynamic.
    public var allowDynamicOrKinematic = false
    /// Lets a kinematic body collide with static and other kinematic bodies.
    public var collideKinematicVsNonDynamic = false
    public var useManifoldReduction = true
    /// Reduces ghost collisions on internal edges of meshes and compounds, at some cost.
    public var enhancedInternalEdgeRemoval = false
    /// Solver iteration overrides. 0 uses the world's setting.
    public var velocityStepsOverride = 0
    public var positionStepsOverride = 0
    /// nil picks `.nonMoving` for static bodies and `.moving` for the rest.
    public var objectLayer: ObjectLayer?
    public var collisionGroup: CollisionGroup = .ungrouped
    /// A sensor reports contacts to the contact listener but has no collision response.
    /// Static sensors only detect moving bodies; kinematic sensors also need
    /// `collideKinematicVsNonDynamic` to detect static ones.
    public var isSensor = false
    /// Axes the body may move and rotate along. Ignored for static bodies.
    public var allowedDegreesOfFreedom: AllowedDegreesOfFreedom = .all
    /// Any value the app wants to attach to the body, such as an index into its own tables.
    public var userData: UInt64 = 0

    public init(
        linearVelocity: SIMD3<Float> = .zero,
        angularVelocity: SIMD3<Float> = .zero,
        linearDamping: Float = 0.05,
        angularDamping: Float = 0.05,
        maxLinearVelocity: Float = 500,
        maxAngularVelocity: Float = 0.25 * .pi * 60,
        gravityFactor: Float = 1,
        motionQuality: MotionQuality = .discrete,
        allowSleeping: Bool = true,
        applyGyroscopicForce: Bool = false,
        inertiaMultiplier: Float = 1,
        mass: Float? = nil,
        allowDynamicOrKinematic: Bool = false,
        collideKinematicVsNonDynamic: Bool = false,
        useManifoldReduction: Bool = true,
        enhancedInternalEdgeRemoval: Bool = false,
        velocityStepsOverride: Int = 0,
        positionStepsOverride: Int = 0,
        objectLayer: ObjectLayer? = nil,
        collisionGroup: CollisionGroup = .ungrouped,
        isSensor: Bool = false,
        allowedDegreesOfFreedom: AllowedDegreesOfFreedom = .all,
        userData: UInt64 = 0
    ) {
        precondition(!allowedDegreesOfFreedom.isEmpty, "A body needs at least one degree of freedom")
        if let mass {
            precondition(mass > 0, "Mass must be positive")
        }
        precondition(velocityStepsOverride >= 0 && positionStepsOverride >= 0, "Step overrides must not be negative")
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
        self.linearDamping = linearDamping
        self.angularDamping = angularDamping
        self.maxLinearVelocity = maxLinearVelocity
        self.maxAngularVelocity = maxAngularVelocity
        self.gravityFactor = gravityFactor
        self.motionQuality = motionQuality
        self.allowSleeping = allowSleeping
        self.applyGyroscopicForce = applyGyroscopicForce
        self.inertiaMultiplier = inertiaMultiplier
        self.mass = mass
        self.allowDynamicOrKinematic = allowDynamicOrKinematic
        self.collideKinematicVsNonDynamic = collideKinematicVsNonDynamic
        self.useManifoldReduction = useManifoldReduction
        self.enhancedInternalEdgeRemoval = enhancedInternalEdgeRemoval
        self.velocityStepsOverride = velocityStepsOverride
        self.positionStepsOverride = positionStepsOverride
        self.objectLayer = objectLayer
        self.collisionGroup = collisionGroup
        self.isSensor = isSensor
        self.allowedDegreesOfFreedom = allowedDegreesOfFreedom
        self.userData = userData
    }

    public static let `default` = Self()

    func apply(to settings: OpaquePointer) {
        var linear = JPH_Vec3(linearVelocity)
        var angular = JPH_Vec3(angularVelocity)
        JPH_BodyCreationSettings_SetLinearVelocity(settings, &linear)
        JPH_BodyCreationSettings_SetAngularVelocity(settings, &angular)
        JPH_BodyCreationSettings_SetLinearDamping(settings, linearDamping)
        JPH_BodyCreationSettings_SetAngularDamping(settings, angularDamping)
        JPH_BodyCreationSettings_SetMaxLinearVelocity(settings, maxLinearVelocity)
        JPH_BodyCreationSettings_SetMaxAngularVelocity(settings, maxAngularVelocity)
        JPH_BodyCreationSettings_SetGravityFactor(settings, gravityFactor)
        JPH_BodyCreationSettings_SetMotionQuality(settings, motionQuality.jph)
        JPH_BodyCreationSettings_SetAllowSleeping(settings, allowSleeping)
        JPH_BodyCreationSettings_SetApplyGyroscopicForce(settings, applyGyroscopicForce)
        JPH_BodyCreationSettings_SetInertiaMultiplier(settings, inertiaMultiplier)
        JPH_BodyCreationSettings_SetAllowDynamicOrKinematic(settings, allowDynamicOrKinematic)
        JPH_BodyCreationSettings_SetCollideKinematicVsNonDynamic(settings, collideKinematicVsNonDynamic)
        JPH_BodyCreationSettings_SetUseManifoldReduction(settings, useManifoldReduction)
        JPH_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(settings, enhancedInternalEdgeRemoval)
        JPH_BodyCreationSettings_SetNumVelocityStepsOverride(settings, UInt32(velocityStepsOverride))
        JPH_BodyCreationSettings_SetNumPositionStepsOverride(settings, UInt32(positionStepsOverride))
        JPH_BodyCreationSettings_SetIsSensor(settings, isSensor)
        JPH_BodyCreationSettings_SetUserData(settings, userData)
        JPH_BodyCreationSettings_SetAllowedDOFs(settings, JPH_AllowedDOFs(rawValue: allowedDegreesOfFreedom.rawValue))
        var group = collisionGroup.jph
        JPH_BodyCreationSettings_SetCollisionGroup(settings, &group)
        if let mass {
            var properties = JPH_MassProperties()
            properties.mass = mass
            JPH_BodyCreationSettings_SetMassPropertiesOverride(settings, &properties)
            JPH_BodyCreationSettings_SetOverrideMassProperties(settings, JPH_OverrideMassProperties_CalculateInertia)
        }
    }
}
