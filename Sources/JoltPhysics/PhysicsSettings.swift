import CJolt

/// World-wide solver and sleep settings. Read and change with `PhysicsWorld.settings`.
/// Defaults are Jolt's; see Jolt's `PhysicsSettings` for details on each value.
public struct PhysicsSettings: Sendable, Equatable {
    public var maxInFlightBodyPairs: Int
    public var stepListenersBatchSize: Int
    public var stepListenerBatchesPerJob: Int
    /// Fraction of position error fixed per step (0...1).
    public var baumgarte: Float
    public var speculativeContactDistance: Float
    /// How far bodies may overlap before the solver pushes them apart.
    public var penetrationSlop: Float
    public var linearCastThreshold: Float
    public var linearCastMaxPenetration: Float
    public var manifoldTolerance: Float
    public var maxPenetrationDistance: Float
    public var bodyPairCacheMaxDeltaPositionSquared: Float
    public var bodyPairCacheCosMaxDeltaRotationDiv2: Float
    public var contactNormalCosMaxDeltaRotation: Float
    public var contactPointPreserveLambdaMaxDistanceSquared: Float
    /// Friction needs at least 2.
    public var velocitySteps: Int
    public var positionSteps: Int
    public var minVelocityForRestitution: Float
    /// Seconds a body must be nearly still before it sleeps.
    public var timeBeforeSleep: Float
    public var pointVelocitySleepThreshold: Float
    public var deterministicSimulation: Bool
    public var constraintWarmStart: Bool
    public var useBodyPairContactCache: Bool
    public var useManifoldReduction: Bool
    public var useLargeIslandSplitter: Bool
    public var allowSleeping: Bool
    public var checkActiveEdges: Bool

    init(_ settings: JPH_PhysicsSettings) {
        maxInFlightBodyPairs = Int(settings.maxInFlightBodyPairs)
        stepListenersBatchSize = Int(settings.stepListenersBatchSize)
        stepListenerBatchesPerJob = Int(settings.stepListenerBatchesPerJob)
        baumgarte = settings.baumgarte
        speculativeContactDistance = settings.speculativeContactDistance
        penetrationSlop = settings.penetrationSlop
        linearCastThreshold = settings.linearCastThreshold
        linearCastMaxPenetration = settings.linearCastMaxPenetration
        manifoldTolerance = settings.manifoldTolerance
        maxPenetrationDistance = settings.maxPenetrationDistance
        bodyPairCacheMaxDeltaPositionSquared = settings.bodyPairCacheMaxDeltaPositionSq
        bodyPairCacheCosMaxDeltaRotationDiv2 = settings.bodyPairCacheCosMaxDeltaRotationDiv2
        contactNormalCosMaxDeltaRotation = settings.contactNormalCosMaxDeltaRotation
        contactPointPreserveLambdaMaxDistanceSquared = settings.contactPointPreserveLambdaMaxDistSq
        velocitySteps = Int(settings.numVelocitySteps)
        positionSteps = Int(settings.numPositionSteps)
        minVelocityForRestitution = settings.minVelocityForRestitution
        timeBeforeSleep = settings.timeBeforeSleep
        pointVelocitySleepThreshold = settings.pointVelocitySleepThreshold
        deterministicSimulation = settings.deterministicSimulation
        constraintWarmStart = settings.constraintWarmStart
        useBodyPairContactCache = settings.useBodyPairContactCache
        useManifoldReduction = settings.useManifoldReduction
        useLargeIslandSplitter = settings.useLargeIslandSplitter
        allowSleeping = settings.allowSleeping
        checkActiveEdges = settings.checkActiveEdges
    }

    var jph: JPH_PhysicsSettings {
        JPH_PhysicsSettings(
            maxInFlightBodyPairs: Int32(maxInFlightBodyPairs),
            stepListenersBatchSize: Int32(stepListenersBatchSize),
            stepListenerBatchesPerJob: Int32(stepListenerBatchesPerJob),
            baumgarte: baumgarte,
            speculativeContactDistance: speculativeContactDistance,
            penetrationSlop: penetrationSlop,
            linearCastThreshold: linearCastThreshold,
            linearCastMaxPenetration: linearCastMaxPenetration,
            manifoldTolerance: manifoldTolerance,
            maxPenetrationDistance: maxPenetrationDistance,
            bodyPairCacheMaxDeltaPositionSq: bodyPairCacheMaxDeltaPositionSquared,
            bodyPairCacheCosMaxDeltaRotationDiv2: bodyPairCacheCosMaxDeltaRotationDiv2,
            contactNormalCosMaxDeltaRotation: contactNormalCosMaxDeltaRotation,
            contactPointPreserveLambdaMaxDistSq: contactPointPreserveLambdaMaxDistanceSquared,
            numVelocitySteps: UInt32(velocitySteps),
            numPositionSteps: UInt32(positionSteps),
            minVelocityForRestitution: minVelocityForRestitution,
            timeBeforeSleep: timeBeforeSleep,
            pointVelocitySleepThreshold: pointVelocitySleepThreshold,
            deterministicSimulation: deterministicSimulation,
            constraintWarmStart: constraintWarmStart,
            useBodyPairContactCache: useBodyPairContactCache,
            useManifoldReduction: useManifoldReduction,
            useLargeIslandSplitter: useLargeIslandSplitter,
            allowSleeping: allowSleeping,
            checkActiveEdges: checkActiveEdges
        )
    }
}

extension PhysicsWorld {
    public var settings: PhysicsSettings {
        get {
            var result = JPH_PhysicsSettings()
            JPH_PhysicsSystem_GetPhysicsSettings(system, &result)
            return PhysicsSettings(result)
        }
        set {
            precondition(newValue.velocitySteps >= 0 && newValue.positionSteps >= 0, "Step counts must not be negative")
            var jph = newValue.jph
            JPH_PhysicsSystem_SetPhysicsSettings(system, &jph)
        }
    }

    public var maxBodies: Int {
        Int(JPH_PhysicsSystem_GetMaxBodies(system))
    }

    /// Every body in the world, including sleeping and static ones.
    public var bodies: [BodyID] {
        let count = Int(JPH_PhysicsSystem_GetNumBodies(system))
        var ids = [JPH_BodyID](repeating: 0, count: count)
        JPH_PhysicsSystem_GetBodies(system, &ids, UInt32(count))
        return ids.map(BodyID.init(rawValue:))
    }

    /// Awake rigid bodies.
    public var activeBodies: [BodyID] {
        let count = Int(JPH_PhysicsSystem_GetNumActiveBodies(system, JPH_BodyType_Rigid))
        var ids = [JPH_BodyID](repeating: 0, count: count)
        JPH_PhysicsSystem_GetActiveBodies(system, JPH_BodyType_Rigid, &ids, UInt32(count))
        return ids.map(BodyID.init(rawValue:))
    }

    /// Constraints added through this world, including vehicles.
    public var allConstraints: [Constraint] {
        Array(constraints.values)
    }

    /// True if the two bodies touched during the last update.
    public func wereBodiesInContact(_ body1: BodyID, _ body2: BodyID) -> Bool {
        JPH_PhysicsSystem_WereBodiesInContact(system, body1.rawValue, body2.rawValue)
    }

    /// Wakes every body in `layer` whose bounds overlap the box.
    public func activateBodies(inBoxFrom min: SIMD3<Float>, to max: SIMD3<Float>, layer: ObjectLayer = .moving) {
        var box = JPH_AABox(min: min, max: max)
        JPH_PhysicsSystem_ActivateBodiesInAABox(system, &box, layer.rawValue)
    }
}
