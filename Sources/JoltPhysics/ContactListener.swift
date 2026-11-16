import CJolt
import simd

public enum ContactValidation: Sendable {
    case acceptAllContactsForThisBodyPair
    case acceptContact
    case rejectContact
    case rejectAllContactsForThisBodyPair

    var jph: JPH_ValidateResult {
        switch self {
        case .acceptAllContactsForThisBodyPair: return JPH_ValidateResult_AcceptAllContactsForThisBodyPair
        case .acceptContact: return JPH_ValidateResult_AcceptContact
        case .rejectContact: return JPH_ValidateResult_RejectContact
        case .rejectAllContactsForThisBodyPair: return JPH_ValidateResult_RejectAllContactsForThisBodyPair
        }
    }
}

/// Identifies a part of a shape, such as a compound child or mesh triangle. Only meaningful for
/// the shape it came from.
public struct SubShapeID: Sendable, Hashable {
    public var rawValue: UInt32

    public init(rawValue: UInt32) {
        self.rawValue = rawValue
    }
}

public struct Contact: Sendable {
    public var body1: BodyID
    public var body2: BodyID
    /// World-space normal pointing from body1 to body2.
    public var normal: SIMD3<Float>
    public var penetrationDepth: Float
    /// World-space contact points on body1.
    public var pointsOn1: [SIMD3<Float>]
    /// World-space contact points on body2.
    public var pointsOn2: [SIMD3<Float>]
    /// Which part of body1's shape is touching.
    public var subShapeID1: SubShapeID
    /// Which part of body2's shape is touching.
    public var subShapeID2: SubShapeID
    var callbackScope: ContactCallbackScope?

    /// Predicts the velocities and impulses that solving this contact will produce, for example
    /// to pick an impact sound or apply damage. Only valid inside `contactAdded` or
    /// `contactPersisted`; calling it later is a precondition failure.
    public func estimateResponse(
        friction: Float,
        restitution: Float,
        minVelocityForRestitution: Float = 1,
        iterations: Int = 10
    ) -> CollisionResponseEstimate {
        guard let callbackScope, callbackScope.isValid else {
            preconditionFailure("Contact.estimateResponse is only valid inside a contact listener callback")
        }
        var result = JPH_CollisionEstimationResult()
        JPH_EstimateCollisionResponse(
            callbackScope.body1, callbackScope.body2, callbackScope.manifold,
            friction, restitution, minVelocityForRestitution, UInt32(iterations), &result
        )
        defer { JPH_CollisionEstimationResult_FreeMembers(&result) }
        return CollisionResponseEstimate(result)
    }
}

public struct CollisionResponseEstimate: Sendable {
    public struct Impulse: Sendable, Hashable {
        /// Impulse along the contact normal.
        public var contact: Float
        /// Friction impulse along `tangent1`.
        public var friction1: Float
        /// Friction impulse along `tangent2`.
        public var friction2: Float
    }

    public var linearVelocity1: SIMD3<Float>
    public var angularVelocity1: SIMD3<Float>
    public var linearVelocity2: SIMD3<Float>
    public var angularVelocity2: SIMD3<Float>
    public var tangent1: SIMD3<Float>
    public var tangent2: SIMD3<Float>
    /// One entry per contact point.
    public var impulses: [Impulse]

    init(_ result: JPH_CollisionEstimationResult) {
        linearVelocity1 = result.linearVelocity1.simd
        angularVelocity1 = result.angularVelocity1.simd
        linearVelocity2 = result.linearVelocity2.simd
        angularVelocity2 = result.angularVelocity2.simd
        tangent1 = result.tangent1.simd
        tangent2 = result.tangent2.simd
        impulses = (0..<Int(result.impulseCount)).map { index in
            let impulse = result.impulses[index]
            return Impulse(contact: impulse.contactImpulse, friction1: impulse.frictionImpulse1, friction2: impulse.frictionImpulse2)
        }
    }
}

/// The Jolt pointers behind a `Contact`, valid only while the listener callback runs.
final class ContactCallbackScope: @unchecked Sendable {
    let body1: OpaquePointer
    let body2: OpaquePointer
    let manifold: OpaquePointer
    var isValid = true

    init(body1: OpaquePointer, body2: OpaquePointer, manifold: OpaquePointer) {
        self.body1 = body1
        self.body2 = body2
        self.manifold = manifold
    }
}

/// Per-contact settings that a listener can change in `contactAdded` and `contactPersisted`.
public struct ContactSettings: Sendable {
    public var combinedFriction: Float
    public var combinedRestitution: Float
    public var isSensor: Bool
}

/// Receives contact events from a `PhysicsWorld`.
///
/// All methods run on Jolt's worker threads, possibly concurrently, during `PhysicsWorld.update`.
/// Do not call back into the world's body interface from them.
public protocol ContactListener: AnyObject, Sendable {
    func contactValidate(body1: BodyID, body2: BodyID) -> ContactValidation
    func contactAdded(_ contact: Contact, settings: inout ContactSettings)
    func contactPersisted(_ contact: Contact, settings: inout ContactSettings)
    func contactRemoved(body1: BodyID, body2: BodyID)
}

public extension ContactListener {
    func contactValidate(body1 _: BodyID, body2 _: BodyID) -> ContactValidation {
        .acceptAllContactsForThisBodyPair
    }
    func contactAdded(_: Contact, settings _: inout ContactSettings) {}
    func contactPersisted(_: Contact, settings _: inout ContactSettings) {}
    func contactRemoved(body1 _: BodyID, body2 _: BodyID) {}
}

final class ContactListenerBox {
    let listener: any ContactListener

    init(_ listener: any ContactListener) {
        self.listener = listener
    }

    static func from(_ userData: UnsafeMutableRawPointer?) -> any ContactListener {
        Unmanaged<ContactListenerBox>.fromOpaque(userData!).takeUnretainedValue().listener
    }
}

// The C procs are process-wide, so install them once. Each listener finds its Swift object via userData.
// joltc keeps the pointer it is given, so the procs struct is allocated once and never freed.
private let installContactListenerProcs: Void = {
    let procs = UnsafeMutablePointer<JPH_ContactListener_Procs>.allocate(capacity: 1)
    procs.initialize(to: JPH_ContactListener_Procs(
        OnContactValidate: { userData, body1, body2, _, _ in
            ContactListenerBox.from(userData)
                .contactValidate(body1: BodyID(body1!), body2: BodyID(body2!))
                .jph
        },
        OnContactAdded: { userData, body1, body2, manifold, settings in
            let contact = Contact(body1!, body2!, manifold!)
            defer { contact.callbackScope?.isValid = false }
            var swiftSettings = ContactSettings(settings!.pointee)
            ContactListenerBox.from(userData).contactAdded(contact, settings: &swiftSettings)
            swiftSettings.apply(to: settings!)
        },
        OnContactPersisted: { userData, body1, body2, manifold, settings in
            let contact = Contact(body1!, body2!, manifold!)
            defer { contact.callbackScope?.isValid = false }
            var swiftSettings = ContactSettings(settings!.pointee)
            ContactListenerBox.from(userData).contactPersisted(contact, settings: &swiftSettings)
            swiftSettings.apply(to: settings!)
        },
        OnContactRemoved: { userData, pair in
            ContactListenerBox.from(userData).contactRemoved(
                body1: BodyID(rawValue: pair!.pointee.Body1ID),
                body2: BodyID(rawValue: pair!.pointee.Body2ID)
            )
        }
    ))
    JPH_ContactListener_SetProcs(procs)
}()

extension BodyID {
    init(_ body: OpaquePointer) {
        self.init(rawValue: JPH_Body_GetID(body))
    }
}

extension Contact {
    init(_ body1: OpaquePointer, _ body2: OpaquePointer, _ manifold: OpaquePointer) {
        var normal = JPH_Vec3()
        JPH_ContactManifold_GetWorldSpaceNormal(manifold, &normal)
        let count = JPH_ContactManifold_GetPointCount(manifold)
        var pointsOn1: [SIMD3<Float>] = []
        var pointsOn2: [SIMD3<Float>] = []
        for index in 0..<count {
            var point = JPH_RVec3()
            JPH_ContactManifold_GetWorldSpaceContactPointOn1(manifold, index, &point)
            pointsOn1.append(point.simd)
            JPH_ContactManifold_GetWorldSpaceContactPointOn2(manifold, index, &point)
            pointsOn2.append(point.simd)
        }
        self.init(
            body1: BodyID(body1),
            body2: BodyID(body2),
            normal: normal.simd,
            penetrationDepth: JPH_ContactManifold_GetPenetrationDepth(manifold),
            pointsOn1: pointsOn1,
            pointsOn2: pointsOn2,
            subShapeID1: SubShapeID(rawValue: JPH_ContactManifold_GetSubShapeID1(manifold)),
            subShapeID2: SubShapeID(rawValue: JPH_ContactManifold_GetSubShapeID2(manifold)),
            callbackScope: ContactCallbackScope(body1: body1, body2: body2, manifold: manifold)
        )
    }
}

extension ContactSettings {
    init(_ settings: JPH_ContactSettings) {
        self.init(
            combinedFriction: settings.combinedFriction,
            combinedRestitution: settings.combinedRestitution,
            isSensor: settings.isSensor != 0
        )
    }

    func apply(to settings: UnsafeMutablePointer<JPH_ContactSettings>) {
        settings.pointee.combinedFriction = combinedFriction
        settings.pointee.combinedRestitution = combinedRestitution
        settings.pointee.isSensor = isSensor ? 1 : 0
    }
}

extension PhysicsWorld {
    func installContactListener() {
        guard let contactListener else {
            JPH_PhysicsSystem_SetContactListener(system, nil)
            releaseContactListener()
            return
        }
        _ = installContactListenerProcs
        let box = Unmanaged.passRetained(ContactListenerBox(contactListener))
        let handle = JPH_ContactListener_Create(box.toOpaque())
        JPH_PhysicsSystem_SetContactListener(system, handle)
        releaseContactListener()
        contactListenerBox = box
        contactListenerHandle = handle
    }

    func releaseContactListener() {
        if let contactListenerHandle {
            JPH_ContactListener_Destroy(contactListenerHandle)
        }
        contactListenerBox?.release()
        contactListenerHandle = nil
        contactListenerBox = nil
    }
}
