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
            var swiftSettings = ContactSettings(settings!.pointee)
            ContactListenerBox.from(userData).contactAdded(contact, settings: &swiftSettings)
            swiftSettings.apply(to: settings!)
        },
        OnContactPersisted: { userData, body1, body2, manifold, settings in
            let contact = Contact(body1!, body2!, manifold!)
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
            pointsOn2: pointsOn2
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
