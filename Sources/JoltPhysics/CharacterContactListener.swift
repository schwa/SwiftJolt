import CJolt
import simd

/// Identifies a virtual character, e.g. in `characterContactRemoved` where the other character may be gone.
public struct CharacterID: Hashable, Sendable {
    public let rawValue: UInt32
}

/// Per-contact choices a `CharacterContactListener` can make.
public struct CharacterContactSettings: Sendable {
    /// The other object can push the character.
    public var canPushCharacter: Bool
    /// The character can push the other object.
    public var canReceiveImpulses: Bool
}

/// Receives contact events for one `VirtualCharacter`. All callbacks run synchronously on the
/// thread that calls the character's `update`, `refreshContacts`, `walkStairs`, `stickToFloor` or
/// `setShape`. Do not call those methods, or `PhysicsWorld.update`, from a callback.
public protocol CharacterContactListener: AnyObject {
    /// Lets you change the velocity the character sees for `body`, e.g. for conveyor belts.
    func adjustBodyVelocity(of body: BodyID, for character: VirtualCharacter, linear: inout SIMD3<Float>, angular: inout SIMD3<Float>)
    /// Return false to ignore `body`.
    func contactValidate(_ character: VirtualCharacter, body: BodyID) -> Bool
    /// Return false to ignore `other`.
    func characterContactValidate(_ character: VirtualCharacter, other: VirtualCharacter) -> Bool
    func contactAdded(_ character: VirtualCharacter, body: BodyID, position: SIMD3<Float>, normal: SIMD3<Float>, settings: inout CharacterContactSettings)
    func contactPersisted(_ character: VirtualCharacter, body: BodyID, position: SIMD3<Float>, normal: SIMD3<Float>, settings: inout CharacterContactSettings)
    func contactRemoved(_ character: VirtualCharacter, body: BodyID)
    func characterContactAdded(_ character: VirtualCharacter, other: VirtualCharacter, position: SIMD3<Float>, normal: SIMD3<Float>, settings: inout CharacterContactSettings)
    func characterContactPersisted(_ character: VirtualCharacter, other: VirtualCharacter, position: SIMD3<Float>, normal: SIMD3<Float>, settings: inout CharacterContactSettings)
    func characterContactRemoved(_ character: VirtualCharacter, other: CharacterID)
    /// Lets you change the character's velocity when it slides along `body`. `newVelocity` starts at Jolt's answer.
    func contactSolve(_ character: VirtualCharacter, body: BodyID, position: SIMD3<Float>, normal: SIMD3<Float>, contactVelocity: SIMD3<Float>, characterVelocity: SIMD3<Float>, newVelocity: inout SIMD3<Float>)
    func characterContactSolve(_ character: VirtualCharacter, other: VirtualCharacter, position: SIMD3<Float>, normal: SIMD3<Float>, contactVelocity: SIMD3<Float>, characterVelocity: SIMD3<Float>, newVelocity: inout SIMD3<Float>)
}

public extension CharacterContactListener {
    func adjustBodyVelocity(of _: BodyID, for _: VirtualCharacter, linear _: inout SIMD3<Float>, angular _: inout SIMD3<Float>) {}
    func contactValidate(_: VirtualCharacter, body _: BodyID) -> Bool { true }
    func characterContactValidate(_: VirtualCharacter, other _: VirtualCharacter) -> Bool { true }
    func contactAdded(_: VirtualCharacter, body _: BodyID, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {}
    func contactPersisted(_: VirtualCharacter, body _: BodyID, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {}
    func contactRemoved(_: VirtualCharacter, body _: BodyID) {}
    func characterContactAdded(_: VirtualCharacter, other _: VirtualCharacter, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {}
    func characterContactPersisted(_: VirtualCharacter, other _: VirtualCharacter, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {}
    func characterContactRemoved(_: VirtualCharacter, other _: CharacterID) {}
    func contactSolve(_: VirtualCharacter, body _: BodyID, position _: SIMD3<Float>, normal _: SIMD3<Float>, contactVelocity _: SIMD3<Float>, characterVelocity _: SIMD3<Float>, newVelocity _: inout SIMD3<Float>) {}
    func characterContactSolve(_: VirtualCharacter, other _: VirtualCharacter, position _: SIMD3<Float>, normal _: SIMD3<Float>, contactVelocity _: SIMD3<Float>, characterVelocity _: SIMD3<Float>, newVelocity _: inout SIMD3<Float>) {}
}

final class CharacterContactListenerBox {
    let listener: any CharacterContactListener
    unowned let character: VirtualCharacter

    init(_ listener: any CharacterContactListener, character: VirtualCharacter) {
        self.listener = listener
        self.character = character
    }

    static func from(_ userData: UnsafeMutableRawPointer?) -> CharacterContactListenerBox {
        Unmanaged<CharacterContactListenerBox>.fromOpaque(userData!).takeUnretainedValue()
    }

    /// The Swift object for another character in the same world. Characters always exist while Jolt reports them.
    func other(_ pointer: OpaquePointer?) -> VirtualCharacter {
        guard let pointer, let other = character.world.virtualCharacters[pointer]?.character else {
            preconditionFailure("Contact with an unknown virtual character")
        }
        return other
    }
}

extension CharacterContactSettings {
    init(_ settings: JPH_CharacterContactSettings) {
        canPushCharacter = settings.canPushCharacter
        canReceiveImpulses = settings.canReceiveImpulses
    }

    func write(to pointer: UnsafeMutablePointer<JPH_CharacterContactSettings>?) {
        pointer?.pointee.canPushCharacter = canPushCharacter
        pointer?.pointee.canReceiveImpulses = canReceiveImpulses
    }
}

private func withSettings(_ pointer: UnsafeMutablePointer<JPH_CharacterContactSettings>?, _ body: (inout CharacterContactSettings) -> Void) {
    var settings = pointer.map { CharacterContactSettings($0.pointee) } ?? CharacterContactSettings(canPushCharacter: true, canReceiveImpulses: true)
    body(&settings)
    settings.write(to: pointer)
}

// joltc keeps the pointer it is given, so the procs struct is allocated once and never freed.
private let installCharacterContactListenerProcs: Void = {
    var procs = JPH_CharacterContactListener_Procs()
    addValidateProcs(to: &procs)
    addBodyContactProcs(to: &procs)
    addCharacterContactProcs(to: &procs)
    addSolveProcs(to: &procs)
    let pointer = UnsafeMutablePointer<JPH_CharacterContactListener_Procs>.allocate(capacity: 1)
    pointer.initialize(to: procs)
    JPH_CharacterContactListener_SetProcs(pointer)
}()

private func addValidateProcs(to procs: inout JPH_CharacterContactListener_Procs) {
    procs.OnAdjustBodyVelocity = { userData, _, body, linear, angular in
        guard let body, let linear, let angular else {
            return
        }
        let box = CharacterContactListenerBox.from(userData)
        var linearVelocity = linear.pointee.simd
        var angularVelocity = angular.pointee.simd
        box.listener.adjustBodyVelocity(of: BodyID(rawValue: JPH_Body_GetID(body)), for: box.character, linear: &linearVelocity, angular: &angularVelocity)
        linear.pointee = JPH_Vec3(linearVelocity)
        angular.pointee = JPH_Vec3(angularVelocity)
    }
    procs.OnContactValidate = { userData, _, body, _ in
        let box = CharacterContactListenerBox.from(userData)
        return box.listener.contactValidate(box.character, body: BodyID(rawValue: body))
    }
    procs.OnCharacterContactValidate = { userData, _, other, _ in
        let box = CharacterContactListenerBox.from(userData)
        return box.listener.characterContactValidate(box.character, other: box.other(other))
    }
}

private func addBodyContactProcs(to procs: inout JPH_CharacterContactListener_Procs) {
    procs.OnContactAdded = { userData, _, body, _, position, normal, settings in
        let box = CharacterContactListenerBox.from(userData)
        withSettings(settings) { contactSettings in
            box.listener.contactAdded(box.character, body: BodyID(rawValue: body), position: position!.pointee.simd, normal: normal!.pointee.simd, settings: &contactSettings)
        }
    }
    procs.OnContactPersisted = { userData, _, body, _, position, normal, settings in
        let box = CharacterContactListenerBox.from(userData)
        withSettings(settings) { contactSettings in
            box.listener.contactPersisted(box.character, body: BodyID(rawValue: body), position: position!.pointee.simd, normal: normal!.pointee.simd, settings: &contactSettings)
        }
    }
    procs.OnContactRemoved = { userData, _, body, _ in
        let box = CharacterContactListenerBox.from(userData)
        box.listener.contactRemoved(box.character, body: BodyID(rawValue: body))
    }
}

private func addCharacterContactProcs(to procs: inout JPH_CharacterContactListener_Procs) {
    procs.OnCharacterContactAdded = { userData, _, other, _, position, normal, settings in
        let box = CharacterContactListenerBox.from(userData)
        withSettings(settings) { contactSettings in
            box.listener.characterContactAdded(box.character, other: box.other(other), position: position!.pointee.simd, normal: normal!.pointee.simd, settings: &contactSettings)
        }
    }
    procs.OnCharacterContactPersisted = { userData, _, other, _, position, normal, settings in
        let box = CharacterContactListenerBox.from(userData)
        withSettings(settings) { contactSettings in
            box.listener.characterContactPersisted(box.character, other: box.other(other), position: position!.pointee.simd, normal: normal!.pointee.simd, settings: &contactSettings)
        }
    }
    procs.OnCharacterContactRemoved = { userData, _, otherID, _ in
        let box = CharacterContactListenerBox.from(userData)
        box.listener.characterContactRemoved(box.character, other: CharacterID(rawValue: otherID))
    }
}

private func addSolveProcs(to procs: inout JPH_CharacterContactListener_Procs) {
    procs.OnContactSolve = { userData, _, body, _, position, normal, contactVelocity, _, characterVelocity, newVelocity in
        guard let newVelocity else {
            return
        }
        let box = CharacterContactListenerBox.from(userData)
        var velocity = newVelocity.pointee.simd
        box.listener.contactSolve(
            box.character, body: BodyID(rawValue: body), position: position!.pointee.simd, normal: normal!.pointee.simd,
            contactVelocity: contactVelocity!.pointee.simd, characterVelocity: characterVelocity!.pointee.simd, newVelocity: &velocity
        )
        newVelocity.pointee = JPH_Vec3(velocity)
    }
    procs.OnCharacterContactSolve = { userData, _, other, _, position, normal, contactVelocity, _, characterVelocity, newVelocity in
        guard let newVelocity else {
            return
        }
        let box = CharacterContactListenerBox.from(userData)
        var velocity = newVelocity.pointee.simd
        box.listener.characterContactSolve(
            box.character, other: box.other(other), position: position!.pointee.simd, normal: normal!.pointee.simd,
            contactVelocity: contactVelocity!.pointee.simd, characterVelocity: characterVelocity!.pointee.simd, newVelocity: &velocity
        )
        newVelocity.pointee = JPH_Vec3(velocity)
    }
}

final class WeakVirtualCharacter {
    weak var character: VirtualCharacter?

    init(_ character: VirtualCharacter) {
        self.character = character
    }
}

extension VirtualCharacter {
    public var id: CharacterID {
        CharacterID(rawValue: JPH_CharacterVirtual_GetID(handle))
    }

    /// True if the character touched `other` during the last update.
    public func hasCollided(with other: VirtualCharacter) -> Bool {
        JPH_CharacterVirtual_HasCollidedWithCharacter(handle, other.handle)
    }

    /// Lets Jolt report removed contacts for contact changes made between the two calls. `update`
    /// does this for you; use it around the step-by-step methods.
    public func startTrackingContactChanges() {
        JPH_CharacterVirtual_StartTrackingContactChanges(handle)
    }

    public func finishTrackingContactChanges() {
        JPH_CharacterVirtual_FinishTrackingContactChanges(handle)
    }

    func installContactListener() {
        let previous = (contactListenerHandle, contactListenerBox)
        if let contactListener {
            _ = installCharacterContactListenerProcs
            let box = Unmanaged.passRetained(CharacterContactListenerBox(contactListener, character: self))
            let listenerHandle = JPH_CharacterContactListener_Create(box.toOpaque())!
            JPH_CharacterVirtual_SetListener(handle, listenerHandle)
            contactListenerHandle = listenerHandle
            contactListenerBox = box
        } else {
            JPH_CharacterVirtual_SetListener(handle, nil)
            contactListenerHandle = nil
            contactListenerBox = nil
        }
        if let oldHandle = previous.0 {
            JPH_CharacterContactListener_Destroy(oldHandle)
        }
        previous.1?.release()
    }

    func updateCharacterCollision(from oldValue: Bool) {
        guard collidesWithCharacters != oldValue else {
            return
        }
        let collision = world.characterVsCharacterCollision
        if collidesWithCharacters {
            JPH_CharacterVsCharacterCollisionSimple_AddCharacter(collision, handle)
            JPH_CharacterVirtual_SetCharacterVsCharacterCollision(handle, collision)
        } else {
            JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter(collision, handle)
            // joltc ignores nil, so point at an empty collection instead.
            JPH_CharacterVirtual_SetCharacterVsCharacterCollision(handle, world.emptyCharacterVsCharacterCollision)
        }
    }
}
