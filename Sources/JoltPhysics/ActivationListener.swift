import CJolt

/// Receives notifications when bodies wake up or fall asleep.
///
/// Jolt calls these while holding internal locks, possibly from worker threads during
/// `PhysicsWorld.update`, and also from `addBody` and velocity setters. Do not call back into
/// the world from them; record the event and handle it later.
public protocol ActivationListener: AnyObject, Sendable {
    func bodyActivated(_ body: BodyID)
    func bodyDeactivated(_ body: BodyID)
}

public extension ActivationListener {
    func bodyActivated(_: BodyID) {}
    func bodyDeactivated(_: BodyID) {}
}

final class ActivationListenerBox {
    let listener: any ActivationListener

    init(_ listener: any ActivationListener) {
        self.listener = listener
    }

    static func from(_ userData: UnsafeMutableRawPointer?) -> any ActivationListener {
        Unmanaged<ActivationListenerBox>.fromOpaque(userData!).takeUnretainedValue().listener
    }
}

// joltc keeps the pointer it is given, so the procs struct is allocated once and never freed.
private let installActivationListenerProcs: Void = {
    let procs = UnsafeMutablePointer<JPH_BodyActivationListener_Procs>.allocate(capacity: 1)
    procs.initialize(to: JPH_BodyActivationListener_Procs(
        OnBodyActivated: { userData, bodyID, _ in
            ActivationListenerBox.from(userData).bodyActivated(BodyID(rawValue: bodyID))
        },
        OnBodyDeactivated: { userData, bodyID, _ in
            ActivationListenerBox.from(userData).bodyDeactivated(BodyID(rawValue: bodyID))
        }
    ))
    JPH_BodyActivationListener_SetProcs(procs)
}()

extension PhysicsWorld {
    func installActivationListener() {
        guard let activationListener else {
            JPH_PhysicsSystem_SetBodyActivationListener(system, nil)
            releaseActivationListener()
            return
        }
        _ = installActivationListenerProcs
        let box = Unmanaged.passRetained(ActivationListenerBox(activationListener))
        let handle = JPH_BodyActivationListener_Create(box.toOpaque())
        JPH_PhysicsSystem_SetBodyActivationListener(system, handle)
        releaseActivationListener()
        activationListenerBox = box
        activationListenerHandle = handle
    }

    func releaseActivationListener() {
        if let activationListenerHandle {
            JPH_BodyActivationListener_Destroy(activationListenerHandle)
        }
        activationListenerBox?.release()
        activationListenerHandle = nil
        activationListenerBox = nil
    }
}
