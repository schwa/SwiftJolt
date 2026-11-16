import CJolt
import Darwin

/// The world whose step listener is running on the current thread, if any.
private let stepListenerWorldKey: pthread_key_t = {
    var key = pthread_key_t()
    pthread_key_create(&key, nil)
    return key
}()

public struct StepContext: Sendable {
    /// Duration of this collision step (the update's deltaTime divided by collisionSteps).
    public var deltaTime: Float
    public var isFirstStep: Bool
    public var isLastStep: Bool
}

/// Called before every collision step of `PhysicsWorld.update`, for forces and controllers that
/// must run at the simulation rate.
///
/// Runs on Jolt's worker threads, and different listeners may run at the same time. Reading and
/// changing bodies and running queries through `PhysicsWorld` is allowed; on this thread the world
/// skips body locks, because Jolt already holds them. Adding or removing bodies or constraints is not allowed.
/// The world holds the listener strongly, so capture the world weakly or unowned to avoid a cycle.
public protocol StepListener: AnyObject, Sendable {
    func step(_ context: StepContext)
}

final class StepListenerRegistration {
    let listener: any StepListener
    unowned let world: PhysicsWorld
    var handle: OpaquePointer?

    init(_ listener: any StepListener, world: PhysicsWorld) {
        self.listener = listener
        self.world = world
    }
}

// joltc keeps the pointer it is given, so the procs struct is allocated once and never freed.
private let installStepListenerProcs: Void = {
    let procs = UnsafeMutablePointer<JPH_PhysicsStepListener_Procs>.allocate(capacity: 1)
    procs.initialize(to: JPH_PhysicsStepListener_Procs { userData, context in
        guard let userData, let context else {
            return
        }
        let registration = Unmanaged<StepListenerRegistration>.fromOpaque(userData).takeUnretainedValue()
        let previous = pthread_getspecific(stepListenerWorldKey)
        pthread_setspecific(stepListenerWorldKey, Unmanaged.passUnretained(registration.world).toOpaque())
        defer { pthread_setspecific(stepListenerWorldKey, previous) }
        registration.listener.step(StepContext(
            deltaTime: context.pointee.deltaTime,
            isFirstStep: context.pointee.isFirstStep != 0,
            isLastStep: context.pointee.isLastStep != 0
        ))
    })
    JPH_PhysicsStepListener_SetProcs(procs)
}()

extension PhysicsWorld {
    var isInStepListener: Bool {
        pthread_getspecific(stepListenerWorldKey) == Unmanaged.passUnretained(self).toOpaque()
    }

    /// Adds `listener`. The world keeps a strong reference until `removeStepListener` is called.
    /// Adding the same listener twice has no effect.
    public func addStepListener(_ listener: any StepListener) {
        let key = ObjectIdentifier(listener)
        guard stepListeners[key] == nil else {
            return
        }
        _ = installStepListenerProcs
        let registration = StepListenerRegistration(listener, world: self)
        let handle = JPH_PhysicsStepListener_Create(Unmanaged.passUnretained(registration).toOpaque())!
        registration.handle = handle
        JPH_PhysicsSystem_AddStepListener(system, handle)
        stepListeners[key] = registration
    }

    public func removeStepListener(_ listener: any StepListener) {
        guard let registration = stepListeners.removeValue(forKey: ObjectIdentifier(listener)) else {
            return
        }
        release(registration)
    }

    func removeAllStepListeners() {
        for registration in stepListeners.values {
            release(registration)
        }
        stepListeners.removeAll()
    }

    private func release(_ registration: StepListenerRegistration) {
        guard let handle = registration.handle else {
            return
        }
        JPH_PhysicsSystem_RemoveStepListener(system, handle)
        JPH_PhysicsStepListener_Destroy(handle)
        registration.handle = nil
    }
}
