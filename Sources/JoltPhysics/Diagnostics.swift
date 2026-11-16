import CJolt
import Synchronization

/// A failed Jolt assertion. Only reported when Jolt is built with asserts enabled; the bundled
/// release build has them off.
public struct JoltAssertionFailure: Sendable {
    public var expression: String
    public var message: String?
    public var file: String
    public var line: Int
}

private let traceHandler = Mutex<(@Sendable (String) -> Void)?>(nil)
private let assertionHandler = Mutex<(@Sendable (JoltAssertionFailure) -> Bool)?>(nil)

extension JoltEngine {
    /// Receives Jolt's diagnostic messages. Called from any thread, including Jolt's workers.
    /// nil restores the default, which prints to stdout.
    @preconcurrency
    public static func setTraceHandler(_ handler: (@Sendable (String) -> Void)?) {
        traceHandler.withLock { $0 = handler }
        if handler == nil {
            JPH_SetTraceHandler(nil)
        } else {
            JPH_SetTraceHandler { message in
                guard let message, let current = traceHandler.withLock({ $0 }) else {
                    return
                }
                current(String(cString: message))
            }
        }
    }

    /// Receives Jolt assertion failures. Return true to stop in the debugger. nil restores the
    /// default, which prints to stderr and breaks. Has no effect unless Jolt is built with asserts.
    @preconcurrency
    public static func setAssertionHandler(_ handler: (@Sendable (JoltAssertionFailure) -> Bool)?) {
        assertionHandler.withLock { $0 = handler }
        if handler == nil {
            JPH_SetAssertFailureHandler(nil)
        } else {
            JPH_SetAssertFailureHandler { expression, message, file, line in
                guard let current = assertionHandler.withLock({ $0 }) else {
                    return true
                }
                return current(JoltAssertionFailure(
                    expression: expression.map { String(cString: $0) } ?? "",
                    message: message.map { String(cString: $0) },
                    file: file.map { String(cString: $0) } ?? "",
                    line: Int(line)
                ))
            }
        }
    }
}
