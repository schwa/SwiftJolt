import JoltPhysics
import Testing

extension JoltTests {
    // The bundled Jolt build has no trace or assert that the public API can trigger on purpose,
    // so these only check that installing and removing handlers is safe.
    @Suite(.serialized)
    struct DiagnosticsTests {
        @Test func installAndRemoveHandlers() throws {
            JoltEngine.setTraceHandler { _ in }
            JoltEngine.setAssertionHandler { _ in false }
            let world = try makeWorld()
            world.addBody(.sphere(radius: 1))
            simulate(world, seconds: 0.1)
            JoltEngine.setTraceHandler(nil)
            JoltEngine.setAssertionHandler(nil)
        }
    }
}
