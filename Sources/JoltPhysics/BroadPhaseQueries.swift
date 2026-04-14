import CJolt
import simd

/// Fast, coarse queries against body bounding boxes only. Results can include bodies whose actual
/// shape does not touch the query; use `PhysicsWorld.castRay`, `collideShape` or
/// `bodies(containing:)` for exact answers. Access through `PhysicsWorld.broadPhase`.
public struct BroadPhaseQuery {
    let world: PhysicsWorld

    private var query: OpaquePointer {
        JPH_PhysicsSystem_GetBroadPhaseQuery(world.system)
    }

    /// Bodies whose bounding box overlaps the box from `min` to `max`.
    public func bodies(overlappingBoxFrom min: SIMD3<Float>, to max: SIMD3<Float>) -> [BodyID] {
        var box = JPH_AABox(min: min, max: max)
        return collectBodies { callback, context in
            JPH_BroadPhaseQuery_CollideAABox(query, &box, callback, context, nil, nil)
        }
    }

    /// Bodies whose bounding box overlaps the sphere.
    public func bodies(overlappingSphereAt center: SIMD3<Float>, radius: Float) -> [BodyID] {
        var c = JPH_Vec3(center)
        return collectBodies { callback, context in
            JPH_BroadPhaseQuery_CollideSphere(query, &c, radius, callback, context, nil, nil)
        }
    }

    /// Bodies whose bounding box contains `point`.
    public func bodies(containing point: SIMD3<Float>) -> [BodyID] {
        var p = JPH_Vec3(point)
        return collectBodies { callback, context in
            JPH_BroadPhaseQuery_CollidePoint(query, &p, callback, context, nil, nil)
        }
    }

    /// Bodies whose bounding box the ray enters, sorted nearest first. `fraction` is where the ray
    /// enters the box, as a fraction of `direction`.
    public func castRay(origin: SIMD3<Float>, direction: SIMD3<Float>) -> [(body: BodyID, fraction: Float)] {
        var o = JPH_Vec3(origin)
        var d = JPH_Vec3(direction)
        var results: [JPH_BroadPhaseCastResult] = []
        withUnsafeMutablePointer(to: &results) { resultsPointer in
            _ = JPH_BroadPhaseQuery_CastRay2(
                query, &o, &d, JPH_CollisionCollectorType_AllHitSorted,
                { context, result in
                    guard let context, let result else {
                        return
                    }
                    context.assumingMemoryBound(to: [JPH_BroadPhaseCastResult].self).pointee.append(result.pointee)
                },
                resultsPointer, nil, nil
            )
        }
        return results.map { (BodyID(rawValue: $0.bodyID), $0.fraction) }
    }

    private typealias BodyCollector = @convention(c) (UnsafeMutableRawPointer?, JPH_BodyID) -> Float

    private func collectBodies(
        _ run: (BodyCollector, UnsafeMutableRawPointer) -> Bool
    ) -> [BodyID] {
        var results: [BodyID] = []
        withUnsafeMutablePointer(to: &results) { resultsPointer in
            _ = run(
                { context, bodyID in
                    context!.assumingMemoryBound(to: [BodyID].self).pointee.append(BodyID(rawValue: bodyID))
                    // Returning the largest fraction keeps the collector going.
                    return .greatestFiniteMagnitude
                },
                resultsPointer
            )
        }
        return results
    }
}

extension PhysicsWorld {
    public var broadPhase: BroadPhaseQuery {
        BroadPhaseQuery(world: self)
    }
}
