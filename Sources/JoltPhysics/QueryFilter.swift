import CJolt

/// Limits which bodies a ray cast, shape cast, shape overlap or point query can return.
public struct QueryFilter: Sendable {
    /// Only bodies in these layers. Empty allows every layer.
    public var layers: Set<ObjectLayer>
    /// Bodies to skip, e.g. the caster's own body.
    public var excludedBodies: Set<BodyID>
    /// Called for each candidate body; return false to skip it. Runs on the calling thread.
    public var shouldInclude: (@Sendable (BodyID) -> Bool)?

    @preconcurrency
    public init(layers: Set<ObjectLayer> = [], excludedBodies: Set<BodyID> = [], shouldInclude: (@Sendable (BodyID) -> Bool)? = nil) {
        self.layers = layers
        self.excludedBodies = excludedBodies
        self.shouldInclude = shouldInclude
    }

    /// Allows every body.
    public static let all = Self()

    func includes(_ body: BodyID) -> Bool {
        !excludedBodies.contains(body) && (shouldInclude?(body) ?? true)
    }

    /// Runs `body` with Jolt filter objects for this filter, or nils when there is nothing to filter.
    func withJoltFilters<Result>(_ body: (_ objectLayerFilter: OpaquePointer?, _ bodyFilter: OpaquePointer?) -> Result) -> Result {
        _ = installQueryFilterProcs
        let box = QueryFilterBox(self)
        return withExtendedLifetime(box) {
            let context = Unmanaged.passUnretained(box).toOpaque()
            let objectLayerFilter = layers.isEmpty ? nil : JPH_ObjectLayerFilter_Create(context)
            let bodyFilter = excludedBodies.isEmpty && shouldInclude == nil ? nil : JPH_BodyFilter_Create(context)
            defer {
                if let objectLayerFilter {
                    JPH_ObjectLayerFilter_Destroy(objectLayerFilter)
                }
                if let bodyFilter {
                    JPH_BodyFilter_Destroy(bodyFilter)
                }
            }
            return body(objectLayerFilter, bodyFilter)
        }
    }
}

final class QueryFilterBox {
    let filter: QueryFilter

    init(_ filter: QueryFilter) {
        self.filter = filter
    }

    static func from(_ userData: UnsafeMutableRawPointer?) -> QueryFilter {
        Unmanaged<QueryFilterBox>.fromOpaque(userData!).takeUnretainedValue().filter
    }
}

// joltc keeps the pointers it is given, so the procs structs are allocated once and never freed.
private let installQueryFilterProcs: Void = {
    let layerProcs = UnsafeMutablePointer<JPH_ObjectLayerFilter_Procs>.allocate(capacity: 1)
    layerProcs.initialize(to: JPH_ObjectLayerFilter_Procs { userData, layer in
        let layers = QueryFilterBox.from(userData).layers
        return layers.isEmpty || layers.contains(ObjectLayer(rawValue: layer))
    })
    JPH_ObjectLayerFilter_SetProcs(layerProcs)

    let bodyProcs = UnsafeMutablePointer<JPH_BodyFilter_Procs>.allocate(capacity: 1)
    bodyProcs.initialize(to: JPH_BodyFilter_Procs(
        ShouldCollide: { userData, bodyID in
            QueryFilterBox.from(userData).includes(BodyID(rawValue: bodyID))
        },
        ShouldCollideLocked: { userData, body in
            guard let body else {
                return true
            }
            return QueryFilterBox.from(userData).includes(BodyID(rawValue: JPH_Body_GetID(body)))
        }
    ))
    JPH_BodyFilter_SetProcs(bodyProcs)
}()
