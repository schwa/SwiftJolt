import CJolt

// MARK: - ObjectLayer

/// A body's object layer. `CollisionLayers` decides which layers collide.
public struct ObjectLayer: RawRepresentable, Hashable, Sendable {
    public let rawValue: JPH_ObjectLayer

    public init(rawValue: JPH_ObjectLayer) {
        self.rawValue = rawValue
    }

    /// Default layer for static bodies.
    public static let nonMoving = Self(rawValue: 0)
    /// Default layer for everything else, including characters, vehicles, soft bodies and ragdolls.
    public static let moving = Self(rawValue: 1)
}

// MARK: - CollisionLayers

/// Which object layers collide. Fixed when the `PhysicsWorld` is created.
///
/// Layers 0 (`.nonMoving`) and 1 (`.moving`) always exist because the library uses them by default.
public struct CollisionLayers: Sendable, Equatable {
    public let count: Int
    /// Layers whose bodies never move. Jolt keeps them in a separate broad-phase tree, which is faster.
    public var staticLayers: Set<ObjectLayer>
    private var pairs: Set<Pair> = []

    private struct Pair: Hashable, Sendable {
        let low: JPH_ObjectLayer
        let high: JPH_ObjectLayer

        init(_ a: ObjectLayer, _ b: ObjectLayer) {
            low = min(a.rawValue, b.rawValue)
            high = max(a.rawValue, b.rawValue)
        }
    }

    /// `count` layers with no collisions enabled.
    public init(count: Int, staticLayers: Set<ObjectLayer> = [.nonMoving]) {
        precondition(count >= 2, "Need at least the .nonMoving and .moving layers")
        self.count = count
        self.staticLayers = staticLayers
        staticLayers.forEach(checkLayer)
    }

    /// Two layers: static bodies collide with moving ones, and moving bodies collide with each other.
    public static let `default`: Self = {
        var layers = Self(count: 2)
        layers.enableCollision(.nonMoving, .moving)
        layers.enableCollision(.moving, .moving)
        return layers
    }()

    public mutating func enableCollision(_ a: ObjectLayer, _ b: ObjectLayer) {
        checkLayer(a)
        checkLayer(b)
        pairs.insert(Pair(a, b))
    }

    public mutating func disableCollision(_ a: ObjectLayer, _ b: ObjectLayer) {
        pairs.remove(Pair(a, b))
    }

    public func collides(_ a: ObjectLayer, _ b: ObjectLayer) -> Bool {
        pairs.contains(Pair(a, b))
    }

    func checkLayer(_ layer: ObjectLayer) {
        precondition(Int(layer.rawValue) < count, "Object layer \(layer.rawValue) is out of range (count \(count))")
    }

    /// Builds Jolt's broad-phase layer interface, pair filter, and object-vs-broad-phase filter.
    func makeJoltFilters() -> (broadPhaseLayerInterface: OpaquePointer, objectLayerPairFilter: OpaquePointer, objectVsBroadPhaseLayerFilter: OpaquePointer) {
        // Broad-phase layer 0 holds static layers, 1 holds the rest.
        let broadPhaseLayerCount: UInt32 = 2
        let interface = JPH_BroadPhaseLayerInterfaceTable_Create(UInt32(count), broadPhaseLayerCount)!
        for raw in 0..<UInt32(count) {
            let broadPhaseLayer: JPH_BroadPhaseLayer = staticLayers.contains(ObjectLayer(rawValue: raw)) ? 0 : 1
            JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(interface, raw, broadPhaseLayer)
        }
        let pairFilter = JPH_ObjectLayerPairFilterTable_Create(UInt32(count))!
        for pair in pairs {
            JPH_ObjectLayerPairFilterTable_EnableCollision(pairFilter, pair.low, pair.high)
        }
        let broadPhaseFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(interface, broadPhaseLayerCount, pairFilter, UInt32(count))!
        return (interface, pairFilter, broadPhaseFilter)
    }
}

// MARK: - Collision groups

/// A table of which sub-groups collide inside the same collision group. All sub-group pairs start enabled.
///
/// Changing the table while `PhysicsWorld.update` runs is not safe.
public final class CollisionGroupFilter: @unchecked Sendable {
    let handle: OpaquePointer
    public let subGroupCount: Int

    public init(subGroupCount: Int) {
        precondition(subGroupCount >= 0, "Sub-group count must not be negative")
        self.subGroupCount = subGroupCount
        handle = JPH_GroupFilterTable_Create(UInt32(subGroupCount))!
    }

    deinit {
        JPH_GroupFilter_Destroy(handle)
    }

    public func enableCollision(_ subGroup1: UInt32, _ subGroup2: UInt32) {
        checkSubGroups(subGroup1, subGroup2)
        JPH_GroupFilterTable_EnableCollision(handle, subGroup1, subGroup2)
    }

    public func disableCollision(_ subGroup1: UInt32, _ subGroup2: UInt32) {
        checkSubGroups(subGroup1, subGroup2)
        JPH_GroupFilterTable_DisableCollision(handle, subGroup1, subGroup2)
    }

    public func isCollisionEnabled(_ subGroup1: UInt32, _ subGroup2: UInt32) -> Bool {
        checkSubGroups(subGroup1, subGroup2)
        return JPH_GroupFilterTable_IsCollisionEnabled(handle, subGroup1, subGroup2)
    }

    private func checkSubGroups(_ subGroup1: UInt32, _ subGroup2: UInt32) {
        precondition(subGroup1 != subGroup2, "Sub-groups must differ")
        precondition(Int(max(subGroup1, subGroup2)) < subGroupCount, "Sub-group out of range (count \(subGroupCount))")
    }
}

/// A body's collision group. Bodies in the same group with a filter ask the filter whether their
/// sub-groups collide. Bodies in different groups, or without a filter, always collide (if their
/// layers allow it).
public struct CollisionGroup: Sendable, Equatable {
    public var filter: CollisionGroupFilter?
    public var groupID: UInt32
    public var subGroupID: UInt32

    public init(filter: CollisionGroupFilter? = nil, groupID: UInt32 = .max, subGroupID: UInt32 = .max) {
        if let filter, groupID != .max {
            precondition(Int(subGroupID) < filter.subGroupCount, "Sub-group \(subGroupID) out of range (count \(filter.subGroupCount))")
        }
        self.filter = filter
        self.groupID = groupID
        self.subGroupID = subGroupID
    }

    /// No group: collides with everything its layer allows.
    public static let ungrouped = Self()

    public static func == (lhs: Self, rhs: Self) -> Bool {
        lhs.filter === rhs.filter && lhs.groupID == rhs.groupID && lhs.subGroupID == rhs.subGroupID
    }

    var jph: JPH_CollisionGroup {
        // Jolt reads the filter as a GroupFilter; GroupFilterTable inherits from it.
        JPH_CollisionGroup(groupFilter: filter?.handle, groupID: groupID, subGroupID: subGroupID)
    }
}

// MARK: - PhysicsWorld

extension PhysicsWorld {
    public func objectLayer(of body: BodyID) -> ObjectLayer {
        ObjectLayer(rawValue: JPH_BodyInterface_GetObjectLayer(bodyInterface, body.rawValue))
    }

    public func setObjectLayer(of body: BodyID, to layer: ObjectLayer) {
        collisionLayers.checkLayer(layer)
        JPH_BodyInterface_SetObjectLayer(bodyInterface, body.rawValue, layer.rawValue)
    }

    public func collisionGroup(of body: BodyID) -> CollisionGroup {
        var result = JPH_CollisionGroup()
        JPH_BodyInterface_GetCollisionGroup(bodyInterface, body.rawValue, &result)
        let filter = result.groupFilter.flatMap { groupFilters[$0] }
        return CollisionGroup(filter: filter, groupID: result.groupID, subGroupID: result.subGroupID)
    }

    public func setCollisionGroup(of body: BodyID, to group: CollisionGroup) {
        registerGroupFilter(group.filter)
        var jph = group.jph
        JPH_BodyInterface_SetCollisionGroup(bodyInterface, body.rawValue, &jph)
    }

    /// Remembers filters so `collisionGroup(of:)` can return the Swift object.
    func registerGroupFilter(_ filter: CollisionGroupFilter?) {
        if let filter {
            groupFilters[filter.handle] = filter
        }
    }
}
