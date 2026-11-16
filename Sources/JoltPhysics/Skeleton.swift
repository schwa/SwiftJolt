import CJolt

/// A joint hierarchy used by ragdolls and poses.
public final class Skeleton {
    public struct Joint: Sendable, Equatable {
        public var name: String
        public var parentName: String?
        public var parentIndex: Int?

        public init(name: String, parentName: String? = nil, parentIndex: Int? = nil) {
            self.name = name
            self.parentName = parentName
            self.parentIndex = parentIndex
        }
    }

    public let engine: JoltEngine
    let handle: OpaquePointer

    public init(engine: JoltEngine) {
        self.engine = engine
        handle = JPH_Skeleton_Create()
    }

    deinit {
        JPH_Skeleton_Destroy(handle)
    }

    /// Adds a joint and returns its index. Parents must be added before their children.
    @discardableResult
    public func addJoint(_ name: String, parent: Int? = nil) -> Int {
        if let parent {
            precondition((0..<jointCount).contains(parent), "Parent index out of range")
        }
        return Int(JPH_Skeleton_AddJoint2(handle, name, Int32(parent ?? -1)))
    }

    @discardableResult
    public func addJoint(_ name: String, parentName: String) -> Int {
        guard let parent = jointIndex(named: parentName) else {
            preconditionFailure("Unknown parent joint \(parentName)")
        }
        return addJoint(name, parent: parent)
    }

    public var jointCount: Int {
        Int(JPH_Skeleton_GetJointCount(handle))
    }

    public func joint(at index: Int) -> Joint {
        precondition((0..<jointCount).contains(index), "Joint index out of range")
        var joint = JPH_SkeletonJoint()
        JPH_Skeleton_GetJoint(handle, Int32(index), &joint)
        let parentName = joint.parentName.map { String(cString: $0) }
        return Joint(
            name: String(cString: joint.name),
            parentName: parentName?.isEmpty == false ? parentName : nil,
            parentIndex: joint.parentJointIndex >= 0 ? Int(joint.parentJointIndex) : nil
        )
    }

    public var joints: [Joint] {
        (0..<jointCount).map(joint(at:))
    }

    public func jointIndex(named name: String) -> Int? {
        let index = JPH_Skeleton_GetJointIndex(handle, name)
        return index >= 0 ? Int(index) : nil
    }

    /// Resolves parent indices from parent names.
    public func calculateParentJointIndices() {
        JPH_Skeleton_CalculateParentJointIndices(handle)
    }

    /// True when every parent comes before its children.
    public var areJointsCorrectlyOrdered: Bool {
        JPH_Skeleton_AreJointsCorrectlyOrdered(handle)
    }
}
