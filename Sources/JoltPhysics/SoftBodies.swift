import CJolt

/// How a soft body resists bending across shared edges.
public enum SoftBodyBendType: Sendable {
    case noBending
    case distance
    case dihedral

    var jph: JPH_SoftBodyBendType {
        switch self {
        case .noBending: JPH_SoftBodyBendType_None
        case .distance: JPH_SoftBodyBendType_Distance
        case .dihedral: JPH_SoftBodyBendType_Dihedral
        }
    }
}

/// Rest-state geometry and constraints, shareable between soft body instances.
public final class SoftBodySharedSettings {
    let handle: OpaquePointer

    public init() {
        handle = JPH_SoftBodySharedSettings_Create()
    }

    deinit {
        JPH_SoftBodySharedSettings_Destroy(handle)
    }

    public var vertexCount: Int {
        Int(JPH_SoftBodySharedSettings_GetVertexCount(handle))
    }

    /// An inverse mass of 0 makes the vertex immovable.
    public func addVertex(_ position: SIMD3<Float>, inverseMass: Float = 1) {
        var point = JPH_Vec3(position)
        JPH_SoftBodySharedSettings_AddVertex(handle, &point, inverseMass)
    }

    public func addVertices(_ positions: [SIMD3<Float>], inverseMass: Float = 1) {
        let points = positions.map(JPH_Vec3.init)
        let inverseMasses = [Float](repeating: inverseMass, count: positions.count)
        JPH_SoftBodySharedSettings_AddVertices(handle, points, inverseMasses, UInt32(positions.count))
    }

    public func addFace(_ vertex1: Int, _ vertex2: Int, _ vertex3: Int) {
        let count = vertexCount
        precondition([vertex1, vertex2, vertex3].allSatisfy { (0..<count).contains($0) }, "Face vertex index out of range")
        JPH_SoftBodySharedSettings_AddFace(handle, UInt32(vertex1), UInt32(vertex2), UInt32(vertex3))
    }

    /// Pin vertices before calling `createConstraints`.
    public func pinVertex(_ index: Int) {
        precondition((0..<vertexCount).contains(index), "Vertex index out of range")
        JPH_SoftBodySharedSettings_AddPinnedVertex(handle, UInt32(index))
    }

    /// Generates edge, shear, and bend constraints from the faces.
    public func createConstraints(compliance: Float = 1.0e-5, bendType: SoftBodyBendType = .distance) {
        JPH_SoftBodySharedSettings_CreateConstraints(handle, compliance, bendType.jph)
    }

    /// Reorders constraints for parallel solving. Call after all constraints exist.
    public func optimize() {
        JPH_SoftBodySharedSettings_Optimize(handle)
    }

    public func vertexPosition(at index: Int) -> SIMD3<Float> {
        precondition((0..<vertexCount).contains(index), "Vertex index out of range")
        var position = JPH_Vec3()
        JPH_SoftBodySharedSettings_GetVertexPosition(handle, UInt32(index), &position)
        return position.simd
    }
}
