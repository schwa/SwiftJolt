import CJolt
import Foundation

/// Build options for `Shape.mesh`. Defaults match Jolt's.
public struct MeshOptions: Sendable, Equatable {
    public enum BuildQuality: Sendable {
        case favorRuntimePerformance
        case favorBuildSpeed
    }

    /// Edges between triangles that meet at less than this angle (radians) are treated as
    /// inactive, so objects slide over them without catching.
    public var activeEdgeThreshold: Float = 5 * .pi / 180
    public var maxTrianglesPerLeaf = 8
    public var buildQuality = BuildQuality.favorRuntimePerformance
    /// One value per triangle, readable later with `PhysicsWorld.triangleUserData(of:at:)`. Empty stores none.
    public var triangleUserData: [UInt32] = []

    public init(
        activeEdgeThreshold: Float = 5 * .pi / 180,
        maxTrianglesPerLeaf: Int = 8,
        buildQuality: BuildQuality = .favorRuntimePerformance,
        triangleUserData: [UInt32] = []
    ) {
        precondition((1...8).contains(maxTrianglesPerLeaf), "maxTrianglesPerLeaf must be in 1...8")
        self.activeEdgeThreshold = activeEdgeThreshold
        self.maxTrianglesPerLeaf = maxTrianglesPerLeaf
        self.buildQuality = buildQuality
        self.triangleUserData = triangleUserData
    }

    func apply(to settings: OpaquePointer) {
        JPH_MeshShapeSettings_SetActiveEdgeCosThresholdAngle(settings, cos(activeEdgeThreshold))
        JPH_MeshShapeSettings_SetMaxTrianglesPerLeaf(settings, UInt32(maxTrianglesPerLeaf))
        JPH_MeshShapeSettings_SetBuildQuality(
            settings,
            buildQuality == .favorBuildSpeed ? JPH_Mesh_Shape_BuildQuality_FavorBuildSpeed : JPH_Mesh_Shape_BuildQuality_FavorRuntimePerformance
        )
        JPH_MeshShapeSettings_SetPerTriangleUserData(settings, !triangleUserData.isEmpty)
    }
}

/// Build options for `Shape.heightField`. Defaults match Jolt's.
public struct HeightFieldOptions: Sendable, Equatable {
    /// Samples per block side in Jolt's internal hierarchy: 2, 4 or 8. `sampleCount` must be a multiple of it.
    public var blockSize = 2
    /// Bits used to store each sample, 1...8. Fewer bits use less memory but lose precision.
    public var bitsPerSample = 8
    /// See `MeshOptions.activeEdgeThreshold`.
    public var activeEdgeThreshold: Float = 5 * .pi / 180
    /// Fixes the quantization range instead of using the samples' range, e.g. so terrain tiles
    /// share one range. In sample units, before `scale`.
    public var heightRange: ClosedRange<Float>?

    public init(blockSize: Int = 2, bitsPerSample: Int = 8, activeEdgeThreshold: Float = 5 * .pi / 180, heightRange: ClosedRange<Float>? = nil) {
        precondition([2, 4, 8].contains(blockSize), "blockSize must be 2, 4 or 8")
        precondition((1...8).contains(bitsPerSample), "bitsPerSample must be in 1...8")
        self.blockSize = blockSize
        self.bitsPerSample = bitsPerSample
        self.activeEdgeThreshold = activeEdgeThreshold
        self.heightRange = heightRange
    }

    func apply(to settings: OpaquePointer) {
        JPH_HeightFieldShapeSettings_SetBlockSize(settings, UInt32(blockSize))
        JPH_HeightFieldShapeSettings_SetBitsPerSample(settings, UInt32(bitsPerSample))
        JPH_HeightFieldShapeSettings_SetActiveEdgeCosThresholdAngle(settings, cos(activeEdgeThreshold))
        if let heightRange {
            JPH_HeightFieldShapeSettings_SetMinHeightValue(settings, heightRange.lowerBound)
            JPH_HeightFieldShapeSettings_SetMaxHeightValue(settings, heightRange.upperBound)
        }
    }

    /// The fewest bits per sample that keep every sample within `maxError` (in sample units) of its true height.
    public static func bitsPerSample(
        forMaxError maxError: Float,
        samples: [Float],
        sampleCount: Int,
        blockSize: Int = 2
    ) -> Int {
        precondition(samples.count == sampleCount * sampleCount, "Height field needs sampleCount * sampleCount samples")
        // Jolt must be initialized while the settings exist.
        guard let engine = try? JoltEngine() else {
            preconditionFailure("Could not initialize Jolt")
        }
        defer { withExtendedLifetime(engine) {} }
        var offset = JPH_Vec3(.zero)
        var scale = JPH_Vec3(.one)
        let settings = JPH_HeightFieldShapeSettings_Create(samples, &offset, &scale, UInt32(sampleCount), nil)!
        defer { JPH_ShapeSettings_Destroy(settings) }
        JPH_HeightFieldShapeSettings_SetBlockSize(settings, UInt32(blockSize))
        return Int(JPH_HeightFieldShapeSettings_CalculateBitsPerSampleForError(settings, maxError))
    }
}

extension PhysicsWorld {
    /// The user data of the mesh triangle at `subShape` (from a ray hit or contact), or nil when
    /// that part of the body is not a mesh built with `MeshOptions.triangleUserData`.
    public func triangleUserData(of body: BodyID, at subShape: SubShapeID) -> UInt32? {
        guard let shape = JPH_BodyInterface_GetShape(bodyInterface, body.rawValue) else {
            return nil
        }
        var remainder = JPH_SubShapeID()
        guard let leaf = JPH_Shape_GetLeafShape(shape, subShape.rawValue, &remainder),
              JPH_Shape_GetSubType(leaf) == JPH_ShapeSubType_Mesh else {
            return nil
        }
        return JPH_MeshShape_GetTriangleUserData(leaf, remainder)
    }
}
