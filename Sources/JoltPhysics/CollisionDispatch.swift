import CJolt
import simd

/// A contact between two lone shapes, from `Shape.collide(with:...)`. Positions are in world space.
public struct ShapePairContact: Sendable {
    public var pointOn1: SIMD3<Float>
    public var pointOn2: SIMD3<Float>
    /// Direction to move shape 2 out of collision along the shortest path.
    public var penetrationAxis: SIMD3<Float>
    /// Overlap depth. Negative when separated but within `maxSeparationDistance`.
    public var penetrationDepth: Float
    public var subShapeID1: SubShapeID
    public var subShapeID2: SubShapeID
}

/// A hit from sweeping one lone shape against another, from `Shape.cast(...)`. Positions are in world space.
public struct ShapePairCastHit: Sendable {
    /// How far along the sweep the hit happened, as a fraction of `direction`, in 0...1.
    public var fraction: Float
    public var pointOn1: SIMD3<Float>
    public var pointOn2: SIMD3<Float>
    public var penetrationAxis: SIMD3<Float>
    /// Non-zero when the shapes already overlapped at the start of the sweep.
    public var penetrationDepth: Float
    public var isBackFaceHit: Bool
    public var subShapeID1: SubShapeID
    public var subShapeID2: SubShapeID
}

extension Shape {
    /// Tests this shape against `other` without a physics world. Each shape is placed with its
    /// origin at the given position and rotation.
    public func collide(
        position: SIMD3<Float> = .zero,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        with other: Self,
        otherPosition: SIMD3<Float>,
        otherRotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        maxSeparationDistance: Float = 0
    ) -> [ShapePairContact] {
        withHandle { handle1, centerOfMass1 in
            other.withHandle { handle2, centerOfMass2 in
                var transform1 = JPH_Mat4(centerOfMassTransform(position, rotation, centerOfMass1))
                var transform2 = JPH_Mat4(centerOfMassTransform(otherPosition, otherRotation, centerOfMass2))
                var scale = JPH_Vec3(SIMD3<Float>.one)
                var settings = JPH_CollideShapeSettings()
                JPH_CollideShapeSettings_Init(&settings)
                settings.maxSeparationDistance = maxSeparationDistance
                var results: [ShapePairContact] = []
                withUnsafeMutablePointer(to: &results) { resultsPointer in
                    _ = JPH_CollisionDispatch_CollideShapeVsShape(
                        handle1, handle2, &scale, &scale, &transform1, &transform2, &settings,
                        collectShapePairContact, resultsPointer, nil
                    )
                }
                return results
            }
        }
    }

    /// Sweeps this shape along `direction` (whose length is the sweep distance) against `other`,
    /// without a physics world. Returns every hit.
    public func cast(
        position: SIMD3<Float> = .zero,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        direction: SIMD3<Float>,
        against other: Self,
        otherPosition: SIMD3<Float>,
        otherRotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    ) -> [ShapePairCastHit] {
        withHandle { handle1, centerOfMass1 in
            other.withHandle { handle2, centerOfMass2 in
                var transform1 = JPH_Mat4(centerOfMassTransform(position, rotation, centerOfMass1))
                var transform2 = JPH_Mat4(centerOfMassTransform(otherPosition, otherRotation, centerOfMass2))
                var scale = JPH_Vec3(SIMD3<Float>.one)
                var jphDirection = JPH_Vec3(direction)
                var settings = JPH_ShapeCastSettings()
                JPH_ShapeCastSettings_Init(&settings)
                var results: [ShapePairCastHit] = []
                withUnsafeMutablePointer(to: &results) { resultsPointer in
                    _ = JPH_CollisionDispatch_CastShapeVsShapeWorldSpace(
                        &jphDirection, handle1, handle2, &scale, &scale, &transform1, &transform2, &settings,
                        collectShapePairCastHit, resultsPointer, nil
                    )
                }
                return results.sorted { $0.fraction < $1.fraction }
            }
        }
    }
}

// Returning the largest float keeps Jolt collecting every hit.
private func collectShapePairContact(_ context: UnsafeMutableRawPointer?, _ result: UnsafePointer<JPH_CollideShapeResult>?) -> Float {
    guard let context, let result else {
        return .greatestFiniteMagnitude
    }
    var hit = result.pointee
    context.assumingMemoryBound(to: [ShapePairContact].self).pointee.append(ShapePairContact(
        pointOn1: hit.contactPointOn1.simd,
        pointOn2: hit.contactPointOn2.simd,
        penetrationAxis: hit.penetrationAxis.simd,
        penetrationDepth: hit.penetrationDepth,
        subShapeID1: SubShapeID(rawValue: hit.subShapeID1),
        subShapeID2: SubShapeID(rawValue: hit.subShapeID2)
    ))
    // joltc mallocs face arrays for each hit and does not free them.
    JPH_CollideShapeResult_FreeMembers(&hit)
    return .greatestFiniteMagnitude
}

private func collectShapePairCastHit(_ context: UnsafeMutableRawPointer?, _ result: UnsafePointer<JPH_ShapeCastResult>?) -> Float {
    guard let context, let hit = result?.pointee else {
        return .greatestFiniteMagnitude
    }
    context.assumingMemoryBound(to: [ShapePairCastHit].self).pointee.append(ShapePairCastHit(
        fraction: hit.fraction,
        pointOn1: hit.contactPointOn1.simd,
        pointOn2: hit.contactPointOn2.simd,
        penetrationAxis: hit.penetrationAxis.simd,
        penetrationDepth: hit.penetrationDepth,
        isBackFaceHit: hit.isBackFaceHit,
        subShapeID1: SubShapeID(rawValue: hit.subShapeID1),
        subShapeID2: SubShapeID(rawValue: hit.subShapeID2)
    ))
    return .greatestFiniteMagnitude
}

// Jolt places shapes by their center of mass, which is not always their origin.
private func centerOfMassTransform(_ position: SIMD3<Float>, _ rotation: simd_quatf, _ centerOfMass: SIMD3<Float>) -> simd_float4x4 {
    var transform = simd_float4x4(rotation)
    transform.columns.3 = SIMD4(position + rotation.act(centerOfMass), 1)
    return transform
}
