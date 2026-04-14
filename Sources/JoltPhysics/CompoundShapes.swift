import CJolt
import simd

/// A shape placed inside a compound shape, relative to the compound's origin.
public struct CompoundChild: Sendable {
    public var shape: Shape
    public var position: SIMD3<Float>
    public var rotation: simd_quatf

    public init(_ shape: Shape, position: SIMD3<Float> = .zero, rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)) {
        self.shape = shape
        self.position = position
        self.rotation = rotation
    }

    func add(to compoundSettings: OpaquePointer) {
        var p = JPH_Vec3(position)
        var r = JPH_Quat(rotation)
        let child = shape.createHandle()
        defer { JPH_Shape_Destroy(child) }
        JPH_CompoundShapeSettings_AddShape2(compoundSettings, &p, &r, child, 0)
    }
}

extension PhysicsWorld {
    public func compoundChildCount(of body: BodyID) -> Int {
        let shape = JPH_BodyInterface_GetShape(bodyInterface, body.rawValue)!
        let subType = JPH_Shape_GetSubType(shape)
        precondition(
            subType == JPH_ShapeSubType_StaticCompound || subType == JPH_ShapeSubType_MutableCompound,
            "Body \(body.rawValue) does not have a compound shape"
        )
        return Int(JPH_CompoundShape_GetNumSubShapes(shape))
    }

    /// Adds a child to a body with a `.mutableCompound` shape. Returns the child's index.
    @discardableResult
    public func addCompoundChild(_ child: CompoundChild, to body: BodyID) -> Int {
        modifyMutableCompound(of: body) { shape in
            var p = JPH_Vec3(child.position)
            var r = JPH_Quat(child.rotation)
            let childHandle = child.shape.createHandle()
            defer { JPH_Shape_Destroy(childHandle) }
            return Int(JPH_MutableCompoundShape_AddShape(shape, &p, &r, childHandle, 0, .max))
        }
    }

    /// Removes a child from a body with a `.mutableCompound` shape. Later children move down one index.
    public func removeCompoundChild(at index: Int, from body: BodyID) {
        modifyMutableCompound(of: body) { shape in
            precondition(index >= 0 && index < Int(JPH_CompoundShape_GetNumSubShapes(shape)), "Compound child index out of range")
            JPH_MutableCompoundShape_RemoveShape(shape, UInt32(index))
        }
    }

    /// Moves a child of a body with a `.mutableCompound` shape.
    public func moveCompoundChild(at index: Int, of body: BodyID, to position: SIMD3<Float>, rotation: simd_quatf) {
        modifyMutableCompound(of: body) { shape in
            precondition(index >= 0 && index < Int(JPH_CompoundShape_GetNumSubShapes(shape)), "Compound child index out of range")
            var p = JPH_Vec3(position)
            var r = JPH_Quat(rotation)
            JPH_MutableCompoundShape_ModifyShape(shape, UInt32(index), &p, &r)
        }
    }

    private func modifyMutableCompound<Result>(of body: BodyID, _ action: (OpaquePointer) -> Result) -> Result {
        let shape = JPH_BodyInterface_GetShape(bodyInterface, body.rawValue)!
        precondition(
            JPH_Shape_GetSubType(shape) == JPH_ShapeSubType_MutableCompound,
            "Body \(body.rawValue) does not have a mutable compound shape"
        )
        var previousCenterOfMass = JPH_Vec3()
        JPH_Shape_GetCenterOfMass(shape, &previousCenterOfMass)
        let result = action(shape)
        JPH_MutableCompoundShape_AdjustCenterOfMass(shape)
        JPH_BodyInterface_NotifyShapeChanged(bodyInterface, body.rawValue, &previousCenterOfMass, true, JPH_Activation_Activate)
        return result
    }
}
