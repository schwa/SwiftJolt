import CJolt
@testable import JoltPhysics
import Testing

/// Reads a Jolt RefTarget's reference count. Relies on Jolt's layout: a vtable pointer followed by
/// a 32-bit count at offset 8 (seen in the disassembly of joltc's *_Create functions).
func referenceCount(of shape: OpaquePointer) -> UInt32 {
    UnsafeRawPointer(shape).load(fromByteOffset: 8, as: UInt32.self)
}

extension JoltTests {
    @Suite
    struct ShapeOwnershipTests {
        @Test func bodyHoldsTheOnlyShapeReference() throws {
            let world = try makeWorld()
            let body = world.addBody(.sphere(radius: 1))
            let shape = JPH_BodyInterface_GetShape(world.bodyInterface, body.rawValue)!
            #expect(referenceCount(of: shape) == 1)
        }

        @Test func compoundHoldsTheOnlyChildReferences() throws {
            let world = try makeWorld()
            let children = [CompoundChild(.sphere(radius: 1)), CompoundChild(.box(halfExtents: [1, 1, 1]), position: [3, 0, 0])]
            for shape in [Shape.compound(children), .mutableCompound(children)] {
                let body = world.addBody(shape)
                let compound = JPH_BodyInterface_GetShape(world.bodyInterface, body.rawValue)!
                #expect(referenceCount(of: compound) == 1)
                for index in 0..<2 {
                    var child: OpaquePointer?
                    JPH_CompoundShape_GetSubShape(compound, UInt32(index), &child, nil, nil, nil)
                    #expect(referenceCount(of: child!) == 1, "child \(index) of \(shape)")
                }
            }
        }

        @Test func addedMutableCompoundChildHasOneReference() throws {
            let world = try makeWorld()
            let body = world.addBody(.mutableCompound([CompoundChild(.sphere(radius: 1))]))
            world.addCompoundChild(CompoundChild(.sphere(radius: 1), position: [3, 0, 0]), to: body)
            let compound = JPH_BodyInterface_GetShape(world.bodyInterface, body.rawValue)!
            var child: OpaquePointer?
            JPH_CompoundShape_GetSubShape(compound, 1, &child, nil, nil, nil)
            #expect(referenceCount(of: child!) == 1)
        }

        @Test func shapesSurviveRemovalChurn() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [20, 0.5, 20]), position: [0, -0.5, 0], motionType: .static)
            let children = [CompoundChild(.sphere(radius: 0.3)), CompoundChild(.box(halfExtents: [0.2, 0.2, 0.2]), position: [0.5, 0, 0])]
            for round in 0..<20 {
                var bodies: [BodyID] = []
                for index in 0..<10 {
                    let shape: Shape = index.isMultiple(of: 2) ? .compound(children) : .mutableCompound(children)
                    bodies.append(world.addBody(shape, position: [Float(index) - 5, 2, 0]))
                }
                world.addCompoundChild(CompoundChild(.capsule(halfHeightOfCylinder: 0.2, radius: 0.1)), to: bodies[1])
                var character: Character? = world.addCharacter(shape: .capsule(halfHeightOfCylinder: 0.5, radius: 0.3), position: [0, 1, 5])
                let virtualCharacter = world.addVirtualCharacter(shape: .sphere(radius: 0.3), position: [3, 1, 5])
                simulate(world, seconds: 0.1)
                virtualCharacter.update(deltaTime: 1.0 / 60.0)
                for body in bodies where !body.rawValue.isMultiple(of: 3) || round.isMultiple(of: 2) {
                    world.removeBody(body)
                }
                character = nil
                _ = character
            }
            simulate(world, seconds: 0.5)
        }

        @Test func charactersHoldTheOnlyShapeReference() throws {
            let world = try makeWorld()
            let shape = Shape.capsule(halfHeightOfCylinder: 0.5, radius: 0.4)
            let character = world.addCharacter(shape: shape, position: [0, 1, 0])
            // The character and its body each hold one.
            #expect(referenceCount(of: JPH_CharacterBase_GetShape(character.handle)!) == 2)
            let virtualCharacter = world.addVirtualCharacter(shape: shape, position: [5, 1, 0])
            #expect(referenceCount(of: JPH_CharacterBase_GetShape(virtualCharacter.handle)!) == 1)
        }
    }
}
