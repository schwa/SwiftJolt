import JoltPhysics
import Testing

extension JoltTests {
    @Suite
    struct UserDataTests {
        @Test func bodyUserData() throws {
            let world = try makeWorld()
            let plain = world.addBody(.sphere(radius: 1))
            #expect(world.userData(of: plain) == 0)
            let body = world.addBody(.sphere(radius: 1), options: BodyOptions(userData: 42))
            #expect(world.userData(of: body) == 42)
            world.setUserData(of: body, to: .max)
            #expect(world.userData(of: body) == .max)
            world.writeBody(body) { $0.userData = 7 }
            #expect(world.readBody(body) { $0.userData } == 7)
        }

        @Test func softBodyUserData() throws {
            let world = try makeWorld()
            let cloth = try makeCloth(engine: world.engine)
            cloth.createConstraints()
            var options = SoftBodyOptions()
            options.userData = 99
            let body = world.addSoftBody(cloth, options: options)
            #expect(world.userData(of: body) == 99)
        }

        @Test func constraintUserData() throws {
            let world = try makeWorld()
            let bob = world.addBody(.sphere(radius: 0.5), position: [0, -1, 0])
            let constraint = world.addPointConstraint(between: nil, and: bob, point: .zero)
            #expect(constraint.userData == 0)
            constraint.userData = 5
            #expect(constraint.userData == 5)
        }

        @Test func virtualCharacterUserData() throws {
            let world = try makeWorld()
            let character = world.addVirtualCharacter(shape: .capsule(halfHeightOfCylinder: 0.5, radius: 0.4), position: [0, 1, 0])
            character.userData = 11
            #expect(character.userData == 11)
        }
    }
}
