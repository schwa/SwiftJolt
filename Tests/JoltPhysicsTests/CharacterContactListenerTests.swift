import JoltPhysics
import simd
import Testing

final class RecordingCharacterListener: CharacterContactListener {
    var added: [BodyID] = []
    var persisted = 0
    var removed: [BodyID] = []
    var characterAdded: [CharacterID] = []
    var characterRemoved: [CharacterID] = []
    var solved = 0
    var ignore: BodyID?
    var conveyor: (body: BodyID, velocity: SIMD3<Float>)?

    func contactValidate(_: VirtualCharacter, body: BodyID) -> Bool {
        body != ignore
    }

    func adjustBodyVelocity(of body: BodyID, for _: VirtualCharacter, linear: inout SIMD3<Float>, angular _: inout SIMD3<Float>) {
        if let conveyor, conveyor.body == body {
            linear = conveyor.velocity
        }
    }

    func contactAdded(_: VirtualCharacter, body: BodyID, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {
        added.append(body)
    }

    func contactPersisted(_: VirtualCharacter, body _: BodyID, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {
        persisted += 1
    }

    func contactRemoved(_: VirtualCharacter, body: BodyID) {
        removed.append(body)
    }

    func characterContactAdded(_: VirtualCharacter, other: VirtualCharacter, position _: SIMD3<Float>, normal _: SIMD3<Float>, settings _: inout CharacterContactSettings) {
        characterAdded.append(other.id)
    }

    func characterContactRemoved(_: VirtualCharacter, other: CharacterID) {
        characterRemoved.append(other)
    }

    func contactSolve(_: VirtualCharacter, body _: BodyID, position _: SIMD3<Float>, normal _: SIMD3<Float>, contactVelocity _: SIMD3<Float>, characterVelocity _: SIMD3<Float>, newVelocity _: inout SIMD3<Float>) {
        solved += 1
    }
}

extension JoltTests {
    @Suite
    struct CharacterContactListenerTests {
        let capsule = Shape.capsule(halfHeightOfCylinder: 0.5, radius: 0.4)
        let deltaTime: Float = 1.0 / 60.0

        func step(_ characters: [VirtualCharacter], velocity: SIMD3<Float>, frames: Int) {
            for _ in 0..<frames {
                for character in characters {
                    var newVelocity = velocity
                    newVelocity.y = character.isSupported ? 0 : character.linearVelocity.y - 9.81 * deltaTime
                    character.linearVelocity = newVelocity
                    character.update(deltaTime: deltaTime)
                }
            }
        }

        @Test func reportsBodyContacts() throws {
            let world = try makeWorld()
            let floor = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let wall = world.addBody(.box(halfExtents: [0.5, 2, 5]), position: [3, 2, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            let listener = RecordingCharacterListener()
            character.contactListener = listener
            step([character], velocity: [3, 0, 0], frames: 60)
            #expect(Set(listener.added) == [floor, wall])
            #expect(listener.persisted > 0)
            #expect(listener.solved > 0)
            step([character], velocity: [-3, 0, 0], frames: 30)
            #expect(listener.removed.contains(wall))
        }

        @Test func validateCanIgnoreBody() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let wall = world.addBody(.box(halfExtents: [0.5, 2, 5]), position: [3, 2, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            let listener = RecordingCharacterListener()
            listener.ignore = wall
            character.contactListener = listener
            step([character], velocity: [3, 0, 0], frames: 120)
            #expect(character.position.x > 4)
        }

        @Test func adjustBodyVelocityMakesConveyor() throws {
            let world = try makeWorld()
            let belt = world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            let listener = RecordingCharacterListener()
            listener.conveyor = (belt, [0, 0, 2])
            character.contactListener = listener
            for _ in 0..<60 {
                character.updateGroundVelocity()
                character.linearVelocity = character.isSupported ? character.groundVelocity : character.linearVelocity - [0, 9.81 * deltaTime, 0]
                character.update(deltaTime: deltaTime)
            }
            #expect(character.position.z > 1)
        }

        @Test func charactersCollideOnlyWhenEnabled() throws {
            func finalX(enabled: Bool) throws -> (Float, RecordingCharacterListener, CharacterID) {
                let world = try makeWorld()
                world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
                let mover = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
                let blocker = world.addVirtualCharacter(shape: capsule, position: [3, 0.9, 0])
                mover.collidesWithCharacters = enabled
                blocker.collidesWithCharacters = enabled
                let listener = RecordingCharacterListener()
                mover.contactListener = listener
                step([mover], velocity: [3, 0, 0], frames: 90)
                #expect(mover.hasCollided(with: blocker) == enabled)
                return (mover.position.x, listener, blocker.id)
            }
            let (blockedX, listener, blockerID) = try finalX(enabled: true)
            #expect(blockedX < 2.3)
            #expect(listener.characterAdded == [blockerID])
            let (freeX, freeListener, _) = try finalX(enabled: false)
            #expect(freeX > 4)
            #expect(freeListener.characterAdded.isEmpty)
        }

        @Test func disablingCharacterCollisionLetsCharacterPass() throws {
            let world = try makeWorld()
            world.addBody(.box(halfExtents: [50, 0.5, 50]), position: [0, -0.5, 0], motionType: .static)
            let mover = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            let blocker = world.addVirtualCharacter(shape: capsule, position: [3, 0.9, 0])
            mover.collidesWithCharacters = true
            blocker.collidesWithCharacters = true
            let listener = RecordingCharacterListener()
            mover.contactListener = listener
            step([mover], velocity: [3, 0, 0], frames: 60)
            #expect(mover.position.x < 2.3)
            mover.collidesWithCharacters = false
            step([mover], velocity: [3, 0, 0], frames: 60)
            #expect(mover.position.x > 4)
            #expect(listener.characterRemoved == [blocker.id])
        }

        @Test func listenerAndCharacterAreReleased() throws {
            weak var weakListener: RecordingCharacterListener?
            let world = try makeWorld()
            do {
                let character = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
                character.collidesWithCharacters = true
                do {
                    let replaced = RecordingCharacterListener()
                    weakListener = replaced
                    character.contactListener = replaced
                }
                character.contactListener = RecordingCharacterListener()
                #expect(weakListener == nil)
                let listener = RecordingCharacterListener()
                weakListener = listener
                character.contactListener = listener
            }
            #expect(weakListener == nil)
            // A new character still works after the old one left the collision set.
            let other = world.addVirtualCharacter(shape: capsule, position: [0, 0.9, 0])
            other.collidesWithCharacters = true
            other.update(deltaTime: deltaTime)
        }
    }
}
