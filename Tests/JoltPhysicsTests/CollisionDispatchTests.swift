import JoltPhysics
import simd
import Testing

extension JoltTests {
    @Suite
    struct CollisionDispatchTests {
        @Test func overlappingSpheresCollide() throws {
            let contacts = Shape.sphere(radius: 1).collide(with: .sphere(radius: 1), otherPosition: [1.5, 0, 0])
            #expect(contacts.count == 1)
            let contact = try #require(contacts.first)
            #expect(abs(contact.penetrationDepth - 0.5) < 1e-3)
            #expect(abs(contact.pointOn1.x - 1) < 1e-3)
            #expect(abs(contact.pointOn2.x - 0.5) < 1e-3)
        }

        @Test func separatedSpheresDoNotCollide() {
            #expect(Shape.sphere(radius: 1).collide(with: .sphere(radius: 1), otherPosition: [3, 0, 0]).isEmpty)
            let near = Shape.sphere(radius: 1).collide(with: .sphere(radius: 1), otherPosition: [2.5, 0, 0], maxSeparationDistance: 1)
            #expect(near.count == 1)
            #expect(near.first.map { $0.penetrationDepth < 0 } == true)
        }

        @Test func collideUsesShapeOrigin() {
            // The center of mass is moved, but the collision geometry stays at the origin.
            let shifted = Shape.offsetCenterOfMass(.sphere(radius: 1), offset: [5, 0, 0])
            #expect(shifted.collide(with: .sphere(radius: 1), otherPosition: [1.5, 0, 0]).count == 1)
        }

        @Test func castHitsAlongSweep() throws {
            let hits = Shape.sphere(radius: 1).cast(direction: [10, 0, 0], against: .box(halfExtents: [1, 1, 1]), otherPosition: [6, 0, 0])
            let hit = try #require(hits.first)
            // The sphere surface meets the box face at x = 5 after moving 4 of 10 units.
            #expect(abs(hit.fraction - 0.4) < 1e-2)
            #expect(abs(hit.pointOn2.x - 5) < 1e-2)
        }

        @Test func castMissesWhenSweepFallsShort() {
            #expect(Shape.sphere(radius: 1).cast(direction: [2, 0, 0], against: .box(halfExtents: [1, 1, 1]), otherPosition: [6, 0, 0]).isEmpty)
        }
    }
}
