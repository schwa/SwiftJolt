import JoltPhysics
import Testing

func makeCloth(engine: JoltEngine? = nil, size: Int = 5, spacing: Float = 0.5) throws -> SoftBodySharedSettings {
    let settings = SoftBodySharedSettings(engine: try engine ?? JoltEngine())
    for z in 0..<size {
        for x in 0..<size {
            settings.addVertex([Float(x) * spacing, 0, Float(z) * spacing])
        }
    }
    for z in 0..<(size - 1) {
        for x in 0..<(size - 1) {
            let index = z * size + x
            settings.addFace(index, index + size, index + 1)
            settings.addFace(index + 1, index + size, index + size + 1)
        }
    }
    return settings
}

extension JoltTests {
    @Suite
    struct SoftBodyTests {
        @Test func clothGridStoresVertices() throws {
            let cloth = try makeCloth()
            #expect(cloth.vertexCount == 25)
            #expect(cloth.vertexPosition(at: 0) == [0, 0, 0])
            #expect(cloth.vertexPosition(at: 24) == [2, 0, 2])
        }

        @Test func pinAndCreateConstraints() throws {
            let cloth = try makeCloth()
            cloth.pinVertex(0)
            cloth.pinVertex(4)
            cloth.createConstraints(bendType: .dihedral)
            cloth.optimize()
            #expect(cloth.vertexCount == 25)
        }

        @Test func addSoftBodyToWorld() throws {
            let world = try makeWorld()
            let cloth = try makeCloth(engine: world.engine)
            cloth.createConstraints()
            world.addSoftBody(cloth, position: [0, 5, 0])
            #expect(world.bodyCount == 1)
        }

        @Test func unpinnedClothFalls() throws {
            let world = try makeWorld()
            let cloth = try makeCloth(engine: world.engine)
            cloth.createConstraints()
            let body = world.addSoftBody(cloth, position: [0, 10, 0])
            simulate(world, seconds: 1)
            #expect(world.position(of: body).y < 7)
        }

        @Test func pinnedClothHangs() throws {
            let world = try makeWorld()
            let loose = try makeCloth(engine: world.engine)
            loose.createConstraints()
            let pinned = try makeCloth(engine: world.engine)
            pinned.pinVertex(0)
            pinned.pinVertex(4)
            pinned.createConstraints()
            let looseBody = world.addSoftBody(loose, position: [-10, 10, 0])
            let pinnedBody = world.addSoftBody(pinned, position: [10, 10, 0])
            simulate(world, seconds: 1)
            let pinnedY = world.position(of: pinnedBody).y
            #expect(pinnedY < 10)
            #expect(pinnedY > 8)
            #expect(pinnedY > world.position(of: looseBody).y + 2)
        }

        @Test func worldKeepsSettingsAlive() throws {
            let world = try makeWorld()
            weak var weakCloth: SoftBodySharedSettings?
            let body: BodyID = try {
                let cloth = try makeCloth(engine: world.engine)
                cloth.createConstraints()
                weakCloth = cloth
                return world.addSoftBody(cloth, position: [0, 10, 0])
            }()
            simulate(world, seconds: 0.5)
            #expect(weakCloth != nil)
            world.removeBody(body)
            #expect(weakCloth == nil)
        }

        @Test func settingsLockedOnceInUse() throws {
            let world = try makeWorld()
            let cloth = try makeCloth(engine: world.engine)
            cloth.createConstraints()
            #expect(!cloth.isOptimized)
            world.addSoftBody(cloth)
            #expect(cloth.isOptimized)
            #expect(cloth.isInUse)
        }

        @Test func optionsApply() throws {
            let world = try makeWorld()
            let cloth = try makeCloth(engine: world.engine)
            cloth.createConstraints()
            var options = SoftBodyOptions()
            options.gravityFactor = 0
            let body = world.addSoftBody(cloth, position: [0, 10, 0], options: options)
            simulate(world, seconds: 1)
            #expect(abs(world.position(of: body).y - 10) < 0.5)
        }

        @Test func addVerticesInBulk() throws {
            let settings = SoftBodySharedSettings(engine: try JoltEngine())
            settings.addVertices([[0, 0, 0], [1, 0, 0], [0, 0, 1]])
            settings.addFace(0, 2, 1)
            #expect(settings.vertexCount == 3)
            #expect(settings.vertexPosition(at: 1) == [1, 0, 0])
        }
    }
}
