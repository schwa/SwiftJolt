import JoltPhysics
import Testing

func makeCloth(size: Int = 5, spacing: Float = 0.5) -> SoftBodySharedSettings {
    let settings = SoftBodySharedSettings()
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
        @Test func clothGridStoresVertices() {
            let cloth = makeCloth()
            #expect(cloth.vertexCount == 25)
            #expect(cloth.vertexPosition(at: 0) == [0, 0, 0])
            #expect(cloth.vertexPosition(at: 24) == [2, 0, 2])
        }

        @Test func pinAndCreateConstraints() {
            let cloth = makeCloth()
            cloth.pinVertex(0)
            cloth.pinVertex(4)
            cloth.createConstraints(bendType: .dihedral)
            cloth.optimize()
            #expect(cloth.vertexCount == 25)
        }

        @Test func addVerticesInBulk() {
            let settings = SoftBodySharedSettings()
            settings.addVertices([[0, 0, 0], [1, 0, 0], [0, 0, 1]])
            settings.addFace(0, 2, 1)
            #expect(settings.vertexCount == 3)
            #expect(settings.vertexPosition(at: 1) == [1, 0, 0])
        }
    }
}
