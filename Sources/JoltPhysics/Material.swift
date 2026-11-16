import CJolt
import simd

/// Surface and bulk properties applied to a body when it is added.
public struct Material: Sendable, Equatable {
    public var friction: Float
    public var restitution: Float
    /// Density in kg/m³. Jolt's shapes default to 1000 (water).
    public var density: Float

    public init(friction: Float = 0.2, restitution: Float = 0.3, density: Float = Self.defaultDensity) {
        precondition(density > 0, "Density must be positive")
        self.friction = friction
        self.restitution = restitution
        self.density = density
    }

    public static let defaultDensity: Float = 1000

    public static let `default` = Self()
    public static let wood = Self(friction: 0.5, restitution: 0.3, density: 700)
    public static let steel = Self(friction: 0.4, restitution: 0.2, density: 7850)
    public static let rubber = Self(friction: 0.9, restitution: 0.8, density: 1100)
    public static let ice = Self(friction: 0.02, restitution: 0.1, density: 917)
}

extension PhysicsWorld {
    /// Add a body using a material's friction, restitution, and density.
    @discardableResult
    public func addBody(
        _ shape: Shape,
        position: SIMD3<Float> = .zero,
        rotation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        motionType: MotionType = .dynamic,
        material: Material,
        options: BodyOptions = .default
    ) -> BodyID {
        let body = addBody(
            shape,
            position: position,
            rotation: rotation,
            motionType: motionType,
            restitution: material.restitution,
            friction: material.friction,
            options: options
        )
        // An explicit mass wins over density.
        if motionType == .dynamic, options.mass == nil, material.density != Material.defaultDensity {
            // Shapes are built at the default density, so scale the computed mass.
            setMass(of: body, to: mass(of: body) * material.density / Material.defaultDensity)
        }
        return body
    }

    public func friction(of body: BodyID) -> Float {
        JPH_BodyInterface_GetFriction(bodyInterface, body.rawValue)
    }

    public func setFriction(of body: BodyID, to friction: Float) {
        JPH_BodyInterface_SetFriction(bodyInterface, body.rawValue, friction)
    }

    public func restitution(of body: BodyID) -> Float {
        JPH_BodyInterface_GetRestitution(bodyInterface, body.rawValue)
    }

    public func setRestitution(of body: BodyID, to restitution: Float) {
        JPH_BodyInterface_SetRestitution(bodyInterface, body.rawValue, restitution)
    }
}
