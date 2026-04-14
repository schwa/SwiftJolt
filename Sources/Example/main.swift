import Foundation
import JoltPhysics

let engine = try JoltEngine()
let world = PhysicsWorld(engine: engine)

// Ground: big flat box
let ground = world.addBody(
    .box(halfExtents: [50, 0.5, 50]),
    position: [0, -0.5, 0],
    motionType: .static,
    restitution: 0.5,
    friction: 0.5
)
_ = ground

// Stack of boxes — 3 rows, angry-birds style
var boxes: [BodyID] = []
for row in 0..<3 {
    for col in 0..<(3 - row) {
        let x = Float(col) - Float(2 - row) / 2.0
        let y = Float(row) * 1.05 + 0.5
        let body = world.addBody(
            .box(halfExtents: [0.5, 0.5, 0.5]),
            position: [x, y, 0],
            motionType: .dynamic,
            restitution: 0.4,
            friction: 0.3
        )
        boxes.append(body)
    }
}

// Projectile sphere
let ball = world.addBody(
    .sphere(radius: 0.3),
    position: [-5, 1.5, 0],
    motionType: .dynamic,
    restitution: 0.6,
    friction: 0.3
)
world.setLinearVelocity(of: ball, to: [15, 5, 0])

world.optimizeBroadPhase()

func fmt(_ v: Float) -> String {
    v.formatted(.number.precision(.fractionLength(2)))
}

func fmtPos(_ p: SIMD3<Float>) -> String {
    "(\(fmt(p.x)), \(fmt(p.y)), \(fmt(p.z)))"
}

print("Bodies: \(world.bodyCount), Active: \(world.activeBodyCount)")
print("Gravity: \(world.gravity)")
print()

// Simulate 2 seconds at 60fps
let steps = 120
let dt: Float = 1.0 / 60.0

for step in 0..<steps {
    world.update(deltaTime: dt)

    if step % 20 == 0 {
        let t = Float(step) * dt
        let ballPos = world.position(of: ball)
        print("t=\(fmt(t))  ball=\(fmtPos(ballPos))  active=\(world.activeBodyCount)")
    }
}

print()
print("Final positions:")
for (i, box) in boxes.enumerated() {
    let p = world.position(of: box)
    let v = world.linearVelocity(of: box)
    let speed = (v.x * v.x + v.y * v.y + v.z * v.z).squareRoot()
    print("  box[\(i)]: pos=\(fmtPos(p))  speed=\(fmt(speed))")
}

let ballPos = world.position(of: ball)
print("  ball:   pos=\(fmtPos(ballPos))")

print("\nDone.")
