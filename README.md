# SwiftJolt

Swift wrapper for [Jolt Physics](https://github.com/jrouwe/JoltPhysics), built on [JoltC](https://github.com/amerkoleci/joltc). Supports macOS 15+, iOS 18+ and visionOS 2+.

```swift
import JoltPhysics

let world = try PhysicsWorld(engine: JoltEngine())
let ball = world.addBody(.sphere(radius: 0.5), position: [0, 10, 0])
world.update(deltaTime: 1.0 / 60.0)
print(world.position(of: ball))
```

Rebuild `Frameworks/CJolt.xcframework` with `Scripts/build-xcframework.sh` (needs CMake). It clones JoltC and JoltPhysics at the commits pinned in `Scripts/jolt-pins.txt`; pass `--update` to move the pins to the latest upstream commits. The built commits are recorded in `Frameworks/CJolt-versions.txt`.

## License

MIT. See [LICENSE](LICENSE). The bundled xcframework contains MIT-licensed code from [Jolt Physics](https://github.com/jrouwe/JoltPhysics) (Copyright 2021 Jorrit Rouwe) and [JoltC](https://github.com/amerkoleci/joltc) (Copyright (c) Amer Koleci and Contributors).
