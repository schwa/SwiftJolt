// swift-tools-version: 6.0

import PackageDescription

let package = Package(
    name: "SwiftJolt",
    platforms: [
        .macOS(.v15),
        .iOS(.v18),
        .visionOS(.v2)
    ],
    products: [
        .library(name: "JoltPhysics", targets: ["JoltPhysics"])
    ],
    targets: [
        .binaryTarget(
            name: "CJolt",
            path: "Frameworks/CJolt.xcframework"
        ),
        .target(
            name: "JoltPhysics",
            dependencies: ["CJolt"],
            linkerSettings: [
                .linkedLibrary("c++")
            ]
        ),
        .executableTarget(
            name: "Example",
            dependencies: ["CJolt", "JoltPhysics"]
        ),
        .testTarget(
            name: "JoltPhysicsTests",
            dependencies: ["CJolt", "JoltPhysics"]
        )
    ]
)
