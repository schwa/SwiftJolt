#!/bin/bash
# build-xcframework.sh — Build JoltC xcframeworks for macOS, iOS, and visionOS
#
# Usage:
#   scripts/build-xcframework.sh [--update] [--output-dir <path>] [--jobs <n>]
#
# Sources are cloned at the commits pinned in Scripts/jolt-pins.txt.
# --update moves the pins to the latest commit on each pinned branch first.
#
# Prerequisites:
#   - CMake 3.16+
#   - Xcode with macOS, iOS, and visionOS SDKs

set -euo pipefail

# ---------- defaults ----------
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PINS_FILE="${REPO_ROOT}/Scripts/jolt-pins.txt"
SOURCE_ROOT="${REPO_ROOT}/.vendor"
UPDATE=0
OUTPUT_DIR="${REPO_ROOT}/Frameworks"
JOBS="$(sysctl -n hw.ncpu)"
BUILD_ROOT="${REPO_ROOT}/.xcframework-build"
DEPLOYMENT_TARGET_MACOS="15.0"
DEPLOYMENT_TARGET_IOS="18.0"
DEPLOYMENT_TARGET_VISIONOS="2.0"

# ---------- parse args ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --update)     UPDATE=1; shift ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --jobs)       JOBS="$2"; shift 2 ;;
        *)            echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ---------- fetch pinned sources ----------
# JoltPhysics is cloned beside JoltC, where JoltC's CMakeLists.txt looks for it.
PINS_TEMP="$(mktemp)"
trap 'rm -f "$PINS_TEMP"' EXIT

while read -r name url branch commit; do
    [[ -z "$name" || "$name" == \#* ]] && { echo "$name $url $branch $commit" | sed 's/ *$//' >> "$PINS_TEMP"; continue; }
    dir="${SOURCE_ROOT}/${name}"
    if [[ ! -d "$dir/.git" ]]; then
        git clone --quiet "$url" "$dir"
    fi
    git -C "$dir" fetch --quiet origin "$branch"
    if [[ $UPDATE -eq 1 ]]; then
        commit="$(git -C "$dir" rev-parse FETCH_HEAD)"
    elif ! git -C "$dir" cat-file -e "${commit}^{commit}" 2>/dev/null; then
        git -C "$dir" fetch --quiet origin "$commit"
    fi
    git -C "$dir" checkout --quiet --force --detach "$commit"
    git -C "$dir" clean --quiet -fdx
    echo "$name $url $branch $commit" >> "$PINS_TEMP"
    echo "$name @ $(git -C "$dir" describe --tags --always)"
done < "$PINS_FILE"

if [[ $UPDATE -eq 1 ]]; then
    cp "$PINS_TEMP" "$PINS_FILE"
fi

JOLTC_DIR="${SOURCE_ROOT}/JoltC"
JOLT_DIR="${SOURCE_ROOT}/JoltPhysics"

# ---------- common cmake flags ----------
COMMON_FLAGS=(
    -G "Unix Makefiles"
    -DJPH_BUILD_SHARED=OFF
    -DJPH_SAMPLES=OFF
    -DJPH_TESTS=OFF
    -DJPH_INSTALL=OFF
    -DCPP_EXCEPTIONS_ENABLED=OFF
    -DCPP_RTTI_ENABLED=OFF
    -DCMAKE_BUILD_TYPE=Release
    -DOVERRIDE_CXX_FLAGS=ON
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
    -DINTERPROCEDURAL_OPTIMIZATION=OFF
)

# ---------- helper: build one slice ----------
build_slice() {
    local name="$1"
    local build_dir="${BUILD_ROOT}/${name}"
    shift
    local extra_flags=("$@")

    echo "━━━ Building: ${name} ━━━"
    cmake -S "$JOLTC_DIR" -B "$build_dir" "${COMMON_FLAGS[@]}" "${extra_flags[@]}"
    cmake --build "$build_dir" --config Release -j"$JOBS"

    # Merge Jolt + JoltC into one static lib
    libtool -static -o "${build_dir}/libjoltc.a" \
        "${build_dir}/lib/libjoltc.a" \
        "${build_dir}/lib/libJolt.a"

    echo "  → ${build_dir}/libjoltc.a"
}

# ---------- helper: create per-platform fat lib + headers dir ----------
prepare_platform() {
    local platform_name="$1"
    local platform_dir="${BUILD_ROOT}/platforms/${platform_name}"
    shift
    local slice_dirs=("$@")

    mkdir -p "${platform_dir}"

    if [[ ${#slice_dirs[@]} -eq 1 ]]; then
        cp "${slice_dirs[0]}/libjoltc.a" "${platform_dir}/libjoltc.a"
    else
        # lipo multiple architectures together
        local lipo_args=()
        for d in "${slice_dirs[@]}"; do
            lipo_args+=("${d}/libjoltc.a")
        done
        lipo -create "${lipo_args[@]}" -output "${platform_dir}/libjoltc.a"
    fi

    # Copy headers
    mkdir -p "${platform_dir}/Headers"
    cp "$JOLTC_DIR/include/joltc.h" "${platform_dir}/Headers/"

    # Create module map
    cat > "${platform_dir}/Headers/module.modulemap" <<'EOF'
module CJolt {
    header "joltc.h"
    link "c++"
    export *
}
EOF

    echo "  → ${platform_dir}/libjoltc.a ($(lipo -archs "${platform_dir}/libjoltc.a"))"
}

# ---------- clean ----------
echo "Cleaning previous build..."
rm -rf "$BUILD_ROOT"
mkdir -p "$BUILD_ROOT"

# ==========================================================================
# macOS — arm64 + x86_64 (built in one cmake invocation)
# ==========================================================================
build_slice "macos" \
    -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET_MACOS" \
    -DUSE_SSE4_1=OFF -DUSE_SSE4_2=OFF -DUSE_AVX=OFF -DUSE_AVX2=OFF \
    -DUSE_AVX512=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DUSE_F16C=OFF -DUSE_FMADD=OFF

prepare_platform "macos" \
    "${BUILD_ROOT}/macos"

# ==========================================================================
# iOS device — arm64
# ==========================================================================
build_slice "ios-arm64" \
    -DCMAKE_SYSTEM_NAME=iOS \
    -DCMAKE_OSX_ARCHITECTURES="arm64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET_IOS" \
    -DUSE_SSE4_1=OFF -DUSE_SSE4_2=OFF -DUSE_AVX=OFF -DUSE_AVX2=OFF \
    -DUSE_AVX512=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DUSE_F16C=OFF -DUSE_FMADD=OFF

prepare_platform "ios-device" \
    "${BUILD_ROOT}/ios-arm64"

# ==========================================================================
# iOS Simulator — arm64 + x86_64 (built in one cmake invocation)
# ==========================================================================
build_slice "ios-simulator" \
    -DCMAKE_SYSTEM_NAME=iOS \
    -DCMAKE_OSX_SYSROOT=iphonesimulator \
    -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET_IOS" \
    -DUSE_SSE4_1=OFF -DUSE_SSE4_2=OFF -DUSE_AVX=OFF -DUSE_AVX2=OFF \
    -DUSE_AVX512=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DUSE_F16C=OFF -DUSE_FMADD=OFF

prepare_platform "ios-simulator" \
    "${BUILD_ROOT}/ios-simulator"

# ==========================================================================
# visionOS device — arm64
# ==========================================================================
build_slice "visionos-arm64" \
    -DCMAKE_SYSTEM_NAME=visionOS \
    -DCMAKE_OSX_ARCHITECTURES="arm64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET_VISIONOS" \
    -DUSE_SSE4_1=OFF -DUSE_SSE4_2=OFF -DUSE_AVX=OFF -DUSE_AVX2=OFF \
    -DUSE_AVX512=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DUSE_F16C=OFF -DUSE_FMADD=OFF

prepare_platform "visionos-device" \
    "${BUILD_ROOT}/visionos-arm64"

# ==========================================================================
# visionOS Simulator — arm64
# ==========================================================================
build_slice "visionos-sim-arm64" \
    -DCMAKE_SYSTEM_NAME=visionOS \
    -DCMAKE_OSX_SYSROOT=xrsimulator \
    -DCMAKE_OSX_ARCHITECTURES="arm64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET_VISIONOS" \
    -DUSE_SSE4_1=OFF -DUSE_SSE4_2=OFF -DUSE_AVX=OFF -DUSE_AVX2=OFF \
    -DUSE_AVX512=OFF -DUSE_LZCNT=OFF -DUSE_TZCNT=OFF -DUSE_F16C=OFF -DUSE_FMADD=OFF

prepare_platform "visionos-simulator" \
    "${BUILD_ROOT}/visionos-sim-arm64"

# ==========================================================================
# Assemble xcframework
# ==========================================================================
echo ""
echo "━━━ Creating xcframework ━━━"

XCFW_PATH="${OUTPUT_DIR}/CJolt.xcframework"
rm -rf "$XCFW_PATH"
mkdir -p "$OUTPUT_DIR"

xcodebuild -create-xcframework \
    -library "${BUILD_ROOT}/platforms/macos/libjoltc.a" \
    -headers "${BUILD_ROOT}/platforms/macos/Headers" \
    -library "${BUILD_ROOT}/platforms/ios-device/libjoltc.a" \
    -headers "${BUILD_ROOT}/platforms/ios-device/Headers" \
    -library "${BUILD_ROOT}/platforms/ios-simulator/libjoltc.a" \
    -headers "${BUILD_ROOT}/platforms/ios-simulator/Headers" \
    -library "${BUILD_ROOT}/platforms/visionos-device/libjoltc.a" \
    -headers "${BUILD_ROOT}/platforms/visionos-device/Headers" \
    -library "${BUILD_ROOT}/platforms/visionos-simulator/libjoltc.a" \
    -headers "${BUILD_ROOT}/platforms/visionos-simulator/Headers" \
    -output "$XCFW_PATH"

# ---------- record source versions ----------
describe_repo() {
    echo "$(git -C "$1" rev-parse HEAD) $(git -C "$1" describe --tags --always)"
}

VERSIONS_FILE="${OUTPUT_DIR}/CJolt-versions.txt"
{
    echo "built: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "JoltC: $(describe_repo "$JOLTC_DIR")"
    echo "JoltPhysics: $(describe_repo "$JOLT_DIR")"
} > "$VERSIONS_FILE"
cat "$VERSIONS_FILE"

echo ""
echo "✅ xcframework created at: $XCFW_PATH"
echo ""
# Show structure
find "$XCFW_PATH" -maxdepth 3 -not -name '.*' | sort
echo ""
echo "Done! Update Package.swift to reference this xcframework with .binaryTarget"
