#!/usr/bin/env bash
set -euo pipefail

BUILD_TYPE=${1:-Release}
BUILD_DIR="build"

cmake -S . -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DADSIM_BUILD_TESTS=ON

cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

echo ""
echo "Build complete: ${BUILD_DIR}/adsim"
echo "Run tests with: ctest --test-dir ${BUILD_DIR} --output-on-failure"
