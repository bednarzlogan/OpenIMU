# 1) Configure + build with tests enabled
TESTS=ON ./scripts/build.sh

# 2) Run all tests with ctest (pretty output, parallel)
ctest --test-dir build --output-on-failure -j"$(nproc)"
