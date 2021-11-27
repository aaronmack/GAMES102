mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="C:\DEV_PROJECT\dev-repository\vcpkg\scripts\buildsystems\vcpkg.cmake" ..
cmake --build . --config Debug -- /M:4