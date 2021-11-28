Reference: GuoqingZhang: https://github.com/MyEvolution/Dragon : DragonView2D

Reference: 马玺凯

Build&Run:

```
vcpkg install libigl[core,glfw,opengl] openmesh
cmake -DCMAKE_TOOLCHAIN_FILE="C:\DEV_PROJECT\dev-repository\vcpkg\scripts\buildsystems\vcpkg.cmake" ..
cmake --build . -- /M:4

```

NOTICE:
    FIXME
        Can not run: CanvasSystem.cpp
        Cause: CGAL uninclude
