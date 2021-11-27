cmake_minimum_required(VERSION 3.1)
project(Minimal_Surface)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)

# *****MODIFIED
#find_package(LIBIGL REQUIRED QUIET)
find_package(libigl CONFIG REQUIRED)

find_package(OpenMesh REQUIRED)

#Error (active)	E0035	#error directive: "You have to define _USE_MATH_DEFINES in the compiler settings!"

add_definitions(-D _USE_MATH_DEFINES)

# Add your project files
file(GLOB SRCFILES *.cpp)
add_executable(${PROJECT_NAME} ${SRCFILES} minimal_surface.h minimal_surface.cpp Base.hpp Base.cpp Harmonic_map.hpp)
target_link_libraries(${PROJECT_NAME} PRIVATE igl::core igl::opengl_glfw)

# *****MODIFIED
#target_link_libraries(${PROJECT_NAME} ${OpenMesh_LIBS} OpenMeshCore OpenMeshTools)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENMESH_LIBRARIES})
