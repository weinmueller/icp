add_executable(demo demo.cpp)
target_link_libraries(demo icp)

find_package(glfw3 QUIET)
find_package(OpenGL QUIET)

if(glfw3_FOUND AND OpenGL_FOUND)
    add_executable(viewer viewer.cpp)
    target_link_libraries(viewer icp glfw OpenGL::GL)
else()
    message(STATUS "GLFW or OpenGL not found — skipping viewer example")
endif()
