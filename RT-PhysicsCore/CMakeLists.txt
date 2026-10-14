# RT-PhysicsCore/CMakeLists.txt

# 1. Define glad first so all subdirectories can see it
add_library(glad external/glad/src/gl.c)
target_include_directories(glad PUBLIC external/glad/include)

# 2. Add subdirectories
add_subdirectory(external)
add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(tests/)
add_subdirectory(tools/profiler)

# 3. Main executable (RT_PhysicsEngine comes before glad)
add_executable(RT-PhysicsCore main.cpp)
target_link_libraries(RT-PhysicsCore PRIVATE RT_PhysicsEngine glad imgui)