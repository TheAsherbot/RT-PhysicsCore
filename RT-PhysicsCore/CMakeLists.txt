# add_subdirectory(external/glfw)
add_subdirectory(src)


add_executable (RT-PhysicsCore "main.cpp")
target_link_libraries(RT-PhysicsCore PRIVATE RT_PhysicsEngine)
# add_subdirectory(examples)
