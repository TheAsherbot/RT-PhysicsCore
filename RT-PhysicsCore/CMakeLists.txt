# RT-PhysicsCore\CMakeLists.txt 

# add_subdirectory(external/glfw)
add_subdirectory(src)


add_executable (RT-PhysicsCore main.cpp)
target_link_libraries(RT-PhysicsCore PRIVATE RT_PhysicsEngine)
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)
# add_subdirectory(examples)
