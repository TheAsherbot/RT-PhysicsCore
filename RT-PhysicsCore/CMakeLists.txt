# RT-PhysicsCore\CMakeLists.txt		

add_subdirectory(src)

add_library(glad external/glad/src/gl.c)
target_include_directories(glad PUBLIC external/glad/include)

add_executable (RT-PhysicsCore main.cpp)
target_link_libraries(RT-PhysicsCore PRIVATE glad)
target_link_libraries(RT-PhysicsCore PRIVATE RT_PhysicsEngine)
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)
# add_subdirectory(examples)
