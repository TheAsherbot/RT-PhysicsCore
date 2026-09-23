# RT-PhysicsCore\tests\physics_tests\CMakeLists.txt		

add_executable (PhysicsTests_Collision CollisionTest.cpp)
target_link_libraries(PhysicsTests_Collision PRIVATE glad)
target_link_libraries(PhysicsTests_Collision PRIVATE RT_PhysicsEngine)
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)
# add_subdirectory(examples)
