# RT-PhysicsCore\src\core\ecs\CMakeLists.txt 

add_subdirectory(core)
add_subdirectory(systems)

file(GLOB CORE_SRC *.cpp *.h)

set(SRC_FILES ${SRC_FILES} ${CORE_SRC} PARENT_SCOPE)

# target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)
