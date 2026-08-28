file(GLOB SRC_FILES *.cpp *.h)

add_library(RT_PhysicsEngine STATIC ${SRC_FILES})

target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)


# target_link_libraries(RT_PhysicsEngine PRIVATE glfw)