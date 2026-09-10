# RT-PhysicsCore\src\CMakeLists.txt 

# file(GLOB SRC_FILES *.cpp *.h)


add_subdirectory(core)
add_subdirectory(utils)

add_library(RT_PhysicsEngine STATIC ${SRC_FILES})

target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)


# target_link_libraries(RT_PhysicsEngine PRIVATE glfw)

include(FetchContent)

FetchContent_Declare(
	glm
	GIT_REPOSITORY	https://github.com/g-truc/glm.git
	GIT_TAG 	8d1fd52 #refs/tags/1.0.1
)

FetchContent_MakeAvailable(glm)

target_link_libraries(RT_PhysicsEngine PUBLIC glm::glm-header-only)

