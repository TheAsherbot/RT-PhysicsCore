# RT-PhysicsCore\src\CMakeLists.txt 

# file(GLOB SRC_FILES *.cpp *.h)


add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(physics)
add_subdirectory(rendering)

add_library(RT_PhysicsEngine STATIC ${SRC_FILES})

target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include)
target_include_directories(RT_PhysicsEngine PUBLIC ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/external/glad/include)


# target_link_libraries(RT_PhysicsEngine PRIVATE glfw)

include(FetchContent)

FetchContent_Declare(
	glm
	GIT_REPOSITORY	https://github.com/g-truc/glm.git
	GIT_TAG 	8d1fd52 #refs/tags/1.0.1
)

FetchContent_MakeAvailable(glm)

target_link_libraries(RT_PhysicsEngine PUBLIC glm::glm-header-only)
target_include_directories(RT_PhysicsEngine
    PUBLIC
        ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)


# ------------------------------
# GLFW 3.5.1
# ------------------------------
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.5.1
)

FetchContent_MakeAvailable(glfw)

# Link GLFW
target_link_libraries(RT_PhysicsEngine PUBLIC glfw)
