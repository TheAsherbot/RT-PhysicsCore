# RT-PhysicsCore\tools\profiler\CMakeLists.txt

add_executable(RT-ProfilerViewer ProfilerApplication.cpp)

target_link_libraries(RT-ProfilerViewer PRIVATE
    RT_PhysicsEngine
    glad
    imgui
)

target_include_directories(RT-ProfilerViewer PRIVATE
    ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include
    ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/external/glad/include
)