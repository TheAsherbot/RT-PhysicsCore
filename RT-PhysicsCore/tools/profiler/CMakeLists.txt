# RT-PhysicsCore\tools\profiler\CMakeLists.txt

add_executable(RT-ProfilerViewer ProfilerApplicaiton.cpp)

target_link_libraries(RT-ProfilerViewer PRIVATE
    glad
    imgui
    RT_PhysicsEngine
)

target_include_directories(RT-ProfilerViewer PRIVATE
    ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/include
    ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/external/glad/include
)