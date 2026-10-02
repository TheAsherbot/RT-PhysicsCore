# RT-PhysicsCore\external\imgui\CMakeLists.txt

add_library(imgui STATIC
    src/imgui.cpp
    src/imgui_demo.cpp
    src/imgui_draw.cpp
    src/imgui_tables.cpp
    src/imgui_widgets.cpp
    src/backend/imgui_impl_glfw.cpp
    src/backend/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(imgui PRIVATE 
    ${CMAKE_SOURCE_DIR}/RT-PhysicsCore/external/glad/include
)

target_link_libraries(imgui PUBLIC 
    glfw
)