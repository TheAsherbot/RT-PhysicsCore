# RT-PhysicsCore\src\rendering\CMakeLists.txt 

file(GLOB CORE_SRC *.cpp *.h)

add_subdirectory(systems)

set(SRC_FILES ${SRC_FILES} ${CORE_SRC} PARENT_SCOPE)
