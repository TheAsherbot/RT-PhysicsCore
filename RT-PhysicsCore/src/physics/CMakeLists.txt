# RT-PhysicsCore\src\physics\CMakeLists.txt 

file(GLOB CORE_SRC *.cpp *.h)

add_subdirectory(systems)
add_subdirectory(solver)
add_subdirectory(collision)

set(SRC_FILES ${SRC_FILES} ${CORE_SRC} PARENT_SCOPE)