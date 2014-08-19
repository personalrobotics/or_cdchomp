cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS openrave_catkin)
catkin_package()
catkin_python_setup()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenRAVE REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

openrave_plugin("${PROJECT_NAME}_plugin"
    src/orcdchomp.cpp
    src/orcdchomp_mod.cpp
    src/orcdchomp_kdata.cpp
    src/orcwrap.cpp
    src/libcd/chomp.c
    src/libcd/grid.c
    src/libcd/grid_flood.c
    src/libcd/kin.c
    src/libcd/mat.c
    src/libcd/os.c
    src/libcd/spatial.c
    src/libcd/util.c
    src/libcd/util_shparse.c
)
target_link_libraries("${PROJECT_NAME}_plugin"
    blas
    lapacke
    lapack
    gsl
    ${catkin_LIBRARIES}
)
