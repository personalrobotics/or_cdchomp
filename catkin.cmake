cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenRAVE REQUIRED)
include_directories(
    ${OpenRAVE_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library("${PROJECT_NAME}_plugin" SHARED
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
   src/libcd/util_shparse.c)
target_link_libraries("${PROJECT_NAME}_plugin"
    blas
    lapacke
    lapack
    gsl
    ${OpenRAVE_LIBRARIES}
    ${catkin_LIBRARIES}
)
set_target_properties("${PROJECT_NAME}_plugin" PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

install(TARGETS "${PROJECT_NAME}_plugin"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}")
