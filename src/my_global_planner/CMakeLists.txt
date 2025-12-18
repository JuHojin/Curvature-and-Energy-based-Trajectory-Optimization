cmake_minimum_required(VERSION 3.10.2)
project(my_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O3 -march=native -DNDEBUG -mfma)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  nav_core
  pluginlib
  roscpp
  std_msgs
  message_generation
)

find_package(osqp REQUIRED)


# --- MKL 라이브러리 찾기 시작 ---
# MKL 루트 경로 힌트 추가 (시스템 환경에 맞게 경로 수정 가능)
set(MKL_LIB_HINTS /opt/intel/oneapi/mkl/2025.1/lib) # libmkl_* 파일들이 있는 경로
set(INTEL_COMPILER_LIB_HINTS /opt/intel/oneapi/compiler/2025.1/lib) # libiomp5.so 파일이 있는 경로

# Advisor에서 요구하는 라이브러리 찾기
find_library(MKL_INTEL_ILP64_LIBRARY NAMES mkl_intel_ilp64 HINTS ${MKL_LIB_HINTS})
find_library(MKL_INTEL_THREAD_LIBRARY NAMES mkl_intel_thread HINTS ${MKL_LIB_HINTS})
find_library(MKL_CORE_LIBRARY NAMES mkl_core HINTS ${MKL_LIB_HINTS})
find_library(IOMP5_LIBRARY NAMES libiomp5 HINTS ${INTEL_COMPILER_LIB_HINTS})

# 찾은 라이브러리 변수들을 Advisor 순서대로 리스트로 묶기
set(MKL_LIBRARIES) # 리스트 초기화
if(MKL_INTEL_ILP64_LIBRARY)
  list(APPEND MKL_LIBRARIES ${MKL_INTEL_ILP64_LIBRARY})
else()
  message(FATAL_ERROR "MKL ILP64 Interface library (mkl_intel_ilp64) not found in ${MKL_LIB_HINTS}!")
endif()
if(MKL_INTEL_THREAD_LIBRARY)
  list(APPEND MKL_LIBRARIES ${MKL_INTEL_THREAD_LIBRARY})
else()
  message(FATAL_ERROR "MKL Intel Threading library (mkl_intel_thread) not found in ${MKL_LIB_HINTS}!")
endif()
if(MKL_CORE_LIBRARY)
  list(APPEND MKL_LIBRARIES ${MKL_CORE_LIBRARY})
else()
  message(FATAL_ERROR "MKL Core library (mkl_core) not found in ${MKL_LIB_HINTS}!")
endif()
if(IOMP5_LIBRARY)
  list(APPEND MKL_LIBRARIES ${IOMP5_LIBRARY})
else()
  message(WARNING "Intel OpenMP library (libiomp5) not found in ${INTEL_COMPILER_LIB_HINTS}!") # 경고 또는 FATAL_ERROR
endif()

message(STATUS "Linking with MKL Libraries: ${MKL_LIBRARIES}")
# --- MKL 라이브러리 찾기 끝 ---

# --- MKL 컴파일러 옵션 추가 시작 ---
# Advisor에서 제시된 컴파일러 옵션 추가
# 전역적으로 추가하거나 필요한 타겟에만 추가할 수 있음
# target_compile_definitions(${PROJECT_NAME} PRIVATE MKL_ILP64) # MKL_ILP64 정의 추가
add_definitions(-DMKL_ILP64) # 전역적으로 추가하는 것이 간단할 수 있음
# target_include_directories(${PROJECT_NAME} PRIVATE ${MKL_INCLUDE_DIRS}) # 필요시 MKL 헤더 경로 추가 (find_path로 MKL_INCLUDE_DIRS 설정 필요)
# --- MKL 컴파일러 옵션 추가 끝 ---


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PinPoints.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ReplanningService.srv
  BSplineSmoothing.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_global_planner dummy_recovery
  CATKIN_DEPENDS costmap_2d nav_core pluginlib roscpp message_runtime
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/my_global_planner.cpp
  src/a_star.cpp
  src/opt_curv.cpp
  src/path_func.cpp
)

add_library(dummy_recovery src/dummy_recovery.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(replanning_trigger src/replanning_trigger.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  osqp::osqp          # OSQP 라이브러리 타겟
  ${MKL_LIBRARIES}    # 찾은 MKL 라이브러리 리스트 (순서 중요)
  pthread             # POSIX Threads
  m                   # Math library
  dl                  # Dynamic linking library
)

target_link_libraries(replanning_trigger
  ${catkin_LIBRARIES}
)

target_link_libraries(dummy_recovery
  ${catkin_LIBRARIES}
)

# target_link_libraries(replanning_trigger
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_global_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
