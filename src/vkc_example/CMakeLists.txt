cmake_minimum_required(VERSION 2.8.3)
project(vkc_example)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall -Wextra -DCMAKE_BUILD_TYPE=Debug -g -ggdb)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_rosutils
  rospy
  tesseract_monitoring
  roslib # added: wanglei@bigai.ai, to get package path
)

find_package(Boost COMPONENTS system python REQUIRED)
find_package(vkc REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajopt REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(ompl REQUIRED)
find_package(kmeans_rex REQUIRED)

find_package(PCL REQUIRED COMPONENTS
  core
  features
  filters
  io
  segmentation
  surface
)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
# add_message_files(
# FILES
# Message1.msg
# Message2.msg
# )

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# Service1.srv
# Service2.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
# generate_messages(
# DEPENDENCIES
# std_msgs  # Or other packages containing msgs
# )

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES vkc
  # CATKIN_DEPENDS
  # roscpp
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(
# include
# ${catkin_INCLUDE_DIRS}
# )

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/vkc.cpp
# )

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vkc_node.cpp)

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
# ${catkin_LIBRARIES}
# )
add_library(${PROJECT_NAME}_utils SHARED src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_utils
  tesseract::tesseract_environment
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_ompl
  ${catkin_LIBRARIES}
)
target_compile_options(
  ${PROJECT_NAME}_utils
  PUBLIC -Wsuggest-override -Wconversion -Wsign-conversion
)

if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_utils PUBLIC -std=c++11)
else()
  target_compile_features(${PROJECT_NAME}_utils PUBLIC cxx_std_11)
endif()

target_include_directories(
  ${PROJECT_NAME}_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(
  ${PROJECT_NAME}_utils SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_arena_env_node src/arena_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_arena_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_arena_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   tesseract::tesseract_motion_planners_ompl
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_arena_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_arena_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_arena_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_arena_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_open_door_env_node src/open_door_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_open_door_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_open_door_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   tesseract::tesseract_motion_planners_ompl
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_open_door_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_open_door_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_open_door_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_open_door_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_uam_env_node src/uam_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_uam_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_uam_env
#   vkc::vkc_urdf_scene_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   tesseract::tesseract_motion_planners_ompl
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_uam_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_uam_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_uam_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_uam_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_sr_env_node src/sr_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_sr_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_urdf_scene_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   tesseract::tesseract_motion_planners_ompl
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_sr_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_sr_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_sr_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_sr_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_benchmark_env_node src/benchmark_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_benchmark_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_benchmark_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   tesseract::tesseract_motion_planners_ompl
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_benchmark_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_benchmark_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_benchmark_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_benchmark_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_urdf_scene_env_node src/urdf_scene_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_urdf_scene_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_urdf_scene_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_urdf_scene_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_urdf_scene_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_urdf_scene_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_urdf_scene_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_household_env_node src/household_env_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_household_env_node
  ${PROJECT_NAME}_utils
  vkc::vkc_construct_vkc
  vkc::vkc_urdf_scene_env
  vkc::vkc_prob_generator
  vkc::vkc_vkc_env_basic
  ${catkin_LIBRARIES}
)

if(CXX_FEATURE_FOUND EQUAL "-1")
  target_compile_options(${PROJECT_NAME}_household_env_node PRIVATE -std=c++11)
else()
  target_compile_features(${PROJECT_NAME}_household_env_node PRIVATE cxx_std_11)
endif()

target_include_directories(${PROJECT_NAME}_household_env_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_household_env_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})


# add_executable(${PROJECT_NAME}_tieta_env_node src/tieta_env_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_tieta_env_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_urdf_scene_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   ${catkin_LIBRARIES}
# )


# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_tieta_env_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_tieta_env_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_tieta_env_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_tieta_env_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_tieta_tablechairenv_node src/tieta_tablechairenv_node.cpp)
# target_link_libraries(
#   ${PROJECT_NAME}_tieta_tablechairenv_node
#   ${PROJECT_NAME}_utils
#   vkc::vkc_construct_vkc
#   vkc::vkc_urdf_scene_env
#   vkc::vkc_prob_generator
#   vkc::vkc_vkc_env_basic
#   ${catkin_LIBRARIES}
# )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#   target_compile_options(${PROJECT_NAME}_tieta_tablechairenv_node PRIVATE -std=c++11)
# else()
#   target_compile_features(${PROJECT_NAME}_tieta_tablechairenv_node PRIVATE cxx_std_11)
# endif()

# target_include_directories(${PROJECT_NAME}_tieta_tablechairenv_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_tieta_tablechairenv_node SYSTEM PUBLIC
#   ${catkin_INCLUDE_DIRS})




# add_executable(${PROJECT_NAME}_big_task_node
# src/big_task_node.cpp
# src/motion_plan_actions.cpp
# )
# target_link_libraries(
# ${PROJECT_NAME}_big_task_node
# ${PROJECT_NAME}_utils
# vkc::vkc_construct_vkc
# vkc::vkc_urdf_scene_env
# vkc::vkc_prob_generator
# vkc::vkc_vkc_env_basic
# tesseract::tesseract_motion_planners_ompl
# ${catkin_LIBRARIES}
# )
# if(CXX_FEATURE_FOUND EQUAL "-1")
# target_compile_options(${PROJECT_NAME}_big_task_node PRIVATE -std=c++11)
# else()
# target_compile_features(${PROJECT_NAME}_big_task_node PRIVATE cxx_std_11)
# endif()
# target_include_directories(${PROJECT_NAME}_big_task_node PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_big_task_node SYSTEM PUBLIC
# ${catkin_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME}_pog_env_node
# src/pog_env_node.cpp
# )
# target_link_libraries(
# ${PROJECT_NAME}_pog_env_node
# ${PROJECT_NAME}_utils
# vkc::vkc_construct_vkc
# vkc::vkc_urdf_scene_env
# vkc::vkc_prob_generator
# vkc::vkc_vkc_env_basic
# tesseract::tesseract_motion_planners_ompl
# ${catkin_LIBRARIES}
# )
# if(CXX_FEATURE_FOUND EQUAL "-1")
# target_compile_options(${PROJECT_NAME}_pog_env_node PRIVATE -std=c++11)
# else()
# target_compile_features(${PROJECT_NAME}_pog_env_node PRIVATE cxx_std_11)
# endif()
# target_include_directories(${PROJECT_NAME}_pog_env_node PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
# "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_pog_env_node SYSTEM PUBLIC
# ${catkin_INCLUDE_DIRS})
# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/ros_movement.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_open_door_env_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_uam_env_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_sr_env_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_arena_env_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_benchmark_env_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_tieta_env_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(TARGETS ${PROJECT_NAME}_tieta_tablechairenv_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vkc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
