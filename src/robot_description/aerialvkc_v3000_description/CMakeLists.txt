cmake_minimum_required(VERSION 2.8.3)
project(aerialvkc_v3000_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
