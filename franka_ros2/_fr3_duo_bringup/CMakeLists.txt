cmake_minimum_required(VERSION 3.8)
project(fr3_duo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz srdf urdf DESTINATION share/${PROJECT_NAME})

ament_package()
