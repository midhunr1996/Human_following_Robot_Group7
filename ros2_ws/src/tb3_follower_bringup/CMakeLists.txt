cmake_minimum_required(VERSION 3.8)
project(tb3_follower_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
