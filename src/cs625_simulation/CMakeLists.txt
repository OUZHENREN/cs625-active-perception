cmake_minimum_required(VERSION 3.8)
project(cs625_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
