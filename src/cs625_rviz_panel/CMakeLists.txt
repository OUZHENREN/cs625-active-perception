cmake_minimum_required(VERSION 3.16)
project(cs625_rviz_panel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Threads REQUIRED)

# Optional: the Elite driver messages used for /io_and_status_controller/
# {safety_mode,robot_mode}.  They live in the mandatory underlay, but the panel
# still builds (without those two subscriptions) when the driver is absent.
find_package(eli_common_interface QUIET)

# Small Qt-only widgets and palette helpers used by the panel and unit tested
# on their own: PillGroup, StatusDot and PanelTheme.
add_library(cs625_panel_widgets STATIC
  src/task_widgets.cpp
  include/cs625_rviz_panel/task_widgets.hpp
)
set_target_properties(cs625_panel_widgets PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cs625_panel_widgets PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(cs625_panel_widgets PUBLIC Qt5::Widgets)

add_library(${PROJECT_NAME} SHARED
  src/task_panel.cpp
  src/task_ros.cpp
  include/cs625_rviz_panel/task_panel.hpp
  include/cs625_rviz_panel/task_ros.hpp
  include/cs625_rviz_panel/task_command.hpp
  include/cs625_rviz_panel/task_state.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  pluginlib::pluginlib
  Qt5::Widgets
  cs625_panel_widgets
  rclcpp::rclcpp
  std_msgs::std_msgs
  sensor_msgs::sensor_msgs
  controller_manager_msgs::controller_manager_msgs
  Threads::Threads
)

if(eli_common_interface_FOUND)
  target_link_libraries(${PROJECT_NAME} eli_common_interface::eli_common_interface)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CS625_HAVE_ELI_INTERFACE=1)
  message(STATUS "cs625_rviz_panel: safety_mode/robot_mode subscriptions enabled")
else()
  message(STATUS "cs625_rviz_panel: eli_common_interface not found, robot mode display limited")
endif()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_task_command test/test_task_command.cpp)
  target_include_directories(test_task_command PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(test_task_command Qt5::Core)

  # Pure JSON / state / model / mode mapping: Qt Core only, no ROS, no widgets.
  ament_add_gtest(test_task_state test/test_task_state.cpp)
  target_include_directories(test_task_state PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(test_task_state Qt5::Core)

  # PillGroup exclusivity / locking and palette-derived theme need Qt Widgets.
  ament_add_gtest(test_task_widgets test/test_task_widgets.cpp)
  target_link_libraries(test_task_widgets cs625_panel_widgets Qt5::Widgets)

  # Offscreen render of the real TaskPanel: proves the widget can be built
  # without a DisplayContext and saves a PNG for visual review.
  ament_add_gtest(test_task_panel_render test/test_task_panel_render.cpp)
  target_link_libraries(test_task_panel_render ${PROJECT_NAME} Qt5::Widgets)
  set_tests_properties(test_task_panel_render PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

  # Service calls must finish (reply or 3 s timeout) without blocking the Qt loop.
  ament_add_gtest(test_task_ros_timeout test/test_task_ros_timeout.cpp)
  target_link_libraries(test_task_ros_timeout ${PROJECT_NAME} Qt5::Widgets)

  # Success path against a fake controller manager on a private service base.
  ament_add_gtest(test_task_ros_services test/test_task_ros_services.cpp)
  target_link_libraries(test_task_ros_services ${PROJECT_NAME} Qt5::Widgets)
endif()

ament_package()
