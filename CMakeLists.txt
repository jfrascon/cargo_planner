cmake_minimum_required(VERSION 3.8)
project(cargo_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ── Find dependencies ────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cargo_planner_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# ── Core library (ROS-free: grid_map_utils + planner + heuristics) ──────────
# Keeping the algorithm in a separate library makes unit testing
# straightforward: tests link against cargo_planner_core without needing a ROS
# node.

add_library(${PROJECT_NAME}_core src/grid_map_utils.cpp src/greedy_placer.cpp
                                 src/cargo_planner.cpp src/visualization.cpp)

target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS})

ament_target_dependencies(${PROJECT_NAME}_core rclcpp nav_msgs geometry_msgs
                          visualization_msgs cargo_planner_msgs)

# ── Executable node ──────────────────────────────────────────────────────────

add_executable(${PROJECT_NAME}_node src/cargo_planner_node.cpp src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  visualization_msgs
  cargo_planner_msgs)

target_include_directories(${PROJECT_NAME}_node PRIVATE include)

# ── Python scripts ────────────────────────────────────────────────────────────

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/synthetic_container_occupancy_grid_publisher.py
                 scripts/container_occupancy_grid_forwarder.py
        DESTINATION lib/${PROJECT_NAME})

# ── Install ───────────────────────────────────────────────────────────────────

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY imgs/ DESTINATION share/${PROJECT_NAME}/imgs)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_grid_map_utils test/test_grid_map_utils.cpp)
  target_link_libraries(test_grid_map_utils ${PROJECT_NAME}_core)
  target_include_directories(test_grid_map_utils PRIVATE include)

  ament_add_gtest(test_cargo_planner test/test_cargo_planner.cpp)
  target_link_libraries(test_cargo_planner ${PROJECT_NAME}_core)
  target_include_directories(test_cargo_planner PRIVATE include)
endif()

ament_package()
