cmake_minimum_required(VERSION 3.8)
project(cargo_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cargo_planner_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)

# Planning algorithm

add_library(
  ${PROJECT_NAME}_core
  src/cargo_planner.cpp
  src/greedy_placer.cpp)

target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_core PUBLIC ${OpenCV_LIBS})

# ROS conversion and visualization support

add_library(
  ${PROJECT_NAME}_ros_support
  src/grid_map_utils.cpp
  src/visualization.cpp)

target_include_directories(
  ${PROJECT_NAME}_ros_support
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_ros_support ${PROJECT_NAME}_core ${OpenCV_LIBS})

ament_target_dependencies(
  ${PROJECT_NAME}_ros_support
  cargo_planner_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  visualization_msgs)

# ROS node

add_executable(${PROJECT_NAME}_node src/cargo_planner_node.cpp src/main.cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE include)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${PROJECT_NAME}_ros_support)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  cargo_planner_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  visualization_msgs)

# Python tools

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/cargo_planner_visualizer.py
    scripts/container_occupancy_grid_forwarder.py
    scripts/run_visualizer.sh
    scripts/synthetic_container_occupancy_grid_publisher.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES scripts/example_visualizer_pallets.yaml
        DESTINATION share/${PROJECT_NAME}/scripts)

# Installation

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config imgs launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros_support ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

ament_export_dependencies(cargo_planner_msgs geometry_msgs nav_msgs OpenCV
                          rclcpp visualization_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Tests

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_grid_map_utils test/test_grid_map_utils.cpp)
  target_link_libraries(test_grid_map_utils ${PROJECT_NAME}_ros_support)
  target_include_directories(test_grid_map_utils PRIVATE include)

  ament_add_gtest(test_cargo_planner test/test_cargo_planner.cpp)
  target_link_libraries(test_cargo_planner ${PROJECT_NAME}_core)
  target_include_directories(test_cargo_planner PRIVATE include)

  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
  ament_add_pytest_test(test_python_tools test/test_python_tools.py)

  # Register configurable linters once with the package settings.
  list(
    APPEND
    AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_flake8)

  ament_cppcheck(include src INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  # Allow Cppcheck 1.88 and 2.x through the ROS 2 Jazzy wrapper.
  set_tests_properties(
    cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

  ament_cpplint(
    MAX_LINE_LENGTH
    120
    FILTERS
    -whitespace/newline
    -whitespace/forcolon
    -build/include_order
    -runtime/indentation_namespace
    -legal/copyright)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  # Register remaining linters declared as test dependencies.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
