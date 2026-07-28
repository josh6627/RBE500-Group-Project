cmake_minimum_required(VERSION 3.8)
project(controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetJointTarget.srv"
  "srv/SetJointVelocity.srv"
)

# Get the generated C++ interface library
rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# PID library
add_library(pid
  src/PID.cpp
)

add_executable(position_controller
  src/position_controller.cpp
)

add_executable(velocity_controller
  src/velocity_controller.cpp
)

ament_target_dependencies(position_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(velocity_controller
  rclcpp
  std_msgs
  sensor_msgs
)

target_include_directories(pid
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(position_controller
  pid
  "${cpp_typesupport_target}"
)
target_link_libraries(velocity_controller
  pid
  "${cpp_typesupport_target}"
)

add_dependencies(position_controller
  ${PROJECT_NAME}__rosidl_generator_cpp
)

add_dependencies(velocity_controller
  ${PROJECT_NAME}__rosidl_generator_cpp
)

# Install targets
install(
  TARGETS
    pid
    position_controller
    velocity_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
