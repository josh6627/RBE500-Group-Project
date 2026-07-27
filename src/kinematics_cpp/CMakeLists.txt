cmake_minimum_required(VERSION 3.8)
project(kinematics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Eigen
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Custom services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IK.srv"
  "srv/SetJointTarget.srv"
  "srv/CartesianToJointVelocity.srv"
  "srv/JointToCartesianVelocity.srv"
  DEPENDENCIES geometry_msgs
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

target_include_directories(pid
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Executables
add_executable(fwd_kinematics
  src/fwd_kinematics.cpp
)

add_executable(inv_kinematics
  src/inv_kinematics.cpp
)

add_executable(controller
  src/position_controller.cpp
)

add_executable(vel_kinematics
  src/velocity_kinematics.cpp
)

# ROS dependencies
ament_target_dependencies(fwd_kinematics
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(inv_kinematics
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(vel_kinematics
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Link libraries
target_link_libraries(fwd_kinematics
  Eigen3::Eigen
)

target_link_libraries(inv_kinematics
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)

target_link_libraries(controller
  pid
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)

target_link_libraries(vel_kinematics
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)

# Ensure generated service headers exist before compiling
add_dependencies(inv_kinematics
  ${PROJECT_NAME}__rosidl_generator_cpp
)

add_dependencies(controller
  ${PROJECT_NAME}__rosidl_generator_cpp
)

add_dependencies(vel_kinematics
  ${PROJECT_NAME}__rosidl_generator_cpp
)

# Install targets
install(
  TARGETS
    pid
    fwd_kinematics
    inv_kinematics
    controller
    vel_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()