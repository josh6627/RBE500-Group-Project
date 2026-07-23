cmake_minimum_required(VERSION 3.8)
project(kinematics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IK.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(fwd_kinematics
  src/fwd_kinematics.cpp
)

add_executable(inv_kinematics
  src/inv_kinematics.cpp
)

ament_target_dependencies(
  fwd_kinematics
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(
  inv_kinematics
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(
  fwd_kinematics
  Eigen3::Eigen
)

target_link_libraries(
  inv_kinematics
  Eigen3::Eigen
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  inv_kinematics
  "${cpp_typesupport_target}"
)

add_dependencies(
  inv_kinematics
  ${PROJECT_NAME}__rosidl_generator_cpp
)

install(
  TARGETS
    fwd_kinematics
    inv_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()