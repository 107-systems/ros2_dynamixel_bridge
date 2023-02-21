#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_ros_dynamixel_bridge)
set(L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET ${PROJECT_NAME}_node)
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#######################################################################################
add_subdirectory(external/libdynamixelplusplus)
#######################################################################################
include_directories(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET}
  include
)
#######################################################################################
add_executable(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET}
  src/MX28ARSingle.cpp
  src/MX28ARSyncGroup.cpp
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET} PRIVATE -Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
target_compile_features(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET} PRIVATE cxx_std_17)
#######################################################################################
target_link_libraries(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET} dynamixel++)
#######################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mode.msg"
)
#######################################################################################
rosidl_target_interfaces(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET} ${PROJECT_NAME} "rosidl_typesupport_cpp")
#######################################################################################
ament_target_dependencies(${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET} rclcpp std_msgs)
#######################################################################################
ament_export_dependencies(rosidl_default_runtime)
#######################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#######################################################################################
install(TARGETS
  ${L3XZ_ROS_DYNAMIXEL_BRIDGE_TARGET}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
