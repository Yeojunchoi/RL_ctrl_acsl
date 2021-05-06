cmake_minimum_required(VERSION 3.5)
project(RL_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# include directory setting

include_directories(include)


#Excutables here
add_executable(offboard_control 
	src/offboard_control.cpp
	)
ament_target_dependencies(offboard_control
  px4_msgs
  rclcpp
)

add_executable(sensor_combined 
	src/sensor_combined.cpp
	)
ament_target_dependencies(sensor_combined
  px4_msgs
  rclcpp
)

add_executable(vehicle_gps_position_listener 
	src/vehicle_gps_position_listener.cpp
	)
ament_target_dependencies(vehicle_gps_position_listener
  px4_msgs
  rclcpp
)



# install excutable

install(TARGETS
  offboard_control
  sensor_combined
  vehicle_gps_position_listener
  DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



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

ament_package()
