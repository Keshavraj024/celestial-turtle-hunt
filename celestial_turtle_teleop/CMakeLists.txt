cmake_minimum_required(VERSION 3.8)
project(celestial_turtle_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_teleop_node src/turtle_teleop.cpp )
target_include_directories(turtle_teleop_node PRIVATE include)
ament_target_dependencies(turtle_teleop_node rclcpp turtlesim geometry_msgs)

install(TARGETS
        turtle_teleop_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
