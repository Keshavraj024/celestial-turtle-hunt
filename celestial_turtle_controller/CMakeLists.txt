cmake_minimum_required(VERSION 3.8)
project(celestial_turtle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(celestial_turtle_interface REQUIRED)
find_package(celestial_turtle_lib REQUIRED)

add_executable(turtle_controller_node
  src/turtle_controller.cpp
)

target_include_directories(turtle_controller_node PRIVATE include)
ament_target_dependencies(turtle_controller_node rclcpp turtlesim celestial_turtle_interface celestial_turtle_lib)

install(TARGETS
        turtle_controller_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
