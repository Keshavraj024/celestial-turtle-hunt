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

add_executable(celestial_turtle_controller
  src/celestial_controller.cpp
)

target_include_directories(celestial_turtle_controller PRIVATE include)
ament_target_dependencies(celestial_turtle_controller rclcpp turtlesim celestial_turtle_interface)

install(TARGETS
  celestial_turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
