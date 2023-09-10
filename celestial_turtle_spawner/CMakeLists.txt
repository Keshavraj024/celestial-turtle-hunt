cmake_minimum_required(VERSION 3.8)
project(celestial_turtle_spawner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(celestial_turtle_interface REQUIRED)

add_executable(celestial_turtle_spawner 
  src/turtle_spawner.cpp
)

target_include_directories(celestial_turtle_spawner PRIVATE include)
ament_target_dependencies(celestial_turtle_spawner rclcpp turtlesim celestial_turtle_interface)

add_executable(celestial_turtle_mover
  src/turtle_mover.cpp
)

target_include_directories(celestial_turtle_mover PRIVATE include)
ament_target_dependencies(celestial_turtle_mover rclcpp turtlesim celestial_turtle_interface geometry_msgs)

install(TARGETS
  celestial_turtle_spawner
  celestial_turtle_mover
  DESTINATION lib/${PROJECT_NAME}
)

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
