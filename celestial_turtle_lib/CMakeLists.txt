cmake_minimum_required(VERSION 3.8)
project(celestial_turtle_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

add_library(
  celestial_turtle_utils
  src/utils.cpp
)
ament_target_dependencies(celestial_turtle_utils rclcpp turtlesim)

ament_export_targets(celestial_turtle_utils HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/celestial_turtle_lib
  DESTINATION include
)

install(
  TARGETS celestial_turtle_utils
  EXPORT celestial_turtle_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_package()
