cmake_minimum_required(VERSION 3.5)
project(scenario_runner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_auto REQUIRED)
# we require all dependencies but by default ament_auto_find_build_dependencies doesn't require packages
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

ament_auto_add_library(scenario_runner SHARED
  src/scenario_terminator.cpp
  src/scenario_runner.cpp
)

ament_auto_add_executable(scenario_runner_node
  src/scenario_runner_node.cpp)

ament_auto_package(INSTALL_TO_SHARE launch)
