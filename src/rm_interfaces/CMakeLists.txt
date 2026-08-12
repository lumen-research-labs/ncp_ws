cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Phase 2: SafetyStatus message.
# Phase 4 will add msg/PolicyStatus.msg (not yet authored — add it here then).
#
# Kill (/safety/kill) and reset (/safety/reset) use std_srvs/srv/Trigger and
# are NOT defined here — rm_interfaces ships no custom srv. The std_srvs
# dependency belongs to rm_safety, which calls those services, not here.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
