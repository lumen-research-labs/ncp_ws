cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message and service definitions are added in Phase 2 (SafetyStatus) and
# Phase 4 (PolicyStatus). Intentionally empty for now.
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/SafetyStatus.msg"
#   "msg/PolicyStatus.msg"
#   "srv/Kill.srv"
#   "srv/Reset.srv"
#   DEPENDENCIES std_msgs builtin_interfaces
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
