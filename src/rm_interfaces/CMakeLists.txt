cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Phase 2: SafetyStatus message.
# BaseDebug is rm_base's diagnostics topic (/base/debug) — never a runtime
# dependency of the control path.
# Phase 4 will add msg/PolicyStatus.msg (not yet authored — add it here then).
#
# WaypointStatus is rm_waypoint's diagnostics topic (/waypoint/status) — also
# never a runtime dependency of the control path.
#
# Kill (/safety/kill) and reset (/safety/reset) use std_srvs/srv/Trigger and
# are NOT defined here: a Trigger says everything those two need to say, and a
# custom srv adding nothing would be a rebuild the robot has to do for no
# reason. That std_srvs dependency belongs to rm_safety, which calls them.
#
# srv/SetWaypoint.srv IS custom, because it carries what a Trigger cannot: a
# frame and a point going in, and a REFUSAL WITH A REASON coming back. See the
# file for why the waypoint path is a service rather than a topic.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  "msg/BaseDebug.msg"
  "msg/WaypointStatus.msg"
  "srv/SetWaypoint.srv"
  "srv/Jog.srv"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
