cmake_minimum_required(VERSION 3.8)
project(buildscan_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# ── Interface generation ───────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/InspectionResult.msg"

  # Services
  "srv/SetInspectionConfig.srv"
  "srv/SetMode.srv"

  # Actions
  "action/InspectArea.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    action_msgs
)

# ── Linting ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
