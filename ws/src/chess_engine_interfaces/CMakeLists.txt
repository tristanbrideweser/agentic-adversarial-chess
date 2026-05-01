cmake_minimum_required(VERSION 3.8)
project(chess_interfaces)

# Default to C99 / C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Interface generation
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}

  # Messages
  "msg/GameState.msg"
  "msg/PickPlaceTask.msg"

  # Services
  "srv/GetSquarePose.srv"
  "srv/GetGraveyardPose.srv"
  "srv/WorldToSquare.srv"

  # Actions
  "action/PickPlace.action"
  "action/GraspPlanning.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    action_msgs
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()