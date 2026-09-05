cmake_minimum_required(VERSION 3.8)
project(mira2_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(control_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Glob all source files in the src/ directory and its subdirectories
#file(GLOB_RECURSE SRC_FILES "src/*.cpp")
set(SRC_FILES
  src/common.cpp
  src/main.cpp
  src/actions/DiveToDepth.cpp
  src/actions/Search.cpp
  src/actions/TrackAndApproach.cpp
  src/actions/BlindMove.cpp
  src/actions/EmergencySurface.cpp
  src/actions/SetFlightMode.cpp
)
# Create executable
add_executable(mira2_actions_exe ${SRC_FILES})
add_executable(sauvc_controller_node src/sauvc/sauvc_controller_node.cpp)
# Link dependencies
ament_target_dependencies(mira2_actions_exe
  rclcpp
  behaviortree_cpp
  control_utils
  custom_msgs
  vision_msgs
  ament_index_cpp
)
ament_target_dependencies(sauvc_controller_node
  rclcpp
  std_msgs
  custom_msgs
  vision_msgs
  control_utils
)

install(TARGETS
  sauvc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install executable
install(TARGETS mira2_actions_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (the XML behavior tree)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()