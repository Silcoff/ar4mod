cmake_minimum_required(VERSION 3.8)
# --- Fetch FTXUI --------------------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/arthursonzogni/ftxui.git
  GIT_TAG        v5.0.0
  GIT_PROGRESS   TRUE
  GIT_SHALLOW    TRUE
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(ftxui)
# ------------------------------------------------------------------------------
project(man_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Include directories
# include_directories(
#   include
#   ${moveit_msgs_INCLUDE_DIRS}
# )

# # teleop ----------------- 
# add_executable(teleop src/teleop.cpp)
# target_link_libraries(teleop ftxui::screen ftxui::dom ftxui::component)
# ament_target_dependencies(teleop rclcpp std_msgs )
# target_include_directories(teleop PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(teleop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# # -----------------
# include_directories(/opt/ros/iron/include)


# servo ----------------- 
add_executable(servo   src/servo_keyboard.cpp)
# target_include_directories(servo PRIVATE /opt/ros/iron/include/moveit_msgs/moveit_msgs/srv/servo_command_type.hpp)

# target_link_libraries(servo moveit_msgs)
ament_target_dependencies(servo
  Eigen3
  rclcpp
  geometry_msgs
  std_srvs
  moveit
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
)
target_include_directories(servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(servo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# -----------------
install(TARGETS 
    servo
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    config/
    DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
