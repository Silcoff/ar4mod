cmake_minimum_required(VERSION 3.22)
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
project(debugger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)


# controller ---------------
add_executable(controller_debug src/controller_debug.cpp)
target_link_libraries(controller_debug ftxui::screen ftxui::dom ftxui::component)
ament_target_dependencies(controller_debug rclcpp std_msgs control_msgs)
target_include_directories(controller_debug PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(controller_debug PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# -----------------

# test_ftxui ---------------
add_executable(test_ftxui src/test_ftxui.cpp)
target_link_libraries(test_ftxui ftxui::screen ftxui::dom ftxui::component)
ament_target_dependencies(test_ftxui rclcpp std_msgs )
target_include_directories(test_ftxui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(test_ftxui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# -----------------

# pub ---------------
add_executable(pub src/pub.cpp)
target_link_libraries(pub ftxui::screen ftxui::dom ftxui::component)
ament_target_dependencies(pub rclcpp std_msgs )
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# -----------------

install(TARGETS controller_debug test_ftxui pub
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
