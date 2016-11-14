cmake_minimum_required(VERSION 3.5)

project(rclcpp_lifecycle)

if(NOT WIN32)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# get the rmw implementations ahead of time
find_package(rmw_implementation_cmake REQUIRED)
get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
endforeach()

set(lifecycle_msgs_files
  "msg/Transition.msg"
)
rosidl_generate_interfaces(lifecycle_msgs
  ${lifecycle_msgs_files}
)

include_directories(
  include
)

set(rcl_lifecycle_sources
  src/rcl_lifecycle/default_state_machine.c
  src/rcl_lifecycle/rcl_lifecycle.c
  src/rcl_lifecycle/transition_map.c
)
set_source_files_properties(
  ${rcl_lifecycle_sources}
  PROPERTIES language "C"
)

macro(targets)
  get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "c")

  ### C-Library depending only on RCL
  add_library(
    rcl_lifecycle${target_suffix}
    SHARED
    ${rcl_lifecycle_sources})

  foreach(typesupport_impl ${typesupport_impls})
    message("MY TYPESUpoort_IMPL ${typesupport_impl}")
    rosidl_target_interfaces(rcl_lifecycle${target_suffix}
      lifecycle_msgs
      ${typesupport_impl}
      )
  endforeach()

  ament_target_dependencies(rcl_lifecycle${target_suffix}
    "rcl${target_suffix}"
    "std_msgs")

  install(TARGETS rcl_lifecycle${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  ### CPP High level library+exe
  add_library(rclcpp_lifecycle${target_suffix}
    SHARED
    src/rclcpp_lifecycle/lifecycle_manager.cpp)
  target_link_libraries(rclcpp_lifecycle${target_suffix}
    rcl_lifecycle${target_suffix})
  ament_target_dependencies(rclcpp_lifecycle${target_suffix}
    "rclcpp${target_suffix}")

  add_executable(lifecycle_talker${target_suffix}
    src/rclcpp_lifecycle/lifecycle_talker.cpp)
  target_link_libraries(lifecycle_talker${target_suffix}
    rclcpp_lifecycle${target_suffix})
  ament_target_dependencies(lifecycle_talker${target_suffix}
    "rclcpp${target_suffix}"
    "std_msgs")

  install(TARGETS
    rclcpp_lifecycle${target_suffix}
    lifecycle_talker${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
