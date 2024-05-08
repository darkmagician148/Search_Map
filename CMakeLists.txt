cmake_minimum_required(VERSION 3.8)
project(search_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(execute_node search_map)
set(library_name ${execute_node}_package)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(dependencies
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  pluginlib
  nav2_util
  nav2_map_server
  glog
  visualization_msgs
)

foreach(name IN LISTS dependencies)
  find_package(${name} REQUIRED)
endforeach()

include_directories(include)

add_library(${library_name} SHARED
  src/data_interface.cpp
  src/alogrithm/dijkstra.cpp
  # src/alogrithm/a_start.cpp
  src/alogrithm/rrt_start.cpp
)

target_include_directories(${library_name} PUBLIC 
  ${GLOG_INCLUDE_DIRS}
)

target_link_libraries(${library_name} 
  ${GLOG_LIBRARIES}
  glog::glog
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${execute_node} 
  src/Main.cpp
)

ament_target_dependencies(${execute_node} 
  ${dependencies}
)

target_link_libraries(${execute_node}
  ${library_name}
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${execute_node}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  map
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


pluginlib_export_plugin_description_file(search_map search_algorithm_lib.xml)

ament_package()
