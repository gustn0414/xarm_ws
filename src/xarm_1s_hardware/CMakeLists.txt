cmake_minimum_required(VERSION 3.8)
project(xarm_1s_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

add_library(xarm_1s_hardware SHARED
  src/xarm_1s_hardware_interface.cpp
)

target_include_directories(xarm_1s_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${HIDAPI_INCLUDE_DIRS}
)

target_link_libraries(xarm_1s_hardware
  ${HIDAPI_LIBRARIES}
)

ament_target_dependencies(xarm_1s_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface xarm_1s_hardware_plugin.xml)

install(TARGETS xarm_1s_hardware
  EXPORT export_xarm_1s_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES xarm_1s_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(xarm_1s_hardware)
ament_export_targets(export_xarm_1s_hardware)

ament_package()
