cmake_minimum_required(VERSION 3.5)
project(romea_cmd_mux_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(romea_cmd_mux_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
 src/cmd_mux_subscription_client.cpp
 src/cmd_mux_unsubscription_client.cpp
 src/cmd_mux_interface.cpp
 )

ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 romea_cmd_mux_msgs
 )

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
 PRIVATE "ROMEA_CMD_MUX_UTILS_BUILDING_DLL")

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(romea_cmd_mux_msgs)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
