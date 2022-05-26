cmake_minimum_required(VERSION 3.5)
project(romea_cmd_mux)

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
find_package(rclcpp_components REQUIRED)
find_package(romea_cmd_mux_msgs REQUIRED)
find_package(romea_cmd_mux_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
 src/cmd_mux.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME}
 PRIVATE "ROMEA_CMD_MUX_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME}
 rclcpp
 rclcpp_components
 romea_cmd_mux_msgs
 romea_cmd_mux_utils
 )

rclcpp_components_register_nodes(${PROJECT_NAME} "romea::CmdMux")

install(TARGETS
 ${PROJECT_NAME}
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin)

add_executable(cmd_mux_node src/cmd_mux_node.cpp)
target_link_libraries(cmd_mux_node ${PROJECT_NAME})
ament_target_dependencies(cmd_mux_node rclcpp)

install(TARGETS
 cmd_mux_node
 DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

ament_package()
