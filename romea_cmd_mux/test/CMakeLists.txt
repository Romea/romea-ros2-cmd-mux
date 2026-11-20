find_package(ament_cmake_gtest REQUIRED)

#get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
#configure_file(test_helper.h.in test_helper.h)
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_cmd_mux_services_test cmd_mux_services_test.cpp)

target_link_libraries(${PROJECT_NAME}_cmd_mux_services_test
    romea_cmd_mux
    rclcpp::rclcpp
    romea_cmd_mux_utils::romea_cmd_mux_utils
    ${diagnostic_msgs_TARGETS}
    ${romea_cmd_mux_msgs_TARGETS}
)

