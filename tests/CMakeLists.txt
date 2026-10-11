cmake_minimum_required(VERSION 3.22)
project(balancing_robot_host_tests LANGUAGES CXX)

enable_testing()

add_executable(motors_test motors_test.cpp ../main/motors_l298n.cpp)
target_include_directories(motors_test PRIVATE stubs ../main)
target_compile_features(motors_test PRIVATE cxx_std_17)
target_compile_options(motors_test PRIVATE -Wall -Wextra -Werror)

foreach(scenario invalid_config startup commands single_motor_test full_duty invalid_command
                 mapping_identity mapping_calibrated mapping_limit invalid_mapping invalid_mapped_command
                 gpio_failure duty_failure update_failure stop_failure
                 timer_failure channel_failure)
    add_test(NAME motors_${scenario} COMMAND motors_test ${scenario})
endforeach()

add_executable(control_test control_test.cpp ../main/control.cpp)
target_include_directories(control_test PRIVATE ../main)
target_compile_features(control_test PRIVATE cxx_std_17)
target_compile_options(control_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME control COMMAND control_test)

add_executable(wireless_test wireless_test.cpp ../main/wireless_protocol.cpp)
target_include_directories(wireless_test PRIVATE ../main)
target_compile_features(wireless_test PRIVATE cxx_std_17)
target_compile_options(wireless_test PRIVATE -Wall -Wextra -Werror)
foreach(scenario parsing stop_priority timeout reconnect)
    add_test(NAME wireless_${scenario} COMMAND wireless_test ${scenario})
endforeach()

find_program(NODE_EXECUTABLE node)
if(NODE_EXECUTABLE)
    add_test(NAME web_controls COMMAND ${NODE_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/web_test.cjs)
endif()
