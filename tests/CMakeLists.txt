cmake_minimum_required(VERSION 3.22)
project(balancing_robot_host_tests LANGUAGES CXX)

enable_testing()

add_executable(motors_test motors_test.cpp ../main/motors_l298n.cpp)
target_include_directories(motors_test PRIVATE stubs ../main)
target_compile_features(motors_test PRIVATE cxx_std_17)
target_compile_options(motors_test PRIVATE -Wall -Wextra -Werror)

foreach(scenario invalid_config startup commands single_motor_test full_duty invalid_command
                 gpio_failure duty_failure update_failure stop_failure
                 timer_failure channel_failure)
    add_test(NAME motors_${scenario} COMMAND motors_test ${scenario})
endforeach()
