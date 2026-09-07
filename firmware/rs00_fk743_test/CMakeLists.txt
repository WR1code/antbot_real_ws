cmake_minimum_required(VERSION 3.16)
project(rs00_protocol_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(rs00_protocol
    firmware/App/Src/rs00_protocol.c
)
target_include_directories(rs00_protocol PUBLIC firmware/App/Inc)
target_compile_options(rs00_protocol PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(test_rs00_protocol tests/test_rs00_protocol.c)
target_link_libraries(test_rs00_protocol PRIVATE rs00_protocol m)
target_compile_options(test_rs00_protocol PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(test_mini_drive_protocol
    tests/test_mini_drive_protocol.c
    firmware/App/Src/mini_drive_protocol.c
)
target_include_directories(test_mini_drive_protocol PRIVATE firmware/App/Inc)
target_compile_options(test_mini_drive_protocol PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)

add_executable(test_drive_controller
    tests/test_drive_controller.c
    firmware/App/Src/mini_drive_protocol.c
    firmware/App/Src/drive_controller.c
)
target_include_directories(test_drive_controller PRIVATE
    tests/mocks
    firmware/App/Inc
)
target_link_libraries(test_drive_controller PRIVATE m)
target_compile_options(test_drive_controller PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)

add_executable(test_steering_controller
    tests/test_steering_controller.c
    firmware/App/Src/chassis_debug.c
    firmware/App/Src/rs00_protocol.c
    firmware/App/Src/rs00_stm32_fdcan.c
    firmware/App/Src/steering_controller.c
)

add_executable(test_chassis_direction
    tests/test_chassis_direction.c
    firmware/App/Src/chassis_direction.c
)
target_include_directories(test_chassis_direction PRIVATE firmware/App/Inc)
target_link_libraries(test_chassis_direction PRIVATE m)
target_compile_options(test_chassis_direction PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)

add_executable(test_chassis_translation_controller
    tests/test_chassis_translation_controller.c
    firmware/App/Src/chassis_debug.c
    firmware/App/Src/chassis_direction.c
    firmware/App/Src/chassis_translation_controller.c
    firmware/App/Src/mini_drive_protocol.c
    firmware/App/Src/drive_controller.c
)

add_executable(test_host_cmd_vel_protocol
    tests/test_host_cmd_vel_protocol.c
    firmware/App/Src/host_cmd_vel_protocol.c
    firmware/App/Src/chassis_uart_ack.c
)
target_include_directories(test_host_cmd_vel_protocol PRIVATE firmware/App/Inc)
target_compile_options(test_host_cmd_vel_protocol PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)
target_include_directories(test_chassis_translation_controller PRIVATE
    tests/mocks
    firmware/App/Inc
)
target_link_libraries(test_chassis_translation_controller PRIVATE m)
target_compile_options(test_chassis_translation_controller PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)
target_compile_definitions(test_chassis_translation_controller PRIVATE
    DRIVE_SAFETY_FEEDBACK_TIMEOUT_MS=100000U
)
target_include_directories(test_steering_controller PRIVATE
    tests/mocks
    firmware/App/Inc
)
target_link_libraries(test_steering_controller PRIVATE m)
target_compile_options(test_steering_controller PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)

add_library(stm32_app_compile_check OBJECT
    firmware/App/Src/chassis_debug.c
    firmware/App/Src/rs00_stm32_fdcan.c
    firmware/App/Src/steering_controller.c
    firmware/App/Src/chassis_direction.c
    firmware/App/Src/chassis_translation_controller.c
    firmware/App/Src/mini_drive_protocol.c
    firmware/App/Src/drive_controller.c
)
target_include_directories(stm32_app_compile_check PRIVATE
    tests/mocks
    firmware/App/Inc
)
target_compile_options(stm32_app_compile_check PRIVATE
    -Wall -Wextra -Wpedantic -Werror
)

# The host tests use the standard assert() macro as their test oracle. Keep
# assertions enabled even when a caller configures this test project as
# Release; otherwise NDEBUG removes both checks and variables used by them.
foreach(test_target
        test_rs00_protocol
        test_mini_drive_protocol
        test_drive_controller
        test_steering_controller
        test_chassis_direction
        test_chassis_translation_controller
        test_host_cmd_vel_protocol)
    target_compile_options(${test_target} PRIVATE -UNDEBUG)
endforeach()

enable_testing()
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME rs00_protocol COMMAND test_rs00_protocol)
add_test(NAME mini_drive_protocol COMMAND test_mini_drive_protocol)
add_test(NAME drive_controller COMMAND test_drive_controller)
add_test(NAME steering_controller COMMAND test_steering_controller)
add_test(NAME chassis_direction COMMAND test_chassis_direction)
add_test(NAME chassis_translation_controller
         COMMAND test_chassis_translation_controller)
add_test(NAME host_cmd_vel_protocol COMMAND test_host_cmd_vel_protocol)
add_test(NAME chassis_uart_python_protocol
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_chassis_uart_protocol.py)
add_test(NAME chassis_dashboard_model
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_chassis_dashboard.py)
add_test(NAME serial_port_resolution
         COMMAND ${Python3_EXECUTABLE}
                 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_serial_port.py)
