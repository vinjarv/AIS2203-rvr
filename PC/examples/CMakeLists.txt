
add_executable(GUI_v1_1 GUI_v1_1.cpp)

add_executable(gamepad_test gamepad_test.cpp)
target_link_libraries(gamepad_test XInput.lib)

add_executable(serial_test serial_test.cpp)
