add_executable(CVUI_test CVUI_test.cpp)

add_executable(gamepad_test gamepad_test.cpp)
target_link_libraries(gamepad_test XInput.lib)
