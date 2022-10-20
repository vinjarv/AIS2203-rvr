add_executable(opencv_pub_server opencv_pub_server.cpp)

add_executable(opencv_sub_client opencv_sub_client.cpp)

add_executable(CVUI_test CVUI_test.cpp)

add_executable(gamepad_test gamepad_test.cpp)
target_link_libraries(gamepad_test XInput.lib)
