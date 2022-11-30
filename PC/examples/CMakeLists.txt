
add_executable(GUI_v1_1 GUI_v1_1.cpp)

add_executable(gamepad_test gamepad_test.cpp)
target_link_libraries(gamepad_test XInput.lib)

add_executable(serial_test serial_joystick_test.cpp)

add_executable(json_map_parse json_map_parse.cpp)

add_executable(udp_client udp_client_img_test.cpp)

add_executable(aruco_est_pos aruco_est_pos.cpp)

