add_executable(GUI_v1_1 GUI_v1_1.cpp)

add_executable(joystick_test serial_joystick_test.cpp)

add_executable(joystick_manual_control serial_manual_control.cpp)

add_executable(json_map_parse json_map_parse.cpp)

add_executable(udp_img_test udp_client_img_test.cpp)
target_link_libraries(udp_img_test imagestream)

add_executable(get_servodata servodata.cpp)
