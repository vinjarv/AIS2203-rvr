#include <iostream>
#include <cmath>
#include <vector>

#include "Windows.h"
#include "Xinput.h"

/*
 * Testing Xbox gamepad functionality with Xinput.h
 * Find number of connected gamepads
 * Read some buttons and set rumble according to left joystick position
 *
 * NB: Xinput.h also requires Windows.h
 * In addition, XInput.lib needs to be linked with target_link_libraries()
 */

int main(){
    int gamepad_count = 0;
    XINPUT_STATE state = {};
    std::vector<int> gamepad_ids{};

    // Check number of connected gamepads, store available user indices
    for (int i = 0; i<XUSER_MAX_COUNT; i++){
        double result = XInputGetState(i, &state);
        if (result == 0x0){ // ERROR_SUCCESS (0x0) means successful completion
            std::cout << "Gamepad " << i << " is connected!" << std::endl;
            gamepad_ids.emplace_back(i);
            gamepad_count++;
        }
    }
    std::cout << gamepad_ids.size() << " gamepad(s) connected" << std::endl;

    // Rumble test
    bool done = gamepad_ids.empty();
    bool rumble_enabled = false;
    while(!done){

        // Try fetching state
        double result = XInputGetState(gamepad_ids[0], &state);
        if (result == 0x0){ // ERROR_SUCCESS (0x0) - read successful
            bool a_pressed = state.Gamepad.wButtons & XINPUT_GAMEPAD_A;
            bool b_pressed = state.Gamepad.wButtons & XINPUT_GAMEPAD_B;

            // Switch on/off output
            if (a_pressed)
                rumble_enabled = true;
            if (b_pressed)
                rumble_enabled = false;

            // Print thumbstick coordinates
            std::cout << "X:" << state.Gamepad.sThumbLX << "\t \t" << " Y:" << state.Gamepad.sThumbLY << std::endl;

            // Find absolute value of thumbstick coordinates
            double lstick_abs = std::hypot(state.Gamepad.sThumbLX,state.Gamepad.sThumbLY);
            unsigned short vibration_level = min(lstick_abs*2, USHRT_MAX); // Rescale and limit

            // Output to controller
            XINPUT_VIBRATION vib;
            if (rumble_enabled)
                vib = {vibration_level, vibration_level};
            else
                vib = {0, 0};
            XInputSetState(gamepad_ids[0], &vib);

        }else{ // On disconnect
            done = true;
        }
    }

    return 0;
}
