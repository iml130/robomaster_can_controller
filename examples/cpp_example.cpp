// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/robomaster.h"
#include <iostream>

/**
 * @brief Callback to print the state of the RoboMaster.
 */
void callback(const robomaster_can_controller::DataRoboMasterState &state)
{
    std::cout << state << std::endl;
}

/**
 * @brief Get the color value for a rainbow scale for a given position.
 * 
 * @param position The position of the reinbow scale.
 * @param r Color red.
 * @param g Color green.
 * @param b Color blue.
 */
void rainbow(uint8_t position, uint8_t &r, uint8_t &g, uint8_t &b) {
    if(position < 85) {
        r = position * 3;
        g = 255 - position * 3;
        b = 0;
    }
    else if(position < 170)  
    {
        position -= 85;
        r = 255 - position * 3;
        g = 0;
        b = position * 3;
    } 
    else 
    {
        position -= 170;
        r = 0;
        g = position * 3;
        b = 255 - position * 3;
    }
}

/**
 * Example for the usage of the robomaster_can_controller library.
 */
int main()
{
    // Using namespace for simplicity
    using namespace robomaster_can_controller;

    // Create the robomaster interface class.
    RoboMaster robomaster;

    // Init the robomaster with the can0 device als default.
    if(robomaster.init())
    {
        // bind the callback function to print state of the robomaster like wheel position, imu data, etc.
        robomaster.bindCallbackDataRobotMasterState(callback);

        // Enable the robomaster to execute drive commands.
        robomaster.commandEnable();

        // CAUTION: Sleep for a small time to not overfill the can bus communication. 
        std::this_thread::sleep_for(std::chrono::milliseconds(25));

        uint8_t r,g,b;
        
        // A small presentation of the LED breath effekt.
        for (size_t i = 0; i < 512; i += 20)
        {
            rainbow(i,r,g,b);
            robomaster.commandLedBreath(LED_MASK_FRONT, r,g,b, float(0.4), float(0.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 

            rainbow(i + 64,r,g,b);
            robomaster.commandLedBreath(LED_MASK_RIGHT, r,g,b, float(0.4), float(0.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            rainbow(i + 128,r,g,b);
            robomaster.commandLedBreath(LED_MASK_BACK, r,g,b, float(0.4), float(0.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            rainbow(i + 195,r,g,b);
            robomaster.commandLedBreath(LED_MASK_LEFT, r,g,b, float(0.4), float(0.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    
        // Let the robomaster drive forward with incresing wheel speed and increase set led brightness.
        for (size_t i = 0; i < 100; i++)
        {
            robomaster.commandLedOn(LED_MASK_ALL, i*2, i*2, i*2);
            robomaster.commandWheelRPM(i*2, i*2, i*2, i*2);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        // Slow the robomaster and decrease the led light. 
        for (size_t i = 100; i --> 0;)
        {
            robomaster.commandLedOn(LED_MASK_ALL, i*2, i*2, i*2);
            robomaster.commandWheelRPM(i*2, i*2, i*2, i*2);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        // Stop the wheel of the robomaster.
        robomaster.commandStop();

        // Use the LED Flash of all LED.
        robomaster.commandLedFlash(LED_MASK_ALL, 255, 0, 0, float(0.4), float(0.1));
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Turn of the LED.
        robomaster.commandLedOff(LED_MASK_ALL);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Disable the robomaster after finish the example.
        robomaster.commandDisable();

        return 0;
    }

    std::cerr << "Could not init the robomaster!" << std::endl;

    return 1;
}
