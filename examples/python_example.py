#!/usr/bin/env python3

# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Import the RoboMaster class from robomaster_can_crontroller_py
from robomaster_can_controller_py import RoboMaster, State
from time import sleep

def callback(state: State):
    """ Callback function to print the RoboMaster states."""
    print(state)


def rainbow(position: int): 
    """ Get the color value for a rainbow scale for a given position."""
    position = min(max(0, position), 255)

    if(position < 85):
        r = position * 3
        g = 255 - position * 3
        b = 0
    elif position < 170:  
        position -= 85
        r = 255 - position * 3
        g = 0
        b = position * 3
    else:
        position -= 170
        r = 0
        g = position * 3
        b = 255 - position * 3

    return r,g,b

def main():
    # create RoboMaster object
    robo = RoboMaster()

    # initialize RoboMaster with can interface 'can0'
    if not robo.init("can0"):
        print("Error: Could not init RoboMaster")
        exit(1)

    # register the callback function to print the robomaster states
    robo.bind(callback)

    # Enable the robomaster to execute drive commands.
    robo.enable()

    # CAUTION: Sleep for a small time to not overfill the can bus communication. 
    sleep(0.025)


    # A small presentation of the LED breath effekt.
    for position in range(0, 512, 20):
        r, g, b = rainbow(position % 256)

        robo.led_breath(r,g,b, 0.4)
        sleep(0.4)

    # Let the robomaster drive forward with incresing wheel speed and increase set led brightness.
    for i in range(0, 200, 2):
        robo.led_on(i, i, i)
        robo.wheel_rpm(i, i, i, i)
        sleep(0.025)

    # Slow the robomaster and decrease the led light. 
    for i in range(200, 0, -2):
        robo.led_on(i, i, i)
        robo.wheel_rpm(i, i, i, i)
        sleep(0.025)

    # Stop the wheel of the robomaster.
    robo.stop()

    # Use the LED Flash of all LED.
    robo.led_flash(255, 0, 0, 0.5)
    sleep(2.0)

    # Turn of the LED.
    robo.led_off()
    sleep(0.01)

    # Disable the robomaster after finish the example.
    robo.disable()

    return 0

if __name__ == '__main__':
    main()
