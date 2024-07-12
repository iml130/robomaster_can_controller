# robomaster_can_controller

**robomaster_can_controller** is a library for controlling the RoboMaster via a CAN Bus interface, designed for anyone who wants to use the RoboMaster's chassis as a mobile robot platform. This library provides a simple C++ interface, also accessible via Python3 through pybind11 bindings.

To use the library, you need a computer with a CAN bus interface, such as NVIDIA Jetson boards or a Raspberry Pi with a CAN Bus module extension. One advantage of direct communication over the CAN Bus is that the intelligent controller is no longer needed, eliminating detours via WiFi, USB, etc., to the intelligent controller and back to the motion controller. Also, the 12-volt power supply from the CAN bus can be used as a power source.

Currently, the library is limited to controlling the RoboMaster chassis and LEDs.

**Caution when using this library:**
The user decides when to send a command, and the library can handle multiple commands using a small queue. However, the CAN Bus is limited by its bandwidth. To prevent the CAN Bus from overfilling with data, it is recommended to send a command only every ~10-20 milliseconds. Otherwise, commands may be ignored, and the RoboMaster could behave unintentionally.

## Requirements

For Python with pybind11 bindings:

```sh
sudo apt install pybind11-dev ninja-build
```

To run tests with gtest:

```sh
sudo apt install gtest-dev
```

## Usage C++

Build the library with python bindings and tests enabled: 

```sh
cd robomaster_can_controller
mkdir build
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_RUN_TESTS=ON
make 
```

Run the example in the build directory. Ensure the RoboMaster has enough space to move or place it on a box so the wheels don't touch the ground:

```sh
./robomaster_can_controller_example
```

Add **robomaster_can_controller** to your project as a submodule to use it with **C++**.

## Usage Python

For **Python**, use pip to build and install **robomaster_can_controller**. Ensure **ninja-build** and **pybind11-dev** are installed:

```sh
cd robomaster_can_controller
pip3 install .
```

Run the Python example. Ensure the RoboMaster has enough space to move or place it on a box so the wheels don't touch the ground:

```sh
python3 examples/python_example.py
```

## Class RoboMaster C++

The RoboMaster class provides simple access to control the chassis and LEDs.

| Method | Description |
| ------ | ----------- |
| `bool init(const std::string &can_interface="can0")` | Initialize the RoboMaster by opening the CAN bus with the given can_interface. Returns true on success. |
| `bool isRunning() const` | Returns true when the RoboMaster is successfully initialized and running. Switches to false when an error occurs. |
| `void bindCallbackDataRobotMasterState(std::function< void(const DataRoboMasterState&)> func)` | Register a callback function that returns the states of the RoboMaster at a rate of 50 Hertz. |
| `void commandEnable()` | Enable the RoboMaster and supply power to the motors. | 
| `void commandDisable()` | Disable the RoboMaster and stop supplying power to the motors. |
| `void commandStop()` | Immediately stop the wheels. | 
| `void commandWheelRPM(const int16_t fr, const int16_t fl, const int16_t rl, const int16_t rr)` | Set the wheel speed in RPM for front right, front left, rear left, and rear right [-1000, 1000]. |
| `void commandVelocity(const float x, const float y, const float z)` | Set the linear x, linear y, and angular z velocity. The velocity and acceleration limits are handled by the motion controller's config. |
| `void commandLedOff(const uint16_t mask)` | Turn off the LEDs with the selected mask. Mask flags can be concatenated and represented as enums: LED_MASK_ALL, LED_MASK_FRONT, LED_MASK_RIGHT, LED_MASK_LEFT, and LED_MASK_BACK. |
| `void commandLedOn(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b)` | Turn on the LEDs with the selected mask and the given RGB values [0, 255]. |
| `void commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_rise, const uint16_t t_down)` | Create a breathing effect on the LEDs with the selected mask and given RGB values. The LEDs' rise and down phases are set in milliseconds [0, 60000]. | 
| `void commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_rise, const float t_down)` | Create a breathing effect on the LEDs with the selected mask and given RGB values. The LEDs' rise and down phases are set in seconds [0.0, 60.0]. |
| `void commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate)` | Create a breathing effect on the LEDs with the selected mask and given RGB values. The rate is set in seconds [0.0, 60.0]. |
| `void commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_on, const uint16_t t_off)` | Make the LEDs flash with the selected mask and given RGB values. The LEDs' on and off phases are set in milliseconds [0, 60000]. |
| `void commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_on, const float t_off)` | Make the LEDs flash with the selected mask and given RGB values. The LEDs' on and off phases are set in seconds [0.0, 60.0]. |
| `void commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate)` | Make the LEDs flash with the selected mask and given RGB values. The rate is set in seconds [0.0, 60.0]. |
| `void commandServoAngle(const uint8_t index, const int32_t angle)` | Set the angle in degree [-180, 180] of the servo by the given index [1, 3]. |
| `void commandServoAngle(const uint8_t index, const float angle)` | Set the angle of in degree [-180, 180] the servo by the given index [1, 3]. |
| `void commandServoSpeed(const uint8_t index, const int32_t speed)` | Set the speed in rotation per minutes [-12, 12] of the servo by the given index [1, 3]. |
| `void commandServoSpeed(const uint8_t index, const float speed)` | Set the speed in rotation per minutes [-12.0, 12.0] of the servo by the given index [1, 3]. |
| `void commandServoStop(const uint8_t index)` | Stop the servo by the given index [1, 3]. |
| `void commandPwmFrequenz(const uint16_t freq1, const uint16_t freq2, const uint16_t freq3, const uint16_t freq4, const uint16_t freq5, const uint16_t freq6)` | Set the PWM frequency of the motion controller channels in the range of 0-50000 Hz. |
| `void commandPwmValue(const uint16_t value1, const uint16_t value2, const uint16_t value3, const uint16_t value4, const uint16_t value5, const uint16_t value6)` | Set the PWM duty cycle of the motion controller channels in the range of 0-100 percent. |
| `void commandPwmValue(const float value1, const float value2, const float value3, const float value4, const float value5, const float value6)` | Set the PWM duty cycle of the motion controller channels in the range of 0.0-100.0 percent. |

## Class RoboMaster Python

The Python class RoboMaster is implemented as Python bindings of the C++ class with fewer LED functions.

| Method | Description |
| ------ | ----------- |
| `init(can_interface: str="can0")` | Initialize the RoboMaster by opening the CAN bus with the given can_interface. Returns true on success. |
| `is_running()` | Returns true when the RoboMaster is successfully initialized and running. Switches to false when an error occurs. |
| `bind(func)` | Register a callback function that returns the states of the RoboMaster at a rate of 50 Hertz. |
| `enable()` | Enable the RoboMaster and supply power to the motors. | 
| `disable()` | Disable the RoboMaster and stop supplying power to the motors. |
| `stop()` | Immediately stop the wheels. | 
| `wheelRPM(fr: int, fl: int, rl: int, rr: int)` | Set the wheel speed in RPM for front right, front left, rear left, and rear right [-1000, 1000]. |
| `velocity(x: float, y: float, z: float)` | Set the linear x, linear y, and angular z velocity. The velocity and acceleration limits are handled by the motion controller's config. |
| `led_off()` | Turn off all LEDs. |
| `led_on(r: int, g: int, b: int)` | Turn on all LEDs with the RGB values [0, 255]. |
| `led_breath(r: int, g: int, b: int, rate: float)` | Create a breathing effect on all LEDs with the RGB values [0, 255] at a rate in seconds [0.0, 60.0]. | 
| `led_flash(r: int, g: int, b: int, rate: float)` | Make all LEDs flash with the RGB values [0, 255] at a rate in seconds [0.0, 60.0]. | 
| `servo_angle(id: int, angle: float)` | Set the angle in degree [-180.0, 180.0] of the servo by the given index [1, 3]. |
| `servo_speed(id: int, speed: float)` | Set the speed in rotation per second [-12.0, 12.0] of the servo by the given index [1, 3]. |
| `servo_stop(id: int)` | Stop the servo by the given index. |
| `pwm_frequenz(freq1: int=0, freq2: int=0, freq3: int=0, freq4: int=0, freq5: int=0, freq6: int=0)` | Set the PWM frequency of the motion controller channels in the range of 0-50000 Hz. |
| `pwm_value(v1: float=0.0, v2: float=0.0, v3: float=0.0, v4: float=0.0, v5: float=0.0, v6: float=0.0)` | Set the PWM duty cycle of the motion controller channels in the range of 0.0-100.0 percent. |

## Struct DataRoboMasterState

The struct DataRoboMasterState represents the state of the RoboMaster, passed by the registered callback function.

| struct (C++) | class (Python) | Description |
| ------------ | -------------- | ----------- |
| `struct DataBattery battery` | `Battery`| Contains the state of the RoboMaster's battery. |
| `struct DataEsc esc` | `Esc`| Contains the states of the wheels. |
| `struct DataImu imu` | `Imu`| Contains the sensor data of the IMU in the motion controller. |
| `struct DataVelocity velocity` | `Velocity`| Contains the estimated velocities of the RoboMaster from the motion controller. |
| `struct DataPosition position` | `Position`| Contains the odometry data of the motion controller. |
| `struct DataAttitude attitude` | `Attitude`| Contains the attitude of the RoboMaster estimated by the motion controller. |

## Struct DataBattery

Information about the RoboMaster battery.

| Name | Datatype | Description |
| ---- | -------- | ----------- |
| `adc_value` | `uint16_t` | ADC value of the battery in millivolts. |
| `temperature` | `int16_t` | Temperature in 10*e-1. |
| `current` | `int32_t` | Current in milliamperes. |
| `percent` | `uint8_t` | Battery percentage [0, 100]. |
| `recv` | `uint8_t` | N/A |

## Struct DataEsc

This struct represents the state of the wheels. The wheel order in the arrays is front left, front right, rear left, and rear right.

| Name | Datatype | Description |
| ---- | -------- | ----------- |
| `speed` | `int16_t[4]` | Speed in RPM -> value range -8192~8191. |
| `angle` | `int16_t[4]` | Angular position -> value range 0-32767 maps to -> 0-360. |
| `timestamp` | `uint32_t[4]` | Timestamp -> units N/A. |
| `state` | `uint8_t[4]` | State of the ESC -> units N/A. |

## Struct DataImu

The measured sensor data from the IMU in the motion controller.

| Name | Datatype | Description |
| ---- | -------- | ----------- |
| `acc_x` | `float` | Acceleration on x-axis in 9.81 m/s². |
| `acc_y` | `float` | Acceleration on y-axis in 9.81 m/s². | 
| `acc_z` | `float` | Acceleration on z-axis in 9.81 m/s². |
| `gyro_x` | `float` | Angular velocity on x-axis in radians. |
| `gyro_y` | `float` | Angular velocity on y-axis in radians. |
| `gyro_z` | `float` | Angular velocity on z-axis in radians. |

## Struct DataVelocity

Information on the chassis velocities measured by the motion controller.

| Name | Datatype | Description |
| ---- | -------- | ----------- |
| `vgx` | `float` | Velocity in m/s on the x-axis in the global coordinate system where the RoboMaster was powered on. |
| `vgy` | `float` | Velocity in m/s on the y-axis in the global coordinate system where the RoboMaster was powered on. |
| `vgz` | `float` | Velocity in m/s on the z-axis in the global coordinate system where the RoboMaster was powered on. Always 0.0. |
| `vbx` | `float` | Velocity in m/s on the x-axis in the local coordinate system. |
| `vby` | `float` | Velocity in m/s on the y-axis in the local coordinate system. |
| `vbz` | `float` | Velocity in m/s on the z-axis in the local coordinate system. Always 0.0. |

## Struct DataPosition

This struct represents the measured position of the RoboMaster since it was powered on.

| Name | Datatype | Description |
| ---- | -------- | ----------- |
| `x` | `float` | X position on the x-axis in the global coordinate system where the RoboMaster was powered on. |
| `y` | `float` | Y position on the y-axis in the global coordinate system where the RoboMaster was powered on. |
| `z` | `float` | Z position on the z-axis in the global coordinate system where the RoboMaster was powered on. Always 0.0. |

## Struct DataAttitude

Information on the attitude measured by the motion controller.

| Name | Datatype | Description |
| ---- | -------- | ----------- |
| `roll` | `float` | Roll in degrees. |
| `pitch` | `float` | Pitch in degrees. |
| `yaw` | `float` | Yaw in degrees. |

## Licensing

This project is licensed under the MIT License - see the LICENSE file for details.
