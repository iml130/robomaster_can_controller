// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_ROBOMASTER_H_
#define ROBOMASTER_CAN_CONTROLLER_ROBOMASTER_H_

#include "handler.h"
#include "definitions.h"
#include "data.h"

namespace robomaster_can_controller
{

/**
 * @brief This class manage the controll of the RoboMaster vai can socket.
 * 
 */
class RoboMaster
{
private:
    /**
     * @brief Handler class for the RoboMaster message io and thread managment.
     */
    Handler handler_;
    /**
     * @brief Callbackfunction to trigger when new RoboMasterState data arte received.
     */
    std::function<void(const DataRoboMasterState &)> callback_data_robomaster_state_;
    /**
     * @brief Counter for the message sequence of the led messages.
     */
    uint16_t counter_drive_;
    /**
     * @brief Counter for the message sequence of the drive messages.
     */
    uint16_t counter_led_;

    /**
     * @brief The boot sequence to configure the RoboMasterState messages.
     */
    void bootSequence();

    /**
     * @brief Decode the RoboMasterState message and trigger the callbackfunction.
     * 
     * @param msg The RoboMasterState message.
     */
    void decodeDataRoboMasterState(const Message &msg);
    /**
     * @brief look up the message index byte for the given servo index.
     * 
     * @param index Servo index from 1 to 4.
     * @return uint8_t Message index byte.
     */
    uint8_t servoIndexTable(const uint8_t index);
public:
    /**
     * @brief Constructor of the RoboMaster class.
     */
    RoboMaster(/* args */);
    /**
     * @brief Destructor of the RoboMaster class.
     */
    ~RoboMaster();

    /**
     * @brief Enable the RoboMaster to drive. Give current to the motor.
     */
    void commandEnable();

    /**
     * @brief Disable the RoboMaster to drive. Take the current of the motor.
     */
    void commandDisable();

    /**
     * @brief Drive the RoboMaster with the given velocities.
     * 
     * @param x Linear x velocity in m/s.
     * @param y Linear y velocity in m/s.
     * @param z Angular velocity in radiant/s.
     */
    void commandVelocity(const float x, const float y, const float z);

    /**
     * @brief Controll each individuell wheel of the RoboMaster in rpm.
     * 
     * @param fr Front right wheel in rpm.
     * @param fl Front left wheel in rpm.
     * @param rl Rear left wheel in rpm.
     * @param rr Rear right wheel in rpm.
     */
    void commandWheelRPM(const int16_t fr, const int16_t fl, const int16_t rl, const int16_t rr);

    /**
     * @brief Stop the RoboMaster with zero velocities.
     */
    void commandStop();

    /**
     * @brief Set the Led off by the given mask.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     */
    void commandLedOff(const uint16_t mask);

    /**
     * @brief Set the Led on by the given mask.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     */
    void commandLedOn(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b);

    /**
     * @brief @brief Set the led with a breath effect with given mask and timer.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     * @param r Red value colour between 0-255.
     * @param g Green value colour between 0-255.
     * @param b Blue value colour between 0-255.
     * @param t_rise The rising time of the led in seconds.
     * @param t_down The falling time of the led in seconds.
     */
    void commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_rise, const uint16_t t_down);

    /**
     * @brief Set the led with a breath effect with given mask and timer.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     * @param r Red value colour between 0-255.
     * @param g Green value colour between 0-255.
     * @param b Blue value colour between 0-255.
     * @param t_rise The rising time of the led in seconds in milliseconds.
     * @param t_down The falling time of the led in seconds in milliseconds.
     */
    void commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_rise, const float t_down);

    /**
     * @brief Set the led with a breath effect with given mask and rate.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     * @param r Red value colour between 0-255.
     * @param g Green value colour between 0-255.
     * @param b Blue value colour between 0-255.
     * @param rate The rate of the breath effect.
     */
    void commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate);

    /**
     * @brief Set the led with a flash effect with given mask and timer.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     * @param r Red value colour between 0-255.
     * @param g Green value colour between 0-255.
     * @param b Blue value colour between 0-255.
     * @param t_on The on time of the led in seconds.
     * @param t_off The off time of the led in seconds.
     */
    void commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_on, const uint16_t t_off);

    /**
     * @brief Set the led with a flash effect with given mask and timer.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     * @param r Red value colour between 0-255.
     * @param g Green value colour between 0-255.
     * @param b Blue value colour between 0-255.
     * @param t_on The on time of the led in milliseconds.
     * @param t_off The off time of the led in milliseconds.
     */
    void commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_on, const float t_off);

    /**
     * @brief Set the led with a flash effect with given mask and rate.
     * 
     * @param mask Mask for selecting the led. LED_MASK_ALL for all Leds or select specific led with LED_MASK_FRONT | LED_MASK_BACK etc.
     * @param r Red value colour between 0-255.
     * @param g Green value colour between 0-255.
     * @param b Blue value colour between 0-255.
     * @param rate The rate of the flash effect.
     */
    void commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate);

    /**
     * @brief Set the angle of the servo by the given index.
     * 
     * @param index Index of the servo from 1 to 3.
     * @param angle Angle between -180-180.
     */
    void commandServoAngle(const uint8_t index, const int32_t angle);

    /**
     * @brief Set the angle of the servo by the given index.
     * 
     * @param index Index of the servo from 1 to 3.
     * @param angle Angle between -180.0-180.0.
     */
    void commandServoAngle(const uint8_t index, const float angle);

    /**
     * @brief Set the speed  of the servo by the given index.
     * 
     * @param index Index of the servo from 1 to 3.
     * @param speed Speed between -12-12 rotation per minute.
     */
    void commandServoSpeed(const uint8_t index, const int32_t speed);

    /**
     * @brief Set the speed of the servo by the given index.
     * 
     * @param index Index of the servo from 1 to 3.
     * @param speed Speed between -12.0-12.0 roation per minute.
     */
    void commandServoSpeed(const uint8_t index, const float speed);

    /**
     * @brief Stop the servo by the given index.
     * 
     * @param index Index of the servo from 1 to 3.
     */
    void commandServoStop(const uint8_t index);

    /**
     * @brief Set the pwm frequence of the motion controller channel in the range of 0-50000 hertz.
     * 
     * @param freq1 Frequence of the pwm channel 1.
     * @param freq2 Frequence of the pwm channel 2.
     * @param freq3 Frequence of the pwm channel 3.
     * @param freq4 Frequence of the pwm channel 4.
     * @param freq5 Frequence of the pwm channel 5.
     * @param freq6 Frequence of the pwm channel 6.
     */
    void commandPwmFrequenz(const uint16_t freq1, const uint16_t freq2, const uint16_t freq3, const uint16_t freq4, const uint16_t freq5, const uint16_t freq6);

    /**
     * @brief Set the pwm dutycycle of the moton controller channels in the range of 0-100 percent.
     * 
     * @param value1 Dutycycle of the pwm channel 1.
     * @param value2 Dutycycle of the pwm channel 2.
     * @param value3 Dutycycle of the pwm channel 3.
     * @param value4 Dutycycle of the pwm channel 4.
     * @param value5 Dutycycle of the pwm channel 5.
     * @param value6 Dutycycle of the pwm channel 6.
     */
    void commandPwmValue(const uint16_t value1, const uint16_t value2, const uint16_t value3, const uint16_t value4, const uint16_t value5, const uint16_t value6);

    /**
     * @brief Set the pwm dutycycle of the moton controller channels in the range of 0.0-100.0 percent
     * 
     * @param value1 Dutycycle of the pwm channel 1.
     * @param value2 Dutycycle of the pwm channel 2.
     * @param value3 Dutycycle of the pwm channel 3.
     * @param value4 Dutycycle of the pwm channel 4.
     * @param value5 Dutycycle of the pwm channel 5.
     * @param value6 Dutycycle of the pwm channel 6.
     */
    void commandPwmValue(const float value1, const float value2, const float value3, const float value4, const float value5, const float value6);

    /**
     * @brief Bind a function to the callbackfunction which get triggered when a new RoboMasterState message is received.
     * 
     * @param func Function to bind as callback.
     */
    void bindCallbackDataRobotMasterState(std::function<void(const DataRoboMasterState&)> func);

    /**
     * @brief Init the RoboMaster can socket to communicate with the motion controller.
     * 
     * @param can_interface Can interface name.
     * @return true, by success.
     * @return false, when initialization failed.
     */
    bool init(const std::string &can_interface="can0");

    /**
     * @brief True when the robomaster is sucessful initiliased and ready to receive and send messages.
     * 
     * @return true if the robomaster is successufl initiliased and is running. false when a can error is happend.
     */
    bool isRunning() const;
};

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_ROBOMASTER_H_