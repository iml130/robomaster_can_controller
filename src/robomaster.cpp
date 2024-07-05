// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/robomaster.h"
#include "robomaster_can_controller/definitions.h"
#include "robomaster_can_controller/utils.h"

#include <cmath>

namespace robomaster_can_controller
{

RoboMaster::RoboMaster():counter_drive_(0), counter_led_(0)
{
    this->handler_.bindCallbackDataRoboMasterState(std::bind(&RoboMaster::decodeDataRoboMasterState, this, std::placeholders::_1));
}

RoboMaster::~RoboMaster()
{

}

void RoboMaster::bindCallbackDataRobotMasterState(std::function<void(const DataRoboMasterState&)> func)
{
    this->callback_data_robomaster_state_ = func;
}

void RoboMaster::bootSequence()
{
    // configuration for sub
    //   imu
    //   esc
    //   velocity
    //   battery
    //   attitude
    //   position
    this->handler_.pushMessage(Message( DEVICE_ID_INTELLI_CONTROLLER, 0x0309,     0, { 0x40, 0x48, 0x04, 0x00, 0x09, 0x00 }));
    this->handler_.pushMessage(Message( DEVICE_ID_INTELLI_CONTROLLER, 0x0309,     1, { 0x40, 0x48, 0x01, 0x09, 0x00, 0x00, 0x00, 0x03 }));
    this->handler_.pushMessage(Message( DEVICE_ID_INTELLI_CONTROLLER, 0x0309,     2, { 0x40, 0x48, 0x03, 0x09, 0x01, 0x03, 0x00, 0x07, 0xa7, 0x02, 0x29, 0x88, 0x03, 0x00, 0x02, 0x00, 0x66, 0x3e, 0x3e, 0x4c, 0x03, 0x00, 0x02, 0x00, 0xfb, 0xdc, 0xf5, 0xd7, 0x03, 0x00, 0x02, 0x00, 0x09, 0xa3, 0x26, 0xe2, 0x03, 0x00, 0x02, 0x00, 0xf4, 0x1d, 0x1c, 0xdc, 0x03, 0x00, 0x02, 0x00, 0x42, 0xee, 0x13, 0x1d, 0x03, 0x00, 0x02, 0x00, 0xb3, 0xf7, 0xe6, 0x47, 0x03, 0x00, 0x02, 0x00, 0x32, 0x00 })); 
}

void RoboMaster::commandEnable()
{
    this->handler_.pushMessage(Message( DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x01 }));
}

void RoboMaster::commandDisable()
{
    this->handler_.pushMessage(Message( DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x00 }));
}

void RoboMaster::commandStop()
{
    const Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, {0x40, 0x3F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandWheelRPM(const int16_t fr, const int16_t fl, const int16_t rl, const int16_t rr)
{
    const int16_t w1 = clip<int16_t>( fr, -1000, 1000);
    const int16_t w2 = clip<int16_t>( fl, -1000, 1000);
    const int16_t w3 = clip<int16_t>( rl, -1000, 1000);
    const int16_t w4 = clip<int16_t>( rr, -1000, 1000);

    Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, {0x40, 0x3F, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

    msg.setValueInt16(3, w1);
    msg.setValueInt16(5, w2);
    msg.setValueInt16(7, w3);
    msg.setValueInt16(9, w4);

    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandVelocity(const float x, const float y, const float z)
{
    const float cx = clip<float>(x,   -3.5f,   3.5f);
    const float cy = clip<float>(y,   -3.5f,   3.5f);
    const float cz = clip<float>(z, -600.0f, 600.0f);

    Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, {0x00, 0x3f, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

    msg.setValueFloat( 3, cx);
    msg.setValueFloat( 7, cy);
    msg.setValueFloat(11, cz);

    this->handler_.pushMessage(std::move(msg));
}

bool RoboMaster::init(const std::string &can_interface)
{
    if (this->handler_.init(can_interface))
    {
        this->bootSequence();
        return true;
    }

    return false;
}

void RoboMaster::commandLedOff(const uint16_t mask)
{
    // LED msg                                                                HEADER          | effectmode|   0 | r   | g   | b   |   0 | timer1    | timer2    | mask
    Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 3, 0x70); // effect mode off
    msg.setValueUInt16(14, mask);

    this->handler_.pushMessage(std::move(msg));
}


void RoboMaster::commandLedOn( const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b)
{
    // LED msg                                                                HEADER          | effectmode|   0 | r   | g   | b   |   0 | timer1    | timer2    | mask
    Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 3, 0x71); // effect mode on
    msg.setValueUInt8 ( 6, r);
    msg.setValueUInt8 ( 7, g);
    msg.setValueUInt8 ( 8, b);
    msg.setValueUInt16(14, mask);

    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_rise, const uint16_t t_down)
{
    // LED msg                                                                HEADER          | effectmode|   0 | r   | g   | b   |   0 | timer1    | timer2    | mask
    Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 3, 0x72); // effect mode breath
    msg.setValueUInt8 ( 6, r);
    msg.setValueUInt8 ( 7, g);
    msg.setValueUInt8 ( 8, b);
    msg.setValueUInt16(10, t_rise);
    msg.setValueUInt16(12, t_down);
    msg.setValueUInt16(14, mask);

    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_rise, const float t_down)
{
    const uint16_t rise = clip<float>(t_rise * 1000.0f, 0.0f, 60000.0f); // near to the 65535
    const uint16_t down = clip<float>(t_down * 1000.0f, 0.0f, 60000.0f); // near to the 65535

    this->commandLedBreath(mask, r, g, b, rise, down);
}

void RoboMaster::commandLedBreath(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate)
{
    const uint16_t freq = clip<float>(rate * 1000.0f, 0.0f, 60000.0f); // near to the 65535

    this->commandLedBreath(mask, r, g, b, freq, freq);
}

void RoboMaster::commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const uint16_t t_on, const uint16_t t_off)
{
    // LED msg                                                                HEADER          | effectmode|   0 | r   | g   | b   |   0 | timer1    | timer2    | mask
    Message msg(DEVICE_ID_INTELLI_CONTROLLER, 0x1809, this->counter_led_++, { 0x00, 0x3f, 0x32, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 3, 0x73); // effect mode flash
    msg.setValueUInt8 ( 6, r);
    msg.setValueUInt8 ( 7, g);
    msg.setValueUInt8 ( 8, b);
    msg.setValueUInt16(10, t_on);
    msg.setValueUInt16(12, t_off);
    msg.setValueUInt16(14, mask);

    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float t_on, const float t_off)
{
    const uint16_t on  = clip<float>(t_on * 1000.0f, 0.0f, 60000.0f); // near to the 65535
    const uint16_t off = clip<float>(t_off * 1000.0f, 0.0f, 60000.0f); // near to the 65535

    this->commandLedFlash(mask, r, g, b, on, off);
}

void RoboMaster::commandLedFlash(const uint16_t mask, const uint8_t r, const uint8_t g, const uint8_t b, const float rate)
{
    const uint16_t freq = clip<float>(rate * 1000.0f, 0.0f, 60000.0f); // near to the 65535

    this->commandLedFlash(mask, r, g, b, freq, freq);
}

void RoboMaster::commandServoAngle(const uint8_t index, const int32_t angle)
{
    if(1 <= index && index <= 3 && -180 <= angle && angle <= 180)
    {
        // SERVO ANGLE msg                                                           HEADER          | cnt | ?   | ind | angle            
        Message msg( DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3f, 0xb7, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 });

        msg.setValueUInt8(5, this->servoIndexTable(index));
        msg.setValueInt32(6, (angle + 180)* 10);

        std::cout << (angle + 180)* 10 << std::endl;

        this->handler_.pushMessage(std::move(msg));
    }
}

void RoboMaster::commandServoAngle(const uint8_t index, const float angle)
{
    if(1 <= index && index <= 3 && -180.0f <= angle && angle <= 180.0f)
    {
        // SERVO ANGLE msg                                                           HEADER          | cnt | ?   | ind | angle            
        Message msg( DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3f, 0xb7, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 });

        msg.setValueUInt8(5, this->servoIndexTable(index));
        msg.setValueInt32(6, static_cast<int16_t>(std::roundf((angle + 180.0f) * 10.0)));

        this->handler_.pushMessage(std::move(msg));
    }
}

void RoboMaster::commandServoSpeed(const uint8_t index, const int32_t speed)
{
    if(1 <= index && index <= 3 && -12 <= speed && speed <= 12)
    {   
        // SERVO MODE msg                                                                HEADER          | ind | mode
        Message msg_mode ( DEVICE_ID_INTELLI_CONTROLLER, 0xa3c9, this->counter_drive_, { 0x40, 0x33, 0x16, 0x00, 0x01 });

        msg_mode.setValueUInt8(3, this->servoIndexTable(index));

        this->handler_.pushMessage(std::move(msg_mode));
        

        // SERVO SPEED msg                                                               HEADER          | ind | ena | speed
        Message msg_speed( DEVICE_ID_INTELLI_CONTROLLER, 0xa3c9, this->counter_drive_, { 0x40, 0x33, 0x17, 0x00, 0x01, 0x00, 0x00 });

        msg_speed.setValueUInt8( 3, this->servoIndexTable(index));
        msg_speed.setValueUInt16(5, static_cast<uint16_t>(((speed + 49) * 900) / 98));

        this->handler_.pushMessage(std::move(msg_speed));
    }
}

void RoboMaster::commandServoSpeed(const uint8_t index, const float speed)
{
    if(1 <= index && index <= 3 && -12.0f <= speed && speed <= 12.0f)
    {   
        // SERVO MODE msg                                                                HEADER          | ind | mode
        Message msg_mode ( DEVICE_ID_INTELLI_CONTROLLER, 0xa3c9, this->counter_drive_, { 0x40, 0x33, 0x16, 0x00, 0x01 });

        msg_mode.setValueUInt8(3, this->servoIndexTable(index));

        this->handler_.pushMessage(std::move(msg_mode));
        

        // SERVO SPEED msg                                            HEADER          | ind | ena | speed
        Message msg_speed( DEVICE_ID_INTELLI_CONTROLLER, 0xa3c9, 0, { 0x40, 0x33, 0x17, 0x00, 0x01, 0x00, 0x00 });

        msg_speed.setValueUInt8( 3, this->servoIndexTable(index));
        msg_speed.setValueUInt16(5, static_cast<uint16_t>(std::roundf(((speed + 49.0f) * 900.0f) / 98.0f)));

        this->handler_.pushMessage(std::move(msg_speed));
    }
}

void RoboMaster::commandServoStop(const uint8_t index)
{
    if(1 <= index && index <= 3)
    {   
        // SERVO MODE msg                                                                HEADER          | ind | mode
        Message msg_mode ( DEVICE_ID_INTELLI_CONTROLLER, 0xa3c9, this->counter_drive_, { 0x40, 0x33, 0x16, 0x00, 0x01 });

        msg_mode.setValueUInt8(3, this->servoIndexTable(index));

        this->handler_.pushMessage(std::move(msg_mode));
        

        // SERVO SPEED msg                                            HEADER          | ind | ena | speed
        Message msg_speed( DEVICE_ID_INTELLI_CONTROLLER, 0xa3c9, 0, { 0x40, 0x33, 0x17, 0x00, 0x00, 0x00, 0x00 });

        msg_speed.setValueUInt8( 3, this->servoIndexTable(index));
        msg_speed.setValueUInt16(5, static_cast<uint16_t>(std::roundf(((0 + 49) * 900) / 98)));

        this->handler_.pushMessage(std::move(msg_speed));
    }
}

void RoboMaster::commandPwmFrequenz(const uint16_t freq1, const uint16_t freq2, const uint16_t freq3, const uint16_t freq4, const uint16_t freq5, const uint16_t freq6)
{
    // PWM FREQUENCE msg                                                         HEADER          | mask| pwm1      | pwm2      | pwm3      | pwm4      | pwm5      | pwm6
    Message msg( DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3f, 0x2b, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 4, clip<uint16_t>(freq1, 0, 50000));
    msg.setValueUInt16( 6, clip<uint16_t>(freq2, 0, 50000));
    msg.setValueUInt16( 8, clip<uint16_t>(freq3, 0, 50000));
    msg.setValueUInt16(10, clip<uint16_t>(freq4, 0, 50000));
    msg.setValueUInt16(12, clip<uint16_t>(freq5, 0, 50000));
    msg.setValueUInt16(14, clip<uint16_t>(freq6, 0, 50000));
    
    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandPwmValue(const uint16_t value1, const uint16_t value2, const uint16_t value3, const uint16_t value4, const uint16_t value5, const uint16_t value6)
{
    // PWM ALUE msg                                                              HEADER          | mask| pwm1      | pwm2      | pwm3      | pwm4      | pwm5      | pwm6
    Message msg( DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3f, 0x3c, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 4, clip<uint16_t>(value1*10, 0, 1000));
    msg.setValueUInt16( 6, clip<uint16_t>(value2*10, 0, 1000));
    msg.setValueUInt16( 8, clip<uint16_t>(value3*10, 0, 1000));
    msg.setValueUInt16(10, clip<uint16_t>(value4*10, 0, 1000));
    msg.setValueUInt16(12, clip<uint16_t>(value5*10, 0, 1000));
    msg.setValueUInt16(14, clip<uint16_t>(value6*10, 0, 1000));
    
    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::commandPwmValue(const float value1, const float value2, const float value3, const float value4, const float value5, const float value6)
{
    // PWM ALUE msg                                                              HEADER          | mask| pwm1      | pwm2      | pwm3      | pwm4      | pwm5      | pwm6
    Message msg( DEVICE_ID_INTELLI_CONTROLLER, 0xc3c9, this->counter_drive_++, { 0x40, 0x3f, 0x3c, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

    msg.setValueUInt16( 4, clip<uint16_t>(value1*10.0, 0, 1000));
    msg.setValueUInt16( 6, clip<uint16_t>(value2*10.0, 0, 1000));
    msg.setValueUInt16( 8, clip<uint16_t>(value3*10.0, 0, 1000));
    msg.setValueUInt16(10, clip<uint16_t>(value4*10.0, 0, 1000));
    msg.setValueUInt16(12, clip<uint16_t>(value5*10.0, 0, 1000));
    msg.setValueUInt16(14, clip<uint16_t>(value6*10.0, 0, 1000));
    
    this->handler_.pushMessage(std::move(msg));
}

void RoboMaster::decodeDataRoboMasterState(const Message &msg)
{
    // check if the callbackfunction is binded to send the information
    if (this->callback_data_robomaster_state_)
    {
        DataRoboMasterState data;
        
        data.velocity = decodeDataVelocity(27, msg);  // 24 bytes
        data.battery  = decodeDataBattery(51, msg);   // 10 bytes
        data.esc      = decodeDataEsc(61, msg);       // 32 bytes
        data.imu      = decodeDataImu(97, msg);       // 24 bytes
        data.attitude = decodeDataAttitude(121, msg); // 12 byte
        data.position = decodeDataPosition(133, msg); // 12 bytes

        this->callback_data_robomaster_state_(data);
    }
}

uint8_t RoboMaster::servoIndexTable(const uint8_t index)
{
    return (index << 5) | 0x19; 
}

bool RoboMaster::isRunning() const
{
    return this->handler_.isRunning();
}

} // namespace robomaster_can_controller