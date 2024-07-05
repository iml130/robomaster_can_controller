// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/data.h"

#include <iostream>

namespace robomaster_can_controller
{


struct DataPosition decodeDataPosition(const size_t index, const Message &msg)
{
    DataPosition data;

    // Check the length of the index and then fill the data.
    if (index + 12 <= msg.getPayload().size())
    {
        data.x  = msg.getValueFloat(index);
        data.y  = msg.getValueFloat(index +  4);
        data.z  = msg.getValueFloat(index +  8);

        data.has_data = true;
    }
    
    return data;
}

struct DataEsc decodeDataEsc(const size_t index, const Message &msg)
{
    DataEsc data;

    // Check the length of the index and then fill the data.
    if (index + 36 <= msg.getPayload().size())
    {
        data.speed[0]      =  msg.getValueInt16 (index);
        data.speed[1]      =  msg.getValueInt16 (index +  2);
        data.speed[2]      =  msg.getValueInt16 (index +  4);
        data.speed[3]      =  msg.getValueInt16 (index +  6);
        data.angle[0]      =  msg.getValueInt16 (index +  8);
        data.angle[1]      =  msg.getValueInt16 (index + 10);
        data.angle[2]      =  msg.getValueInt16 (index + 12);
        data.angle[3]      =  msg.getValueInt16 (index + 14);
        data.time_stamp[0] =  msg.getValueUInt32(index + 16);
        data.time_stamp[1] =  msg.getValueUInt32(index + 20);
        data.time_stamp[2] =  msg.getValueUInt32(index + 24);
        data.time_stamp[3] =  msg.getValueUInt32(index + 28);
        data.state[0]      =  msg.getValueUInt8 (index + 32);
        data.state[1]      =  msg.getValueUInt8 (index + 33);
        data.state[2]      =  msg.getValueUInt8 (index + 34);
        data.state[3]      =  msg.getValueUInt8 (index + 35);

        data.has_data = true;
    }

    return data;
}

struct DataImu decodeDataImu(const size_t index, const Message &msg)
{
    DataImu data;

    // Check the length of the index and then fill the data.
    if (index + 24 <= msg.getPayload().size())
    {
        data.acc_x  = msg.getValueFloat(index);
        data.acc_y  = msg.getValueFloat(index +  4);
        data.acc_z  = msg.getValueFloat(index +  8);
        data.gyro_x = msg.getValueFloat(index + 12);
        data.gyro_y = msg.getValueFloat(index + 16);
        data.gyro_z = msg.getValueFloat(index + 20);

        data.has_data = true;
    }
    
    return data;
}

struct DataAttitude decodeDataAttitude(const size_t index, const Message &msg)
{
    DataAttitude data;

    // Check the length of the index and then fill the data.
    if (index + 12 <= msg.getPayload().size())
    {
        data.yaw   = msg.getValueFloat(index);
        data.pitch = msg.getValueFloat(index + 4);
        data.roll  = msg.getValueFloat(index + 8);

        data.has_data = true;
    }
    
    return data;
}

struct DataBattery decodeDataBattery(const size_t index, const Message &msg)
{
    DataBattery data;

    // Check the length of the index and then fill the data.
    if (index + 10 <= msg.getPayload().size())
    {
        data.adc_value   = msg.getValueUInt16(index);;
        data.temperature = msg.getValueUInt16(index + 2);
        data.current     = msg.getValueInt32 (index + 4);
        data.percent     = msg.getValueUInt8 (index + 8);
        data.recv        = msg.getValueUInt8 (index + 9);

        data.has_data = true;
    }
    
    return data;
}

struct DataVelocity decodeDataVelocity(const size_t index, const Message &msg)
{
    DataVelocity data;

    // Check the length of the index and then fill the data.
    if (index + 24 <= msg.getPayload().size())
    {
        data.vgx = msg.getValueFloat(index);
        data.vgy = msg.getValueFloat(index +  4);
        data.vgz = msg.getValueFloat(index +  8);
        data.vbx = msg.getValueFloat(index + 12);
        data.vby = msg.getValueFloat(index + 16);
        data.vbz = msg.getValueFloat(index + 20);

        data.has_data = true;
    }
    
    return data;
}

std::ostream& operator<<(std::ostream& os, const DataEsc &data)
{
    if (data.has_data)
    {
        os << "Esc{"
        "speed["<<data.speed[0]<<", "<<data.speed[1]<<", "<<data.speed[2]<<", "<<data.speed[3]<<"], "
        "angle["<<data.angle[0]<<", "<<data.angle[1]<<", "<<data.angle[2]<<", "<<data.angle[3]<<"], "
        "time_stamp["<<data.time_stamp[0]<<", "<<data.time_stamp[1]<<", "<<data.time_stamp[2]<<", "<<data.time_stamp[3]<<"], "
        "state["<<static_cast<uint16_t>(data.state[0])<<", "<<static_cast<uint16_t>(data.state[1])<<", "<<static_cast<uint16_t>(data.state[2])<<", "<<static_cast<uint16_t>(data.state[3])<<"]"
        "}";
    }
    else
    {
        os << "Esc{no data}";
    }
    
    return os;
}

std::ostream& operator<<(std::ostream& os, const DataImu &data)
{
    if (data.has_data)
    {
        os << "Imu{"
              "accel["<<data.acc_x<<", "<<data.acc_y<<", "<<data.acc_z<<"], "
              "gyro["<<data.gyro_x<<", "<<data.gyro_y<<", "<<data.gyro_z<<"]"
              "}";
        
    }
    else
    {
        os << "Imu{no data}";
    }
    
    return os;
}

std::ostream& operator<<(std::ostream& os, const DataAttitude &data)
{
    if (data.has_data)
    {
        os << "Attitude{"
              "roll: "<<data.roll<<", pitch: "<<data.pitch<<", yaw: "<<data.yaw<<
              "}";
    }
    else
    {
        os << "Attitude{no data}";
    }
    
    return os;
}

std::ostream& operator<<(std::ostream& os, const DataBattery &data)
{
    if (data.has_data)
    {
        os << "Battery{"
              "adc: "<<data.adc_value<<", temp: "<<data.temperature<<", current: "<<data.current<<", percent: "<<static_cast<uint16_t>(data.percent)<<"]"
              "}";
    }
    else
    {
        os << "Battery{no data}";
    }
    
    return os;
}

std::ostream& operator<<(std::ostream& os, const DataVelocity &data)
{
    if (data.has_data)
    {
        os << "Velocity{"
              "global["<<data.vgx<<", "<<data.vgy<<", "<<data.vgz<<"], "
              "body["<<data.vbx<<", "<<data.vby<<", "<<data.vbz<<"]"
              "}";
    }
    else
    {
        os << "Velocity{no data}";
    }
    
    return os;
}

std::ostream& operator<<(std::ostream& os, const DataPosition &data)
{
    if (data.has_data)
    {
        os << "Position{"
                "x: "<<data.x<< ", y: "<<data.y<<", z: "<<data.z<<"]"
                "}";
    }
    else
    {
        os << "Position{no data}";
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const DataRoboMasterState &data)
{
    os << "RoboMasterState:"
       << "\n    " << data.battery 
       << "\n    " << data.esc
       << "\n    " << data.imu
       << "\n    " << data.velocity
       << "\n    " << data.position 
       << "\n    " << data.attitude;

    return os;
}

} // namespace robomaster_can_controller