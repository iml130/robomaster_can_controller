// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/data.h"
#include "gtest/gtest.h"

namespace robomaster_can_controller
{

TEST(DataTest, DecodeDataEsc) {
    Message msg = Message(0, 0, 0, std::vector<uint8_t>(36, 0));

    msg.setValueInt16(0, 0);
    msg.setValueInt16(2, 1);
    msg.setValueInt16(4, 2);
    msg.setValueInt16(6, 3);

    msg.setValueInt16(8, 10);
    msg.setValueInt16(10, 11);
    msg.setValueInt16(12, 12);
    msg.setValueInt16(14, 13);

    msg.setValueUInt32(16, 20);
    msg.setValueUInt32(20, 21);
    msg.setValueUInt32(24, 22);
    msg.setValueUInt32(28, 23);

    msg.setValueUInt8(32, 30);
    msg.setValueUInt8(33, 31);
    msg.setValueUInt8(34, 32);
    msg.setValueUInt8(35, 33);
    
    DataEsc esc = decodeDataEsc(0, msg);

    ASSERT_TRUE(esc.has_data);

    ASSERT_EQ(esc.speed[0], 0);
    ASSERT_EQ(esc.speed[1], 1);
    ASSERT_EQ(esc.speed[2], 2);
    ASSERT_EQ(esc.speed[3], 3);

    ASSERT_EQ(esc.angle[0], 10);
    ASSERT_EQ(esc.angle[1], 11);
    ASSERT_EQ(esc.angle[2], 12);
    ASSERT_EQ(esc.angle[3], 13);

    ASSERT_EQ(esc.time_stamp[0], 20);
    ASSERT_EQ(esc.time_stamp[1], 21);
    ASSERT_EQ(esc.time_stamp[2], 22);
    ASSERT_EQ(esc.time_stamp[3], 23);

    ASSERT_EQ(esc.state[0], 30);
    ASSERT_EQ(esc.state[1], 31);
    ASSERT_EQ(esc.state[2], 32);
    ASSERT_EQ(esc.state[3], 33);
}

TEST(DataTest, DecodeDataImu) {
    Message msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

    msg.setValueFloat(0, 0.0f);
    msg.setValueFloat(4, 1.0f);
    msg.setValueFloat(8, 2.0f);

    msg.setValueFloat(12, 10.0f);
    msg.setValueFloat(16, 11.0f);
    msg.setValueFloat(20, 12.0f);

    DataImu imu = decodeDataImu(0, msg);

    ASSERT_TRUE(imu.has_data);

    ASSERT_FLOAT_EQ(imu.acc_x, 0.0f);
    ASSERT_FLOAT_EQ(imu.acc_y, 1.0f);
    ASSERT_FLOAT_EQ(imu.acc_z, 2.0f);

    ASSERT_FLOAT_EQ(imu.gyro_x, 10.0f);
    ASSERT_FLOAT_EQ(imu.gyro_y, 11.0f);
    ASSERT_FLOAT_EQ(imu.gyro_z, 12.0f);
}

TEST(DataTest, DecodeDataAttitude) {
    Message msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

    msg.setValueFloat(0, 0.0f);
    msg.setValueFloat(4, 1.0f);
    msg.setValueFloat(8, 2.0f);

    DataAttitude attitude = decodeDataAttitude(0, msg);

    ASSERT_TRUE(attitude.has_data);

    ASSERT_FLOAT_EQ(attitude.yaw,  0.0f);
    ASSERT_FLOAT_EQ(attitude.pitch, 1.0f);
    ASSERT_FLOAT_EQ(attitude.roll,   2.0f);
}

TEST(DataTest, DecodeDataBattery) {
    Message msg = Message(0, 0, 0, std::vector<uint8_t>(10, 0));

    msg.setValueInt16(0, 0);
    msg.setValueInt16(2, 1);
    msg.setValueInt32(4, 2);
    msg.setValueUInt8(8, 3);
    msg.setValueUInt8(9, 4);

    DataBattery battery = decodeDataBattery(0, msg);

    ASSERT_TRUE(battery.has_data);

    ASSERT_EQ(battery.adc_value,   0);
    ASSERT_EQ(battery.temperature, 1);
    ASSERT_EQ(battery.current,     2);
    ASSERT_EQ(battery.percent,     3);
    ASSERT_EQ(battery.recv,        4);
}

TEST(DataTest, DecodeDataPosition) {
    Message msg = Message(0, 0, 0, std::vector<uint8_t>(12, 0));

    msg.setValueFloat(0, 0.0f);
    msg.setValueFloat(4, 1.0f);
    msg.setValueFloat(8, 2.0f);

    DataPosition position = decodeDataPosition(0, msg);

    ASSERT_TRUE(position.has_data);

    ASSERT_FLOAT_EQ(position.x, 0.0f);
    ASSERT_FLOAT_EQ(position.y, 1.0f);
    ASSERT_FLOAT_EQ(position.z, 2.0f);
}

TEST(DataTest, DecodeDataVelocity) {
    Message msg = Message(0, 0, 0, std::vector<uint8_t>(24, 0));

    msg.setValueFloat(0, 0.0f);
    msg.setValueFloat(4, 1.0f);
    msg.setValueFloat(8, 2.0f);

    msg.setValueFloat(12, 10.0f);
    msg.setValueFloat(16, 11.0f);
    msg.setValueFloat(20, 12.0f);

    DataVelocity velocity = decodeDataVelocity(0, msg);

    ASSERT_TRUE(velocity.has_data);

    ASSERT_FLOAT_EQ(velocity.vgx, 0.0f);
    ASSERT_FLOAT_EQ(velocity.vgy, 1.0f);
    ASSERT_FLOAT_EQ(velocity.vgz, 2.0f);

    ASSERT_FLOAT_EQ(velocity.vbx, 10.0f);
    ASSERT_FLOAT_EQ(velocity.vby, 11.0f);
    ASSERT_FLOAT_EQ(velocity.vbz, 12.0f);
}

} // namespace robomaster_can_controller
