// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/data.h"
#include "gtest/gtest.h"

namespace robomaster_can_controller
{

TEST(MessageTest, ValueUint8)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{1,2});

    ASSERT_EQ(msg.getValueUInt8(0), 1);
    ASSERT_EQ(msg.getValueUInt8(1), 2);

    msg.setValueUInt8(0, 10);
    msg.setValueUInt8(1, 11);

    ASSERT_EQ(msg.getPayload()[0], 10);
    ASSERT_EQ(msg.getPayload()[1], 11);    
}

TEST(MessageTest, ValueUint16)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

    ASSERT_EQ(msg.getValueUInt16(0), 0xADDE);
    ASSERT_EQ(msg.getValueUInt16(2), 0xEFBE);

    msg.setValueUInt16(0, 0xDEAD);
    msg.setValueUInt16(2, 0xBEEF);

    ASSERT_EQ(msg.getPayload()[0], 0xAD);
    ASSERT_EQ(msg.getPayload()[1], 0xDE);
    ASSERT_EQ(msg.getPayload()[2], 0xEF);
    ASSERT_EQ(msg.getPayload()[3], 0xBE);
}

TEST(MessageTest, ValueUint32)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xCA, 0xFB, 0xAD});

    ASSERT_EQ(msg.getValueUInt32(0), 0xEFBEADDE);
    ASSERT_EQ(msg.getValueUInt32(4), 0xADFBCADE);

    msg.setValueUInt32(0, 0xDECAFBAD);
    msg.setValueUInt32(4, 0xDEADBEEF);

    ASSERT_EQ(msg.getPayload()[0], 0xAD);
    ASSERT_EQ(msg.getPayload()[1], 0xFB);
    ASSERT_EQ(msg.getPayload()[2], 0xCA);
    ASSERT_EQ(msg.getPayload()[3], 0xDE);
    ASSERT_EQ(msg.getPayload()[4], 0xEF);
    ASSERT_EQ(msg.getPayload()[5], 0xBE);
    ASSERT_EQ(msg.getPayload()[6], 0xAD);
    ASSERT_EQ(msg.getPayload()[7], 0xDE);
}


TEST(MessageTest, ValueInt8)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{255,1});

    ASSERT_EQ(msg.getValueInt8(0), -1);
    ASSERT_EQ(msg.getValueInt8(1), 1);

    msg.setValueInt8(0, -10);
    msg.setValueInt8(1,  10);

    ASSERT_EQ(msg.getPayload()[0], 246);
    ASSERT_EQ(msg.getPayload()[1], 10);    
}

TEST(MessageTest, ValueInt16)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

    ASSERT_EQ(msg.getValueInt16(0), -21026);
    ASSERT_EQ(msg.getValueInt16(2), -4162);

    msg.setValueInt16(0, 0xDEAD);
    msg.setValueInt16(2, 0xBEEF);

    ASSERT_EQ(msg.getPayload()[0], 0xAD);
    ASSERT_EQ(msg.getPayload()[1], 0xDE);
    ASSERT_EQ(msg.getPayload()[2], 0xEF);
    ASSERT_EQ(msg.getPayload()[3], 0xBE);
}

TEST(MessageTest, ValueInt32)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xCA, 0xFB, 0xAD});

    ASSERT_EQ(msg.getValueInt32(0), 0xEFBEADDE);
    ASSERT_EQ(msg.getValueInt32(4), 0xADFBCADE);

    msg.setValueInt32(0, 0xDECAFBAD);
    msg.setValueInt32(4, 0xDEADBEEF);

    ASSERT_EQ(msg.getPayload()[0], 0xAD);
    ASSERT_EQ(msg.getPayload()[1], 0xFB);
    ASSERT_EQ(msg.getPayload()[2], 0xCA);
    ASSERT_EQ(msg.getPayload()[3], 0xDE);
    ASSERT_EQ(msg.getPayload()[4], 0xEF);
    ASSERT_EQ(msg.getPayload()[5], 0xBE);
    ASSERT_EQ(msg.getPayload()[6], 0xAD);
    ASSERT_EQ(msg.getPayload()[7], 0xDE);
}

TEST(MessageTest, ValueFloat32)
{
    Message msg = Message(0, 0, 0, std::vector<uint8_t>{0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F});

    ASSERT_FLOAT_EQ(msg.getValueFloat(0), -1.0f);
    ASSERT_FLOAT_EQ(msg.getValueFloat(4),  0.0);
    ASSERT_FLOAT_EQ(msg.getValueFloat(8),  1.0f);

    msg.setValueFloat(0, 1337.0f); // 0x44a72000
    msg.setValueFloat(4, 3.14f);   // 0x4048f5c3
    msg.setValueFloat(8, 0.0f);    // 0x00000000

    ASSERT_EQ(msg.getPayload()[0], 0x00);
    ASSERT_EQ(msg.getPayload()[1], 0x20);
    ASSERT_EQ(msg.getPayload()[2], 0xA7);
    ASSERT_EQ(msg.getPayload()[3], 0x44);
    ASSERT_EQ(msg.getPayload()[4], 0xC3);
    ASSERT_EQ(msg.getPayload()[5], 0xF5);
    ASSERT_EQ(msg.getPayload()[6], 0x48);
    ASSERT_EQ(msg.getPayload()[7], 0x40);
    ASSERT_EQ(msg.getPayload()[8], 0x00);
    ASSERT_EQ(msg.getPayload()[9], 0x00);
    ASSERT_EQ(msg.getPayload()[10], 0x00);
    ASSERT_EQ(msg.getPayload()[11], 0x00);
}

TEST(MessageTest, Creation)
{
    Message msg = Message(0, 1337, 1, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

    ASSERT_EQ(msg.getDeviceId(), 0);
    ASSERT_EQ(msg.getType(), 1337);
    ASSERT_EQ(msg.getSequence(), 1);
    ASSERT_EQ(msg.getLength(), 14);
    ASSERT_TRUE(msg.isValid());
    ASSERT_EQ(msg.getPayload()[0], 0xDE);
    ASSERT_EQ(msg.getPayload()[1], 0xAD);
    ASSERT_EQ(msg.getPayload()[2], 0xBE);
    ASSERT_EQ(msg.getPayload()[3], 0xEF);

    msg = Message(1337, std::vector<uint8_t>{0xDE, 0xAD, 0xBE, 0xEF});

    ASSERT_EQ(msg.getDeviceId(), 1337);
    ASSERT_EQ(msg.getType(), 0);
    ASSERT_EQ(msg.getSequence(), 0);
    ASSERT_EQ(msg.getLength(), 10);
    ASSERT_FALSE(msg.isValid());
    ASSERT_EQ(msg.getPayload().size(), 0);
}

} // namespace robomaster_can_controller
