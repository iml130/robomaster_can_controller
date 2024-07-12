// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/message.h"
#include "robomaster_can_controller/utils.h"

#include <cassert>
#include <iomanip>

namespace robomaster_can_controller
{

Message::Message(const uint32_t device_id, const std::vector<uint8_t> &data)
    : is_valid_(false),
      device_id_(device_id),
      sequence_(0),
      type_(0)

{
    // message must be at least 10 bytes or higher 
    // 4 bytes header
    // 2 byte type
    // 2 byte sequence
    // 2 byte crc16
    if(10 < data.size())
    {
        this->type_ = littleToUint16(data[4], data[5]);
        this->sequence_ = littleToUint16(data[6], data[7]);

        this->payload_.clear();
        this->payload_.insert(std::begin(this->payload_), std::cbegin(data) + 8, std::cend(data) - 2);

        this->is_valid_ = true;        
    }
}

Message::Message(const uint32_t device_id, const uint16_t type, const uint16_t sequence, const std::vector<uint8_t> payload)
    : is_valid_(true),
      device_id_(device_id),
      sequence_(sequence),
      type_(type),
      payload_(payload)
{

}

uint32_t Message::getDeviceId() const
{
    return this->device_id_;
}

uint16_t Message::getSequence() const
{
    return this->sequence_;
}
uint16_t Message::getType() const
{
    return this->type_;
}

std::vector<uint8_t> Message::getPayload() const
{
    return this->payload_;
}

size_t Message::getLength() const
{
    
    return this->payload_.size() + 10;
}

bool Message::isValid() const
{
    return this->is_valid_;
}

void Message::incrementSequence()
{
    this->sequence_++;
}

void Message::setValueUInt8(const size_t index, const uint8_t value)
{
    assert(index < this->payload_.size());

    this->payload_[index] = value;
}

void Message::setValueInt8(const size_t index, const int8_t value)
{
    assert(index < this->payload_.size());

    this->payload_[index] = value;
}

void Message::setValueUInt16(const size_t index, const uint16_t value)
{
    assert(index + 1 < this->payload_.size());

    this->payload_[index] = static_cast<uint8_t>(value);
    this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
}

void Message::setValueInt16(const size_t index, const int16_t value)
{
    assert(index + 1 < this->payload_.size());

    this->payload_[index] = static_cast<uint8_t>(value);
    this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
}

void Message::setValueUInt32(const size_t index, const uint32_t value)
{
    assert(index + 3 < this->payload_.size());

    this->payload_[index] = static_cast<uint8_t>(value);
    this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
    this->payload_[index + 2] = static_cast<uint8_t>(value >> 16);
    this->payload_[index + 3] = static_cast<uint8_t>(value >> 24);
}

void Message::setValueInt32(const size_t index, const int32_t value)
{
    assert(index + 3 < this->payload_.size());

    this->payload_[index] = static_cast<uint8_t>(value);
    this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
    this->payload_[index + 2] = static_cast<uint8_t>(value >> 16);
    this->payload_[index + 3] = static_cast<uint8_t>(value >> 24);
}

void Message::setValueFloat(const size_t index, const float value)
{
    assert(index + 3 < this->payload_.size());

    union {
        uint32_t u;
        float f;
    } float_uint32_t_union;

    float_uint32_t_union.f = value;
    this->setValueUInt32(index, float_uint32_t_union.u);
}


uint8_t Message::getValueUInt8(const size_t index) const
{
    assert(index < this->payload_.size());

    return this->payload_[index];
}

int8_t Message::getValueInt8(const size_t index) const
{
    assert(index < this->payload_.size());

    return this->payload_[index];
}

uint16_t Message::getValueUInt16(const size_t index) const
{
    assert(index + 1 < this->payload_.size());

    uint16_t value = this->payload_[index + 1];
    value = value << 8 | this->payload_[index];
    return value;
}

int16_t Message::getValueInt16(const size_t index) const
{
    assert(index + 1 < this->payload_.size());

    int16_t value = this->payload_[index + 1];
    value = value << 8 | this->payload_[index];
    return value;
}

uint32_t Message::getValueUInt32(const size_t index) const
{
    assert(index + 3 < this->payload_.size());

    uint32_t value = this->payload_[index + 3];
    value = value << 8 | this->payload_[index + 2];
    value = value << 8 | this->payload_[index + 1];
    value = value << 8 | this->payload_[index];
    return value;
}

int32_t Message::getValueInt32(const size_t index) const
{
    assert(index + 3 < this->payload_.size());

    int32_t value = this->payload_[index + 3];
    value = value << 8 | this->payload_[index + 2];
    value = value << 8 | this->payload_[index + 1];
    value = value << 8 | this->payload_[index];
    return value;
}

float Message::getValueFloat(const size_t index) const
{
    assert(index + 3 < this->payload_.size());

    union {
        uint32_t u;
        float f;
    } float_uint32_t_union;

    float_uint32_t_union.u = this->getValueUInt32(index);
    return float_uint32_t_union.f;
}

void Message::setPayload(const std::vector<uint8_t> &payload)
{
    this->payload_ = payload;
}

std::vector<uint8_t> Message::toVector() const
{
    std::vector<uint8_t> vector;

    if (this->is_valid_)
    {
        // header, crc usw + payload
        vector.resize(10 + this->payload_.size());

        vector[0] = 0x55;
        vector[1] = static_cast<uint8_t>(vector.size());
        vector[2] = 0x04;
        vector[3] = calculateCRC8(vector.data(), 3);
        vector[4] = static_cast<uint8_t>(this->type_);
        vector[5] = static_cast<uint8_t>(this->type_ >> 8);
        vector[6] = static_cast<uint8_t>(this->sequence_);
        vector[7] = static_cast<uint8_t>(this->sequence_ >> 8);

        // fill payload
        for (size_t i = 0; i < this->payload_.size(); i++)
        {
            vector[8 + i] = this->payload_[i];
        }

        // crc16 
        const uint16_t crc16 = calculateCRC16(vector.data(), vector.size() - 2);

        vector[vector.size() - 2] = static_cast<uint8_t>(crc16);
        vector[vector.size() - 1] = static_cast<uint8_t>(crc16 >> 8);
    }
    
    return vector;
}

std::ostream& operator<<(std::ostream& os, const Message& msg)
{
    os << "Message( 0x"
       << std::setfill('0') << std::setw(4) << std::hex << msg.getDeviceId() << ", 0x"
       << std::setfill('0') << std::setw(4) << std::hex << msg.getType() << ", "
       << std::setfill(' ') << std::setw(5) << std::dec << msg.getSequence() << ", { ";

    if (!msg.getPayload().empty())
    {
        os << "0x";

        for (size_t i = 0; i < msg.getPayload().size(); i++)
        {
            os << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(msg.getPayload()[i]);

            if (i < msg.getPayload().size() - 1)
            {
                os << ", 0x";
            }
        }
    }
    
    os << " })" << std::dec;
    
    return os;
}

} // namespace robomaster_can_controller