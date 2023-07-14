// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_MESSAGE_H_
#define ROBOMASTER_CAN_CONTROLLER_MESSAGE_H_

#include "definitions.h"
#include <memory>
#include <vector>
#include <ostream>

namespace robomaster_can_controller
{

/**
 * @brief This class defined the a RoboMaster message. The information values in the messages are saved in little endian. 
 */
class Message
{
private:
    /**
     * @brief Flag for a valid message. This includes when the message is long enough with header and the right crc.
     */
    bool is_valid_;
    /**
     * @brief The can device id for this message.
     */
    uint32_t device_id_;
    /**
     * @brief The sqeuence or counter for the message.
     */
    uint16_t sequence_;
    /**
     * @brief The type of the message. Every message from the intelli controller or motion controller has a fixed value which seems to be a type or command id.
     */
    uint16_t type_;
    /**
     * @brief The payload of the message which contains the information.
     */
    std::vector<uint8_t> payload_;
public:
    /**
     * @brief Construct a new Message object from the given raw data.
     * 
     * @param device_id The can device id.
     * @param msg_data The raw data eg. can bus to parse into a RoboMaster message.
     */
    Message(const uint32_t device_id, const std::vector<uint8_t> &msg_data);

    /**
     * @brief Construct a new Message object.
     * 
     * @param device_id The can device id.
     * @param type The type of the message.
     * @param squence The current squence.
     * @param payload The payload for the information.
     */
    Message(const uint32_t device_id, const uint16_t type, const uint16_t squence, const std::vector<uint8_t> payload=std::vector<uint8_t>());

    /**
     * @brief Get the can device id.
     * 
     * @return uint32_t as device id.
     */
    uint32_t getDeviceId() const;

    /**
     * @brief Get the sequence.
     * 
     * @return uint16_t as sequence.
     */
    uint16_t getSequence() const;

    /**
     * @brief Get the message type.
     * 
     * @return uint16_t as type.
     */
    uint16_t getType() const;

    /**
     * @brief Get the payload from the message.
     * 
     * @return std::vector<uint8_t> as payload.
     */
    std::vector<uint8_t> getPayload() const;

    /**
     * @brief Retuns true for a valid message, when message length and crc is correct.
     * 
     * @return true 
     * @return false 
     */
    bool isValid() const;

    /**
     * @brief Get the complett length from the message including header, crc and payload.
     * 
     * @return size_t as length.
     */
    size_t getLength() const;

    /**
     * @brief Set the a uint8 value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueUInt8(const size_t index, const uint8_t value);

    /**
     * @brief Set the a int8 value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueInt8(const size_t index, const int8_t value);

    /**
     * @brief Set the a uint16 value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueUInt16(const size_t index, const uint16_t value);

    /**
     * @brief Set the a int16 value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueInt16(const size_t index, const int16_t value);

    /**
     * @brief Set the a uint32 value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueUInt32(const size_t index, const uint32_t value);

    /**
     * @brief Set the a int32 value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueInt32(const size_t index, const int32_t value);

    /**
     * @brief Set the a float value into the payload at given index.
     * 
     * @param index The index for the payload position.
     * @param value The value to be set.
     */
    void setValueFloat(const size_t index, const float value);

    /**
     * @brief Get the uint8 value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return uint8_t as value.
     */
    uint8_t getValueUInt8(const size_t index) const;

    /**
     * @brief Get the int8 value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return int8_t as value.
     */
    int8_t getValueInt8(const size_t index) const;

    /**
     * @brief Get the uint16 value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return uint16_t as value.
     */
    uint16_t getValueUInt16(const size_t index) const;

    /**
     * @brief Get the int16 value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return uint16_t as value.
     */
    int16_t getValueInt16(const size_t index) const;

    /**
     * @brief Get the uint32 value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return uint32_t as value.
     */
    uint32_t getValueUInt32(const size_t index) const;

    /**
     * @brief Get the int32 value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return int32_t as value.
     */
    int32_t getValueInt32(const size_t index) const;

    /**
     * @brief Get the float value form the palyoad at given index.
     * 
     * @param index The index for the payload position.
     * @return float as value.
     */
    float getValueFloat(const size_t index) const;

    /**
     * @brief Increment the sequence by one.
     */
    void incrementSequence();
    
    /**
     * @brief Set the payload.
     * 
     * @param payload The payload.
     */
    void setPayload(const std::vector<uint8_t> &payload);

    /**
     * @brief Create a vector as raw data from the message including header, crc and payload.
     * 
     * @return std::vector<uint8_t> as raw data message.
     */
    std::vector<uint8_t> toVector() const;

    friend std::ostream& operator<<(std::ostream& os, const Message &msg);
};

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_ROBOTMASTER_H_