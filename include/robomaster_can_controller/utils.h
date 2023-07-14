// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_UTILS_H_
#define ROBOMASTER_CAN_CONTROLLER_UTILS_H_

#include <algorithm>
#include <arpa/inet.h>
#include <iostream>
#include <sstream>
#include <vector>

namespace robomaster_can_controller
{

/**
 * @brief Clip the value in the given range from min to max.
 * 
 * @tparam T Template for the spezific data type.
 * @param value Value to clip.
 * @param min Min value.
 * @param max Max value.
 * @return T Clipped value.
 */
template <typename T>
T clip(T value, T min, T max) {
    return std::min(std::max(min, value), max);
}

/**
 * @brief Calculated the crc8 for the given data.
 * 
 * @param data Data for the crc8 calculation.
 * @param length Length of the data.
 * @return uint8_t Crc8 value.
 */
uint8_t   calculateCRC8(const uint8_t *data, const size_t length);

/**
 * @brief Calculated the crc16 for the given data.
 * 
 * @param data Data for the crc8 calculation.
 * @param length Length of the data.
 * @return uint8_t Crc16 value.
 */
uint16_t calculateCRC16(const uint8_t *data, const size_t length);


/**
 * @brief Give the given uint8 data array in hex as string back.
 * 
 * @param data Input data.
 * @param length Length of the data.
 * @return std::string Data as string visuallization. 
 */
std::string stringDataAsHex(const uint8_t * data, const size_t length);

/**
 * @brief 
 * 
 * @param data 
 * @return std::string 
 */
std::string stringDataAsHex(const std::vector<uint8_t> &data);

/**
 * @brief Put the two bytes from littlen endian in the right host platform order.
 * 
 * @param lsb Least significant bit.
 * @param msb Most significant bit.
 * @return uint16_t The uint16_t value.
 */
uint16_t littleToUint16(const uint8_t lsb, const uint8_t msb);

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_UTILS_H_