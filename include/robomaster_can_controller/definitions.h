// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_DEFINITIONS_H_
#define ROBOMASTER_CAN_CONTROLLER_DEFINITIONS_H_

#include <stdint.h>
#include <memory>

namespace robomaster_can_controller
{

// definition for the device can id.
const static uint16_t DEVICE_ID_INTELLI_CONTROLLER = 0x201;
const static uint16_t DEVICE_ID_MOTION_CONTROLLER = 0x202;
const static uint16_t DEVICE_ID_GIMBAL = 0x203;
const static uint16_t DEVICE_ID_HIT_DETECTOR_1 = 0x211;
const static uint16_t DEVICE_ID_HIT_DETECTOR_2 = 0x212;
const static uint16_t DEVICE_ID_HIT_DETECTOR_3 = 0x213;
const static uint16_t DEVICE_ID_HIT_DETECTOR_4 = 0x214;

// mask of the led which can be combined with | symbol.
const static uint16_t LED_MASK_ALL   = 0x000f;
const static uint16_t LED_MASK_BACK  = 0x0001;
const static uint16_t LED_MASK_FRONT = 0x0002;
const static uint16_t LED_MASK_LEFT  = 0x0004;
const static uint16_t LED_MASK_RIGHT = 0x0008;

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_DEFINITIONS_H_