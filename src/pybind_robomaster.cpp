// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "pybind11/pybind11.h"
#include "pybind11/functional.h"
#include <pybind11/stl.h>

#include <robomaster_can_controller/robomaster.h>
#include <sstream>

namespace robomaster_can_controller
{

/**
 * @brief Clip the color from integer to uint8_t for boundaries.
 * 
 * @param value Integer value to clip.
 * @return The clipt uint_8 value.
 */
uint8_t clipColor(const int value)
{
    return static_cast<uint8_t>(std::min(std::max(0, value), 255));
}

/**
 * @brief Definition for the pybind11 python bindings of the robomaster_can_controller class.
 */
PYBIND11_MODULE(robomaster_can_controller_py, m) {
    // Optional docstring
    m.doc() = "Python bindings for robomaster_can_controller.";

    auto py_robomaster = pybind11::class_<RoboMaster>(m, "RoboMaster");
    auto py_state = pybind11::class_<DataRoboMasterState>(m, "State");
    auto py_attitude = pybind11::class_<DataAttitude>(m, "Attitude");
    auto py_battery = pybind11::class_<DataBattery>(m, "Battery");
    auto py_esc = pybind11::class_<DataEsc>(m, "Esc");
    auto py_imu = pybind11::class_<DataImu>(m, "Imu");
    auto py_position = pybind11::class_<DataPosition>(m, "Position");
    auto py_velocity = pybind11::class_<DataVelocity>(m, "Velocity");

    py_robomaster.def(pybind11::init());
    py_robomaster.def("init", &RoboMaster::init, pybind11::arg("interface") = std::string("can0"));
    py_robomaster.def("enable", &RoboMaster::commandEnable);
    py_robomaster.def("disable", &RoboMaster::commandDisable);
    py_robomaster.def("is_running", &RoboMaster::isRunning);
    py_robomaster.def("wheel_rpm", &RoboMaster::commandWheelRPM, pybind11::arg("fr"), pybind11::arg("fl"), pybind11::arg("rl"), pybind11::arg("rr"));
    py_robomaster.def("velocity", &RoboMaster::commandVelocity, pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("angular"));
    py_robomaster.def("stop", &RoboMaster::commandStop);
    py_robomaster.def("led_on", [](RoboMaster &robomaster, const int r, const int g, const int b){
        robomaster.commandLedOn(LED_MASK_ALL, clipColor(r), clipColor(g), clipColor(b));
    }, pybind11::arg("r")=0, pybind11::arg("g")=0, pybind11::arg("b")=0);
    py_robomaster.def("led_breath", [](RoboMaster &robomaster, const int r, const int g, const int b, const float rate){
        robomaster.commandLedBreath(LED_MASK_ALL, clipColor(r), clipColor(g), clipColor(b), rate);
    }, pybind11::arg("r")=0, pybind11::arg("g")=0, pybind11::arg("b")=0, pybind11::arg("rate")=1.0f);
    py_robomaster.def("led_flash", [](RoboMaster &robomaster, const int r, const int g, const int b, const float rate){
        robomaster.commandLedFlash(LED_MASK_ALL, clipColor(r), clipColor(g), clipColor(b), rate);
    }, pybind11::arg("r")=0, pybind11::arg("g")=0, pybind11::arg("b")=0, pybind11::arg("rate")=1.0f);
    py_robomaster.def("led_off", [](RoboMaster &robomaster){
        robomaster.commandLedOff(LED_MASK_ALL);
    });
    py_robomaster.def("servo_angle", pybind11::overload_cast<const uint8_t, const float>(&RoboMaster::commandServoAngle), pybind11::arg("id"), pybind11::arg("angle"));
    py_robomaster.def("servo_speed", pybind11::overload_cast<const uint8_t, const float>(&RoboMaster::commandServoSpeed), pybind11::arg("id"), pybind11::arg("speed"));
    py_robomaster.def("servo_stop", &RoboMaster::commandServoStop, pybind11::arg("id"));
    py_robomaster.def("pwm_frequenz", &RoboMaster::commandPwmFrequenz, 
        pybind11::arg("freq1")=0, pybind11::arg("freq2")=0, pybind11::arg("freq3")=0, pybind11::arg("freq4")=0, pybind11::arg("freq5")=0, pybind11::arg("freq6")=0);
    py_robomaster.def("pwm_value", pybind11::overload_cast<const float ,const float, const float, const float, const float, const float> (&RoboMaster::commandPwmValue), 
        pybind11::arg("v1")=0.0f, pybind11::arg("v2")=0.0f, pybind11::arg("v3")=0.0f, pybind11::arg("v4")=0.0f, pybind11::arg("v5")=0.0f, pybind11::arg("v6")=0.0f);
    py_robomaster.def("bind", [](RoboMaster &robomaster, const std::function<void(const DataRoboMasterState &)> &func){
        robomaster.bindCallbackDataRobotMasterState(func);
    });

    py_state.def("__str__", [](const DataRoboMasterState &data){std::stringstream ss; ss << data; return ss.str();});
    py_state.def_readonly("attitude", &DataRoboMasterState::attitude);
    py_state.def_readonly("battery", &DataRoboMasterState::battery);
    py_state.def_readonly("esc", &DataRoboMasterState::esc);
    py_state.def_readonly("imu", &DataRoboMasterState::imu);
    py_state.def_readonly("position", &DataRoboMasterState::position);
    py_state.def_readonly("velocity", &DataRoboMasterState::velocity);

    py_attitude.def("__str__", [](const DataAttitude &data){std::stringstream ss; ss << data; return ss.str();});
    py_attitude.def_readonly("roll", &DataAttitude::roll);
    py_attitude.def_readonly("pitch", &DataAttitude::pitch);
    py_attitude.def_readonly("yaw", &DataAttitude::yaw);

    py_battery.def("__str__", [](const DataBattery &data){std::stringstream ss; ss << data; return ss.str();});
    py_battery.def_readonly("adc_value", &DataBattery::adc_value);
    py_battery.def_readonly("current", &DataBattery::current);
    py_battery.def_readonly("percent", &DataBattery::percent);
    py_battery.def_readonly("recv", &DataBattery::recv);
    py_battery.def_readonly("temperature", &DataBattery::temperature);

    py_esc.def("__str__", [](const DataEsc &data){std::stringstream ss; ss << data; return ss.str();});
    py_esc.def_readonly("angle", &DataEsc::angle);
    py_esc.def_readonly("speed", &DataEsc::speed);
    py_esc.def_readonly("state", &DataEsc::state);
    py_esc.def_readonly("time_stamp", &DataEsc::time_stamp);

    py_imu.def("__str__", [](const DataImu &data){std::stringstream ss; ss << data; return ss.str();});
    py_imu.def_readonly("acc_x", &DataImu::acc_x);
    py_imu.def_readonly("acc_y", &DataImu::acc_y);
    py_imu.def_readonly("acc_z", &DataImu::acc_z);
    py_imu.def_readonly("gyro_x", &DataImu::gyro_x);
    py_imu.def_readonly("gyro_y", &DataImu::gyro_y);
    py_imu.def_readonly("gyro_z", &DataImu::gyro_z);

    py_position.def("__str__", [](const DataPosition &data){std::stringstream ss; ss << data; return ss.str();});
    py_position.def_readonly("x", &DataPosition::x);
    py_position.def_readonly("y", &DataPosition::y);
    py_position.def_readonly("z", &DataPosition::z);

    py_velocity.def("__str__", [](const DataVelocity &data){std::stringstream ss; ss << data; return ss.str();});
    py_velocity.def_readonly("vbx", &DataVelocity::vbx);
    py_velocity.def_readonly("vby", &DataVelocity::vby);
    py_velocity.def_readonly("vbz", &DataVelocity::vbz);
    py_velocity.def_readonly("vgx", &DataVelocity::vgx);
    py_velocity.def_readonly("vgy", &DataVelocity::vgy);
    py_velocity.def_readonly("vgz", &DataVelocity::vgz);
}

} // namespace robomaster_can_controller
