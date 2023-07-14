// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_CAN_SOCKET_H_
#define ROBOMASTER_CAN_CONTROLLER_CAN_SOCKET_H_

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> 

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>

namespace robomaster_can_controller
{

/**
 * @brief This class manage the the io of the can bus.
 */
class CanSocket
{
private:
    /**
     * @brief The Socket for the CanBus.
     */
    int socket_;

    /**
     * @brief Struct to request the Can Bus interface.
     */
    struct ifreq ifr_;

    /**
     * @brief Struct for the Can Bus address.
     */
    struct sockaddr_can addr_;
public:
    /**
     * @brief Construct the the CanSocket object.
     */
    CanSocket(/* args */);

    /**
     * @brief Destroy the Can Socket object and close socket.
     */
    ~CanSocket();

    /**
     * @brief Set the timeout for reading the can socket.
     * 
     * @param seconds Timeout in seconds.
     * @param microseconds Timeouts in microseconds.
     */
    void setTimeout(const size_t seconds, const size_t microseconds);

    /**
     * @brief Set the timeout for the reading the can socket.
     * 
     * @param seconds Float in seconds.
     */
    void setTimeout(const double seconds);

    /**
     * @brief Open the can socket by the given can interface name.
     * 
     * @param can_interface The name of the can interface.
     * @return true, when the socket is open sucessfully.
     * @return false, when this socket failed to open.
     */
    bool init(const std::string &can_interface);

    /**
     * @brief Send a can frame over the socket.
     * 
     * @param id The device id.
     * @param data The data of the can frame.
     * @param length The length of the data.
     * @return true, by success.
     * @return false, when failed.
     */
    bool sendFrame(const uint32_t id, const uint8_t data[8], const size_t length);
    
    /**
     * @brief Read the next incomming can frame from the can socket. This function is blocking until the timeout is reached.
     * 
     * @param id The device id.
     * @param data The data of the can frame.
     * @param length The length of the data. The length is zero, when the timeout is reached.
     * @return true, by success.
     * @return false  when failed.
     */
    bool readFrame(uint32_t &id, uint8_t data[8], size_t &length);
};

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_CAN_SOCKET_H_