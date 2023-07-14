// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_HANDLER_H_
#define ROBOMASTER_CAN_CONTROLLER_HANDLER_H_

#include "can_socket.h"
#include "message.h"
#include "queue_msg.h"

 
#include <thread>
#include <queue>
#include <condition_variable>
#include <functional>

namespace robomaster_can_controller
{

/**
 * @brief This class handles the incoming and outcoming RoboMaster message over the can bus.
 * 
 */
class Handler
{
private:
    /**
     * @brief CanSocket class for the can bus io.
     */
    CanSocket can_socket_;
    /**
     * @brief Thread for reading on the can socket and put valid messages in the receiver queue.
     */
    std::thread thread_receiver_;
    /**
     * @brief Thread for sending messages on the can bus. Also triggers the 10 ms heartbeat to keept the RoboMaster alive.
     */
    std::thread thread_sender_;
    /**
     * @brief Thread for processiong the messages in receiver queue and trigger the callback function for RoboMaster states. 
     */
    std::thread thread_handler_;
    /**
     * @brief Receiver queue for received messages.
     */
    QueueMsg queue_receiver_;
    /**
     * @brief Senmder queue for seding messages.
     */
    QueueMsg queue_sender_;
    /**
     * @brief conditional variable for the handler thread, when new messages put in the receiver queue.
     */
    std::condition_variable cv_handler_;
    /**
     * @brief Mutex of the handler conditional variable.
     */
    std::mutex cv_handler_mutex_;
    /**
     * @brief Conditional varaible for the sender thread, when new messages put into the sender queue.
     */
    std::condition_variable cv_sender_;
    /**
     * @brief Mutex of the sender conditional variable.
     */
    std::mutex cv_sender_mutex_;
    /**
     * @brief callback function for the data of the robomaster motion controller.
     */
    std::function<void(const Message&)> callback_data_robomaster_state_;
    /**
     * @brief Flag of the initilaisation of the handler class. True when the can socket was successful initialised.
     */
    bool flag_initialised_;
    /**
     * @brief Flag then the threads are running to prevent multiply starts.
     */
    bool flag_stop_;

    /**
     * @brief Run function of the sender thread.
     */
    void runSenderThread();

    /**
     * @brief Run function of the receiver thread.
     */
    void runReceiverThread();

    /**
     * @brief Run function of the handler thread.
     */
    void runHandlerThread();

    /**
     * @brief Notify all conditional variable eg. stopping the threads.
     */
    void notifyAll();

    /**
     * @brief Joining all started threads.
     */
    void joinAll();

    /**
     * @brief Send the message to the can socket.
     * 
     * @param id The can device it.
     * @param data Data of the hole message.
     * @return true, by success.
     * @return false, by failing to send the message.
     */
    bool sendMessage(const uint32_t id, const std::vector<uint8_t> &data);

    /**
     * @brief Send the message to the can socket.
     * 
     * @param msg The RoboMaster message.
     * @return true, by success.
     * @return false, by failing to send the message.
     */
    bool sendMessage(const Message &msg);

    /**
     * @brief Process the received messages from the message queue and triggers callback functions.
     * 
     * @param msg RoboMaster message.
     */
    void processReceivedMessage(const Message &msg);
 
public:
    /**
     * @brief Construct a new Handler object.
     * 
     */
    Handler();
    
    /**
     * @brief Destroy the Handler object and stopped the threads.
     */
    ~Handler();

    /**
     * @brief Init the can socket and start the threads.
     * 
     * @param can_interface The can interface name.
     * @return true, when successful initialised.
     * @return false, by failing the initialisation.
     */
    bool init(const std::string &can_interface="can0");

    /**
     * @brief Bind the given callbackfunction for triggering when the message for the RoboMasterState is received.
     * 
     * @param func The callbackfunction to trigger.
     */
    void bindCallbackDataRoboMasterState(std::function<void(const Message&)> func);

    /**
     * @brief Push a message to the sender queue to send it over the can bus.
     * 
     * @param msg A RoboMaster message.
     */
    void pushMessage(const Message &msg);

    /**
     * @brief State if the handler is running or not.
     * 
     * @return true If the Handler is running an ready to receive and send messages.
     * @return false If the handler was stopped due to error.
     */
    bool isRunning() const;
};

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_HANDLER_H_