// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#ifndef ROBOMASTER_CAN_CONTROLLER_QUEUE_MSG_H_
#define ROBOMASTER_CAN_CONTROLLER_QUEUE_MSG_H_

#include <mutex>
#include <queue>
#include "message.h"

namespace robomaster_can_controller
{

/**
 * @brief This class is queue for RoboMaster messages which is protected by a mutex.
 */
class QueueMsg
{
private:
    /**
     * @brief The message qeueu.
     * 
     */
    std::queue<Message> queue_;
    /**
     * @brief The mutex to proteced the critical section.
     */
    std::mutex mutex_;

public:
    /**
     * @brief Construct a new Queue Msg object.
     */
    QueueMsg();

    /**
     * @brief Push a Message into the qeueu. If the maximal queue size is reached the front message will be pop.
     * 
     * @param msg A RoboMaster message.
     */
    void push(const Message &msg);

        /**
     * @brief Push a Message into the qeueu. If the maximal queue size is reached the front message will be pop.
     * 
     * @param msg A RoboMaster message.
     */
    void push(Message && msg);

    /**
     * @brief Pop and return the message of the queue. If the queue is empty a empty message is returned.
     * 
     * @return RoboMaster message.
     */
    Message pop();

    /**
     * @brief The current size of the queue.
     * 
     * @return size_t as size.
     */
    size_t size();

    /**
     * @brief Get the maximal allow queue size.
     * 
     * @return size_t as maximal qeueu size.
     */
    size_t maxQueueSize() const;

    /**
     * @brief True when the queue is empty.
     * 
     * @return true, when empty.
     * @return false, when not empty.
     */
    bool empty();

    /**
     * @brief Clear all RoboMaster messagfe from the queue.
     */
    void clear();
};

} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_QUEUE_MSG_H_