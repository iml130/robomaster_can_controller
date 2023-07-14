// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/queue_msg.h"
#include <iostream>

namespace robomaster_can_controller
{

const static size_t STD_MAX_QUEUE_SIZE = 10;

QueueMsg::QueueMsg()
{

}

void QueueMsg::push(const Message &msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    // When the max queue size is reached, pop the front message from the queue.
    if(STD_MAX_QUEUE_SIZE <= this->queue_.size())
    {
        this->queue_.pop();
    }

    this->queue_.push(msg);
}

void QueueMsg::push(Message && msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);

    // When the max queue size is reached, pop the front message from the queue.
    if(STD_MAX_QUEUE_SIZE <= this->queue_.size())
    {
        this->queue_.pop();
    }

    this->queue_.emplace(std::move(msg));
}


Message QueueMsg::pop()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->queue_.empty())
    {
        // return an empty message, which is invalid to prevent undefined behaviour.
        return Message(0, {});
    }
    else
    {
        const Message msg = queue_.front();
        this->queue_.pop();

        return msg;
    }
}

size_t QueueMsg::size()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->queue_.size();
}

bool QueueMsg::empty()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->queue_.empty();

}

size_t QueueMsg::maxQueueSize() const
{
    return STD_MAX_QUEUE_SIZE;
}

void QueueMsg::clear()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    while(!this->queue_.empty())
    {
        this->queue_.pop();
    }
}

} // namespace robomaster_can_controller