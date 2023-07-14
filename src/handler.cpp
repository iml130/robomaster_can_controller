// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/handler.h"
#include "robomaster_can_controller/utils.h"
#include "robomaster_can_controller/definitions.h"

#include <iostream>
#include <algorithm>
#include <map>

namespace robomaster_can_controller
{


// The maximal number of following frame error for the can bus to stop the thread.
const static size_t STD_MAX_ERROR_COUNT = 3;
const static std::chrono::milliseconds STD_HEARTBEAT_TIME =  std::chrono::milliseconds(10);

Handler::Handler()
    : flag_initialised_(false),
      flag_stop_(false)
{

}

void Handler::notifyAll()
{
    this->cv_handler_.notify_all();
    this->cv_sender_.notify_all();
}

void Handler::joinAll()
{
    this->thread_receiver_.join();
    this->thread_sender_.join();
    this->thread_handler_.join();
}

Handler::~Handler()
{
    if (this->flag_initialised_)
    {
        this->flag_stop_ = true;
        this->notifyAll();
        this->joinAll();
    }
}

bool Handler::init(const std::string &can_interface)
{
    if (this->flag_initialised_)
    {
        std::cerr << "[Handler::init]: Handler is already running!" << std::endl;
        return false;
    }
    else if(this->can_socket_.init(can_interface))
    {
        this->can_socket_.setTimeout(0.1);
        this->flag_initialised_ = true;
        this->thread_receiver_ = std::thread(&Handler::runReceiverThread, this);
        this->thread_sender_ = std::thread(&Handler::runSenderThread, this);
        this->thread_handler_ = std::thread(&Handler::runHandlerThread, this);
        return true;
    }
    else
    {
        std::cerr << "[Handler::init]: Could not init Handler!" << std::endl;
        return false;
    }
}

bool Handler::isRunning() const
{
    return this->flag_initialised_ && !this->flag_stop_;
}

bool Handler::sendMessage(const uint32_t id, const std::vector<uint8_t> &data)
{
    uint8_t frame_data[8];

    for (size_t i = 0; i < data.size(); i += 8)
    {
        const size_t frame_length = std::min(static_cast<size_t>(8), data.size() - i);

        std::copy(data.begin() + i, data.begin() + i + frame_length, frame_data);

        if(!this->can_socket_.sendFrame(id, frame_data, frame_length))
        {
            return false; 
        }

        // sleep to not overfill the can bus
        // std::this_thread::sleep_for(std::chrono::microseconds(75));  
    }

    return true;
}

bool Handler::sendMessage(const Message &msg)
{
    return this->sendMessage(msg.getDeviceId(), msg.toVector());
}

void Handler::runReceiverThread()
{
    struct msg_robomaster{
        std::vector<uint8_t> buffer;
        size_t length = 0; // if lentgh is zero, searching for the header
    };

    std::map<uint32_t, msg_robomaster> map_msg_robomaster{
            //{DEVICE_ID_INTELLI_CONTROLLER, msg_robomaster()},
            {DEVICE_ID_MOTION_CONTROLLER, msg_robomaster()}
            //{DEVICE_ID_GIMBAL, msg_robomaster()},
            //{DEVICE_ID_HIT_DETECTOR_1, msg_robomaster()},
            //{DEVICE_ID_HIT_DETECTOR_2, msg_robomaster()},
            //{DEVICE_ID_HIT_DETECTOR_3, msg_robomaster()},
            //{DEVICE_ID_HIT_DETECTOR_4, msg_robomaster()}
        };

    uint32_t frame_id;
    uint8_t frame_buffer[8];
    size_t frame_length;

    size_t error_counter = 0;
    
    while(error_counter <= STD_MAX_ERROR_COUNT && !this->flag_stop_)
    {
        // read the frame
        if(!can_socket_.readFrame(frame_id, frame_buffer, frame_length))
        {
            error_counter++;
            continue;
        }

        std::map<uint32_t, msg_robomaster>::iterator iter = map_msg_robomaster.find(frame_id);
        
        // check can id 
        if(iter == map_msg_robomaster.end())
        {
            continue;
        }

        msg_robomaster& msg = iter->second;

        msg.buffer.insert(std::end(msg.buffer), frame_buffer, frame_buffer + frame_length);

        // if length zero, looking for header
        if(msg.length == 0)
        {
            std::vector<uint8_t>::const_iterator iter = msg.buffer.cbegin();

            // 
            while(iter != msg.buffer.cend())
            {
                // find header begin
                iter = std::find(iter, std::cend(msg.buffer),0x55);
                // delete all elements till 0x55
                msg.buffer.erase(std::cbegin(msg.buffer), iter);

                // If header is to small wait for next frame
                if(msg.buffer.size() < 4)
                {
                    break;
                }
                // check crc8
                else if(msg.buffer[3] == calculateCRC8(msg.buffer.data(), 3))
                {
                    msg.length = msg.buffer[1];
                    break;
                }
                // go one step further
                else
                {
                    std::cout << "[Handler::runReceiverThread]: crc8 failure" << std::endl;
                    iter++;
                }
            }
        }
        // check if the message is complet
        else if(msg.length <= msg.buffer.size())
        {
            // extract crc16
            const uint16_t crc16 = littleToUint16(msg.buffer[msg.length - 2], msg.buffer[msg.length - 1]);//(static_cast<uint16_t>(msg.buffer[msg.length - 1]) << 8) + static_cast<uint16_t>(msg.buffer[msg.length - 2]);

            if(crc16 == calculateCRC16(msg.buffer.data(), msg.length - 2))
            {
                this->queue_receiver_.push(Message(frame_id, std::vector<uint8_t>(std::cbegin(msg.buffer), std::cbegin(msg.buffer) + msg.length)));
                this->cv_handler_.notify_one();
            }
            else
            {
                std::cerr << "[Handler::runReceiverThread]: crc16 failure" << std::endl;
            }

            msg.buffer.erase(std::cbegin(msg.buffer), std::cbegin(msg.buffer) + msg.length);
            msg.length = 0;
        }
    }

    if(error_counter != 0)
    {
        this->flag_stop_ = true;
        std::cerr << "[Handler::runReceiverThread]: Error frame!" << std::endl;
    }
}

void Handler::runSenderThread()
{
    uint16_t heartbeat_10ms_counter = 0;
    std::chrono::high_resolution_clock::time_point heartbeat_10ms_time_point = std::chrono::high_resolution_clock::now();

    size_t error_counter = 0;

    while (error_counter <= STD_MAX_ERROR_COUNT && !this->flag_stop_)
    {
        // check for the 10ms heartbeat.
        if (heartbeat_10ms_time_point < std::chrono::high_resolution_clock::now())
        {
            if(this->sendMessage(Message(DEVICE_ID_INTELLI_CONTROLLER, 0xc309, heartbeat_10ms_counter++, {0x00, 0x3f, 0x60, 0x00, 0x04, 0x20, 0x00, 0x01, 0x00, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03, 0x00, 0x00})))
            {
                heartbeat_10ms_time_point += STD_HEARTBEAT_TIME;
                error_counter = 0;
            }
            else
            {
                error_counter++;
            }
        }
        // check for message in the queue
        else if(!this->queue_sender_.empty())
        {
            Message msg = queue_sender_.pop();

            // check for valid message
            if (msg.isValid())
            {
                //Send the valid message over can socket
                if (this->sendMessage(msg))
                {
                    error_counter = 0;
                }
                else
                {
                    error_counter++;
                }
            }
        }
        // wait until the next heartbeat is required.
        else
        {
            std::unique_lock<std::mutex> lock(this->cv_sender_mutex_);
            this->cv_sender_.wait_until(lock, heartbeat_10ms_time_point);
        }
    }

    if(error_counter != 0)
    {
        this->flag_stop_ = true;
        std::cerr << "[Handler][runSenderThread]: Error counter e!" << std::endl;
    }
}

void Handler::runHandlerThread()
{
    while (!this->flag_stop_)
    {
        if (!this->queue_receiver_.empty())
        {
            const Message msg = this->queue_receiver_.pop();

            if (msg.isValid())
            {
                this->processReceivedMessage(msg);
            }
        }
        // wait until new message is arrived or a notify for shutdown.
        else
        {
            std::unique_lock<std::mutex> lock(this->cv_handler_mutex_);
            this->cv_handler_.wait(lock);
        }
    }
}

void Handler::pushMessage(const Message &msg)
{
    this->queue_sender_.push(msg);
    this->cv_sender_.notify_one();
}

void Handler::bindCallbackDataRoboMasterState(std::function<void(const Message&)> func)
{
    this->callback_data_robomaster_state_ = func;
}

void Handler::processReceivedMessage(const Message &msg)
{
    if (msg.getDeviceId() == DEVICE_ID_MOTION_CONTROLLER)
    {
        switch (msg.getType())
        {
        case 0x0903:
            //header information of the payload
            if(4 < msg.getPayload().size() 
            && msg.getPayload()[0] == 0x20
            && msg.getPayload()[1] == 0x48
            && msg.getPayload()[2] == 0x08
            && msg.getPayload()[3] == 0x00
            && this->callback_data_robomaster_state_)
            {
                this->callback_data_robomaster_state_(msg);
            }
        
        default:
            break;
        }
    }
}

} // namespace robomaster_can_controller