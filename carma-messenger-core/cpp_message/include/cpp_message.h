#pragma once
/*
 * Copyright (C) 2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

extern "C"
{
#include "MessageFrame.h"
}

#include <vector>
#include <boost/optional.hpp>
#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/ByteArray.h>
#include <j2735_msgs/ControlRequest.h>

namespace cpp_message
{
/**
 * @class Message
 * @brief Is the class responsible for DSRC message encoding and decoding
 * 
 * It currently only supports Geofence message type.
 */
class Message
{
private:

    // node handles
    std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;
    
    // ROS sub, pub and spin rate
    int default_spin_rate_ = 10;
    ros::Publisher outbound_binary_message_pub_;
    ros::Subscriber outbound_geofence_request_message_sub_;
    ros::Subscriber inbound_binary_message_sub_;
    ros::Publisher inbound_geofence_request_message_pub_;

    /**
     * @brief Initialize pub/sub and params.
     */
    void initialize();

    // callbacks for subscribers
    void inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg);
    void outbound_control_request_callback(const j2735_msgs::ControlRequestConstPtr& msg);

public:

    /**
     * @brief Execution function which will start the ROS subscriptions and publications.
     */
    int run();

    // helper functions for message decode/encode
    boost::optional<j2735_msgs::ControlRequest> decode_geofence_request(std::vector<uint8_t>& binary_array);
    boost::optional<std::vector<uint8_t>> encode_geofence_request(j2735_msgs::ControlRequest request_msg);

};
}
