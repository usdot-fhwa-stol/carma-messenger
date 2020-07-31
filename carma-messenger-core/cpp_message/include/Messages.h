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

#include<vector>
#include<boost/optional.hpp>
#include<ros/ros.h>
#include<carma_utils/CARMAUtils.h>
#include<cav_msgs/ByteArray.h>
#include<cav_msgs/MobilityHeader.h>
#include<cav_msgs/MobilityOperation.h>
#include<cav_msgs/MobilityResponse.h>
#include<cav_msgs/MobilityPath.h>

namespace Message_cpp
{
class MessageConsumer
{
    private:
    //node handles
    std::shared_ptr<ros::CARMANodeHandle> nh_,pnh_;

    //ROS sub,pub and spin rate
    int default_spin_rate=10;
    ros::Publisher outbound_binary_message_pub_;    //outgoing byte array after encode
    ros::Publisher mobility_operation_message_pub_;  //incoming mobility operation message after decoded
    ros::Publisher mobility_response_message_pub_;     //incoming mobility response message after decoded
    ros::Subscriber inbound_binary_message_sub_;      //incoming byte array, need to decode
    ros::Subscriber mobility_operation_message_sub_; //outgoing plain mobility operation message 
    ros::Subscriber mobility_response_message_sub_; //outgoing plain mobility response message

    void initialize();

    public:
    int run();
    /**
     * @brief function callback when there is an incoming byte array. Message type is checked and required decoding function called.
     * @param msg container with binary input. Passed to a decoding function depending on message type.
     */
    void inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg); 
    /**
     * @brief function callback when there is an incoming mobility operation message. .
     * @param msg container with Mobility Operation ros message. Passed to an encoding function in Mobility_Operation class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_mobility_operation_message_callback(const cav_msgs::MobilityOperation& msg);  
    /**
     * @brief function callback when there is an incoming mobility response message. .
     * @param msg container with Mobility response ros message. Passed to an encoding function in Mobility_Response class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_mobility_response_message_callback(const cav_msgs::MobilityResponse& msg);   

};

}