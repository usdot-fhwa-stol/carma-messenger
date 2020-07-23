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
    ros::Subscriber inbound_binary_message_sub_;      //incoming byte array, need to decode
    ros::Subscriber mobility_operation_message_sub_; //outgoing plain mobility operation message 

    void initialize();

    public:
    int run();
    //callbacks for subscribers
    void inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg);   //decode the message
    void outbound_mobility_operation_message_callback(const cav_msgs::MobilityOperation& msg);   //Check message type


};

}