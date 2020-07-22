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

namespace Mobility_Operation
{
        /**
         * @class Message
         * @brief Is the class responsible for DSRC message encoding and decoding
         */ 

class Mobility_Operation_Message
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

    //constants
    int STATIC_ID_MAX_LENGTH=16;
    std::string BSM_ID_DEFAULT="00000000";
    int BSM_ID_LENGTH=BSM_ID_DEFAULT.length();
    std::string STRING_DEFAULT="[]";
    int TIMESTAMP_LENGTH=std::to_string(INT64_MAX).length();
    std::string GUID_DEFAULT= "00000000-0000-0000-0000-000000000000";
    int GUID_LENGTH=GUID_DEFAULT.length();
    int STRATEGY_MAX_LENGTH=50;
    int STRATEGY_PARAMS_MAX_LENGTH=1000;

    /**
        * @brief Initialize pub/sub and params.
    */
    void initialize();

    //callbacks for subscribers
    void inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg);   //decode the message
    void outbound_mobility_operation_message_callback(const cav_msgs::MobilityOperation& msg);   //Check message type

    public:
        /**
        * @brief Execution function which will start the ROS subscriptions and publications.
        */
        int run();

        //helper functions for message decode/encode
        boost::optional<cav_msgs::MobilityOperation>decode_mobility_operation_message(std::vector<uint8_t>& binary_array);
        boost::optional<std::vector<uint8_t>> encode_mobility_operation_message(cav_msgs::MobilityOperation plainMessage);
        
};
} 