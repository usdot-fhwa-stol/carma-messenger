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

/**
 * CPP File containing Message implementations 
 */

#include "Messages.h"
#include "MobilityOperation_Message.h"
#include "MobilityResponse_Message.h"

namespace Message_cpp
{
    void MessageConsumer::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        //initialize pub/sub
        outbound_binary_message_pub_=nh_->advertise<cav_msgs::ByteArray>("outbound_binary_msg",5);
        inbound_binary_message_sub_=nh_->subscribe("inbound_binary_msg",5, &MessageConsumer::inbound_binary_callback, this);
        mobility_operation_message_pub_=nh_->advertise<cav_msgs::ByteArray>("incoming_mobility_operation_message_decoded",5);
        mobility_operation_message_sub_=nh_->subscribe<>("outgoing_plain_mobility_operation_message",5, &MessageConsumer::outbound_mobility_operation_message_callback,this);
        mobility_response_message_pub_=nh_->advertise<cav_msgs::ByteArray>("incoming_mobility_response_message_decoded",5);
        mobility_response_message_sub_=nh_->subscribe<>("outgoing_plain_mobility_operation_message",5, &MessageConsumer::outbound_mobility_response_message_callback,this);
    }
    
    void MessageConsumer::inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg)
    {   //find message type and then send for decoding
        if(msg->messageType=="MobilityOperation")   
        {
            std::vector<uint8_t> array=msg->content;
            Mobility_Operation decode;
            auto output=decode.decode_mobility_operation_message(array);

        }
        else if(msg->messageType=="MobilityResponse")
        {
            std::vector<uint8_t> array=msg->content;
            Mobility_Response decode;
            auto output=decode.decode_mobility_response_message(array);
        }
        else
        {
            ROS_WARN_STREAM("Cannot decode mobility operation message.");
        }
    }
    
    int MessageConsumer::run(){
        initialize();
        ros::CARMANodeHandle::spin();
        return 0;
    }

    void MessageConsumer::outbound_mobility_operation_message_callback(const cav_msgs::MobilityOperation& msg)
    {//encode and publish as outbound binary message
        Mobility_Operation encode;
        auto res=encode.encode_mobility_operation_message(msg);
        if(res)
        {
            //copy to byte array msg
            cav_msgs::ByteArray output;
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_.publish(output);
        }
        else
        {
            ROS_WARN_STREAM("Cannot encode mobility operation message.");
        }
    }

    void MessageConsumer::outbound_mobility_response_message_callback(const cav_msgs::MobilityResponse& msg)
        {//encode and publish as outbound binary message
        Mobility_Response encode;
        auto res=encode.encode_mobility_response_message(msg);
        if(res)
        {
            //copy to byte array msg
            cav_msgs::ByteArray output;
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_.publish(output);
        }
        else
        {
            ROS_WARN_STREAM("Cannot encode mobility response message.");
        }
    }
}