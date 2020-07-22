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

// extern "C"
// {
//     #include "MessageFrame.h"
// }

#include<vector>
#include<boost/optional.hpp>
#include<RoadSegment.h/ros.h>
#include<carma_utils/CARMAUtils.h>
#include<cav_msgs/ByteArray.h>

namespace MessageConsumer
{
    class Message
    {
        private:
        //node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_,pnh_;

        //ROS sub,pub and spin rate
        int default_spin_rate=10;

        //Publishers
        ros::Publisher outboundPub_;//outgoing byte array after encode
        ros::Publisher bsmPub_; //incoming BSM after decoded
        ros::Publisher mobilityReqPub_; //incoming mobility request after decode
        ros::Publisher mobilityPathPub_; //incoming mobility path message after decoded
        ros::Publisher mobilityResponsePub_; //incoming mobility response publisher after decoded
        ros::Publisher mobilityOperationPub_; //incoming mobility operation message after decoded
        ros::Publisher mapPub_; //incoming MAP message after decoded
        ros::Publisher spatPub_; //incoming SPAT message after decoded

        //Subscribers
        ros::Subscriber alertSub_;
        ros::Subscriber inboundSub_;  //incoming byte array, need to be decoded
        ros::Subscriber bsmSub_; //outgoing plain BSM
        ros::Subscriber mobilityReqSub_; // outgoing plain mobility request message
        ros::Subscriber mobilityPathSub_; //outgoing plain mobility path message
        ros::Subscriber mobilityResponseSub_; //outgoing plain mobility response message
        ros:Subscriber mobilityOperationSub_; //outgoing plain mobility operation message

    /**
     * @brief Initialize pub/sub and params.
     */
    void initialize();

    //callbacks for subscribers
    

    //Log for this node

    //Recording message frequency

    //Messages to be encoded

    //Configure parameters
    bool publishOutboundBsm_=true;
    bool publishOutboundMobilityRequest_=true;
    bool publishOutboundMobilityPath_=true;
    bool publishOutboundMobilityResponse_=true;
    bool publishOutboundMobilityOperation_=true;

    public:
    /**
     * @brief Execution function which will start the ROS subscriptions and publications.
     */
    int run();

    // helper functions for message decode/encode
    boost::optional<j2735_msgs::ControlRequest> decodeMobilityOperation(std::vector<uint8_t>& binary_array);
    boost::optional<std::vector<uint8_t>> encodeMobilityOperation(j2735_msgs::ControlRequest request_msg);
    }
}