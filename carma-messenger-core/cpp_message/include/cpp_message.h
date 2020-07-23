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
#include <j2735_msgs/TrafficControlRequest.h>
#include <j2735_msgs/TrafficControlMessage.h>

// TODO Delete before PR
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
    ros::Subscriber outbound_geofence_control_message_sub_;
    ros::Subscriber inbound_binary_message_sub_;
    ros::Publisher inbound_geofence_request_message_pub_;
    ros::Publisher inbound_geofence_control_message_pub_;

    /**
     * @brief Initialize pub/sub and params.
     */
    void initialize();

    // callbacks for subscribers
    void inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg);
    void outbound_control_request_callback(const j2735_msgs::ControlRequestConstPtr& msg);
    void outbound_control_message_callback(const j2735_msgs::TrafficControlMessageConstPtr& msg);

public:

    /**
     * @brief Execution function which will start the ROS subscriptions and publications.
     */
    int run();

    // helper functions for control message/request decode/encode
    boost::optional<j2735_msgs::ControlRequest> decode_geofence_request(std::vector<uint8_t>& binary_array);
    boost::optional<std::vector<uint8_t>> encode_geofence_request(j2735_msgs::ControlRequest request_msg);
    boost::optional<j2735_msgs::TrafficControlMessage> decode_geofence_control(std::vector<uint8_t>& binary_array);
    boost::optional<std::vector<uint8_t>> encode_geofence_control(j2735_msgs::TrafficControlMessage control_msg);

    // sub-helper functions for decoding TrafficControlMessage
    j2735_msgs::TrafficControlMessageV01 decode_geofence_control_v01(const TrafficControlMessageV01_t& message);
    j2735_msgs::Id64b decode_id64b(const Id64b_t& message);
    j2735_msgs::Id128b decode_id128b(const Id128b_t& message);
    j2735_msgs::TrafficControlPackage decode_geofence_control_package (const TrafficControlPackage_t& message);
    j2735_msgs::TrafficControlParams decode_geofence_control_params (const TrafficControlParams_t& message);
    j2735_msgs::TrafficControlGeometry decode_geofence_control_geometry(const TrafficControlGeometry_t& message);
    j2735_msgs::TrafficControlVehClass decode_geofence_control_veh_class(const TrafficControlVehClass_t& message);
    j2735_msgs::TrafficControlSchedule decode_geofence_control_schedule(const TrafficControlSchedule_t& message);
    j2735_msgs::TrafficControlDetail decode_geofence_control_detail(const TrafficControlDetail_t& message);
    j2735_msgs::DayOfWeek decode_day_of_week(const DSRC_DayOfWeek_t& message);
    j2735_msgs::DailySchedule decode_daily_schedule(const DailySchedule_t& message);
    j2735_msgs::RepeatParams decode_repeat_params(const RepeatParams_t& message);
    j2735_msgs::PathNode decode_path_node(const PathNode_t& message);
    
    // sub-helper functions for encoding TrafficControlMessage
    TrafficControlMessageV01_t*  encode_geofence_control_v01(const j2735_msgs::TrafficControlMessageV01& msg);
    Id64b_t* encode_id64b(const j2735_msgs::Id64b& msg);
    Id128b_t*    encode_id128b(const j2735_msgs::Id128b& msg);
    TrafficControlPackage_t* encode_geofence_control_package (const j2735_msgs::TrafficControlPackage& msg);
    TrafficControlParams_t*  encode_geofence_control_params (const j2735_msgs::TrafficControlParams& msg);
    TrafficControlGeometry_t*    encode_geofence_control_geometry(const j2735_msgs::TrafficControlGeometry& msg);
    TrafficControlVehClass_t*    encode_geofence_control_veh_class(const j2735_msgs::TrafficControlVehClass& msg);
    TrafficControlSchedule_t*    encode_geofence_control_schedule(const j2735_msgs::TrafficControlSchedule& msg);
    TrafficControlDetail_t*  encode_geofence_control_detail(const j2735_msgs::TrafficControlDetail& msg);
    DSRC_DayOfWeek_t*    encode_day_of_week(const j2735_msgs::DayOfWeek& msg);
    DailySchedule_t* encode_daily_schedule(const j2735_msgs::DailySchedule& msg);
    RepeatParams_t*  encode_repeat_params(const j2735_msgs::RepeatParams& msg);
    PathNode_t*  encode_path_node(const j2735_msgs::PathNode& msg);

};
}
