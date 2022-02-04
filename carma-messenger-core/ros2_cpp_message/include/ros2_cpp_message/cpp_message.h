#pragma once
/*
 * Copyright (C) 2020-2021 LEIDOS.
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
#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <functional>

#include "j2735_v2x_msgs/msg/byte_array.hpp"
#include "carma_v2x_msgs/msg/mobility_header.hpp"
#include "carma_v2x_msgs/msg/mobility_operation.hpp"
#include "carma_v2x_msgs/msg/mobility_response.hpp"
#include "carma_v2x_msgs/msg/mobility_path.hpp"
#include "carma_v2x_msgs/msg/mobility_request.hpp"

#include "carma_driver_msgs/msg/byte_array.hpp"

#include "j2735_v2x_msgs/msg/bsm.hpp"
#include "j2735_v2x_msgs/msg/map_data.hpp"
#include "j2735_v2x_msgs/msg/spat.hpp"
#include "j2735_v2x_msgs/msg/traffic_control_request.hpp"
#include "j2735_v2x_msgs/msg/traffic_control_message.hpp"


namespace cpp_message
{
/**
 * @class Message
 * @brief Is the class responsible for DSRC message encoding and decoding
 * 
 */

// class Message
// {

class Node : public carma_ros2_utils::CarmaLifecycleNode
{

private:
    
    // ROS sub, pub and spin rate
    int default_spin_rate_ = 10;

    carma_ros2_utils::PubPtr<carma_driver_msgs::msg::ByteArray> outbound_binary_message_pub_;
    carma_ros2_utils::SubPtr<carma_driver_msgs::msg::ByteArray> inbound_binary_message_sub_;

    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::TrafficControlRequest> outbound_geofence_request_message_sub_;
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::TrafficControlMessage> outbound_geofence_control_message_sub_;

    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::TrafficControlRequest> inbound_geofence_request_message_pub_;
    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::TrafficControlMessage> inbound_geofence_control_message_pub_;

    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_message_pub_;  //incoming mobility operation message after decoded
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityResponse> mobility_response_message_pub_;     //incoming mobility response message after decoded

    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_message_sub_; //outgoing plain mobility operation message 
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityResponse> mobility_response_message_sub_; //outgoing plain mobility response message

    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityPath> mobility_path_message_pub_;     //incoming mobility path message after decoded
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityPath> mobility_path_message_sub_;    //outgoing plain mobility path message

    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityRequest> mobility_request_message_pub_;     //incoming mobility request message after decoded
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityRequest> mobility_request_message_sub_;    //outgoing plain mobility request message

    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::BSM> bsm_message_pub_;     //incoming bsm message
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::BSM> bsm_message_sub_;    //outgoing plain bsm message

    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::SPAT> spat_message_pub_;    //incoming spat message
    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::MapData> map_message_pub_; //incoming map message

    /**
     * @brief Initialize pub/sub and params.
     */

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    // callbacks for subscribers
    void inbound_binary_callback(carma_driver_msgs::msg::ByteArray::UniquePtr msg);
    void outbound_control_message_callback(j2735_v2x_msgs::msg::TrafficControlMessage::UniquePtr msg);

    void outbound_control_request_callback(j2735_v2x_msgs::msg::TrafficControlRequest::UniquePtr msg);
    /**
     * @brief function callback when there is an outgoing mobility operation message. .
     * @param msg container with Mobility Operation ros message. Passed to an encoding function in Mobility_Operation class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_mobility_operation_message_callback(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg);  
    /**
     * @brief function callback when there is an outgoing mobility response message. .
     * @param msg container with Mobility response ros message. Passed to an encoding function in Mobility_Response class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_mobility_response_message_callback(carma_v2x_msgs::msg::MobilityResponse::UniquePtr msg);
    /**
     * @brief function callback when there is an outgoing mobility path message. .
     * @param msg container with Mobility path ros message. Passed to an encoding function in Mobility_Path class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_mobility_path_message_callback(carma_v2x_msgs::msg::MobilityPath::UniquePtr msg);
    /**
     * @brief function callback when there is an outgoing mobility request message. .
     * @param msg container with Mobility request ros message. Passed to an encoding function in Mobility_Request class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_mobility_request_message_callback(carma_v2x_msgs::msg::MobilityRequest::UniquePtr msg);
    /**
     * @brief function callback when there is an outgoing bsm message.
     * @param msg container with BSM ros message. Passed to an encoding function in BSM_Message class.
     * The encoded message is published as outbound binary message. Failure to encode results in a ROS Warning.
     */
    void outbound_bsm_message_callback(j2735_v2x_msgs::msg::BSM::UniquePtr msg);

public:

    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    // helper functions for control message/request decode/encode
    boost::optional<j2735_v2x_msgs::msg::TrafficControlRequest> decode_geofence_request(std::vector<uint8_t>& binary_array);
    boost::optional<std::vector<uint8_t>> encode_geofence_request(j2735_v2x_msgs::msg::TrafficControlRequest request_msg);
    boost::optional<j2735_v2x_msgs::msg::TrafficControlMessage> decode_geofence_control(std::vector<uint8_t>& binary_array);
    boost::optional<std::vector<uint8_t>> encode_geofence_control(j2735_v2x_msgs::msg::TrafficControlMessage control_msg);

    // sub-helper functions for decoding TrafficControlMessage
    j2735_v2x_msgs::msg::TrafficControlMessageV01 decode_geofence_control_v01(const TrafficControlMessageV01_t& message);
    j2735_v2x_msgs::msg::Id64b decode_id64b(const Id64b_t& message);
    j2735_v2x_msgs::msg::Id128b decode_id128b(const Id128b_t& message);
    j2735_v2x_msgs::msg::TrafficControlPackage decode_geofence_control_package (const TrafficControlPackage_t& message);
    j2735_v2x_msgs::msg::TrafficControlParams decode_geofence_control_params (const TrafficControlParams_t& message);
    j2735_v2x_msgs::msg::TrafficControlGeometry decode_geofence_control_geometry(const TrafficControlGeometry_t& message);
    j2735_v2x_msgs::msg::TrafficControlVehClass decode_geofence_control_veh_class(const TrafficControlVehClass_t& message);
    j2735_v2x_msgs::msg::TrafficControlSchedule decode_geofence_control_schedule(const TrafficControlSchedule_t& message);
    j2735_v2x_msgs::msg::TrafficControlDetail decode_geofence_control_detail(const TrafficControlDetail_t& message);
    j2735_v2x_msgs::msg::DayOfWeek decode_day_of_week(const DSRC_DayOfWeek_t& message);
    j2735_v2x_msgs::msg::DailySchedule decode_daily_schedule(const DailySchedule_t& message);
    j2735_v2x_msgs::msg::RepeatParams decode_repeat_params(const RepeatParams_t& message);
    j2735_v2x_msgs::msg::PathNode decode_path_node(const PathNode_t& message);
    
    // sub-helper functions for encoding TrafficControlMessage
    Id64b_t* encode_id64b(const j2735_v2x_msgs::msg::Id64b& msg);
    Id128b_t*    encode_id128b(const j2735_v2x_msgs::msg::Id128b& msg);
    TrafficControlVehClass_t*    encode_geofence_control_veh_class(const j2735_v2x_msgs::msg::TrafficControlVehClass& msg);
    TrafficControlDetail_t*  encode_geofence_control_detail(const j2735_v2x_msgs::msg::TrafficControlDetail& msg);
    DSRC_DayOfWeek_t*    encode_day_of_week(const j2735_v2x_msgs::msg::DayOfWeek& msg);
    DailySchedule_t* encode_daily_schedule(const j2735_v2x_msgs::msg::DailySchedule& msg);
    RepeatParams_t*  encode_repeat_params(const j2735_v2x_msgs::msg::RepeatParams& msg);
    PathNode_t*  encode_path_node(const j2735_v2x_msgs::msg::PathNode& msg);


};
}
