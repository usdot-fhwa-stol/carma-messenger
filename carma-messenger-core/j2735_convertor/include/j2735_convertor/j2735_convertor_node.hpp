/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace j2735_convertor
{

  /**
   * @class j2735_convertor::Node
   * @brief Is the class responsible for conversion of j2735_msgs to cav_msgs
   *
   * The J2735Convertor is a ROS Node which subscribes to topics containing messages from the j2735_msgs package.
   * The j2735_msgs are then converted to cav_msgs types including any necessary unit conversions.
   *
   * Each j2735 topic has its own thread for processing. This will help prevent larger message types like Map from
   * blocking small high frequency message types like BSMs.
   *
   * Other subscribed topics like system_alert are handled on the default global queue
   *
   * When an internal exception is triggered the node will first broadcast a FATAL message to the system_alert topic
   * before shutting itself down. This node will also shut itself down on receive of a SHUTDOWN message from system_alert
   */
  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::BSM> j2735_bsm_sub_;
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::SPAT> j2735_spat_sub_;
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::MapData> j2735_map_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> outbound_bsm_sub_;
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::TrafficControlMessage> j2735_geofence_control_sub_;
    carma_ros2_utils::SubPtr<j2735_v2x_msgs::msg::TrafficControlRequest> j2735_geofence_request_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::TrafficControlMessage> outbound_geofence_control_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::TrafficControlRequest> outbound_geofence_request_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::BSM> converted_bsm_pub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::SPAT> converted_spat_pub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MapData> converted_map_pub_;
    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::BSM> outbound_j2735_bsm_pub_;
    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::TrafficControlMessage> outbound_j2735_geofence_control_pub_;
    carma_ros2_utils::PubPtr<j2735_v2x_msgs::msg::TrafficControlRequest> outbound_j2735_geofence_request_pub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::TrafficControlMessage> converted_geofence_control_pub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::TrafficControlRequest> converted_geofence_request_pub_;
    

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * @brief Converts carma_v2x_msgs::msg::BSM messages to j2735_v2x_msgs::msg::BSM and publishes the converted messages
     *
     * @param message The message to convert
     */
    void BsmHandler(const carma_v2x_msgs::msg::BSM::UniquePtr  message);

    /**
     * @brief Converts j2735_v2x_msgs::msg::BSM messages to carma_v2x_msgs::msg::BSM and publishes the converted messages
     *
     * @param message The message to convert
     */
    void j2735BsmHandler(j2735_v2x_msgs::msg::BSM::UniquePtr message);

    /**
     * @brief Converts j2735_v2x_msgs::msg::SPAT messages to carma_v2x_msgs::msg::SPAT and publishes the converted messages
     *
     * @param message The message to convert
     */
    void j2735SpatHandler(j2735_v2x_msgs::msg::SPAT::UniquePtr message);

    /**
     * @brief Converts j2735_v2x_msgs::msg::MapData messages to carma_v2x_msgs::msg::MapData and publishes the converted messages
     *
     * @param message The message to convert
     */
    void j2735MapHandler(j2735_v2x_msgs::msg::MapData::UniquePtr message);

    /**
     * @brief Converts carma_v2x_msgs::msg::TrafficControlMessage messages to j2735_v2x_msgs::msg::TrafficControlMessage and publishes the converted messages
     *
     * @param message The message to convert
     */
    void ControlMessageHandler(carma_v2x_msgs::msg::TrafficControlMessage::UniquePtr message);

    /**
     * @brief Converts j2735_v2x_msgs::msg::TrafficControlMessage messages to carma_v2x_msgs::msg::TrafficControlMessage and publishes the converted messages
     *
     * @param message The message to convert
     */
    void j2735ControlMessageHandler(j2735_v2x_msgs::msg::TrafficControlMessage::UniquePtr message);

    /**
     * @brief Converts carma_v2x_msgs::msg::ControlRequest messages to j2735_v2x_msgs::msg::ControlRequest and publishes the converted messages
     *
     * @param message The message to convert
     */
    void ControlRequestHandler(carma_v2x_msgs::msg::TrafficControlRequest::UniquePtr message);

    /**
     * @brief Converts j2735_v2x_msgs::msg::ControlRequest messages to carma_v2x_msgs::msg::ControlRequest and publishes the converted messages
     *
     * @param message The message to convert
     */
    void j2735ControlRequestHandler(j2735_v2x_msgs::msg::TrafficControlRequest::UniquePtr message);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

  };

} // j2735_convertor
