/*
 * Copyright (C) 2022 LEIDOS.
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
#include "j2735_convertor/j2735_convertor_node.hpp"

namespace j2735_convertor
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {}


  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {

    /*
    TODO In ROS1 each topic is given its own thread to spin on using AsyncSpinner.
         This reduces latency in the conversions (bsms are not held up while Maps convert etc.)
         Still need to determine how to implement this multi-threading behavior in ROS2. 
         Documentation should be reviewed.

         
      // Setup node handles here if needed
      default_nh_.reset(new ros::CARMANodeHandle());
      bsm_nh_.reset(new ros::CARMANodeHandle());
      spat_nh_.reset(new ros::CARMANodeHandle());
      map_nh_.reset(new ros::CARMANodeHandle());
      geofence_nh_.reset(new ros::CARMANodeHandle());
      // Set Callback Queues for Node Handles
      bsm_nh_->setCallbackQueue(&bsm_queue_);
      spat_nh_->setCallbackQueue(&spat_queue_);
      map_nh_->setCallbackQueue(&map_queue_);
      geofence_nh_->setCallbackQueue(&geofence_queue_);

      // Attach bsm, spat, map to processing to unique threads
      ros::AsyncSpinner bsm_spinner(1, &bsm_queue_);
      bsm_spinner.start();

      ros::AsyncSpinner spat_spinner(1, &spat_queue_);
      spat_spinner.start();

      ros::AsyncSpinner map_spinner(1, &map_queue_);
      map_spinner.start();

      ros::AsyncSpinner geofence_spinner(1, &geofence_queue_);
      geofence_spinner.start();

    */

      // J2735 BSM Subscriber
    j2735_bsm_sub_ = create_subscription<j2735_v2x_msgs::msg::BSM>("incoming_j2735_bsm", 100, std::bind(&Node::j2735BsmHandler, this, std_ph::_1));

    // BSM Publisher
    converted_bsm_pub_ = create_publisher<carma_v2x_msgs::msg::BSM>("incoming_bsm", 100);

    // Outgoing J2735 BSM Subscriber
    outbound_bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("outgoing_bsm", 1, std::bind(&Node::BsmHandler,
                                          this);  // Queue size of 1 as we should never publish outdated BSMs

    // BSM Publisher
    outbound_j2735_bsm_pub_ = create_publisher<j2735_v2x_msgs::msg::BSM>("outgoing_j2735_bsm", 1);  // Queue size of 1 as we should never publish outdated BSMs

    // J2735 SPAT Subscriber
    j2735_spat_sub_ = create_subscription<j2735_v2x_msgs::msg::SPAT>("incoming_j2735_spat", 100, std::bind(&Node::j2735SpatHandler, this, std_ph::_1));

    // SPAT Publisher TODO think about queue sizes
    converted_spat_pub_ = create_publisher<carma_v2x_msgs::msg::SPAT>("incoming_spat", 100);

    // J2735 MAP Subscriber
    j2735_map_sub_ = create_subscription<j2735_v2x_msgs::msg::MapData>("incoming_j2735_map", 50, std::bind(&Node::j2735MapHandler, this, std_ph::_1));

    // MAP Publisher TODO think about queue sizes
    converted_map_pub_ = create_publisher<carma_v2x_msgs::msg::MapData>("incoming_map", 50);

    // Incoming geofence pub/sub
    converted_geofence_control_pub_ = create_publisher<carma_v2x_msgs::msg::TrafficControlMessage>("incoming_geofence_control", 50);
    converted_geofence_request_pub_ = create_publisher<carma_v2x_msgs::msg::TrafficControlRequest>("incoming_geofence_request", 50);

    j2735_geofence_control_sub_ = create_subscription<j2735_v2x_msgs::msg::TrafficControlMessage>("incoming_j2735_geofence_control", 50, std::bind(&Node::j2735ControlMessageHandler, this, std_ph::_1));
    j2735_geofence_request_sub_ = create_subscription<j2735_v2x_msgs::msg::TrafficControlRequest>("incoming_j2735_geofence_request", 50, std::bind(&Node::j2735ControlRequestHandler, this, std_ph::_1));

    // Outgoing geofence pub/sub
    outbound_geofence_control_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlMessage>("outgoing_geofence_control", 50, std::bind(&Node::ControlMessageHandler, this, std_ph::_1));
    outbound_geofence_request_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlRequest>("outgoing_geofence_request", 50, std::bind(&Node::ControlRequestHandler, this, std_ph::_1));

    outbound_j2735_geofence_control_pub_ = create_publisher<j2735_v2x_msgs::msg::TrafficControlMessage>("outgoing_j2735_geofence_control", 10);
    outbound_j2735_geofence_request_pub_ = create_publisher<j2735_v2x_msgs::msg::TrafficControlRequest>("outgoing_j2735_geofence_request", 10);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void Node::BsmHandler(const carma_v2x_msgs::msg::BSM::UniquePtr  message)
  {
    j2735_v2x_msgs::msg::BSM j2735_msg;
    BSMConvertor::convert(*message, j2735_msg);  // Convert message
    outbound_j2735_bsm_pub_->publish(j2735_msg);  // Publish converted message
  }

  void Node::j2735BsmHandler(j2735_v2x_msgs::msg::BSM::UniquePtr message)
  {
    carma_v2x_msgs::msg::BSM converted_msg;
    BSMConvertor::convert(*message, converted_msg);  // Convert message
    converted_bsm_pub_->publish(converted_msg);       // Publish converted message
  }

  void Node::j2735SpatHandler(j2735_v2x_msgs::msg::SPAT::UniquePtr message)
  {
    carma_v2x_msgs::msg::SPAT converted_msg;
    SPATConvertor::convert(*message, converted_msg);  // Convert message
    converted_spat_pub_->publish(converted_msg);       // Publish converted message
  }

  void Node::j2735MapHandler(j2735_v2x_msgs::msg::MapData::UniquePtr message)
  {
    carma_v2x_msgs::msg::MapData converted_msg;
    MapConvertor::convert(*message, converted_msg);  // Convert message
    converted_map_pub_->publish(converted_msg);       // Publish converted message
  }

  void Node::ControlMessageHandler(carma_v2x_msgs::msg::TrafficControlMessage::UniquePtr message) {
    j2735_v2x_msgs::msg::TrafficControlMessage converted_msg;
    j2735_convertor::geofence_control::convert(*message, converted_msg);  // Convert message
    outbound_j2735_geofence_control_pub_->publish(converted_msg);       // Publish converted message
  }

  void Node::j2735ControlMessageHandler(j2735_v2x_msgs::msg::TrafficControlMessage::UniquePtr message) {
    carma_v2x_msgs::msg::TrafficControlMessage converted_msg;
    j2735_convertor::geofence_control::convert(*message, converted_msg);  // Convert message
    converted_geofence_control_pub_->publish(converted_msg);       // Publish converted message
  }

  void Node::ControlRequestHandler(carma_v2x_msgs::msg::TrafficControlRequest::UniquePtr message) {
    j2735_v2x_msgs::msg::TrafficControlRequest converted_msg;
    j2735_convertor::geofence_request::convert(*message, converted_msg);  // Convert message
    outbound_j2735_geofence_request_pub_->publish(converted_msg);       // Publish converted message
  }

  void Node::j2735ControlRequestHandler(j2735_v2x_msgs::msg::TrafficControlRequest::UniquePtr message) {
    carma_v2x_msgs::msg::TrafficControlRequest converted_msg;
    j2735_convertor::geofence_request::convert(*message, converted_msg);  // Convert message
    converted_geofence_request_pub_->publish(converted_msg);       // Publish converted message
  }

} // j2735_convertor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(j2735_convertor::Node)
