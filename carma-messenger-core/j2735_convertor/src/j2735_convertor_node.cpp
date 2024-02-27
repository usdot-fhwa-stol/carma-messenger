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

      // J2735 BSM Subscriber
    auto j2735_bsm_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j2735_bsm_options;
    j2735_bsm_options.callback_group = j2735_bsm_cb_group;
    j2735_bsm_sub_ = create_subscription<j2735_v2x_msgs::msg::BSM>("incoming_j2735_bsm", 100, std::bind(&Node::j2735BsmHandler, this, std_ph::_1), j2735_bsm_options);

    // BSM Publisher
    converted_bsm_pub_ = create_publisher<carma_v2x_msgs::msg::BSM>("incoming_bsm", 100);

    // Outgoing J2735 BSM Subscriber
    auto outbound_bsm_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions outbound_bsm_options;
    outbound_bsm_options.callback_group = outbound_bsm_cb_group;
    outbound_bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("outgoing_bsm", 1, std::bind(&Node::BsmHandler,
                                          this, std_ph::_1), outbound_bsm_options);  // Queue size of 1 as we should never publish outdated BSMs

    // BSM Publisher
    outbound_j2735_bsm_pub_ = create_publisher<j2735_v2x_msgs::msg::BSM>("outgoing_j2735_bsm", 1);  // Queue size of 1 as we should never publish outdated BSMs

    // J2735 PSM Subscriber
    auto j2735_psm_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j2735_psm_options;
    j2735_psm_options.callback_group = j2735_psm_cb_group;
    j2735_psm_sub_ = create_subscription<j2735_v2x_msgs::msg::PSM>("incoming_j2735_psm", 100, std::bind(&Node::j2735PsmHandler, this, std_ph::_1), j2735_psm_options);

    // PSM Publisher
    converted_psm_pub_ = create_publisher<carma_v2x_msgs::msg::PSM>("incoming_psm", 100);

    // Outgoing J2735 PSM Subscriber
    auto outbound_psm_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions outbound_psm_options;
    outbound_psm_options.callback_group = outbound_psm_cb_group;
    outbound_psm_sub_ = create_subscription<carma_v2x_msgs::msg::PSM>("outgoing_psm", 1, std::bind(&Node::PsmHandler,
                                          this, std_ph::_1), outbound_psm_options);  // Queue size of 1 as we should never publish outdated PSMs

    // PSM Publisher
    outbound_j2735_psm_pub_ = create_publisher<j2735_v2x_msgs::msg::PSM>("outgoing_j2735_psm", 1);  // Queue size of 1 as we should never publish outdated PSMs


    // J2735 SPAT Subscriber
    auto j2735_spat_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j2735_spat_options;
    j2735_spat_options.callback_group = j2735_bsm_cb_group;
    j2735_spat_sub_ = create_subscription<j2735_v2x_msgs::msg::SPAT>("incoming_j2735_spat", 100, std::bind(&Node::j2735SpatHandler, this, std_ph::_1), j2735_spat_options);

    // SPAT Publisher TODO think about queue sizes
    converted_spat_pub_ = create_publisher<carma_v2x_msgs::msg::SPAT>("incoming_spat", 100);

    // J2735 MAP Subscriber
    auto j2735_map_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j2735_map_options;
    j2735_map_options.callback_group = j2735_map_cb_group;
    j2735_map_sub_ = create_subscription<j2735_v2x_msgs::msg::MapData>("incoming_j2735_map", 50, std::bind(&Node::j2735MapHandler, this, std_ph::_1), j2735_map_options);

    // MAP Publisher TODO think about queue sizes
    converted_map_pub_ = create_publisher<carma_v2x_msgs::msg::MapData>("incoming_map", 50);

    // Added, check namespace and standards ///////////////////////////////////////////////////////

    // J3224 SDSM Subscriber
    auto j3224_sdsm_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j3224_sdsm_options;
    j3224_sdsm_options.callback_group = j3224_sdsm_cb_group;
    j3224_sdsm_sub_ = create_subscription<j3224_v2x_msgs::msg::SensorDataSharingMessage>("incoming_j3224_sdsm", 100, std::bind(&Node::j3224SdsmHandler, this, std_ph::_1), j3224_sdsm_options);

    // SDSM carma Publisher
    converted_sdsm_pub_ = create_publisher<carma_v2x_msgs::msg::SensorDataSharingMessage>("incoming_sdsm", 100);

    // Outgoing J3224 SDSM Subscriber
    auto outbound_sdsm_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions outbound_sdsm_options;
    outbound_sdsm_options.callback_group = outbound_sdsm_cb_group;
    outbound_sdsm_sub_ = create_subscription<carma_v2x_msgs::msg::SensorDataSharingMessage>("outgoing_sdsm", 1, std::bind(&Node::SdsmHandler,
                                          this, std_ph::_1), outbound_sdsm_options);  // Queue size of 1 as we should never publish outdated PSMs

    // SDSM j3224 Publisher
    outbound_j3224_sdsm_pub_ = create_publisher<j3224_v2x_msgs::msg::SensorDataSharingMessage>("outgoing_j3224_sdsm", 1);

    // End of SDSM pub/sub ///////////////////////////////////////////////////////


    // Incoming geofence pub/sub
      // NOTE: Currently, intra-process comms must be disabled for the following publisher that are transient_local: https://github.com/ros2/rclcpp/issues/1753
    rclcpp::PublisherOptions intra_proc_disabled;
    intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object

    // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
    auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepAll()); // A publisher with this QoS will store all messages that it has sent on the topic
    pub_qos_transient_local.transient_local();  // A publisher with this QoS will re-send all (when KeepAll is used) messages to all late-joining subscribers
                                          // NOTE: The subscriber's QoS must be set to transient_local() as well for earlier messages to be resent to the later-joiner.

    converted_geofence_control_pub_ = create_publisher<carma_v2x_msgs::msg::TrafficControlMessage>("incoming_geofence_control", pub_qos_transient_local, intra_proc_disabled);

    converted_geofence_request_pub_ = create_publisher<carma_v2x_msgs::msg::TrafficControlRequest>("incoming_geofence_request", 50);

    auto j2735_geofence_control_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j2735_geofence_control_options;
    j2735_geofence_control_options.callback_group = j2735_geofence_control_cb_group;
    j2735_geofence_control_sub_ = create_subscription<j2735_v2x_msgs::msg::TrafficControlMessage>("incoming_j2735_geofence_control", 50, std::bind(&Node::j2735ControlMessageHandler, this, std_ph::_1), j2735_geofence_control_options);


    auto j2735_geofence_request_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions j2735_geofence_request_options;
    j2735_geofence_request_options.callback_group = j2735_geofence_request_cb_group;
    j2735_geofence_request_sub_ = create_subscription<j2735_v2x_msgs::msg::TrafficControlRequest>("incoming_j2735_geofence_request", 50, std::bind(&Node::j2735ControlRequestHandler, this, std_ph::_1), j2735_geofence_request_options);

    // Outgoing geofence pub/sub
    auto outbound_geofence_control_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions outbound_geofence_control_options;
    outbound_geofence_control_options.callback_group = outbound_geofence_control_cb_group;
    outbound_geofence_control_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlMessage>("outgoing_geofence_control", 50, std::bind(&Node::ControlMessageHandler, this, std_ph::_1), outbound_geofence_control_options);


    auto outbound_geofence_request_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions outbound_geofence_request_options;
    outbound_geofence_request_options.callback_group = outbound_geofence_request_cb_group;
    outbound_geofence_request_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlRequest>("outgoing_geofence_request", 50, std::bind(&Node::ControlRequestHandler, this, std_ph::_1), outbound_geofence_request_options);

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

  void Node::PsmHandler(const carma_v2x_msgs::msg::PSM::UniquePtr  message)
  {
    j2735_v2x_msgs::msg::PSM j2735_msg;
    PSMConvertor::convert(*message, j2735_msg);  // Convert message
    outbound_j2735_psm_pub_->publish(j2735_msg);  // Publish converted message
  }

  void Node::j2735PsmHandler(j2735_v2x_msgs::msg::PSM::UniquePtr message)
  {
    carma_v2x_msgs::msg::PSM converted_msg;
    PSMConvertor::convert(*message, converted_msg);  // Convert message
    converted_psm_pub_->publish(converted_msg);       // Publish converted message
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

  // Updated for SDSM ///////////////////////////////////////
    void Node::SdsmHandler(const carma_v2x_msgs::msg::SensorDataSharingMessage::UniquePtr  message)
  {
    j3224_v2x_msgs::msg::SensorDataSharingMessage j3224_msg;
    // RCLCPP_INFO(get_logger(), "Converting outgoing SDSM message");
    // try {
      SDSMConvertor::convert(*message, j3224_msg);  // Convert message
      RCLCPP_INFO(get_logger(), "Done converting outgoing SDSM message");
      outbound_j3224_sdsm_pub_->publish(j3224_msg);  // Publish converted message
    // } catch(std::exception const & e) {
    //   RCLCPP_ERROR_STREAM(get_logger(), "Error while converting outgoing SDSM message: " << e.what());
    // } catch(...) {
    //   RCLCPP_ERROR_STREAM(get_logger(), "Unknown error while converting outgoing SDSM message");
    // }
  }

  void Node::j3224SdsmHandler(j3224_v2x_msgs::msg::SensorDataSharingMessage::UniquePtr message)
  {
    carma_v2x_msgs::msg::SensorDataSharingMessage converted_msg;
    SDSMConvertor::convert(*message, converted_msg);  // Convert message
    converted_sdsm_pub_->publish(converted_msg);       // Publish converted message
  }

  //////////////////////////////////////////////////////

} // j2735_convertor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(j2735_convertor::Node)
