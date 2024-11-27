/*
 * Copyright (C) 2024 LEIDOS.
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

#include "traffic_incident_node.hpp"

namespace traffic
{
namespace std_ph = std::placeholders;

TrafficIncidentNode::TrafficIncidentNode(const rclcpp::NodeOptions & options)
: carma_ros2_utils::CarmaLifecycleNode(options),
  traffic_worker_(
    std::bind(&TrafficIncidentNode::publishTrafficIncidentMobilityOperation, this, std_ph::_1))
{
  declare_parameter<std::string>("sender_id", sender_id_);
  declare_parameter<std::string>("event_reason", event_reason_);
  declare_parameter<std::string>("event_type", event_type_);
  declare_parameter<double>("down_track", down_track_);
  declare_parameter<double>("up_track", up_track_);
  declare_parameter<double>("min_gap", min_gap_);
}

carma_ros2_utils::CallbackReturn TrafficIncidentNode::handle_on_configure(
  const rclcpp_lifecycle::State &)
{
  get_parameter<std::string>("sender_id", sender_id_);
  get_parameter<std::string>("event_reason", event_reason_);
  get_parameter<std::string>("event_type", event_type_);
  get_parameter<double>("down_track", down_track_);
  get_parameter<double>("up_track", up_track_);
  get_parameter<double>("min_gap", min_gap_);

  traffic_worker_.setSenderId(sender_id_);
  traffic_worker_.setDownTrack(down_track_);
  traffic_worker_.setUpTrack(up_track_);
  traffic_worker_.setMinGap(min_gap_);
  traffic_worker_.setEventReason(event_reason_);
  traffic_worker_.setEventType(event_type_);

  // Setup pub/sub
  pinpoint_driver_sub_ = create_subscription<gps_msgs::msg::GPSFix>(
    "gps_common_fix",
    10,
    std::bind(&TrafficIncidentWorker::pinpointDriverCallback, &traffic_worker_, std_ph::_1));
  traffic_mobility_operation_pub_ =
    create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 10);

  // setup services
  start_broadcast_request_service_server = create_service<carma_msgs::srv::SetTrafficEvent>(
    "start_broadcasting_traffic_event",
    std::bind(
      &TrafficIncidentNode::startTrafficBroadcastCallback,
      this,
      std_ph::_1,
      std_ph::_2,
      std_ph::_3));
  stop_broadcast_request_service_server = create_service<std_srvs::srv::Trigger>(
    "stop_broadcasting_traffic_event",
    std::bind(
      &TrafficIncidentNode::stopTrafficBroadcastCallback,
      this,
      std_ph::_1,
      std_ph::_2,
      std_ph::_3));

  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn TrafficIncidentNode::handle_on_activate(
  const rclcpp_lifecycle::State &)
{
  // spin loop
  spin_timer_ = create_timer(
    get_clock(),
    std::chrono::milliseconds(100),
    std::bind(&TrafficIncidentNode::spin_callback, this));
  RCLCPP_INFO_STREAM(get_logger(), "Traffic Incident node is initialized...");
  return CallbackReturn::SUCCESS;
}

void TrafficIncidentNode::publishTrafficIncidentMobilityOperation(
  const carma_v2x_msgs::msg::MobilityOperation & traffic_msg)
{
  traffic_mobility_operation_pub_->publish(traffic_msg);
}


/*****
 * Used by UI to start broadcasting traffic event (geofence)
 * Msg: MobilityOperation
 * **/
bool TrafficIncidentNode::startTrafficBroadcastCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const carma_msgs::srv::SetTrafficEvent::Request::SharedPtr req,
  carma_msgs::srv::SetTrafficEvent::Response::SharedPtr resp)
{
  // update instance variables with incoming request params
  traffic_worker_.setMinGap(req->minimum_gap);
  traffic_worker_.setDownTrack(req->down_track);
  traffic_worker_.setUpTrack(req->up_track);
  traffic_worker_.setAdvisorySpeed(req->advisory_speed);

  // return service response true
  resp->success = true;
  return true;
}

/*****
 * Used by UI to stop broadcasting traffic event (geofence)
 * Msg: MobilityOperation
 * **/
bool TrafficIncidentNode::stopTrafficBroadcastCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  try {
    // reset instance variables
    traffic_worker_.setMinGap(0);
    traffic_worker_.setDownTrack(0);
    traffic_worker_.setUpTrack(0);
    traffic_worker_.setAdvisorySpeed(0);

    resp->success = true;
    resp->message = "stop broadcasting";

    // return service response true
    return true;

  } catch (...) {
    // in case any exception
    resp->success = false;
    return false;
  }
}

void TrafficIncidentNode::spin_callback(void)
{
  if (
    traffic_worker_.getDownTrack() > 0 || traffic_worker_.getUpTrack() > 0 || traffic_worker_.getAdvisorySpeed() > 0) {
    // construct local mobilityOperation msg
    carma_v2x_msgs::msg::MobilityOperation traffic_mobility_msg =
      traffic_worker_.mobilityMessageGenerator(traffic_worker_.getPinPoint());

    // start constantly broadcasting mobilityOperation msg
    traffic_mobility_operation_pub_->publish(traffic_mobility_msg);
  }
}


}  // namespace traffic

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(traffic::TrafficIncidentNode)