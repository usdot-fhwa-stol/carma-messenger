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

#ifndef TRAFFIC_INCIDENT_H
#define TRAFFIC_INCIDENT_H

#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_ros2_utils.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <carma_msgs/srv/set_traffic_event.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <functional>
#include <string>
#include "traffic_incident_worker.hpp"

namespace traffic{

class TrafficIncidentNode : public carma_ros2_utils::CarmaLifecycleNode
{

public:
  
  /*! \fn TrafficIncidentNode()
    \brief TrafficIncidentNode constructor 
  */
  explicit TrafficIncidentNode(const rclcpp::NodeOptions &);

  /*! \fn publishTrafficIncidentMobilityOperation()
    \brief Publish mobility operation message
  */
  void publishTrafficIncidentMobilityOperation(const carma_v2x_msgs::msg::MobilityOperation& traffic_msg);
  
  // Service callback
  bool startTrafficBroadcastCallback(const std::shared_ptr<rmw_request_id_t>, const carma_msgs::srv::SetTrafficEvent::Request::SharedPtr req, carma_msgs::srv::SetTrafficEvent::Response::SharedPtr resp);
  bool stopTrafficBroadcastCallback(const std::shared_ptr<rmw_request_id_t>, const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);

  /*!fn run()
    \brief General starting point to run this node
  */
  void run();

private:

  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

  void spin_callback(void);

  //subscriber
  carma_ros2_utils::SubPtr<gps_msgs::msg::GPSFix> pinpoint_driver_sub_;

  //publisher
  carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> traffic_mobility_operation_pub_;

  //services
  carma_ros2_utils::ServicePtr<carma_msgs::srv::SetTrafficEvent> start_broadcast_request_service_server;
  carma_ros2_utils::ServicePtr<std_srvs::srv::Trigger> stop_broadcast_request_service_server;

  //timer
  rclcpp::TimerBase::SharedPtr spin_timer_;

  //TrafficIncidentWorker class object
  TrafficIncidentWorker traffic_worker_;

  //ROS Params
  std::string sender_id_; 
  std::string event_reason_;
  std::string event_type_;
  double down_track_;
  double up_track_;
  double min_gap_;

};

}//traffic

#endif /* TRAFFIC_INCIDENT_H */