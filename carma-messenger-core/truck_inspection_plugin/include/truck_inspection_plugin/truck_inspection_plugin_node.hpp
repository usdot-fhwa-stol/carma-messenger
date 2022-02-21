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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "truck_inspection_plugin/truck_inspection_plugin_config.hpp"
#include "carma_v2x_msgs/msg/mobility_request.hpp"
#include "carma_v2x_msgs/msg/mobility_operation.hpp"
#include <std_srvs/srv/trigger.hpp>

namespace truck_inspection_plugin
{

  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityOperation> mo_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<std_msgs::msg::String> content_pub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityRequest> mr_pub_;
    carma_ros2_utils::PubPtr<std_msgs::msg::String> cav_detection_pub_;

    // Service Servers
    carma_ros2_utils::ServicePtr<std_srvs::srv::Trigger> inspection_request_service_server_;

    // Node configuration
    Config config_;

    const std::string INSPECTION_STRATEGY = "TruckInspection";
    int number_of_entries;
    std::string safety_log_;

    // Timers
    rclcpp::TimerBase::SharedPtr content_pub_timer_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * \brief callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief subscription callback
     */
    void mobilityOperationCallback(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg);

    /**
      * \brief service callback
      */
    void inspectionRequestCallback(std::shared_ptr<rmw_request_id_t> header,
                                  std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // helper function to verify safety log
    bool isSafetyLogValid(const std::string& log);

    // helper function to check ads_auto_status
    bool isADSAutoEngaged(const std::string& log);

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    void content_pub_timer_callback();

  };

} // truck_inspection_plugin
