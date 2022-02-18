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
#include "truck_inspection_plugin/truck_inspection_plugin_node.hpp"

namespace truck_inspection_plugin
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.num_of_entries = declare_parameter<int>("num_of_entries", config_.num_of_entries);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    // TODO for the USER: Ensure all parameters can be updated dynamically by adding them to this method
    auto error = update_params<int>({{"num_of_entries", config_.num_of_entries}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<int>("num_of_entries", config_.num_of_entries);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    mo_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>("mobility_operation_inbound", 5,
                                                              std::bind(&Node::mobilityOperationCallback, this, std_ph::_1));

    // Setup publishers
    mr_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityRequest>("mobility_request_outbound", 5);
    cav_detection_pub_ = create_publisher<std_msgs::msg::String>("cav_truck_identified", 5);
    content_pub_ = create_publisher<std_msgs::msg::String>("truck_safety_info", 5);

    // Setup service servers
    inspection_request_service_server_ = create_service<std_srvs::srv::Trigger>("send_inspection_request",std::bind(&Node::inspectionRequestCallback, this, std_ph::_1, std_ph::_2, std_ph::_3));
    
    if(!this->safety_log_.empty()) {
        std_msgs::msg::String msg_content;
        msg_content.data = safety_log_;
        content_pub_->publish(msg_content);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Truck inspection plugin is initialized...");

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  // Parameter names not shown to prevent unused compile warning. The user may add them back
  void Node::inspectionRequestCallback( std::shared_ptr<rmw_request_id_t> header,
                                   std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
        carma_v2x_msgs::msg::MobilityRequest msg;
        msg.strategy = Node::INSPECTION_STRATEGY;
        mr_pub_->publish(msg);
        // reset safety log
        this->safety_log_ = "";
        response->success = true;
  }

  void Node::mobilityOperationCallback(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg)
  {
        // if there is a truck around running CARMA
        if(msg->strategy == Node::INSPECTION_STRATEGY)
        {
            // if the incoming message contains a valid safety log
            if(isSafetyLogValid(msg->strategy_params))
            {
                safety_log_ = msg->strategy_params;
                safety_log_ += ",timestamp:" + std::to_string(msg->m_header.timestamp);
            } else {
                std_msgs::msg::String msg_out;
                std::string k_v_pair = msg->strategy_params;
                // get only VIN number, state and license plate
                msg_out.data = k_v_pair;
                // publish message to show there is a cav truck in the radio range of the ego-vehicle
                cav_detection_pub_->publish(msg_out);
            }
        }
  }


  bool Node::isSafetyLogValid(const std::string& log)
  {
      // Check 1: if number of k-v pairs matches expectation
      std::string temp;
      int count = 0;
      std::stringstream ss(log);
      while(std::getline(ss, temp, ','))
      {
          if(!temp.empty())
          {
              count++;
          }
          else
          {
              // do not allow spaces in the safety log
              return false;
          }
      }
      return count == this->number_of_entries;
  }

} // truck_inspection_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(truck_inspection_plugin::Node)
