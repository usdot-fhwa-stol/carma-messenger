#pragma once
/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <j2735_msgs/BSM.h>
#include <j2735_msgs/SPAT.h>
#include <j2735_msgs/MapData.h>
#include <j2735_msgs/ControlMessage.h>
#include <j2735_msgs/TrafficControlRequest.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/SPAT.h>
#include <cav_msgs/MapData.h>
#include <cav_msgs/ControlMessage.h>
#include <cav_msgs/TrafficControlRequest.h>
#include <cav_msgs/MapData.h>
#include <j2735_convertor/bsm_convertor.h>
#include <j2735_convertor/map_convertor.h>
#include <j2735_convertor/spat_convertor.h>
#include <carma_utils/CARMANodeHandle.h>

namespace j2735_convertor
{
/**
 * @class J2735Convertor
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
 * before shutting itself down. This node will also shut itself down on recieve of a SHUTDOWN message from system_alert
 */
class J2735Convertor
{
private:
  // Shutdown flags and mutex
  std::mutex shutdown_mutex_;
  bool shutting_down_ = false;
  // Members used in ROS behavior
  int default_spin_rate_ = 10;
  ros::Publisher converted_bsm_pub_, converted_spat_pub_, converted_map_pub_,
      outbound_j2735_bsm_pub_, outbound_j2735_geofence_control_pub_, outbound_j2735_geofence_request_pub_, 
      converted_geofence_control_pub_, converted_geofence_request_pub_;
  ros::Subscriber j2735_bsm_sub_, j2735_spat_sub_, j2735_map_sub_, outbound_bsm_sub_,
      j2735_geofence_control_sub_, j2735_geofence_request_sub_, outbound_geofence_control_sub_, outbound_geofence_request_sub_;
  std::shared_ptr<ros::CARMANodeHandle> default_nh_;
  std::shared_ptr<ros::CARMANodeHandle> bsm_nh_;
  std::shared_ptr<ros::CARMANodeHandle> spat_nh_;
  std::shared_ptr<ros::CARMANodeHandle> map_nh_;
  std::shared_ptr<ros::CARMANodeHandle> geofence_nh_;
  ros::CallbackQueue bsm_queue_;
  ros::CallbackQueue spat_queue_;
  ros::CallbackQueue map_queue_;
  ros::CallbackQueue geofence_queue_;

public:
  /**
   * @brief Constructor
   * @param argc - command line argument count
   * @param argv - command line arguments
   */
  J2735Convertor(int argc, char** argv)
  {
    ros::init(argc, argv, "j2735_convertor_node");  // Initialize ROS connection
  };

  /**
   * @brief Execution function which will start the ROS subscriptions and publications. Exits on node shutdown.
   */
  int run();

private:
  /**
   * @brief Initializes the subscribers and publishers for this node
   */
  void initialize();

  /**
   * @brief Converts cav_msgs::BSM messages to j2735_msgs::BSM and publishes the converted messages
   *
   * @param message The message to convert
   */
  void BsmHandler(const cav_msgs::BSMConstPtr& message);

  /**
   * @brief Converts j2735_msgs::BSM messages to cav_msgs::BSM and publishes the converted messages
   *
   * @param message The message to convert
   */
  void j2735BsmHandler(const j2735_msgs::BSMConstPtr& message);

  /**
   * @brief Converts j2735_msgs::SPAT messages to cav_msgs::SPAT and publishes the converted messages
   *
   * @param message The message to convert
   */
  void j2735SpatHandler(const j2735_msgs::SPATConstPtr& message);

  /**
   * @brief Converts j2735_msgs::MapData messages to cav_msgs::MapData and publishes the converted messages
   *
   * @param message The message to convert
   */
  void j2735MapHandler(const j2735_msgs::MapDataConstPtr& message);

  /**
   * @brief Converts cav_msgs::ControlMessage messages to j2735_msgs::ControlMessage and publishes the converted messages
   *
   * @param message The message to convert
   */
  void ControlMessageHandler(const cav_msgs::ControlMessageConstPtr& message);

  /**
   * @brief Converts j2735_msgs::ControlMessage messages to cav_msgs::ControlMessage and publishes the converted messages
   *
   * @param message The message to convert
   */
  void j2735ControlMessageHandler(const j2735_msgs::ControlMessageConstPtr& message);

    /**
   * @brief Converts cav_msgs::ControlRequest messages to j2735_msgs::ControlRequest and publishes the converted messages
   *
   * @param message The message to convert
   */
  void ControlRequestHandler(const cav_msgs::TrafficControlRequestConstPtr& message);

  /**
   * @brief Converts j2735_msgs::ControlRequest messages to cav_msgs::ControlRequest and publishes the converted messages
   *
   * @param message The message to convert
   */
  void j2735ControlRequestHandler(const j2735_msgs::TrafficControlRequestConstPtr& message);
};

}  // namespace j2735_convertor