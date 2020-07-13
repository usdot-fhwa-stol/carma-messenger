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

/**
 * CPP File containing J2735Convertor method definitions
 */

#include "j2735_convertor.h"
#include <j2735_convertor/control_message_convertor.h>
#include <j2735_convertor/control_request_convertor.h>

namespace j2735_convertor
{
int J2735Convertor::run()
{
  // Initialize Node
  try
  {
    initialize();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to initialize node with exception: " << e.what());
  }

  // Attach bsm, spat, map to processing to unique threads
  ros::AsyncSpinner bsm_spinner(1, &bsm_queue_);
  bsm_spinner.start();

  ros::AsyncSpinner spat_spinner(1, &spat_queue_);
  spat_spinner.start();

  ros::AsyncSpinner map_spinner(1, &map_queue_);
  map_spinner.start();

  ros::AsyncSpinner geofence_spinner(1, &geofence_queue_);
  geofence_spinner.start();

  // Continuosly process callbacks for default_nh_ using the GlobalCallbackQueue
  ros::Rate r(default_spin_rate_);
  while (ros::ok() && !shutting_down_)
  {
    ros::spinOnce();
    r.sleep();
  }
  // Request ros node shutdown before exit
  ros::shutdown();

  return 0;
}

void J2735Convertor::initialize()
{
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

  // J2735 BSM Subscriber
  j2735_bsm_sub_ = bsm_nh_->subscribe("incoming_j2735_bsm", 100, &J2735Convertor::j2735BsmHandler, this);

  // BSM Publisher
  converted_bsm_pub_ = bsm_nh_->advertise<cav_msgs::BSM>("incoming_bsm", 100);

  // Outgoing J2735 BSM Subscriber
  outbound_bsm_sub_ = bsm_nh_->subscribe("outgoing_bsm", 1, &J2735Convertor::BsmHandler,
                                         this);  // Queue size of 1 as we should never publish outdated BSMs

  // BSM Publisher
  outbound_j2735_bsm_pub_ = bsm_nh_->advertise<j2735_msgs::BSM>(
      "outgoing_j2735_bsm", 1);  // Queue size of 1 as we should never publish outdated BSMs

  // J2735 SPAT Subscriber
  j2735_spat_sub_ = spat_nh_->subscribe("incoming_j2735_spat", 100, &J2735Convertor::j2735SpatHandler, this);

  // SPAT Publisher TODO think about queue sizes
  converted_spat_pub_ = spat_nh_->advertise<cav_msgs::SPAT>("incoming_spat", 100);

  // J2735 MAP Subscriber
  j2735_map_sub_ = map_nh_->subscribe("incoming_j2735_map", 50, &J2735Convertor::j2735MapHandler, this);

  // MAP Publisher TODO think about queue sizes
  converted_map_pub_ = map_nh_->advertise<cav_msgs::MapData>("incoming_map", 50);

  // Incoming geofence pub/sub
  converted_geofence_control_pub_ = geofence_nh_->advertise<cav_msgs::ControlMessage>("incoming_geofence_control", 50);
  converted_geofence_request_pub_ = geofence_nh_->advertise<cav_msgs::TrafficControlRequest>("incoming_geofence_request", 50);

  j2735_geofence_control_sub_ = geofence_nh_->subscribe("incoming_j2735_geofence_control", 50, &J2735Convertor::j2735ControlMessageHandler, this);
  j2735_geofence_request_sub_ = geofence_nh_->subscribe("incoming_j2735_geofence_request", 50, &J2735Convertor::j2735ControlRequestHandler, this);

  // Outgoing geofence pub/sub
  outbound_geofence_control_sub_ = geofence_nh_->subscribe("outgoing_geofence_control", 50, &J2735Convertor::ControlMessageHandler, this);
  outbound_geofence_request_sub_ = geofence_nh_->subscribe("outgoing_geofence_request", 50, &J2735Convertor::ControlRequestHandler, this);

  outbound_j2735_geofence_control_pub_ = geofence_nh_->advertise<j2735_msgs::ControlMessage>("outgoing_j2735_geofence_control", 10);
  outbound_j2735_geofence_request_pub_ = geofence_nh_->advertise<j2735_msgs::TrafficControlRequest>("outgoing_j2735_geofence_request", 10);
}

void J2735Convertor::BsmHandler(const cav_msgs::BSMConstPtr& message)
{
  j2735_msgs::BSM j2735_msg;
  BSMConvertor::convert(*message, j2735_msg);  // Convert message
  outbound_j2735_bsm_pub_.publish(j2735_msg);  // Publish converted message
}

void J2735Convertor::j2735BsmHandler(const j2735_msgs::BSMConstPtr& message)
{
  cav_msgs::BSM converted_msg;
  BSMConvertor::convert(*message, converted_msg);  // Convert message
  converted_bsm_pub_.publish(converted_msg);       // Publish converted message
}

void J2735Convertor::j2735SpatHandler(const j2735_msgs::SPATConstPtr& message)
{
  cav_msgs::SPAT converted_msg;
  SPATConvertor::convert(*message, converted_msg);  // Convert message
  converted_spat_pub_.publish(converted_msg);       // Publish converted message
}

void J2735Convertor::j2735MapHandler(const j2735_msgs::MapDataConstPtr& message)
{
  cav_msgs::MapData converted_msg;
  MapConvertor::convert(*message, converted_msg);  // Convert message
  converted_map_pub_.publish(converted_msg);       // Publish converted message
}

void J2735Convertor::ControlMessageHandler(const cav_msgs::ControlMessageConstPtr& message) {
  j2735_msgs::ControlMessage converted_msg;
  j2735_convertor::geofence_control::convert(*message, converted_msg);  // Convert message
  outbound_j2735_geofence_control_pub_.publish(converted_msg);       // Publish converted message
}

void J2735Convertor::j2735ControlMessageHandler(const j2735_msgs::ControlMessageConstPtr& message) {
  cav_msgs::ControlMessage converted_msg;
  j2735_convertor::geofence_control::convert(*message, converted_msg);  // Convert message
  converted_geofence_control_pub_.publish(converted_msg);       // Publish converted message
}

void J2735Convertor::ControlRequestHandler(const cav_msgs::TrafficControlRequestConstPtr& message) {
  j2735_msgs::TrafficControlRequest converted_msg;
  j2735_convertor::geofence_request::convert(*message, converted_msg);  // Convert message
  outbound_j2735_geofence_request_pub_.publish(converted_msg);       // Publish converted message
}

void J2735Convertor::j2735ControlRequestHandler(const j2735_msgs::TrafficControlRequestConstPtr& message) {
  cav_msgs::TrafficControlRequest converted_msg;
  j2735_convertor::geofence_request::convert(*message, converted_msg);  // Convert message
  converted_geofence_request_pub_.publish(converted_msg);       // Publish converted message
}

}  // namespace j2735_convertor
