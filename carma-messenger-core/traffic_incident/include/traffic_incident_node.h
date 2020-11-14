/*
 * Copyright (C) 2020 LEIDOS.
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

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <functional>
#include <string>
#include "traffic_incident_worker.h"

namespace traffic{

class TrafficIncidentNode
{

 private:
  
  //node handle
  ros::CARMANodeHandle nh_;
  ros::CARMANodeHandle pnh_;
   
  //subscriber
  ros::Subscriber pinpoint_driver_sub_;

  //publisher
  ros::Publisher traffic_mobility_operation_pub_;
  
  //TrafficIncidentWorker class object
  TrafficIncidentWorker traffic_worker_;
  
    /*!fn initialize()
  \brief initialize this node before running
  */
   void initialize();

  //ROS Params
  std::string sender_id_; 
  std::string closed_lane_;
  double down_track_;
  double up_track_; 

 public:
  
  /*! \fn TrafficIncidentNode()
    \brief TrafficIncidentNode constructor 
  */
  TrafficIncidentNode();

  /*! \fn publishTrafficIncidentMobilityOperation()
    \brief Publish mobility operation message
  */
  void publishTrafficIncidentMobilityOperation(const cav_msgs::MobilityOperation& traffic_msg);

  /*!fn run()
    \brief General starting point to run this node
  */
  void run();
  
};

}//traffic

#endif /* TRAFFIC_INCIDENT_H */