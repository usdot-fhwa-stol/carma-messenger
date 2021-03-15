/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#ifndef TRAFFIC_INCIDENT_WORKER_H
#define TRAFFIC_INCIDENT_WORKER_H

#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <gps_common/GPSFix.h>
#include <functional>
#include <iostream>
#include <string>
#include <sstream>

namespace traffic{

class TrafficIncidentWorker
{

 public:

  using PublishTrafficCallback = std::function<void(const cav_msgs::MobilityOperation&)>;

  /*!
   * \brief Constructor
   */
  TrafficIncidentWorker(PublishTrafficCallback traffic_pub);
    
  /*! \fn pinpointDriverCallback(const gps_common::GPSFix &pinpoint_msg)
    \brief pinpointDriverCallback populates lat lon heading from pinpoint driver.
    \param  gps_common::GPSFix.
  */

  void pinpointDriverCallback(const gps_common::GPSFix &pinpoint_msg);

  /*! \fn anytypeToString(T value)
    \brief anytypeToString converts anytype to string value
    \param  value which gets converted.
  */
  template<class T>
  std::string anytypeToString(T value)
  {
    std::stringstream ss;
    ss<<value;
    return ss.str();
  }

  // Setters for the prediction parameters
  void setSenderId(std::string sender_id);
  void setClosedLane(std::string closed_lane);
  void setDownTrack(double down_track);
  void setUpTrack(double up_track);

  // Generate mobility message
  cav_msgs::MobilityOperation mobilityMessageGenerator(const gps_common::GPSFix& msg);


  

 private:

  // local copy of external object publihsers

  PublishTrafficCallback traffic_pub_;
 
 // Prediction parameters
  std::string sender_id_ = "USDOT-49096";
  std::string closed_lane_= "[1]";
  double down_track_= 50.0;
  double up_track_= 50.0;

};

}//traffic

#endif /* EXTERNAL_OBJECT_WORKER_H */