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

#ifndef TRAFFIC_INCIDENT_WORKER_H
#define TRAFFIC_INCIDENT_WORKER_H

#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <optional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_ros2_utils.hpp>

#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <gps_msgs/msg/gps_fix.hpp>

namespace traffic
{
class TrafficIncidentWorker
{
public:
  using PublishTrafficCallback =
    std::function<void(const carma_v2x_msgs::msg::MobilityOperation &)>;

  /*!
   * \brief Constructor
   */
  TrafficIncidentWorker(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh,
    PublishTrafficCallback traffic_pub);

  /*! \fn pinpointDriverCallback(const gps_msgs::msg::GPSFix &pinpoint_msg)
    \brief pinpointDriverCallback populates lat lon heading from pinpoint driver.
    \param  gps_msgs::msg::GPSFix.
  */
  void pinpointDriverCallback(gps_msgs::msg::GPSFix::UniquePtr pinpoint_msg);

  /*! \fn geofenceStartLocCallback()
    \brief Callback for geofence start location
  */
  void geofenceStartLocCallback(gps_msgs::msg::GPSFix::UniquePtr start_loc_msg);

  /*! \fn geofenceEndLocCallback()
    \brief Callback for geofence end location
  */
  void geofenceEndLocCallback(gps_msgs::msg::GPSFix::UniquePtr end_loc_msg);

  /*! \fn anytypeToString(T value)
    \brief anytypeToString converts anytype to string value
    \param  value which gets converted.
  */
  template <class T> std::string anytypeToString(T value)
  {
    std::stringstream ss;
    ss << value;
    return ss.str();
  }

  std::string doubleToString(double value);

  // Setters for the prediction parameters
  void setSenderId(std::string sender_id);
  void setEventReason(std::string event_reason);
  void setEventType(std::string event_type);
  void setDownTrack(double down_track);
  void setUpTrack(double up_track);
  void setMinGap(double min_gap);
  void setPinPoint(gps_msgs::msg::GPSFix pinpoint_msg);
  void setAdvisorySpeed(double advisory_speed);
  void setGeofenceStartEndDataTimeout(double geofence_start_end_data_timeout);
  void setShouldBroadcast(bool should_broadcast){should_broadcast_ = should_broadcast;}

  // Getter for the prediction parameters
  std::string getSenderId();
  std::string getEventReason();
  std::string getEventType();
  double getDownTrack();
  double getUpTrack();
  double getMinGap();
  gps_msgs::msg::GPSFix getPinPoint();
  std::optional<gps_msgs::msg::GPSFix> getGeofenceStartLoc();
  std::optional<gps_msgs::msg::GPSFix> getGeofenceEndLoc();
  double getAdvisorySpeed();

  // Generate mobility message using pinpoint driver data
  carma_v2x_msgs::msg::MobilityOperation mobilityMessageGenerator(
    const gps_msgs::msg::GPSFix & msg);

  // Generate mobility message using geofence start and end data
  carma_v2x_msgs::msg::MobilityOperation mobilityMessageGenerator(
    const gps_msgs::msg::GPSFix & start_zone, const gps_msgs::msg::GPSFix & end_zone);

  // public constant variables
  const std::string USE_CASE_NAME_ = "carma3/Incident_Use_Case";

private:
  // local copy of external object publihsers
  PublishTrafficCallback traffic_pub_;
  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;

  // Prediction parameters
  std::string sender_id_ = "USDOT-49096";
  std::string closed_lane_ = "[1]";
  std::string event_reason_ = "";
  std::string event_type_ = "OPEN";
  double down_track_ = 0;
  double up_track_ = 0;
  double min_gap_ = 0;
  double advisory_speed_ = 0;
  bool should_broadcast_ = false;
  double geofence_start_end_data_timeout_ = 1.0; // seconds
  gps_msgs::msg::GPSFix pinpoint_msg_;
  std::optional<gps_msgs::msg::GPSFix> geo_start_loc_msg_;
  std::optional<gps_msgs::msg::GPSFix> geo_end_loc_msg_;
};

}  // namespace traffic

#endif /* TRAFFIC_INCIDENT_WORKER_H */
