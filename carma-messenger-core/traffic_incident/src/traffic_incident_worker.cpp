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

#include "traffic_incident_worker.hpp"

namespace traffic
{
TrafficIncidentWorker::TrafficIncidentWorker(
  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh,
  PublishTrafficCallback traffic_pub)
: nh_(nh), traffic_pub_(traffic_pub){};

void TrafficIncidentWorker::pinpointDriverCallback(gps_msgs::msg::GPSFix::UniquePtr pinpoint_msg)
{
  setPinPoint(*pinpoint_msg);
}

void TrafficIncidentWorker::setGeofenceStartEndDataTimeout(double geofence_start_end_data_timeout) {
  geofence_start_end_data_timeout_ = geofence_start_end_data_timeout;
}

void TrafficIncidentWorker::geofenceStartLocCallback(gps_msgs::msg::GPSFix::UniquePtr start_loc_msg)
{
  geo_start_loc_msg_ = *start_loc_msg;
}

void TrafficIncidentWorker::geofenceEndLocCallback(gps_msgs::msg::GPSFix::UniquePtr end_loc_msg)
{
  geo_end_loc_msg_ = *end_loc_msg;
}

carma_v2x_msgs::msg::MobilityOperation TrafficIncidentWorker::mobilityMessageGenerator(
  const gps_msgs::msg::GPSFix & pinpoint_msg)
{
  carma_v2x_msgs::msg::MobilityOperation traffic_mobility_msg;

  traffic_mobility_msg.m_header.timestamp = rclcpp::Time(pinpoint_msg.header.stamp).nanoseconds() / 1e6;
  traffic_mobility_msg.m_header.sender_id = sender_id_;

  traffic_mobility_msg.strategy = USE_CASE_NAME_;

  traffic_mobility_msg.strategy_params =
    "lat:" + doubleToString(pinpoint_msg.latitude) + "," +
    "lon:" + doubleToString(pinpoint_msg.longitude) + "," +
    "downtrack:" + anytypeToString(down_track_) + "," + "uptrack:" + anytypeToString(up_track_) +
    "," + "min_gap:" + anytypeToString(min_gap_) + "," +
    "advisory_speed:" + anytypeToString(advisory_speed_) + "," + "event_reason:" + event_reason_ +
    "," + "event_type:" + event_type_;

  return traffic_mobility_msg;
}

carma_v2x_msgs::msg::MobilityOperation TrafficIncidentWorker::mobilityMessageGenerator(
  const gps_msgs::msg::GPSFix & start_zone, const gps_msgs::msg::GPSFix & end_zone)
{
  carma_v2x_msgs::msg::MobilityOperation traffic_mobility_msg;
  // Overload the mobilityMessageGenerator function to use the start and end geofence locations
  // to get the same message. Get the pinpoint location by getting the midpoint of start and end
  // and approximate the downtrack and uptrack values using the midpoint.
  gps_msgs::msg::GPSFix midpoint;
  midpoint.latitude = (start_zone.latitude + end_zone.latitude) / 2.0;
  midpoint.longitude = (start_zone.longitude + end_zone.longitude) / 2.0;
  midpoint.altitude = (start_zone.altitude + end_zone.altitude) / 2.0;
  midpoint.header.stamp = nh_->now();

  // Calculate approximate distances from midpoint to start and end zones
  // Using simple lat/lon to meters conversion (approximate for small distances)
  const double DEGREES_TO_METERS_LAT = 111000.0; // ~111 km per degree latitude
  const double DEGREES_TO_METERS_LON = 111000.0 * cos(midpoint.latitude * M_PI / 180.0); // Adjusted for longitude

  // Calculate distance to start zone (uptrack - behind the midpoint)
  double lat_diff_start = start_zone.latitude - midpoint.latitude;
  double lon_diff_start = start_zone.longitude - midpoint.longitude;
  double dist = sqrt(pow(lat_diff_start * DEGREES_TO_METERS_LAT, 2) +
                            pow(lon_diff_start * DEGREES_TO_METERS_LON, 2));

  // Set the header information
  traffic_mobility_msg.m_header.timestamp = rclcpp::Time(midpoint.header.stamp).nanoseconds() / 1e6;
  traffic_mobility_msg.m_header.sender_id = sender_id_;

  // Set the strategy
  traffic_mobility_msg.strategy = USE_CASE_NAME_;

  // Build the strategy parameters string using the midpoint coordinates and calculated distances
  traffic_mobility_msg.strategy_params =
    "lat:" + doubleToString(midpoint.latitude) + "," +
    "lon:" + doubleToString(midpoint.longitude) + "," +
    "downtrack:" + anytypeToString(dist) + "," + "uptrack:" + anytypeToString(dist) +
    "," + "min_gap:" + anytypeToString(min_gap_) + "," +
    "advisory_speed:" + anytypeToString(advisory_speed_) + "," + "event_reason:" + event_reason_ +
    "," + "event_type:" + event_type_;

  return traffic_mobility_msg;
}


std::string TrafficIncidentWorker::doubleToString(double value)
{
  std::ostringstream ss;
  ss << std::setprecision(10);
  ss << value;
  return ss.str();
}

void TrafficIncidentWorker::setSenderId(std::string sender_id) { this->sender_id_ = sender_id; }

void TrafficIncidentWorker::setEventReason(std::string event_reason)
{
  this->event_reason_ = event_reason;
}

void TrafficIncidentWorker::setEventType(std::string event_type) { this->event_type_ = event_type; }

void TrafficIncidentWorker::setMinGap(double min_gap) { this->min_gap_ = min_gap; }

void TrafficIncidentWorker::setDownTrack(double down_track) { this->down_track_ = down_track; }

void TrafficIncidentWorker::setUpTrack(double up_track) { this->up_track_ = up_track; }


void TrafficIncidentWorker::setPinPoint(gps_msgs::msg::GPSFix pinpoint_msg)
{
  this->pinpoint_msg_ = pinpoint_msg;
}

void TrafficIncidentWorker::setAdvisorySpeed(double advisory_speed)
{
  this->advisory_speed_ = advisory_speed;
}

std::string TrafficIncidentWorker::getSenderId() { return this->sender_id_; }

std::string TrafficIncidentWorker::getEventReason() { return this->event_reason_; }

std::string TrafficIncidentWorker::getEventType() { return this->event_type_; }

double TrafficIncidentWorker::getDownTrack() { return this->down_track_; }

double TrafficIncidentWorker::getUpTrack() { return this->up_track_; }

double TrafficIncidentWorker::getMinGap() { return this->min_gap_; }

gps_msgs::msg::GPSFix TrafficIncidentWorker::getPinPoint() { return this->pinpoint_msg_; }

std::optional<gps_msgs::msg::GPSFix> TrafficIncidentWorker::getGeofenceStartLoc() {
  if (geo_start_loc_msg_.has_value()) {
    double age = nh_->now().seconds() - rclcpp::Time(geo_start_loc_msg_.value().header.stamp).seconds();

    if (age >= 0 && age < geofence_start_end_data_timeout_) {
      return geo_start_loc_msg_.value();
    }
  }
  geo_start_loc_msg_.reset(); // Reset the value if it has expired
  return std::nullopt;
}

std::optional<gps_msgs::msg::GPSFix> TrafficIncidentWorker::getGeofenceEndLoc() {
  // Check if the geofence end location is set and not expired
  if (geo_end_loc_msg_.has_value()) {
    double age = nh_->now().seconds() - rclcpp::Time(geo_end_loc_msg_.value().header.stamp).seconds();

    if (age >= 0 && age < geofence_start_end_data_timeout_) {
      return geo_end_loc_msg_.value();
    }
  }

  geo_end_loc_msg_.reset(); // Reset the value if it has expired
  return std::nullopt;
}

double TrafficIncidentWorker::getAdvisorySpeed() { return this->advisory_speed_; }

}  // namespace traffic
