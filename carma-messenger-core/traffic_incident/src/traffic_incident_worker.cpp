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
TrafficIncidentWorker::TrafficIncidentWorker(PublishTrafficCallback traffic_pub)
: traffic_pub_(traffic_pub){};

void TrafficIncidentWorker::pinpointDriverCallback(gps_msgs::msg::GPSFix::UniquePtr pinpoint_msg)
{
  carma_v2x_msgs::msg::MobilityOperation traffic_mobility_msg =
    mobilityMessageGenerator(*pinpoint_msg);
  // comment out this for now since we are not broadcasting upon receiving GPS signal
  // traffic_pub_(traffic_mobility_msg);
  setPinPoint(*pinpoint_msg);
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

double TrafficIncidentWorker::getAdvisorySpeed() { return this->advisory_speed_; }

}  // namespace traffic