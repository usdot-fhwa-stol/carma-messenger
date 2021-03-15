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

#include "traffic_incident_worker.h"

namespace traffic
{

  TrafficIncidentWorker::TrafficIncidentWorker(PublishTrafficCallback traffic_pub) : traffic_pub_(traffic_pub){};

  void TrafficIncidentWorker::pinpointDriverCallback(const gps_common::GPSFix& pinpoint_msg)
  {

  cav_msgs::MobilityOperation traffic_mobility_msg=mobilityMessageGenerator(pinpoint_msg);
  traffic_pub_(traffic_mobility_msg);
  }

  cav_msgs::MobilityOperation TrafficIncidentWorker::mobilityMessageGenerator(const gps_common::GPSFix& pinpoint_msg)
  {
    cav_msgs::MobilityOperation traffic_mobility_msg;

    traffic_mobility_msg.header.timestamp=pinpoint_msg.header.stamp.sec*1000;
    traffic_mobility_msg.header.sender_id=sender_id_;

    traffic_mobility_msg.strategy="carma3/Incident_Use_Case";

    traffic_mobility_msg.strategy_params="lat:"+anytypeToString(pinpoint_msg.latitude)+","+"lon:"+anytypeToString(pinpoint_msg.longitude)+","+"closed_lanes:"+ closed_lane_ +","+"downtrack:"+anytypeToString(down_track_)+","+"uptrack:"+anytypeToString(up_track_);
  
    return traffic_mobility_msg;
  }
  
  void TrafficIncidentWorker::setSenderId(std::string sender_id)
  {
    sender_id_= sender_id;
  }

  void TrafficIncidentWorker::setClosedLane(std::string closed_lane)
  {
    closed_lane_= closed_lane;
  }

  void TrafficIncidentWorker::setDownTrack(double down_track)
  {
    down_track_= down_track;
  }

  void TrafficIncidentWorker::setUpTrack(double up_track)
  {
    up_track_= up_track;
  }

}//traffic