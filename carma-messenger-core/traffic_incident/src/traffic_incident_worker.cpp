/*
 * Copyright (C) 2020-2021 LEIDOS.
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
  setPinPoint(pinpoint_msg);
  }

  cav_msgs::MobilityOperation TrafficIncidentWorker::mobilityMessageGenerator(const gps_common::GPSFix& pinpoint_msg)
  {
    cav_msgs::MobilityOperation traffic_mobility_msg;

    traffic_mobility_msg.header.timestamp=pinpoint_msg.header.stamp.sec*1000;
    traffic_mobility_msg.header.sender_id=sender_id_;

    traffic_mobility_msg.strategy="carma3/Incident_Use_Case";

    traffic_mobility_msg.strategy_params="lat:"+anytypeToString(pinpoint_msg.latitude)+","+"lon:"+anytypeToString(pinpoint_msg.longitude) +","+"downtrack:"+anytypeToString(down_track_)+","+"uptrack:"+anytypeToString(up_track_) +"," +"min_gap:"+ anytypeToString(min_gap_);
  
    return traffic_mobility_msg;
  }
  
  void TrafficIncidentWorker::setSenderId(std::string sender_id)
  {
    this->sender_id_= sender_id;
  }

  void TrafficIncidentWorker::setMinGap(double min_gap)
  {
   this->min_gap_= min_gap;
  }

  void TrafficIncidentWorker::setDownTrack(double down_track)
  {
    this->down_track_= down_track;
  }

  void TrafficIncidentWorker::setUpTrack(double up_track)
  {
    this->up_track_= up_track;
  }


  void TrafficIncidentWorker::setPinPoint(gps_common::GPSFix pinpoint_msg )
  {
    this->pinpoint_msg_ = pinpoint_msg;
  }


  void TrafficIncidentWorker::setAdvisorySpeed(double advisory_speed ){
      this->advisory_speed_ = advisory_speed;
  }

  std::string  TrafficIncidentWorker::getSenderId(){
    return this->sender_id_;
  }
  
  double TrafficIncidentWorker::getDownTrack(){
      return this->down_track_;
  }

  double TrafficIncidentWorker::getUpTrack(){
    return this->up_track_;
  }
  double TrafficIncidentWorker::getMinGap(){
      return  this->min_gap_;
  }

  gps_common::GPSFix  TrafficIncidentWorker::getPinPoint(){
    return this->pinpoint_msg_;
  }

  double TrafficIncidentWorker::getAdvisorySpeed(){
    return this->advisory_speed_;
  }

}//traffic