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

  #include "traffic_incident_node.h"

  namespace traffic{

  using std::placeholders::_1;

  TrafficIncidentNode::TrafficIncidentNode(): pnh_("~"), traffic_worker_(std::bind(&TrafficIncidentNode::publishTrafficIncidentMobilityOperation, this, _1)){}

  void TrafficIncidentNode::initialize()
  {

	  pnh_.getParam("sender_id", sender_id_);
    pnh_.getParam("down_track", down_track_);
    pnh_.getParam("up_track", up_track_);
    pnh_.getParam("min_gap", min_gap_); 

    traffic_worker_.setSenderId(sender_id_);
    traffic_worker_.setDownTrack(down_track_);
    traffic_worker_.setUpTrack(up_track_);
    traffic_worker_.setMinGap(min_gap_);


    // Setup pub/sub
    pinpoint_driver_sub_=nh_.subscribe("gps_common_fix",10,&TrafficIncidentWorker::pinpointDriverCallback,&traffic_worker_);
    traffic_mobility_operation_pub_=nh_.advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 10);
  }

  void TrafficIncidentNode::publishTrafficIncidentMobilityOperation(const cav_msgs::MobilityOperation& traffic_msg)
  {
  	traffic_mobility_operation_pub_.publish(traffic_msg);
  }

  void TrafficIncidentNode::run()
  {
    initialize();
    nh_.setSpinRate(20);
    nh_.spin();
  }

}//traffic