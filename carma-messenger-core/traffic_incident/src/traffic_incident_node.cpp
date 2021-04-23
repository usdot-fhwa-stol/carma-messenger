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
    pnh_.getParam("event_reason", event_reason_);
    pnh_.getParam("event_type", event_type_);

    traffic_worker_.setSenderId(sender_id_);
    traffic_worker_.setDownTrack(down_track_);
    traffic_worker_.setUpTrack(up_track_);
    traffic_worker_.setMinGap(min_gap_);
    traffic_worker_.setEventReason(event_reason_);
    traffic_worker_.setEventType(event_type_);

    // Setup pub/sub
    pinpoint_driver_sub_=nh_.subscribe("gps_common_fix",10,&TrafficIncidentWorker::pinpointDriverCallback,&traffic_worker_);
    traffic_mobility_operation_pub_=nh_.advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 10);

    //setup services
    start_broadcast_request_service_server = nh_.advertiseService("start_broadcasting_traffic_event", &TrafficIncidentNode::startTrafficBroadcastCallback, this);
    stop_broadcast_request_service_server = nh_.advertiseService("stop_broadcasting_traffic_event", &TrafficIncidentNode::stopTrafficBroadcastCallback, this);

    //spin loop
    ros::CARMANodeHandle::setSpinRate(10.0);
    ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            if(traffic_worker_.getDownTrack() >0  && traffic_worker_.getUpTrack()>0 && traffic_worker_.getMinGap() > 0 && traffic_worker_.getAdvisorySpeed() > 0) 
              {
                //construct local mobilityOperation msg
                cav_msgs::MobilityOperation traffic_mobility_msg = traffic_worker_.mobilityMessageGenerator(traffic_worker_.getPinPoint());      
                
                //start constantly broadcasting mobilityOperation msg
                traffic_mobility_operation_pub_.publish(traffic_mobility_msg);
              }
            return true;
        });
    ROS_INFO_STREAM("Traffic Incident node is initialized...");
    ros::CARMANodeHandle::spin();
  }

  void TrafficIncidentNode::publishTrafficIncidentMobilityOperation(const cav_msgs::MobilityOperation& traffic_msg)
  {
  	traffic_mobility_operation_pub_.publish(traffic_msg);
  }


  /*****
   * Used by UI to start broadcasting traffic event (geofence)
   * Msg: MobilityOperation 
   * **/
  bool TrafficIncidentNode::startTrafficBroadcastCallback(cav_srvs::SetTrafficEventRequest& req, cav_srvs::SetTrafficEventResponse& resp)
  {
      //update instance variables with incoming request params
      traffic_worker_.setMinGap(req.minimum_gap);
      traffic_worker_.setDownTrack(req.down_track);
      traffic_worker_.setUpTrack(req.up_track);
      traffic_worker_.setAdvisorySpeed(req.advisory_speed);
      
      //return service response true 
      resp.success = true;
      return true;
  }

 /*****
   * Used by UI to stop broadcasting traffic event (geofence)
   * Msg: MobilityOperation
   * **/
  bool TrafficIncidentNode::stopTrafficBroadcastCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp){
      try{
          //reset instance variables 
          traffic_worker_.setMinGap(0);
          traffic_worker_.setDownTrack(0);
          traffic_worker_.setUpTrack(0);
          traffic_worker_.setAdvisorySpeed(0);

          resp.success = true;
          resp.message = "stop broadcasting";

          //return service response true 
          return true;

      }catch(...){
          //in case any exception
          resp.success = false;
          return false;
    }
  }


  void TrafficIncidentNode::run()
  {
    initialize();
  }

}//traffic