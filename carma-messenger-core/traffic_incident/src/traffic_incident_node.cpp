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

    //setup services
    start_broadcast_request_service_server = nh_.advertiseService("start_broadcasting_traffic_event", &TrafficIncidentNode::startTrafficBroadcastCallback, this);
    stop_broadcast_request_service_server = nh_.advertiseService("stop_broadcasting_traffic_event", &TrafficIncidentNode::stopTrafficBroadcastCallback, this);
 
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
    try{
          //update instance variables with incoming request params
          traffic_worker_.setMinGap(req.minimum_gap);
          traffic_worker_.setDownTrack(req.down_track);
          traffic_worker_.setUpTrack(req.up_track);
          traffic_worker_.setAdvisorySpeed(req.advisory_speed);
        
          while (ros::ok())
          {
              //construct local mobilityOperation msg
              cav_msgs::MobilityOperation traffic_mobility_msg;
              traffic_mobility_msg.header.timestamp = traffic_worker_.getPinPoint().header.stamp.sec*1000;
              traffic_mobility_msg.header.sender_id = traffic_worker_.getSenderId();
              traffic_mobility_msg.strategy = traffic_worker_.USE_CASE_NAME_;

              if(traffic_worker_.getDownTrack()>0  && traffic_worker_.getUpTrack()>0 && traffic_worker_.getMinGap() > 0) 
              {
                  
                  traffic_mobility_msg.strategy_params = "lat:" + traffic_worker_.anytypeToString(traffic_worker_.getPinPoint().latitude) + ","
                                                        +"lon:" + traffic_worker_.anytypeToString(traffic_worker_.getPinPoint().longitude) + ","
                                                        +"downtrack:" + traffic_worker_.anytypeToString(traffic_worker_.getDownTrack())+ ","
                                                        +"uptrack:" + traffic_worker_.anytypeToString(traffic_worker_.getUpTrack()) + ","
                                                        +"min_gap:"+ traffic_worker_.anytypeToString(traffic_worker_.getMinGap()) +","
                                                        +"advisory_speed:"+ traffic_worker_.anytypeToString(traffic_worker_.getAdvisorySpeed());
                //start constantly broadcasting mobilityOperation msg
                traffic_mobility_operation_pub_.publish(traffic_mobility_msg);

                //return service response true 
                resp.success=true;
              }

              ros::spinOnce();
              ros::Duration(1).sleep();
          }
          return true;

        }catch(...){
              //in case any exception
              resp.success = false;
              return false;
        }    
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
          traffic_worker_.setUpTrack(0);

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
    nh_.setSpinRate(20);
    nh_.spin();
  }

}//traffic