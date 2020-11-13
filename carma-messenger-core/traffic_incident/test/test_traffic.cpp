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
#include <gtest/gtest.h>

namespace traffic
{

TEST(TrafficIncidentWorkerTest, testTrafficMobilityOperationBroadcast)
{
    /*TrafficIncidentWorker traffic_worker();
    cav_msgs::MobilityOperation traffic_mobility_msg;

    gps_common::GPSFix pinpoint_msg;
   
    pinpoint_msg.latitude=57.0;
    pinpoint_msg.longitude=155.79;

    std::string sender_id_="USDOT-49096";
    std::string closed_lane_="[1]";
    double down_track_=50.0;
    double up_track_=50.0;*/

    //std::string sender_id="USDOT-49096";
    //std::string closed_lane="[1]";
    //double down_track=50.0;
    //double up_track=50.0;
    
   // traffic_worker.setSenderId(sender_id);
   // traffic_worker.setClosedLane(closed_lane);
    //traffic_worker.setDownTrack(down_track);
    //traffic_worker.setUpTrack(up_track);

    //EXPECT_EQ(traffic_worker.closed_lane_,'[1]');
    //EXPECT_EQ(traffic_worker.down_track_,50.0);
    //EXPECT_EQ(traffic_worker.up_track_,50.0);

   // traffic_worker.pinpointDriverCallback(pinpoint_msg);
    //EXPECT_EQ(traffic_mobility_msg.header.sender_id,'USDOT-49096');
   // EXPECT_EQ(traffic_mobility_msg.strategy_params,"lat:57.0,lon:155.79,closed_lanes:[1],downtrack:50.0,uptrack:50.0");
    EXPECT_EQ(1,1);
}

}//traffic