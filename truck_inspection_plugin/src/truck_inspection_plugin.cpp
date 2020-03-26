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


#include "truck_inspection_plugin.h"

namespace truck_inspection_plugin
{

    void TruckInspectionPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        mr_pub_ = nh_->advertise<cav_msgs::MobilityRequest>("mobility_request_outbound", 5);
        inspection_request_service_server_ = nh_->advertiseService("send_inspection_request", &TruckInspectionPlugin::inspectionRequestCallback, this);
        ROS_INFO_STREAM("Truck inspection plugin is initialized...");
    }

    void TruckInspectionPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    bool TruckInspectionPlugin::inspectionRequestCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp)
    {
        cav_msgs::MobilityRequest msg;
        msg.strategy = TruckInspectionPlugin::INSPECTION_STRATEGY;
        mr_pub_.publish(msg);
    }

}
