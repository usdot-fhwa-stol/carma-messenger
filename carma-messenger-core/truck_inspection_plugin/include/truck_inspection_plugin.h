#pragma once

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

#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityOperation.h>

namespace truck_inspection_plugin
{

    class TruckInspectionPlugin
    {

    public:

        // general starting point of this node
        void run();

        const std::string INSPECTION_STRATEGY = "TruckInspection";

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // publisher for generated Mobility Request messages
        ros::Publisher  mr_pub_;

        // publisher for passing relevent data to UI
        ros::Publisher content_pub_;
        ros::Publisher cav_detection_pub_;

        // subscriber for incoming mobility operation messages
        ros::Subscriber mo_sub_;

        // service server for sending inspection request
        ros::ServiceServer inspection_request_service_server_;

        // initialize this node
        void initialize();

        // callbacks for the subscriber
        void mobilityOperationCallback(const cav_msgs::MobilityOperationConstPtr& msg);

        // callbacks for the service
        bool inspectionRequestCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

        // helper function to verify safety log
        bool isSafetyLogValid(const std::string& log);

        // helper function to check ads_auto_status
        bool isADSAutoEngaged(const std::string& log);

        int number_of_entries;

        std::string safety_log_;

    };

}
