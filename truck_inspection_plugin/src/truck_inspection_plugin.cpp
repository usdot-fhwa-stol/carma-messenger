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
        pnh_->getParam("num_of_entries", number_of_entries);
        mr_pub_ = nh_->advertise<cav_msgs::MobilityRequest>("mobility_request_outbound", 5);
        cav_detection_pub_ = nh_->advertise<std_msgs::String>("cav_truck_identified", 5);
        content_pub_ = nh_->advertise<std_msgs::String>("truck_safety_info", 5);
        mo_sub_ = nh_->subscribe("mobility_operation_inbound", 5, &TruckInspectionPlugin::mobilityOperationCallback, this);
        inspection_request_service_server_ = nh_->advertiseService("send_inspection_request", &TruckInspectionPlugin::inspectionRequestCallback, this);
        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            if(!this->safety_log_.empty()) {
                std_msgs::String msg_content;
                msg_content.data = safety_log_;
                content_pub_.publish(msg_content);
            }
            return true;
        });
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
        // reset safety log
        this->safety_log_ = "";
        resp.success = true;
    }

    void TruckInspectionPlugin::mobilityOperationCallback(const cav_msgs::MobilityOperationConstPtr& msg)
    {
        // if there is a truck around running CARMA
        if(msg->strategy == TruckInspectionPlugin::INSPECTION_STRATEGY)
        {
            // if the incoming message contains a valid safety log
            if(isSafetyLogValid(msg->strategy_params))
            {
                safety_log_ = msg->strategy_params;
            } else {
                std_msgs::String msg_out;
                std::string k_v_pair = msg->strategy_params;
                // get only VIN number
                msg_out.data = k_v_pair.substr(k_v_pair.find(':') + 1);
                // publish message to show there is a cav truck in the radio range of the ego-vehicle
                cav_detection_pub_.publish(msg_out);
            }
        }
    }

    bool TruckInspectionPlugin::isSafetyLogValid(const std::string& log)
    {
        // Check 1: if number of k-v pairs matches expectation
        std::string temp;
        int count = 0;
        std::stringstream ss(log);
        while(std::getline(ss, temp, ','))
        {
            if(!temp.empty())
            {
                count++;
            }
            else
            {
                // do not allow spaces in the safety log
                return false;
            }
        }
        return count == this->number_of_entries;
    }

}
