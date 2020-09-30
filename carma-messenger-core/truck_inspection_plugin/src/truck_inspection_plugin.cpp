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
        //add header temporarily for ACE testing, will be removed once the mobilityRequest message node is fixed
        cav_msgs::MobilityHeader msg_out_header;
        msg_out_header.sender_id =  "USDOT-10003";
        msg_out_header.recipient_id="USDOT-10003";
        msg_out_header.sender_bsm_id="10ABCDEF";
        msg_out_header.plan_id= "11111111-2222-3333-AAAA-111111111111";
        msg_out_header.timestamp= 9223372036854775807;
        msg.header = msg_out_header;
        msg.expiration=1523372036854775807;
        msg.strategy_params="abs";
        cav_msgs::PlanType plan_type;
        plan_type.type=4;
        msg.plan_type = plan_type;
        cav_msgs::LocationECEF location;
        location.ecef_x=0;
        location.ecef_y=0;
        location.ecef_z=0;
        location.timestamp=1223372036854775807; 
        msg.location=location;
        location.timestamp= 9023372036854775807;
        cav_msgs::Trajectory trajectory;
        trajectory.location=location;
        cav_msgs::LocationOffsetECEF offset;
        offset.offset_x=0;
        offset.offset_y=0;
        offset.offset_z=0;
        trajectory.offsets.push_back(offset);
        msg.trajectory=trajectory;
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
                safety_log_ += ",timestamp:" + std::to_string(msg->header.timestamp);
            } else {
                std_msgs::String msg_out;
                std::string k_v_pair = msg->strategy_params;
                // get only VIN number, state and license plate
                msg_out.data = k_v_pair;
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
