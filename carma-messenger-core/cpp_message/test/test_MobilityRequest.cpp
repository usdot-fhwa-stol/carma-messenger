/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include "cpp_message/MobilityRequest_Message.h"
#include<gtest/gtest.h>

TEST(MobilityRequestMessageTest, testDecodeMobilityRequestMsg)
{
    std::vector<uint8_t> binary_input={0,240,129,225,77,90,113,39,212,90,209,171,22,12,38,173,56,147,234,45,104,213,131,150,172,88,65,133,14,36,88,204,88,177,98,197,139,22,43,89,50,100,201,107,54,108,217,173,131,6,12,21,172,88,177,98,197,139,22,44,88,177,98,229,147,38,108,219,178,96,205,179,134,173,27,183,106,225,131,126,116,60,60,182,225,95,67,102,30,155,247,238,211,187,57,131,36,193,149,6,137,131,42,13,83,6,84,27,49,100,201,155,54,236,152,51,108,225,171,70,237,218,184,96,221,91,237,167,117,253,221,118,226,203,201,211,24,213,101,71,145,18,179,8,115,33,80,112,225,155,69,155,52,227,203,187,158,91,252,54,97,233,149,212,73,245,22,177,96,193,131,53,152,240,242,229,167,47,43,251,176,237,202,234,158,157,157,178,242,65,83,151,92,122,208,70,145,94,10,10,145,164,82,134,179,30,30,92,180,229,229,127,78,71,85,105,196,159,81,3,6,12,24,48,96,197,103,124,186,115,232,232,233,102,28,156,239,243,223,155,167,124,60,178,223,237,151,151,61,59,247,58,167,231,159,76,187,80,86,203,203,158,157,251,144,85,221,175,118,254,251,150,100,195,211,45,253,249,175,236,195,207,165,254,125,48,244,203,127,78,238,124,50,227,233,167,126,231,86,108,217,178,182,108,213,177,34,44,201,135,166,91,251,243,95,217,135,159,75,248,114,115,191,143,14,205,56,185,97,233,167,126,231,86,108,217,178,182,108,213,177,34,44,225,203,45,254,156,180,240,191,135,39,59,250,50,225,217,211,69,252,122,50,227,214,234,63,44,185,119,44,195,147,157,254,125,48,244,235,205,213,44,185,22,105,231,206,255,60,123,249,101,116,209,202,206,25,121,109,211,210,255,44,188,122,233,229,151,35,166,11,58,105,219,151,159,76,59,120,58,98,213,195,87,12,219,55,102,197,195,22,137,131,42,14,83,6,84,26,166,12,168,54,114,193,147,54,109,217,48,102,217,195,86,141,219,181,112,193,184,79,171,234,250,190,207,179,236,197,171,38,108,219,178,96,205,179,134,173,27,183,106,225,131,112};

    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Request worker(node->get_node_logging_interface());

    boost::optional<carma_v2x_msgs::msg::MobilityRequest> res = worker.decode_mobility_request_message(binary_input);
    if(res)
    {
        carma_v2x_msgs::msg::MobilityRequest to_read=res.get();
        // std::cout<<"H_sender_id:"<<to_read.header.sender_id<<std::endl;
        // std::cout<<"H_target id:"<<to_read.header.recipient_id<<std::endl;
        // std::cout<<"H_bsm id:"<<to_read.header.sender_bsm_id<<std::endl;
        // std::cout<<"H_plan id:"<<to_read.header.plan_id<<std::endl;
        // std::cout<<"H_timestamp:"<<to_read.header.timestamp<<std::endl;
        // std::cout<<"strategy:"<<to_read.strategy<<std::endl;
        // std::cout<<"plan type:"<<to_read.plan_type;;
        // std::cout<<"urgency:"<<to_read.urgency<<std::endl;        
        // std::cout<<"location_x:"<<to_read.location.ecef_x<<std::endl;
        // std::cout<<"location_y:"<<to_read.location.ecef_y<<std::endl;
        // std::cout<<"location_z:"<<to_read.location.ecef_z<<std::endl;
        // std::cout<<"location_timestamp:"<<to_read.location.timestamp<<std::endl;
        // std::cout<<"strategy_params:"<<to_read.strategy_params<<std::endl;
        // std::cout<<"trajectory_x:"<<to_read.trajectory.location.ecef_x<<std::endl;
        // std::cout<<"trajectory_y:"<<to_read.trajectory.location.ecef_y<<std::endl;
        // std::cout<<"trajectory_z:"<<to_read.trajectory.location.ecef_z<<std::endl;
        // std::cout<<"trajectory_timestamp:"<<to_read.trajectory.location.timestamp<<std::endl;
        // std::cout<<"trajectory_offsets0_x:"<<to_read.trajectory.offsets[0].offset_x<<std::endl;
        // std::cout<<"trajectory_offsets0_y:"<<to_read.trajectory.offsets[0].offset_y<<std::endl;
        // std::cout<<"trajectory_offsets0_z:"<<to_read.trajectory.offsets[0].offset_z<<std::endl;
        // std::cout<<"trajectory_offsets1_x:"<<to_read.trajectory.offsets[1].offset_x<<std::endl;
        // std::cout<<"trajectory_offsets1_y:"<<to_read.trajectory.offsets[1].offset_y<<std::endl;
        // std::cout<<"trajectory_offsets1_z:"<<to_read.trajectory.offsets[1].offset_z<<std::endl;
        // std::cout<<"expiration:"<<to_read.expiration<<std::endl;
        if(to_read.m_header.sender_bsm_id=="10ABCDEF" && to_read.trajectory.location.ecef_x==5) EXPECT_TRUE(true);
        else EXPECT_TRUE(false);
    }
    else EXPECT_TRUE(false);
}

TEST(MobilityRequestMessageTest, testEncodeMobilityRequestMsg)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Request worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::MobilityRequest message;
    header.sender_id="USDOT-45100";
    header.recipient_id="USDOT-45095";
    header.sender_bsm_id="10ABCDEF";
    header.plan_id="11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.m_header=header;   

    //body
    message.strategy="Carma/Platooning";
    message.plan_type.type=6;
    message.urgency=50;
    
        //location
    carma_v2x_msgs::msg::LocationECEF starting_location;
    starting_location.ecef_x=0;
    starting_location.ecef_y=1;
    starting_location.ecef_z=2;
    starting_location.timestamp=1223372036854775807;

    message.location=starting_location;
        //strategy_params
    message.strategy_params="vin_number:1FUJGHDV0CLBP8834,license_plate:DOT-10003,carrier_name:Silver Truck FHWA TFHRC,carrier_id:USDOT 0000001,weight:,ads_software_version:System Version Unknown,date_of_last_state_inspection:YYYY-MM-DD,date_of_last_ads_calibration:YYYY-MM-DD,pre_trip_ads_health_check:Green,ads_status:Red,iss_score:49,permit_required:0,timestamp:1585836731814";
        //trajectory
    carma_v2x_msgs::msg::Trajectory trajectory;
    carma_v2x_msgs::msg::LocationECEF trajectory_start;
    trajectory_start.ecef_x=5;
    trajectory_start.ecef_y=1;
    trajectory_start.ecef_z=2;
    trajectory_start.timestamp=9023372036854775807;
    trajectory.location=trajectory_start;

            //offsets
    carma_v2x_msgs::msg::LocationOffsetECEF offset1;
    offset1.offset_x=1;
    offset1.offset_y=1;
    offset1.offset_z=1;

    trajectory.offsets.push_back(offset1);
 
    carma_v2x_msgs::msg::LocationOffsetECEF offset2;
    offset2.offset_x=2;
    offset2.offset_y=2;
    offset2.offset_z=2;

    trajectory.offsets.push_back(offset2);

    message.trajectory=trajectory;
    //expiration
    message.expiration=1523372036854775807;
    boost::optional<std::vector<uint8_t>> res = worker.encode_mobility_request_message(message);
    
    if(res)
    {
        std::vector<uint8_t> to_read=res.get();
        for(size_t i=0;i<to_read.size();i++)std::cout<<int(to_read[i])<<",";
        std::cout<<"\n";
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(MobilityRequestMessageTest, testEncodeMobilityRequestMsg_base_case)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Request worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::MobilityRequest message;
    header.sender_id="";
    header.recipient_id="";
    header.sender_bsm_id="";
    header.plan_id="";
    header.timestamp = 0;
    message.m_header=header;   

    //body
    message.strategy="";
    message.plan_type.type=0;
    message.urgency=0;
    
        //location
    carma_v2x_msgs::msg::LocationECEF starting_location;
    starting_location.ecef_x=0;
    starting_location.ecef_y=0;
    starting_location.ecef_z=0;
    starting_location.timestamp=0;

    message.location=starting_location;
        //strategy_params
    message.strategy_params="";
        //trajectory
    carma_v2x_msgs::msg::Trajectory trajectory;
    carma_v2x_msgs::msg::LocationECEF trajectory_start;
    trajectory_start.ecef_x=0;
    trajectory_start.ecef_y=0;
    trajectory_start.ecef_z=0;
    trajectory_start.timestamp=0;
    trajectory.location=trajectory_start;

            //offsets
    carma_v2x_msgs::msg::LocationOffsetECEF offset1;
    offset1.offset_x=0;
    offset1.offset_y=0;
    offset1.offset_z=0;

    trajectory.offsets.push_back(offset1);
 

    message.trajectory=trajectory;
    //expiration
    message.expiration=0;
    boost::optional<std::vector<uint8_t>> res = worker.encode_mobility_request_message(message);
    
    if(res)
    {
        // std::vector<uint8_t> to_read=res.get();
        // for(size_t i=0;i<to_read.size();i++)std::cout<<int(to_read[i])<<",";
        // std::cout<<"\n";
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}



