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
    std::vector<uint8_t> binary_input={0,240,129,225,77,90,113,39,212,90,209,171,22,12,38,173,56,147,234,45,104,213,131,150,172,88,65,133,14,36,88,204,88,177,98,197,139,22,43,89,50,100,201,107,54,108,217,173,131,6,12,21,172,88,177,98,197,139,22,44,88,177,98,229,147,38,108,219,178,96,205,179,134,173,27,183,106,225,131,126,116,60,60,182,225,95,67,102,30,155,247,238,211,187,58,6,73,131,42,13,19,6,84,26,166,12,168,54,98,201,147,54,109,217,48,102,217,195,86,141,219,181,112,193,186,183,219,78,235,251,186,237,197,151,147,166,49,170,202,143,34,37,102,16,230,66,160,225,195,54,139,54,105,199,151,119,60,183,248,108,195,211,43,168,147,234,45,98,193,131,6,107,49,225,229,203,78,94,87,247,97,219,149,213,61,59,59,101,228,130,167,46,184,245,160,141,34,188,20,21,35,72,165,13,102,60,60,185,105,203,202,254,156,142,170,211,137,62,162,6,12,24,48,96,193,138,206,249,116,231,209,209,210,204,57,57,223,231,191,55,78,248,121,101,191,219,47,46,122,119,238,117,79,207,62,153,118,160,173,151,151,61,59,247,32,171,187,94,237,253,247,44,201,135,166,91,251,243,95,217,135,159,75,252,250,97,233,150,254,157,220,248,101,199,211,78,253,206,172,217,179,101,108,217,171,98,68,89,147,15,76,183,247,230,191,179,15,62,151,240,228,231,127,30,29,154,113,114,195,211,78,253,206,172,217,179,101,108,217,171,98,68,89,195,150,91,253,57,105,225,127,14,78,119,244,101,195,179,166,139,248,244,101,199,173,212,126,89,114,238,89,135,39,59,252,250,97,233,215,155,170,89,114,44,211,207,157,254,120,247,242,202,233,163,149,156,50,242,219,167,165,254,89,120,245,211,203,46,71,76,22,116,211,183,47,62,152,118,240,116,197,171,134,174,25,182,110,205,139,134,45,19,6,84,28,166,12,168,53,76,25,80,108,229,131,38,108,219,178,96,205,179,134,173,27,183,106,225,131,112,159,87,213,245,125,159,103,217,139,86,76,217,183,100,193,155,103,13,90,55,110,213,195,6,224};
    
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
    message.plan_type.type=4;
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



