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
#include "MobilityRequest_Message.h"
#include<gtest/gtest.h>
#include<ros/ros.h>

TEST(MobilityRequestMessageTest, testDecodeMobilityRequestMsg)
{
    std::vector<uint8_t> binary_input={0,240,128,145,5,187,161,110,235,6,12,24,48,96,193,131,6,12,24,48,96,193,130,214,12,24,48,90,193,131,6,11,88,48,96,193,107,6,12,24,48,96,193,131,6,12,24,48,96,193,131,6,12,24,48,96,193,131,6,12,24,48,96,195,129,110,234,6,73,131,42,13,19,6,84,26,166,12,168,54,96,193,131,6,12,24,48,96,193,131,6,12,24,48,96,192,0,1,110,234,96,202,131,68,193,149,6,137,131,42,13,24,48,96,193,131,6,12,24,48,96,193,131,6,12,24,48,96,11,232,250,62,140,24,48,96,193,131,6,12,24,48,96,193,131,6,12,24,48};
    cpp_message::Mobility_Request worker;
    boost::optional<cav_msgs::MobilityRequest> res = worker.decode_mobility_request_message(binary_input);
    if(res)
    {
        cav_msgs::MobilityRequest to_read=res.get();
        std::cout<<"H_sender_id:"<<to_read.header.sender_id<<std::endl;
        std::cout<<"H_target id:"<<to_read.header.recipient_id<<std::endl;
        std::cout<<"H_bsm id:"<<to_read.header.sender_bsm_id<<std::endl;
        std::cout<<"H_plan id:"<<to_read.header.plan_id<<std::endl;
        std::cout<<"H_timestamp:"<<to_read.header.timestamp<<std::endl;
        std::cout<<"strategy:"<<to_read.strategy<<std::endl;
        std::cout<<"plan type:"<<to_read.plan_type;;
        std::cout<<"urgency:"<<to_read.urgency<<std::endl;        
        std::cout<<"location_x:"<<to_read.location.ecef_x<<std::endl;
        std::cout<<"location_y:"<<to_read.location.ecef_y<<std::endl;
        std::cout<<"location_z:"<<to_read.location.ecef_z<<std::endl;
        std::cout<<"location_timestamp:"<<to_read.location.timestamp<<std::endl;
        std::cout<<"strategy_params:"<<to_read.strategy_params<<std::endl;
        std::cout<<"trajectory_x:"<<to_read.trajectory.location.ecef_x<<std::endl;
        std::cout<<"trajectory_y:"<<to_read.trajectory.location.ecef_y<<std::endl;
        std::cout<<"trajectory_z:"<<to_read.trajectory.location.ecef_z<<std::endl;
        std::cout<<"trajectory_timestamp:"<<to_read.trajectory.location.timestamp<<std::endl;
        std::cout<<"trajectory_offsets0_x:"<<to_read.trajectory.offsets[0].offset_x<<std::endl;
        std::cout<<"trajectory_offsets0_y:"<<to_read.trajectory.offsets[0].offset_y<<std::endl;
        std::cout<<"trajectory_offsets0_z:"<<to_read.trajectory.offsets[0].offset_z<<std::endl;
        std::cout<<"trajectory_offsets1_x:"<<to_read.trajectory.offsets[1].offset_x<<std::endl;
        std::cout<<"trajectory_offsets1_y:"<<to_read.trajectory.offsets[1].offset_y<<std::endl;
        std::cout<<"trajectory_offsets1_z:"<<to_read.trajectory.offsets[1].offset_z<<std::endl;
        std::cout<<"expiration:"<<to_read.expiration<<std::endl;
        if(res) EXPECT_TRUE(true);
        else EXPECT_TRUE(false);
    }
    else EXPECT_TRUE(false);
}


TEST(MobilityRequestMessageTest, testEncodeMobilityRequestMsg)
{
    cpp_message::Mobility_Request worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityRequest message;
    header.sender_id="USDOT-45100";
    header.recipient_id="USDOT-45095";
    header.sender_bsm_id="10ABCDEF";
    header.plan_id="11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.header=header;   

    //body
    message.strategy="Carma/Platooning";
    message.plan_type.type=4;
    message.urgency=50;
    
        //location
    cav_msgs::LocationECEF starting_location;
    starting_location.ecef_x=0;
    starting_location.ecef_y=1;
    starting_location.ecef_z=2;
    starting_location.timestamp=1223372036854775807;

    message.location=starting_location;
        //strategy_params
    message.strategy_params="vin_number:1FUJGHDV0CLBP8834,license_plate:DOT-10003,carrier_name:Silver Truck FHWA TFHRC,carrier_id:USDOT 0000001,weight:,ads_software_version:System Version Unknown,date_of_last_state_inspection:YYYY-MM-DD,date_of_last_ads_calibration:YYYY-MM-DD,pre_trip_ads_health_check:Green,ads_status:Red,iss_score:49,permit_required:0,timestamp:1585836731814";
        //trajectory
    cav_msgs::Trajectory trajectory;
    cav_msgs::LocationECEF trajectory_start;
    trajectory_start.ecef_x=5;
    trajectory_start.ecef_y=1;
    trajectory_start.ecef_z=2;
    trajectory_start.timestamp=9023372036854775807;
    trajectory.location=trajectory_start;

            //offsets
    cav_msgs::LocationOffsetECEF offset1;
    offset1.offset_x=1;
    offset1.offset_y=1;
    offset1.offset_z=1;

    trajectory.offsets.push_back(offset1);
 
    cav_msgs::LocationOffsetECEF offset2;
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
    cpp_message::Mobility_Request worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityRequest message;
    header.sender_id="";
    header.recipient_id="";
    header.sender_bsm_id="";
    header.plan_id="";
    header.timestamp = 0;
    message.header=header;   

    //body
    message.strategy="";
    message.plan_type.type=4;
    message.urgency=50;
    
        //location
    cav_msgs::LocationECEF starting_location;
    starting_location.ecef_x=0;
    starting_location.ecef_y=1;
    starting_location.ecef_z=2;
    starting_location.timestamp=0;

    message.location=starting_location;
        //strategy_params
    message.strategy_params="";
        //trajectory
    cav_msgs::Trajectory trajectory;
    cav_msgs::LocationECEF trajectory_start;
    trajectory_start.ecef_x=0;
    trajectory_start.ecef_y=0;
    trajectory_start.ecef_z=0;
    trajectory_start.timestamp=0;
    trajectory.location=trajectory_start;

            //offsets
    cav_msgs::LocationOffsetECEF offset1;
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



// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


