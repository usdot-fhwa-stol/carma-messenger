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
#include "MobilityPath_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(CppMessageTest, testEncodeRequestMsg)
{
    std::vector<uint8_t> binary_input={0,242,112,77,90,113,39,212,90,209,171,22,12,38,173,56,147,234,45,104,213,131,150,172,88,65,133,14,36,88,204,88,177,98,197,139,22,43,89,50,100,201,107,54,108,217,173,131,6,12,21,172,88,177,98,197,139,22,44,88,177,98,229,147,38,108,219,178,96,205,179,134,173,27,183,106,225,131,116,193,149,6,137,131,42,13,83,6,84,27,57,100,201,155,54,236,152,51,108,225,171,70,237,218,184,96,220,39,213,245,125,95,103,217,246};
    cpp_message::Mobility_Path worker;
    cav_msgs::MobilityPath res;
    res=(worker.decode_mobility_path_message(binary_input)).get();
    if(res.trajectory.location.ecef_y==1)EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}


TEST(MobilityOperationMessageTest, testEncodeMobilityOperationMsg)
{
    //Mobility_Operation::Mobility_Operation_Message worker;
    cpp_message::Mobility_Path worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityPath message;     
    header.sender_id="USDOT-45100";
    header.recipient_id="USDOT-45095";
    header.sender_bsm_id="10ABCDEF";
    header.plan_id="11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.header=header;
    
    //body-location and trajectory- for encoding provide ros message
    //start location
    cav_msgs::Trajectory trajectory;
    cav_msgs::LocationECEF starting_location;
    starting_location.ecef_x=0;
    starting_location.ecef_y=1;
    starting_location.ecef_z=2;
    starting_location.timestamp=9223372036854775807;
    
    trajectory.location=starting_location;
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
    auto res = worker.encode_mobility_path_message(message);
    
   // std::vector<uint8_t> binary_input={0,242,112,77,90,113,39,212,90,209,171,22,12,38,173,56,147,234,45,104,213,131,150,172,88,65,133,14,36,88,204,88,177,98,197,139,22,43,89,50,100,201,107,54,108,217,173,131,6,12,21,172,88,177,98,197,139,22,44,88,177,98,229,147,38,108,219,178,96,205,179,134,173,27,183,106,225,131,116,193,149,6,137,131,42,13,83,6,84,27,57,100,201,155,54,236,152,51,108,225,171,70,237,218,184,96,220,39,213,245,125,95,103,217,246};
    std::vector<uint8_t> to_read=res.get();
    auto len=to_read.size();
    //for(auto i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
    if(res) { 
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