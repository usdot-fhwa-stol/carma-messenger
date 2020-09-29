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

//#include "Mobility_Operation_Message.h"
#include "MobilityResponse_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(MobilityResponseMessageTest, testDecodeMobilityResponseMsg)
{
    std::vector<uint8_t> binary_input = {0,241,77,77,90,113,39,212,90,209,171,22,12,38,173,56,147,234,45,104,213,131,150,172,88,65,133,14,36,88,204,88,177,98,197,139,22,43,89,50,100,201,107,54,108,217,173,131,6,12,21,172,88,177,98,197,139,22,44,88,177,98,229,147,38,108,219,178,96,205,179,134,173,27,183,106,225,131,112,202};
    cpp_message::Mobility_Response worker;
    boost::optional<cav_msgs::MobilityResponse> res;
    res = worker.decode_mobility_response_message(binary_input);
    if(res){
        cav_msgs::MobilityResponse to_read=res.get();
        // std::cout<<to_read.header.sender_id<<std::endl;
        // std::cout<<to_read.header.recipient_id<<std::endl;
        // std::cout<<to_read.header.sender_bsm_id<<std::endl;
        // std::cout<<to_read.header.plan_id<<std::endl;
        // std::cout<<to_read.header.timestamp<<std::endl;
        // std::cout<<to_read.urgency<<std::endl;
        // std::cout<<int(to_read.is_accepted)<<std::endl;
        if(to_read.header.plan_id=="11111111-2222-3333-AAAA-111111111111" && to_read.urgency==50 ) {
            EXPECT_TRUE(true);
        }
        else EXPECT_TRUE(false);
    }
    else EXPECT_TRUE(false);
}

TEST(MobilityResponseMessageTest, testEncodeMobilityResponseMsg)
{
    cpp_message::Mobility_Response worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityResponse message;     
    header.sender_id="USDOT-45100";
    header.recipient_id="USDOT-45095";
    header.sender_bsm_id="10ABCDEF";
    header.plan_id="11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.header=header;
    message.urgency=50;
    message.is_accepted=1;
    auto res = worker.encode_mobility_response_message(message);
    std::vector<uint8_t> to_read=res.get();
    auto len=to_read.size();
    
    if(res) {
        // for(auto i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
        // std::cout<<"\n";
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(MobilityResponseMessageTest, testEncodeMobilityResponseMsg_base_case)
{
    cpp_message::Mobility_Response worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityResponse message;     
    header.sender_id="";
    header.recipient_id="";
    header.sender_bsm_id="";
    header.plan_id="";
    header.timestamp = 0;
    message.header=header;
    message.urgency=0;
    message.is_accepted=0;
    auto res = worker.encode_mobility_response_message(message);
    std::vector<uint8_t> to_read=res.get();
    auto len=to_read.size();
    
    if(res) {
        // for(auto i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
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