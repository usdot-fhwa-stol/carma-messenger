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
#include "MobilityOperation_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/optional/optional_io.hpp>   //to print boost::optional

TEST(MobilityOperationMessageTest, testDecodeMobilityOperationMsg)
{
    std::vector<uint8_t> binary_input = {0, 243, 66, 5, 187, 161, 110, 235, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 130, 214, 12, 24, 48, 90, 193, 131, 6, 11, 88, 48, 96, 193, 107, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 193, 131, 6, 12, 24, 48, 96, 192, 11, 119, 64, 11, 119, 64};
    cpp_message::Mobility_Operation worker;
    auto res = worker.decode_mobility_operation_message(binary_input);
    cav_msgs::MobilityOperation to_read;
    if(res) {
        to_read=res.get();
        // std::cout<<"sender_id:"<<to_read.header.sender_id<<std::endl;
        // std::cout<<"recipient_id:"<<to_read.header.recipient_id<<std::endl;
        // std::cout<<"bsm_id:"<<to_read.header.sender_bsm_id<<std::endl;
        // std::cout<<"plan_id:"<<to_read.header.plan_id<<std::endl;
        // std::cout<<"timestamp:"<<to_read.header.timestamp<<std::endl;
        // std::cout<<"strategy:"<<to_read.strategy<<std::endl;
        // std::cout<<"strategy_params:"<<to_read.strategy_params<<std::endl;
        if(to_read.header.plan_id=="00000000-0000-0000-0000-000000000000" && to_read.strategy=="[]"){
            EXPECT_TRUE(true);
        }
        else EXPECT_TRUE(false);
    }
    else EXPECT_TRUE(false);
}

TEST(MobilityOperationMessageTest, testEncodeMobilityOperationMsg)
{
    //Mobility_Operation::Mobility_Operation_Message worker;
    cpp_message::Mobility_Operation worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityOperation message;     
    header.sender_id="";
    header.recipient_id="";
    header.sender_bsm_id="";
    header.plan_id="";
    header.timestamp = 0;
    message.header=header;
    message.strategy="";
    message.strategy_params="";
    auto res = worker.encode_mobility_operation_message(message);

    if(res) {
        // std::vector<uint8_t> to_read=res.get();
        // size_t len=to_read.size();
        // for(size_t i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
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