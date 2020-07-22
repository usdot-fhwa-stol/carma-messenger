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

#include "Mobility_Operation_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

// TEST(MobilityOperationMessageTest, testDecodeMobilityOperationMsg)
// {
//     std::vector<uint8_t> binar_input = {0, 13, 127, 115, 77, 90, 113, 39, 44, 90, 47, 85, 22, 12, 38, 83, 56, 109, 22, 45, 104, 43, 125, 106, 84, 88, 65, 123, 14, 36, 88, 52, 88, 79, 98, 59, 117, 22, 43, 89, 50, 100, 55, 107, 54, 108, 39, 83, 125, 6, 12, 21, 84, 88, 79, 98, 59, 117, 22, 44, 88, 79, 98, 63, 125, 6, 12, 24, 48, 96, 63, 125, 22, 76, 38, 53, 108, 35, 61, 109, 95, 31, 27, 73, 10, 6, 27, 48, 12, 33, 65, 118, 99, 39, 43, 66, 38, 119, 95, 35, 41, 110, 44, 68, 99, 49, 115, 86, 84, 121, 17, 43, 48, 121, 50, 21, 7, 14, 25, 76, 89, 77, 78, 60, 69, 71, 27, 65, 61, 102, 30, 103, 93, 68, 97, 81, 107, 22, 12, 24, 51, 89, 113, 15, 46, 90, 114, 14, 65, 69, 14, 36, 82, 87, 23, 39, 37, 47, 36, 21, 57, 117, 57, 83, 4, 105, 21, 32, 96, 87, 26, 69, 40, 107, 49, 31, 27, 53, 78, 94, 87, 12, 28, 117, 86, 100, 73, 11, 16, 48, 96, 63, 125, 6, 12, 86, 119, 53, 89, 62, 114, 114, 106, 97, 55, 50, 1, 61, 7, 70, 119, 61, 53, 45, 2, 39, 121, 115, 45, 65, 115, 86, 126, 121, 12, 53, 75, 5, 108, 68, 71, 23, 33, 71, 5, 93, 38, 9, 111, 17, 71, 102, 76, 61, 50, 33, 33, 102, 2, 52, 60, 6, 95, 25, 45, 15, 76, 73, 12, 18, 25, 61, 46, 62, 102, 119, 18, 117, 102, 51, 101, 43, 102, 51, 91, 18, 34, 52, 104, 122, 101, 65, 65, 53, 3, 104, 121, 12, 65, 121, 39, 59, 8, 16, 20, 45, 117, 106, 30, 102, 119, 18, 117, 102, 51, 101, 43, 102, 51, 91, 18, 34, 50, 28, 78, 33, 23, 53, 79, 11, 8, 114, 115, 65, 93, 46, 29, 99, 52, 95, 57, 93, 46, 61, 110, 93, 14, 53, 105, 114, 52, 57, 57, 33, 25, 45, 15, 78, 68, 35, 82, 53, 111, 102, 98, 124, 17, 13, 57, 65, 106, 87, 77, 28, 84, 31, 105, 106, 35, 61, 47, 14, 53, 57, 82, 98, 89, 114, 58, 96, 77, 90, 99, 71, 121, 12, 61, 73, 125, 90, 45, 92, 53, 112, 51, 77, 118, 108, 92, 49, 104};
//     Mobility_Operation::Mobility_Operation_Message worker;
//     auto res = worker.decode_mobility_operation_message(binar_input);
//     if(res) EXPECT_TRUE(true);
//     else EXPECT_TRUE(false);
// }

TEST(MobilityOperationMessageTest, testEncodeMobilityOperationMsg)
{
    Mobility_Operation::Mobility_Operation_Message worker;
    cav_msgs::MobilityHeader header;
    cav_msgs::MobilityOperation message; 
    message.header=header;
    message.header.sender_id="USDOT-45100";
    message.header.recipient_id="USDOT-45095";
    message.header.sender_bsm_id="10ABCDEF";
    message.header.plan_id="11111111-2222-3333-AAAA-111111111111";
    message.header.timestamp=1585836731814;
    message.strategy="Carma/Platooning";
    message.strategy_params="vin_number:1FUJGHDV0CLBP8834,license_plate:DOT-10003,carrier_name:Silver Truck FHWA TFHRC,carrier_id:USDOT 0000001,weight:,ads_software_version:System Version Unknown,date_of_last_state_inspection:YYYY-MM-DD,date_of_last_ads_calibration:YYYY-MM-DD,pre_trip_ads_health_check:Green,ads_status:Red,iss_score:49,permit_required:0,timestamp:1585836731814";
    
    auto res = worker.encode_mobility_operation_message(message);
    if(res) EXPECT_TRUE(true);
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