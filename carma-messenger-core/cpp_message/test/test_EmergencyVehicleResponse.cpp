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

//#include "emergency_vehicle_response_message.h"
#include "cpp_message/EmergencyVehicleResponse_Message.h"
#include <gtest/gtest.h>
#include <boost/optional/optional_io.hpp> //to print boost::optional

TEST(EmergencyVehicleResponseMessageTest, testDecodeEmergencyVehicleResponseMsg)
{
    // binary copied from the output of testEncodeEmergencyVehicleResponseMsg below
    std::vector<uint8_t> binary_input = {0, 246, 76, 77, 90, 113, 39, 212, 90, 209, 171, 22, 12, 38, 173, 56, 147, 234, 45, 104, 213, 131, 150, 172, 88, 65, 133, 14, 36, 88, 204, 88, 177, 98, 197, 139, 22, 43, 89, 50, 100, 201, 107, 54, 108, 217, 173, 131, 6, 12, 21, 172, 88, 177, 98, 197, 139, 22, 44, 88, 177, 98, 229, 147, 38, 108, 219, 178, 96, 205, 179, 134, 173, 27, 183, 106, 225, 131, 112, 0, 246, 76, 77, 90, 113, 39, 212, 90, 209, 171, 22, 12, 38, 173, 56, 147, 234, 45, 104, 213, 131, 150, 172, 88, 65, 133, 14, 36, 88, 204, 88, 177, 98, 197, 139, 22, 43, 89, 50, 100, 201, 107, 54, 108, 217, 173, 131, 6, 12, 21, 172, 88, 177, 98, 197, 139, 22, 44, 88, 177, 98, 229, 147, 38, 108, 219, 178, 96, 205, 179, 134, 173, 27, 183, 106, 225, 131, 112};
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Emergency_Vehicle_Response worker(node->get_node_logging_interface());
    auto res = worker.decode_emergency_vehicle_response_message(binary_input);
    carma_v2x_msgs::msg::EmergencyVehicleResponse to_read;
    if (res)
    {
        to_read = res.get();
        if (to_read.m_header.plan_id == "11111111-2222-3333-AAAA-111111111111" && to_read.can_change_lanes == 0)
        {
            EXPECT_TRUE(true);
        }
        else
            EXPECT_TRUE(false);
    }
    else
        EXPECT_TRUE(false);
}

TEST(EmergencyVehicleResponseMessageTest, testEncodeEmergencyVehicleResponseMsg)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Emergency_Vehicle_Response worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::EmergencyVehicleResponse message;
    header.sender_id = "USDOT-45100";
    header.recipient_id = "USDOT-45095";
    header.sender_bsm_id = "10ABCDEF";
    header.plan_id = "11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.m_header = header;
    message.can_change_lanes = 0;
    message.reason_exists = 0;
    // message.reason = "vehicles in the way";
    auto res = worker.encode_emergency_vehicle_response_message(message);

    if (res)
    {
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

TEST(EmergencyVehicleResponseMessageTest, testEncodeDecodeEmergencyVehicleResponseMsg)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Emergency_Vehicle_Response worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::EmergencyVehicleResponse message;
    header.sender_id = "USDOT-45100";
    header.recipient_id = "USDOT-45095";
    header.sender_bsm_id = "10ABCDEF";
    header.plan_id = "11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.m_header = header;
    message.can_change_lanes = 1;
    message.reason = "can't do it";
    message.reason_exists = 1;
    auto res = worker.encode_emergency_vehicle_response_message(message);

    if (res)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    auto res_decoded = worker.decode_emergency_vehicle_response_message(res.get());
    if (res_decoded)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    carma_v2x_msgs::msg::EmergencyVehicleResponse result = res_decoded.get();
    EXPECT_EQ(message, result);

    auto res2 = worker.encode_emergency_vehicle_response_message(result);
    if (res2)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "Encoding failed while unit testing BSM encoder!\n";
        EXPECT_TRUE(false);
    }
    auto res2_decoded = worker.decode_emergency_vehicle_response_message(res2.get());
    if (res2_decoded)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    EXPECT_EQ(message, res2_decoded.get());
}

TEST(EmergencyVehicleResponseMessageTest, testEncodeDecodeEmergencyVehicleResponseMsg_base_case)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Emergency_Vehicle_Response worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::EmergencyVehicleResponse message;
    header.sender_id = "";
    header.recipient_id = "";
    header.sender_bsm_id = "";
    header.plan_id = "";
    header.timestamp = 0;
    message.m_header = header;
    message.can_change_lanes = 0;
    message.reason_exists = 0;
    auto res = worker.encode_emergency_vehicle_response_message(message);

    if (res)
    {
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
    auto res_decoded = worker.decode_emergency_vehicle_response_message(res.get());
    if (res_decoded)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
}
