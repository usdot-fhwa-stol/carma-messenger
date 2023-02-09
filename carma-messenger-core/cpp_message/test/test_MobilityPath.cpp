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
#include "cpp_message/MobilityPath_Message.h"
#include <gtest/gtest.h>

TEST(MobilityPathMessageTest, testDecodeRequestMsg)
{
    std::vector<uint8_t> binary_input = {0, 242, 112, 77, 90, 113, 39, 212, 90, 209, 171, 22, 12, 38, 173, 56, 147, 234, 45, 104, 213, 131, 150, 172, 88, 65, 133, 14, 36, 88, 204, 88, 177, 98, 197, 139, 22, 43, 89, 50, 100, 201, 107, 54, 108, 217, 173, 131, 6, 12, 21, 172, 88, 177, 98, 197, 139, 22, 44, 88, 177, 98, 229, 147, 38, 108, 219, 178, 96, 205, 179, 134, 173, 27, 183, 106, 225, 131, 116, 193, 149, 6, 137, 131, 42, 13, 83, 6, 84, 27, 57, 100, 201, 155, 54, 236, 152, 51, 108, 225, 171, 70, 237, 218, 184, 96, 220, 39, 213, 245, 125, 95, 103, 217, 246};
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Path worker(node->get_node_logging_interface());
    boost::optional<carma_v2x_msgs::msg::MobilityPath> res;
    carma_v2x_msgs::msg::MobilityPath to_read;
    res = (worker.decode_mobility_path_message(binary_input));
    if (res)
    {
        to_read = res.get();
        // std::cout<<"Decoded Values:"<<std::endl;
        // std::cout<<to_read.header.sender_id<<std::endl;
        // std::cout<<to_read.header.recipient_id<<std::endl;
        // std::cout<<to_read.header.sender_bsm_id<<std::endl;
        // std::cout<<to_read.header.plan_id<<std::endl;
        // std::cout<<to_read.header.timestamp<<std::endl;
        // std::cout<<to_read.trajectory.location.ecef_x<<std::endl;
        // std::cout<<to_read.trajectory.location.ecef_y<<std::endl;
        // std::cout<<to_read.trajectory.location.ecef_z<<std::endl;
        // std::cout<<to_read.trajectory.location.timestamp<<std::endl;
        // for(int i=0;i<to_read.trajectory.offsets.size();i++) std::cout<<to_read.trajectory.offsets[i]<<" ";

        if (to_read.m_header.sender_id == "USDOT-45100" && to_read.trajectory.location.ecef_z == 2)
            EXPECT_TRUE(true);
        else
            EXPECT_TRUE(false);
    }
    else
        EXPECT_TRUE(false);
}

TEST(MobilityPathMessageTest, testEncodeMobilityPathMsg)
{
    // Mobility_Operation::Mobility_Operation_Message worker;
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Path worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::MobilityPath message;
    header.sender_id = "USDOT-45100";
    header.recipient_id = "USDOT-45095";
    header.sender_bsm_id = "100ABCEF";
    header.plan_id = "11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.m_header = header;

    // body-location and trajectory- for encoding provide ros message
    // start location
    carma_v2x_msgs::msg::Trajectory trajectory;
    carma_v2x_msgs::msg::LocationECEF starting_location;
    starting_location.ecef_x = 0;
    starting_location.ecef_y = 1;
    starting_location.ecef_z = 2;
    starting_location.timestamp = 9223372036854775807;

    trajectory.location = starting_location;
    // offsets
    carma_v2x_msgs::msg::LocationOffsetECEF offset1;
    offset1.offset_x = 1;
    offset1.offset_y = 1;
    offset1.offset_z = 1;

    trajectory.offsets.push_back(offset1);

    carma_v2x_msgs::msg::LocationOffsetECEF offset2;
    offset2.offset_x = 2;
    offset2.offset_y = 2;
    offset2.offset_z = 2;

    trajectory.offsets.push_back(offset2);

    message.trajectory = trajectory;
    auto res = worker.encode_mobility_path_message(message);

    // std::vector<uint8_t> to_read=res.get();
    // size_t len=to_read.size();
    // std::cout<<"Encoded Values:"<<std::endl;
    // for(size_t i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
    // std::cout<<"\n";
    if (res)
    {
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(MobilityPathMessageTest, testEncodeDecodeMobilityPathMsg)
{
    // Mobility_Operation::Mobility_Operation_Message worker;
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Path worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::MobilityPath message;
    header.sender_id = "USDOT-45100";
    header.recipient_id = "USDOT-45095";
    header.sender_bsm_id = "100ABCEF";
    header.plan_id = "11111111-2222-3333-AAAA-111111111111";
    header.timestamp = 9223372036854775807;
    message.m_header = header;

    // body-location and trajectory- for encoding provide ros message
    // start location
    carma_v2x_msgs::msg::Trajectory trajectory;
    carma_v2x_msgs::msg::LocationECEF starting_location;
    starting_location.ecef_x = 0;
    starting_location.ecef_y = 1;
    starting_location.ecef_z = 2;
    starting_location.timestamp = 9223372036854775807;

    trajectory.location = starting_location;
    // offsets
    carma_v2x_msgs::msg::LocationOffsetECEF offset1;
    offset1.offset_x = 1;
    offset1.offset_y = 1;
    offset1.offset_z = 1;

    trajectory.offsets.push_back(offset1);

    carma_v2x_msgs::msg::LocationOffsetECEF offset2;
    offset2.offset_x = 2;
    offset2.offset_y = 2;
    offset2.offset_z = 2;

    trajectory.offsets.push_back(offset2);

    message.trajectory = trajectory;
    auto res = worker.encode_mobility_path_message(message);

    if (res)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed while unit testing Mobility Path encoder! \n";
        EXPECT_TRUE(false);
    }
    auto res_decoded = worker.decode_mobility_path_message(res.get());
    if (res_decoded)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "Decoding failed while unit testing Mobility Path encoder! \n";
        EXPECT_TRUE(false);
    }
    carma_v2x_msgs::msg::MobilityPath result = res_decoded.get();
    EXPECT_EQ(message, result);

    auto res2 = worker.encode_mobility_path_message(result);
    if (res2)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "Encoding failed second time while unit testing Mobility Path encoder! \n";
        EXPECT_TRUE(false);
    }
    auto res2_decoded = worker.decode_mobility_path_message(res2.get());
    if (res2_decoded)
        EXPECT_TRUE(true);
    else
    {
        std::cout << "Decoding failed second time while unit testing Mobility Path encoder! \n";
        EXPECT_TRUE(false);
    }
    EXPECT_EQ(message, res2_decoded.get());
}

TEST(MobilityPathMessageTest, testEncodeMobilityPathMsg_base_case)
{
    // Mobility_Operation::Mobility_Operation_Message worker;
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::Mobility_Path worker(node->get_node_logging_interface());
    carma_v2x_msgs::msg::MobilityHeader header;
    carma_v2x_msgs::msg::MobilityPath message;
    header.sender_id = "";
    header.recipient_id = "";
    header.sender_bsm_id = "";
    header.plan_id = "";
    header.timestamp = 0;
    message.m_header = header;

    // body-location and trajectory- for encoding provide ros message
    // start location
    carma_v2x_msgs::msg::Trajectory trajectory;
    carma_v2x_msgs::msg::LocationECEF starting_location;
    starting_location.ecef_x = 0;
    starting_location.ecef_y = 0;
    starting_location.ecef_z = 0;
    starting_location.timestamp = 0;

    trajectory.location = starting_location;
    // offsets
    carma_v2x_msgs::msg::LocationOffsetECEF offset1;
    offset1.offset_x = 0;
    offset1.offset_y = 0;
    offset1.offset_z = 0;

    trajectory.offsets.push_back(offset1);

    message.trajectory = trajectory;
    auto res = worker.encode_mobility_path_message(message);

    // std::vector<uint8_t> to_read=res.get();
    // size_t len=to_read.size();
    // std::cout<<"Encoded Values:"<<std::endl;
    // for(size_t i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
    // std::cout<<"\n";
    if (res)
    {
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}
