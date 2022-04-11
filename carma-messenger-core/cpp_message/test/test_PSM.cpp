/*
 * Copyright (C) 2022 LEIDOS.
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

#include "cpp_message/PSM_Message.h"
#include "carma_v2x_msgs/msg/psm.hpp"
#include<gtest/gtest.h>

namespace cpp_message
{
    int char2int(char input)
    {
    if(input >= '0' && input <= '9')
        return input - '0';
    if(input >= 'A' && input <= 'F')
        return input - 'A' + 10;
    if(input >= 'a' && input <= 'f')
        return input - 'a' + 10;
    throw std::invalid_argument("Invalid input string");
    }

    void hex2bin(const char* src, char* target)
    {
        while(*src && src[1])
        {
            *(target++) = char2int(*src)*16 + char2int(src[1]);
            src += 2;
        }
    }

    std::vector<char> HexToBytes(const std::string& hex) {
        std::vector<char> bytes;

        for (unsigned int i = 0; i < hex.length(); i += 2) {
            std::string byteString = hex.substr(i, 2);
            char byte = (char) strtol(byteString.c_str(), NULL, 16);
            bytes.push_back(byte);
        }

        return bytes;
    }

    // TEST(PSMTest, testdDecodePSM){
    //     std::string hex_message = "3C4D6573736167654672616D653E0D0A20203C6D65737361676549643E33323C2F6D65737361676549643E0D0A20203C76616C75653E0D0A202020203C506572736F6E616C5361666574794D6573736167653E0D0A2020202020203C6261736963547970653E3C61";
        
    //     //Convert hex string to byte message
    //     std::vector<uint8_t> binary_input;

    //     std::vector<char> new_binary_input = HexToBytes(hex_message);
    //     for(int i =0;i<new_binary_input.size();++i){
    //         std::cout<<new_binary_input[i]<<" "<<",";
    //     }
    //     std::cout<<"\n";
    // }

    TEST(PSMTest, testEncodePSM){
        
        carma_v2x_msgs::msg::PSM message;

        //1.presence_vector
        uint32_t presence_vector = carma_v2x_msgs::msg::PSM::HAS_ACCEL_SET;
        message.presence_vector |= presence_vector;

        //2. basic_type
        j2735_v2x_msgs::msg::PersonalDeviceUserType basic_type;
        basic_type.type |= j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;
        message.basic_type = basic_type;

        //3.sec_mark
        j2735_v2x_msgs::msg::DSecond sec_mark;
        sec_mark.millisecond = 60998;
        message.sec_mark = sec_mark;

        //4.msg_cnt
        j2735_v2x_msgs::msg::MsgCount msg_cnt;
        msg_cnt.msg_cnt = 1;

        message.msg_cnt = msg_cnt;

        //5. id
        j2735_v2x_msgs::msg::TemporaryID id;
        id.id = {0,0,0,0};

        message.id = id;

        //6. position
        carma_v2x_msgs::msg::Position3D position;
        position.latitude = 406680509 * (float)0.0000001;
        position.longitude = -738318466 * (float)0.0000001;
        position.elevation_exists = false;
        // position.elevation = 40 * 0.1;

        message.position = position;

        //7. positional accuracy
        carma_v2x_msgs::msg::PositionalAccuracy accuracy;
        accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
        accuracy.semi_major = 11.7;
        accuracy.semi_minor = 11.7;
        accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
        accuracy.orientation = 300;

        message.accuracy = accuracy;

        //8. velocity
        carma_v2x_msgs::msg::Velocity speed;
        speed.unavailable = false;
        speed.velocity = 75 * 0.02;

        message.speed = speed;
        //9. heading
        carma_v2x_msgs::msg::Heading heading;
        heading.unavailable = false;
        heading.heading = 3672 * 0.0125;

        message.heading = heading;

        rclcpp::NodeOptions options;
        auto node = std::make_shared<rclcpp::Node>("base_node");
        
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ = node->get_node_logging_interface();;
        cpp_message::PSM_Message worker(node_logging_);

       boost::optional<std::vector<uint8_t>> b_array = worker.encode_psm_message(message);
       if(b_array){
        std::vector<uint8_t> array = b_array.get();
        for(size_t i = 0; i < array.size(); i++) std::cout<< int(array[i])<< ", ";
       }

    //    Binary examples -
    //1. Till longitude - 0, 32, 26, 0, 0, 3, 220, 140, 4, 0, 0, 0, 0, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 0, 0, 0, 0, 0, 0
    //2. Till Header -  0, 32, 28, 0, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 128
    }
}
