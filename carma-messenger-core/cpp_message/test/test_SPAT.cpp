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

#include "SPAT_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/optional/optional_io.hpp>   //To print boost::optional

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

    TEST(SPATTest, testDECODESPAT)
    {
        std::string hex_message = "00131c44630800812f6800000c2d40100204342b3fac0a0020232159495f9c";

        //Convert hex string to byte message
        std::vector<uint8_t> binary_input;

        std::vector<char> new_binary_input = HexToBytes(hex_message);
        std::vector<uint8_t> new_binary_input_int;
        for(int i = 0; i<new_binary_input.size();i++){
            new_binary_input_int.push_back(new_binary_input[i]);
        }
        

        cpp_message::SPAT_Message worker;
        auto res = worker.decode_spat_message(new_binary_input_int);
        
        
        if(res){
            EXPECT_EQ(res.get().time_stamp, 287496);
            EXPECT_EQ(int(res.get().intersections.intersection_state_list.front().states.movement_list.front().signal_group), 2);
            EXPECT_EQ(res.get().intersections.intersection_state_list.front().states.movement_list.front().state_time_speed.movement_event_list.front().timing.min_end_time, 22143);
            EXPECT_EQ(res.get().intersections.intersection_state_list.front().states.movement_list.front().state_time_speed.movement_event_list.front().timing.max_end_time, 22548);
        }


    }

    TEST(SPATTest, DISABLED_testENCODESPAT)
    {
        cpp_message::SPAT_Message worker;
        j2735_msgs::SPAT message;
        
        //1. Intersection State List
        j2735_msgs::IntersectionState intersection_state_1;
        intersection_state_1.id.id = 127;
        intersection_state_1.revision = 2;
        intersection_state_1.status.intersection_status_object = 0000000000000000;
        intersection_state_1.moy_exists = true;
        intersection_state_1.moy = 386768;
        intersection_state_1.time_stamp_exists = true;
        intersection_state_1.time_stamp = 40439;
        //intersection_state_1.states.movement_list;// Add movement_state_1 to this
        
        
        j2735_msgs::MovementState movement_state_1;
        movement_state_1.signal_group = 1;
        //movement_state_1.state_time_speed.movement_event_list; //Add movement_event_1 to this

        j2735_msgs::MovementEvent movement_event_1;
        movement_event_1.event_state.movement_phase_state = j2735_msgs::MovementPhaseState::STOP_AND_REMAIN;
        movement_event_1.timing_exists = true;
        movement_event_1.timing.min_end_time = 5786;
        movement_event_1.timing.max_end_time_exists = true;
        movement_event_1.timing.max_end_time = 36001;

        movement_state_1.state_time_speed.movement_event_list.push_back(movement_event_1);
        intersection_state_1.states.movement_list.push_back(movement_state_1);

        //Second Movement state
        j2735_msgs::MovementState movement_state_2;
        movement_state_2.signal_group = 2;
        //movement_state_2.state_time_speed.movement_event_list - Add Movement event to this

        j2735_msgs::MovementEvent movement_event_2;
        movement_event_2.event_state.movement_phase_state = j2735_msgs::MovementPhaseState::PERMISSIVE_MOVEMENT_ALLOWED;
        movement_event_2.timing_exists = true;
        movement_event_2.timing.min_end_time = 5581;
        movement_event_2.timing.max_end_time_exists = true;
        movement_event_2.timing.max_end_time = 5707;

        movement_state_2.state_time_speed.movement_event_list.push_back(movement_event_2);
        intersection_state_1.states.movement_list.push_back(movement_state_2);


        //Third Movement State
        j2735_msgs::MovementState movement_state_3;
        movement_state_3.signal_group = 3;

        j2735_msgs::MovementEvent movement_event_3;
        movement_event_3.event_state.movement_phase_state = j2735_msgs::MovementPhaseState::STOP_AND_REMAIN;
        movement_event_3.timing_exists = true;
        movement_event_3.timing.min_end_time = 5642;
        movement_event_3.timing.max_end_time_exists = true;
        movement_event_3.timing.max_end_time = 5768;

        movement_state_3.state_time_speed.movement_event_list.push_back(movement_event_3);
        intersection_state_1.states.movement_list.push_back(movement_state_3);

        //Fourth Movement State
        j2735_msgs::MovementState movement_state_4;
        movement_state_4.signal_group = 4;

        j2735_msgs::MovementEvent movement_event_4;
        movement_event_4.event_state.movement_phase_state = j2735_msgs::MovementPhaseState::STOP_AND_REMAIN;
        movement_event_4.timing_exists = true;
        movement_event_4.timing.min_end_time = 5786;
        movement_event_4.timing.max_end_time_exists = true;
        movement_event_4.timing.max_end_time = 36001;

        movement_state_4.state_time_speed.movement_event_list.push_back(movement_event_4);
        intersection_state_1.states.movement_list.push_back(movement_state_4);

        //Fifth Movement State
        j2735_msgs::MovementState movement_state_5;
        movement_state_5.signal_group = 5;

        j2735_msgs::MovementEvent movement_event_5;
        movement_event_5.event_state.movement_phase_state = j2735_msgs::MovementPhaseState::STOP_AND_REMAIN;
        movement_event_5.timing_exists = true;
        movement_event_5.timing.min_end_time = 5786;
        movement_event_5.timing.max_end_time_exists = true;
        movement_event_5.timing.max_end_time = 36001;

        movement_state_5.state_time_speed.movement_event_list.push_back(movement_event_5);
        intersection_state_1.states.movement_list.push_back(movement_state_5);

        message.intersections.intersection_state_list.push_back(intersection_state_1);

        auto res = worker.encode_spat_message(message);

    }


}
