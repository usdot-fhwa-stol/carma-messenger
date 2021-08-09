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

#include "Map_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/optional/optional_io.hpp>   //to print boost::optional

namespace cpp_message
{

 int chartoint(char input)
    {
    if(input >= '0' && input <= '9')
        return input - '0';
    if(input >= 'A' && input <= 'F')
        return input - 'A' + 10;
    if(input >= 'a' && input <= 'f')
        return input - 'a' + 10;
    throw std::invalid_argument("Invalid input string");
    }

 void hextobin(const char* src, char* target)
    {
        while(*src && src[1])
        {
            *(target++) = chartoint(*src)*16 + chartoint(src[1]);
            src += 2;
        }
    }

std::vector<char> Hex2Bytes(const std::string& hex)
{
  std::vector<char> bytes;

  for (unsigned int i = 0; i < hex.length(); i += 2) {
    std::string byteString = hex.substr(i, 2);
    char byte = (char) strtol(byteString.c_str(), NULL, 16);
    bytes.push_back(byte);
  }

  return bytes;
}

TEST(MapMessageTest, testDecodeMapMessage)
{

    std::string hex_message = "0012815338033020204bda0d4cdcf8143d4dc48811860224164802280008002297d4bc80a0a0a9825825923a90b2f2e418986f41b7006480602403812020084015480010004521d9f001414160c7c42a1879858619502a42a060e927100662000400105be6bf41c8aded5816ebc050507dcb860ec57aead5079e02828900890001000417223a50728b750f9c6ea9e8ae480a0a0f68746ad447c002828900a0880704404020803b9000200062b68d5305d1f9269a725027d8352f72867d6c82403340004000c53f5b761abbb7d35d3c0813ec1a3baac16bfc048050240301202008402208001000310fe55f849acd608d8ace136b440000dfe4808880008002086365c0017d1612eb34026067404895390907bd848050440302201c100024000200000090026180a0a0f2852600140001000000169fc1585bd1da000b00008000000a3bb2f439459a80060000400000046d55c416c67f40";

        //Convert hex string to byte message
        std::vector<uint8_t> binary_input;

        std::vector<char> new_binary_input = Hex2Bytes(hex_message);
        std::vector<uint8_t> new_binary_input_int;
        for(int i = 0; i<new_binary_input.size();i++){
            new_binary_input_int.push_back(new_binary_input[i]);
        }

    cpp_message::Map_Message worker;

    auto res = worker.decode_map_message(new_binary_input_int);


    if(res)
    {
        //Intersection Handling
        EXPECT_EQ(res.get().intersections_exists, true);
        EXPECT_EQ(res.get().intersections.size(), 1);

        //IntersectionID
        EXPECT_EQ(res.get().intersections[0].id.id, 9709);
        EXPECT_EQ(res.get().intersections[0].id.region_exists, false);
        EXPECT_EQ(res.get().intersections[0].id.region, 0);

        //Intersection Lane Set
        EXPECT_EQ(res.get().intersections[0].lane_set.lane_list.size(), 0);

        //Intersection Lane Width
        EXPECT_EQ(res.get().intersections[0].lane_width_exists, true);
        EXPECT_EQ(res.get().intersections[0].lane_width, 274);

        //Intersection Name
        EXPECT_EQ(res.get().intersections[0].name_exists, false);

        //Intersection Preempt Priority Data
        EXPECT_EQ(res.get().intersections[0].preempt_priority_data_exists, false);

        //Intersection Speed Limits
        EXPECT_EQ(res.get().intersections[0].speed_limits_exists, false);
        EXPECT_EQ(res.get().intersections[0].speed_limits.speed_limits.size(), 0);

        //Intersection Ref Points
        EXPECT_EQ(res.get().intersections[0].ref_point.elevation_exists, true);
        EXPECT_EQ(res.get().intersections[0].ref_point.elevation, 390);
        EXPECT_NEAR(res.get().intersections[0].ref_point.latitude, 3.8955 * pow(10,8), pow(10,8));
        EXPECT_NEAR(res.get().intersections[0].ref_point.longitude, -7.71493 * pow(10, 8), pow(10,8));
        EXPECT_EQ(res.get().intersections[0].revision, 3);


        //Layer ID
        EXPECT_EQ(res.get().layer_id, 1);
        EXPECT_EQ(res.get().layer_id_exists, true);

        //Layer Type
        EXPECT_EQ(res.get().layer_type.layer_type,j2735_msgs::LayerType::INTERSECTION_DATA);

        //Msg Issue Revision
        EXPECT_EQ(res.get().msg_issue_revision,3);

        //Restriction List
        EXPECT_EQ(res.get().restriction_list_exists, false);
        EXPECT_EQ(res.get().restriction_list.restriction_class_list.size(), 0);

        //Road Segments
        EXPECT_EQ(res.get().road_segments_exists, false);
        EXPECT_EQ(res.get().road_segments.road_segment_list.size(), 0);

        //Time Stamp
        EXPECT_EQ(res.get().time_stamp_exists, false);
        EXPECT_EQ(res.get().time_stamp, 0);

        //Data Parameters
        EXPECT_EQ(res.get().data_parameters_exists, false);
    }
    else EXPECT_TRUE(false);

}



}