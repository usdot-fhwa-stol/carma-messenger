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

TEST(MapMessageTest, testDecodeMapMessage)
{
    std::vector<uint8_t> binary_input = {0,20,37,0,64,64,128,193,0,0,90,210,116,128,53,164,233,0,8,0,0,0,0,0,128,0,0,0,126,125,7,208,127,128,0,10,170,0,128,8};
    cpp_message::Map_Message worker;
            ROS_WARN_STREAM("TEST0");

    auto res = worker.decode_map_message(binary_input);
    j2735_msgs::MapData to_read;

        ROS_WARN_STREAM("TEST1");


    if(res)
    {
        to_read=res.get();

        //Intersection Handling
        EXPECT_EQ(to_read.intersections_exists, true);
        EXPECT_EQ(to_read.intersections.size(), 1);

        EXPECT_EQ(to_read.layer_id, (uint8_t)1);
        EXPECT_EQ(to_read.layer_id_exists, true);
        ROS_WARN_STREAM("TEST");
        EXPECT_EQ(to_read.layer_type.layer_type,j2735_msgs::LayerType::NONE );
        EXPECT_EQ(to_read.msg_issue_revision,(uint8_t)1);
        EXPECT_EQ(to_read.restriction_list_exists, true);
        EXPECT_EQ(to_read.restriction_list.restriction_class_list.size(), 1);
        EXPECT_EQ(to_read.road_segments_exists, true);
        EXPECT_EQ(to_read.road_segments.road_segment_list.size(), 1);
        EXPECT_EQ(to_read.time_stamp_exists, true);
        EXPECT_EQ(to_read.time_stamp, 1);




    }
    else EXPECT_TRUE(false);



}