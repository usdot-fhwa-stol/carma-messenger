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

#include "cpp_message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(CppMessageTest, testDecodeRequestMsg1)
{
    std::vector<uint8_t> binar_input = {0, 244, 1, 0};
    cpp_message::Message worker;
    auto res = worker.decode_geofence_request(binar_input);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}

TEST(CppMessageTest, testDecodeRequestMsg2)
{
    std::vector<uint8_t> binar_input = {0, 244, 37, 32, 0, 0, 0, 0, 0, 0, 0, 0, 44,
                                        0, 0, 0, 0, 0, 0, 6, 180, 157, 31, 246, 180, 157, 
                                        32, 16, 0, 16, 0, 16, 0, 16, 0, 16, 0, 16, 0, 0};
    cpp_message::Message worker;
    auto res = worker.decode_geofence_request(binar_input);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}

TEST(CppMessageTest, testEncodeRequestMsg1)
{
    cpp_message::Message worker;

    j2735_msgs::TrafficControlRequest request;
    request.choice = j2735_msgs::TrafficControlRequest::RESERVED;
    auto res = worker.encode_geofence_request(request);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(CppMessageTest, testEncodeRequestMsg2)
{
    cpp_message::Message worker;

    j2735_msgs::TrafficControlRequest request;
    request.choice = j2735_msgs::TrafficControlRequest::TCRV01;

    j2735_msgs::TrafficControlRequestV01 request1;
    j2735_msgs::Id64b id64;
    for(int i = 0; i < id64.id.size(); i++){
        id64.id[i] = 0;
    }
    request1.reqid = id64;
    request1.reqseq = 1;
    request1.scale = 0;
    j2735_msgs::TrafficControlBounds bounds;
    bounds.reflon = 0;
    bounds.reflat = 0;
    bounds.oldest = 0;
    for(int i = 0; i < bounds.offsets.size(); i++){
        j2735_msgs::OffsetPoint point;
        point.deltax = 0;
        point.deltay = 0;
        bounds.offsets[i] = point;
    } 
    request1.bounds.push_back(bounds);
    request.tcrV01 = request1;
    auto res = worker.encode_geofence_request(request);
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