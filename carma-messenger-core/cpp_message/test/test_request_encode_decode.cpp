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

TEST(CppMessageTest, testDecodeRequestMsg)
{
    std::vector<uint8_t> binar_input = {0, 244, 56, 18, 48, 98, 201, 155, 70, 173, 155, 184, 114, 193, 139, 38, 109, 26,
                                        182, 110, 225, 203, 6, 44, 153, 180, 106, 217, 187, 135, 45, 155, 54, 108, 217, 179,
                                        0, 0, 0, 0, 0, 0, 0, 0, 3, 90, 78, 144, 3, 90, 78, 144, 8, 0, 8, 0, 8, 0, 0};
    cpp_message::Message worker;
    auto res = worker.decode_geofence_request(binar_input);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}

TEST(CppMessageTest, testEncodeRequestMsg)
{
    cpp_message::Message worker;
    j2735_msgs::ControlRequest request;
    request.version = "012345678901234567890123456789012345";
    request.scale = 0;
    j2735_msgs::ControlBounds bounds;
    bounds.longitude = 0;
    bounds.latitude = 0;
    bounds.oldest = 0;
    for(int i = 0; i < bounds.offsets.size(); i++) bounds.offsets[i] = 0;
    request.bounds.push_back(bounds);
    auto res = worker.encode_geofence_request(request);
    if(res) EXPECT_TRUE(true);
    else
    {
        //std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
