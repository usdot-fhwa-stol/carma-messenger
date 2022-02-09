/*
 * Copyright (C) <SUB><year> LEIDOS.
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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "ros2_cpp_message/cpp_message.h"
#include "test_encode_decode.cpp"
#include "test_MobilityOperations.cpp"
#include "test_MobilityResponse.cpp"
#include "test_MobilityPath.cpp"
#include "test_MobilityRequest.cpp"
#include "test_BSM.cpp"
#include "test_MapMessage.cpp"
#include "test_SPAT.cpp"
#include "test_BSM.cpp"

     
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);
    
    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}