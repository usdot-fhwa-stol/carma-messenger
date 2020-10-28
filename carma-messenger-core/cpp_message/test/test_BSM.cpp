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

#include "BSM_Message.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/optional/optional_io.hpp>   //to print boost::optional

TEST(BSMTest, testDecodeBSM)
{
    std::vector<uint8_t> binary_input = {0,20,37,0,64,64,128,193,0,0,90,210,116,128,53,164,233,0,8,0,0,0,0,0,128,0,0,0,126,125,7,208,127,128,0,10,170,0,128,8};
    cpp_message::BSM_Message worker;
    auto res = worker.decode_bsm_message(binary_input);
    j2735_msgs::BSM to_read;
    if(res)
    {
        to_read=res.get();
        EXPECT_EQ(to_read.core_data.msg_count, 1);
        
        EXPECT_EQ(to_read.core_data.id[0], (uint8_t)1);
        EXPECT_EQ(to_read.core_data.id[1], (uint8_t)2);
        EXPECT_EQ(to_read.core_data.id[2], (uint8_t)3);
        EXPECT_EQ(to_read.core_data.id[3], (uint8_t)4);

        EXPECT_EQ(to_read.core_data.sec_mark, 1);
        EXPECT_EQ(to_read.core_data.latitude, 0); 
        EXPECT_EQ(to_read.core_data.longitude, 1); 
        EXPECT_EQ(to_read.core_data.elev, 0);
        EXPECT_EQ(to_read.core_data.accuracy.orientation, 1); 
        EXPECT_EQ(to_read.core_data.accuracy.semiMajor, 0); 
        EXPECT_EQ(to_read.core_data.accuracy.semiMinor, 0); 
        EXPECT_EQ(to_read.core_data.transmission.transmission_state, 0); 
        EXPECT_EQ(to_read.core_data.speed, 0); 
        EXPECT_EQ(to_read.core_data.heading, 0); 
        EXPECT_EQ(to_read.core_data.angle, 0); 
        EXPECT_EQ(to_read.core_data.accelSet.lateral, 0); 
        EXPECT_EQ(to_read.core_data.accelSet.longitudinal, 0);
        EXPECT_EQ(to_read.core_data.accelSet.vert, 0);
        EXPECT_EQ(to_read.core_data.accelSet.yaw_rate, 1); 
        EXPECT_EQ(to_read.core_data.brakes.wheelBrakes.brake_applied_status, j2735_msgs::BrakeAppliedStatus::RIGHT_REAR); 
        EXPECT_EQ(to_read.core_data.brakes.traction.traction_control_status, 1); 
        EXPECT_EQ(to_read.core_data.brakes.abs.anti_lock_brake_status, 1); 
        EXPECT_EQ(to_read.core_data.brakes.scs.stability_control_status, 1); 
        EXPECT_EQ(to_read.core_data.brakes.brakeBoost.brake_boost_applied, 1);
        EXPECT_EQ(to_read.core_data.brakes.auxBrakes.auxiliary_brake_status, 1);    
        EXPECT_EQ(to_read.core_data.size.vehicle_length, 1); 
        EXPECT_EQ(to_read.core_data.size.vehicle_width, 1); 
    }
    else EXPECT_TRUE(false);
}

TEST(BSMTest, testEncodeBSM)
{
    cpp_message::BSM_Message worker;
    j2735_msgs::BSM message;
    message.core_data.msg_count = 1;
    message.core_data.id = {1,2,3,4};
    message.core_data.sec_mark = 1;
    message.core_data.longitude = 1;
    message.core_data.accuracy.orientation = 1;
    message.core_data.brakes.wheelBrakes.brake_applied_status = j2735_msgs::BrakeAppliedStatus::RIGHT_REAR;
    message.core_data.brakes.traction.traction_control_status = 1;
    message.core_data.brakes.abs.anti_lock_brake_status = 1;
    message.core_data.brakes.scs.stability_control_status = 1;
    message.core_data.brakes.brakeBoost.brake_boost_applied = 1;
    message.core_data.brakes.auxBrakes.auxiliary_brake_status = 1;
    message.core_data.accelSet.yaw_rate = 1;
    message.core_data.size.vehicle_width = 1;
    message.core_data.size.vehicle_length = 1;
    auto res = worker.encode_bsm_message(message);

    if(res) {
        std::vector<uint8_t> to_read=res.get();
        size_t len=to_read.size();
        for(size_t i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
        std::cout<<"\n";
        
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "Encoding failed while unit testing BSM encoder!\n";
        EXPECT_TRUE(false);
    }
}
