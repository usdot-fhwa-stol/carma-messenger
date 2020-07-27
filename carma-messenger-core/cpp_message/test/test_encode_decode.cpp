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


TEST(CppMessageTest, DISABLED_testDecodeControlMsg)
{
    std::vector<uint8_t> binar_input = {0, 245, 128, 128, 73, 24, 49, 100, 205, 163, 86, 205, 220, 57, 96, 197,
                                        147, 54, 141, 91, 55, 112, 229, 131, 22, 76, 218, 53, 108, 221, 195, 150, 
                                        12, 89, 51, 104, 212, 4, 8, 12, 16, 20, 24, 28, 32, 4, 8, 12, 16, 20, 24, 
                                        28, 32, 0, 0, 0, 0, 0, 0, 0, 0, 8, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 90, 0, 32, 0, 32, 16, 0, 16, 0, 0, 56, 64, 112, 128, 0, 
                                        0, 0, 0, 0, 0, 0, 107, 73, 210, 0, 107, 73, 210, 0, 0, 0, 0, 0, 16, 16, 0, 16, 0, 16, 0, 16, 8, 8, 0, 8, 0, 8, 0, 8, 0};
    cpp_message::Message worker;
    auto res = worker.decode_geofence_control(binar_input);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}

TEST(CppMessageTest, testEncodeControlMsg1)
{
    cpp_message::Message worker;

    j2735_msgs::TrafficControlMessage control;
    control.choice = j2735_msgs::TrafficControlMessage::RESERVED;
    auto res = worker.encode_geofence_control(control);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(CppMessageTest, testEncodeControlMsg2)
{
    cpp_message::Message worker;
    // ControlMessage START
    j2735_msgs::TrafficControlMessage control_main;
    control_main.choice = j2735_msgs::TrafficControlMessage::TCMV01;
    // ControlMessageV01 START
    j2735_msgs::TrafficControlMessageV01 control;
    j2735_msgs::Id64b id64b;
    for(int i = 0; i < id64b.id.size(); i++){
        id64b.id[i] = 0;
    }
    control.reqid = id64b;
    control.reqseq = 111;
    j2735_msgs::Id128b id128b;
    for(int i = 0; i < id128b.id.size(); i++){
        id128b.id[i] = 0;
    }
    control.id = id128b;
    control.msgnum = 5;
    control.msgtot = 6;
    control.updated = 12345678;

    // TrafficControlPackage START
    j2735_msgs::TrafficControlPackage package;
    package.label = "avs";
    package.label_exists = true;
    for (auto i = 0; i < 2; i++) package.tcids.push_back(id128b);
    control.package = package;
    control.package_exists = true;
    // TrafficControlPackage END

    // TrafficControlParams START
    j2735_msgs::TrafficControlParams params;
    params.regulatory = true;
    j2735_msgs::TrafficControlVehClass bycicle;
    bycicle.vehicle_class = j2735_msgs::TrafficControlVehClass::BICYCLE;
    params.vclasses.push_back(bycicle);
    j2735_msgs::TrafficControlSchedule schedule;
    schedule.between_exists = true;
    schedule.dow_exists = true;
    schedule.repeat_exists = true;
    schedule.end_exists = true;
    j2735_msgs::DailySchedule daily_schedule;
    daily_schedule.begin = 1;
    daily_schedule.duration = 2;
    schedule.between.push_back(daily_schedule);
    schedule.start = 123456;
    schedule.end = 123456;
    j2735_msgs::DayOfWeek dow;
    dow.dow = {0,1,0,1,0,1,0};
    schedule.dow = dow;
    j2735_msgs::RepeatParams repeat;
    repeat.offset = 1;
    repeat.period = 2;
    repeat.span = 3;
    schedule.repeat = repeat;
    params.schedule = schedule;
    // TrafficControlDetails START
    j2735_msgs::TrafficControlDetail detail;
    detail.choice = j2735_msgs::TrafficControlDetail::CLOSED_CHOICE;
    detail.closed = j2735_msgs::TrafficControlDetail::OPENLEFT;

    //boost::array<uint8_t, 2UL> stuff {{0 ,1}}; //
    //detail.latperm = stuff;

    //ROS_WARN_STREAM("latperm og " << (int)detail.latperm.elems[0] << " | " << (int)detail.latperm.elems[1]);
    // TrafficControlDetails END
    params.detail = detail;
    control.params = params;
    control.params_exists = true;
    // TrafficControlParams END

    // TrafficControlGeometry START
    j2735_msgs::TrafficControlGeometry geometry;
    geometry.proj = "a";
    geometry.datum = "a";
    geometry.reftime = 0;
    geometry.reflon = 0;
    geometry.reflat = 0;
    geometry.refelv = 0;
    geometry.heading = 0;
    j2735_msgs::PathNode node;
    node.x = 1;
    node.y = 1;
    node.z_exists = false;
    node.z = 1;
    node.width_exists = false;
    node.width = 1;
    geometry.nodes.push_back(node);
    control.geometry = geometry;
    control.geometry_exists = false;    
    // TrafficControlGeometry END
    // TrafficControlMessageV01 END
    control_main.tcmV01 = control;
    // TrafficControlMessage END
    /*
    auto res = worker.encode_geofence_control(control_main);

    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    */
    auto res = worker.encode_geofence_control_combined(control_main);
    
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