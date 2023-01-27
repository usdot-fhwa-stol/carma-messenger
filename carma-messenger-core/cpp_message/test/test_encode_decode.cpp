/*
 * Copyright (C) 2019-2023 LEIDOS.
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

#include "cpp_message/cpp_message.h"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

TEST(CppMessageTest, testDecodeControlMsgPackage)
{
    std::vector<uint8_t> binar_input_package_only = {0, 245, 72, 48, 0, 0, 0, 0, 0, 0, 0, 1, 188, 0, 24, 0, 
                                                    20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                                    2, 241, 133, 58, 22, 30, 220, 193, 0, 0, 0, 0, 0, 0, 0, 0, 
                                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                                                    
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    auto res = worker->decode_geofence_control(binar_input_package_only);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
    
    j2735_v2x_msgs::msg::TrafficControlMessageV01 msg;
    msg = res.get().tcm_v01;

    ASSERT_EQ(msg.reqseq, 111);
    ASSERT_EQ(msg.msgnum, 5);
    ASSERT_EQ(msg.msgtot, 6);
    ASSERT_FALSE(msg.geometry_exists);
    ASSERT_FALSE(msg.params_exists);
    ASSERT_TRUE(msg.package_exists);
    ASSERT_EQ(msg.package.label, "avs");

    auto res_encoded = worker->encode_geofence_control(res.get());
    if(res_encoded) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
    
}


TEST(CppMessageTest, testDecodeControlMsgParams)
{
    // NOTE: Binary obtained from running testEncodeControlMsg2 with only TrafficControlParams included (TrafficControlPackage and TrafficControlGeometry not included) 
    std::vector<uint8_t> binar_input_params_only = {0, 245, 61, 40, 0, 4, 8, 12, 16, 20, 24, 29, 188, 0, 24, 0, 20, 80, 84, 88, 92, 
                                                96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 0, 0, 2, 241, 133, 56, 0, 
                                                47, 0, 0, 0, 15, 15, 243, 0, 0, 0, 9, 251, 241, 7, 190, 0, 1, 0, 64, 4, 4, 128, 57, 34};



    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    auto res = worker->decode_geofence_control(binar_input_params_only);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);

    j2735_v2x_msgs::msg::TrafficControlMessageV01 msg;
    msg = res.get().tcm_v01;

    ASSERT_EQ(msg.reqseq, 111);
    ASSERT_EQ(msg.msgnum, 5);
    ASSERT_EQ(msg.msgtot, 6);
    ASSERT_TRUE(msg.params_exists);
    ASSERT_EQ(msg.params.detail.choice, j2735_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE);
    ASSERT_EQ(msg.params.schedule.between[0].begin, 1);

    auto res_encoded = worker->encode_geofence_control(res.get());
    if(res_encoded) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}

TEST(CppMessageTest, testDecodeControlMsgGeometry)
{
    // NOTE: Binary obtained from running testEncodeControlMsg2 with only TrafficControlGeometry included (TrafficControlPackage and TrafficControlParams not included) 
    std::vector<uint8_t> binar_input_geometry_only = {0, 245, 120, 36, 0, 4, 8, 12, 16, 20, 24, 29, 188, 0, 24, 0, 20, 80, 84, 88, 92, 96, 100, 104, 
                                                    108, 112, 116, 120, 124, 128, 132, 136, 140, 0, 0, 2, 241, 133, 56, 231, 77, 26, 121, 160, 211, 
                                                    205, 6, 20, 28, 240, 237, 225, 179, 42, 14, 28, 183, 234, 65, 207, 167, 45, 59, 179, 187, 211, 70, 
                                                    158, 104, 52, 243, 65, 133, 7, 60, 59, 120, 108, 202, 131, 38, 30, 157, 118, 160, 231, 211, 150, 157, 
                                                    217, 192, 0, 0, 0, 1, 172, 90, 210, 116, 128, 26, 210, 116, 133, 8, 125, 0, 8, 15, 0, 3, 0, 3, 0, 3, 
                                                    3, 64, 0, 64, 0, 192, 0, 0};

    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);
    auto res = worker->decode_geofence_control(binar_input_geometry_only);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);

    j2735_v2x_msgs::msg::TrafficControlMessageV01 msg;
    msg = res.get().tcm_v01;

    ASSERT_EQ(msg.reqseq, 111);
    ASSERT_EQ(msg.msgnum, 5);
    ASSERT_EQ(msg.msgtot, 6);
    ASSERT_TRUE(msg.geometry_exists);
    ASSERT_EQ(msg.geometry.reftime, 1713);
    ASSERT_EQ(msg.geometry.refelv, 250);

    auto res_encoded = worker->encode_geofence_control(res.get());
    if(res_encoded) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}


TEST(CppMessageTest, testEncodeControlMsg1)
{
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    j2735_v2x_msgs::msg::TrafficControlMessage control;
    control.choice = j2735_v2x_msgs::msg::TrafficControlMessage::RESERVED;
    auto res = worker->encode_geofence_control(control);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(CppMessageTest, testEncodeControlMsg2)
{
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);
    // ControlMessage START
    j2735_v2x_msgs::msg::TrafficControlMessage control_main;
    control_main.choice = j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01;
    // ControlMessageV01 START
    j2735_v2x_msgs::msg::TrafficControlMessageV01 control;
    j2735_v2x_msgs::msg::Id64b id64b;
    for(int i = 0; i < id64b.id.size(); i++){
        id64b.id[i] = i;
    }
    control.reqid = id64b;
    control.reqseq = 111;
    j2735_v2x_msgs::msg::Id128b id128b_0;
    for(int i = 0; i < id128b_0.id.size(); i++){
        id128b_0.id[i] = i + 20;
    }
    control.id = id128b_0;
    control.msgnum = 5;
    control.msgtot = 6;
    control.updated = (unsigned)12345678;

    //==============================================
    // ENABLES OPTIONAL FIELDS IN CONTROL MESSAGE
    bool PACKAGE_BOOL = 1;
    bool PARAM_BOOL = 1;
    bool GEO_BOOL = 1;

    //=======================================
    // TrafficControlPackage START
    j2735_v2x_msgs::msg::TrafficControlPackage package;
    package.label = "avs";
    package.label_exists = true;
    j2735_v2x_msgs::msg::Id128b id128b_1;
    for(int i = 0; i < id128b_1.id.size(); i++){
        id128b_1.id[i] = i + 40;
    }
    j2735_v2x_msgs::msg::Id128b id128b_2;
    for(int i = 0; i < id128b_2.id.size(); i++){
        id128b_2.id[i] = i + 60;
    }
    package.tcids.push_back(id128b_1);
    package.tcids.push_back(id128b_2);
    control.package = package;
    control.package_exists = PACKAGE_BOOL;
    // TrafficControlPackage END

    // TrafficControlParams START
    j2735_v2x_msgs::msg::TrafficControlParams params;
    params.regulatory = true;
    j2735_v2x_msgs::msg::TrafficControlVehClass bycicle;
    bycicle.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BICYCLE;
    params.vclasses.push_back(bycicle);
    j2735_v2x_msgs::msg::TrafficControlSchedule schedule;
    schedule.between_exists = true;
    schedule.dow_exists = true;
    schedule.repeat_exists = true;
    schedule.end_exists = true;
    j2735_v2x_msgs::msg::DailySchedule daily_schedule;
    daily_schedule.begin = 1;
    daily_schedule.duration = 2;
    schedule.between.push_back(daily_schedule);
    schedule.start = (unsigned)987123;
    schedule.end = (unsigned)654321;
    j2735_v2x_msgs::msg::DayOfWeek dow;
    dow.dow = {1,0,1,1,1,1,1};
    schedule.dow = dow;
    j2735_v2x_msgs::msg::RepeatParams repeat;
    repeat.offset = (unsigned)1;
    repeat.period = (unsigned)9;
    repeat.span = (unsigned)3;
    schedule.repeat = repeat;
    params.schedule = schedule;
    // TrafficControlDetails START
    j2735_v2x_msgs::msg::TrafficControlDetail detail;
    detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE;
    detail.latperm = {0, 1};

    // TrafficControlDetails END
    params.detail = detail;
    control.params = params;
    control.params_exists = PARAM_BOOL;
    // TrafficControlParams END

    // TrafficControlGeometry START
    j2735_v2x_msgs::msg::TrafficControlGeometry geometry;
    geometry.proj = "this is a sample proj string";
    geometry.datum = "this is a sample datum string";
    geometry.reftime = (unsigned)1713;
    geometry.reflon = 1;
    geometry.reflat = 10;
    geometry.refelv = 250;
    geometry.heading = (unsigned)1;
    j2735_v2x_msgs::msg::PathNode node;
    node.x = 1;
    node.y = 1;
    node.z_exists = true;
    node.z = 1;
    node.width_exists = true;
    node.width = 1;
    j2735_v2x_msgs::msg::PathNode node2;
    node2.x = 0;
    node2.y = 1;
    node2.z_exists = true;
    node2.z = 0;
    node2.width_exists = false;
    geometry.nodes.push_back(node);
    geometry.nodes.push_back(node2);
    control.geometry = geometry;
    control.geometry_exists = GEO_BOOL;    
    // TrafficControlGeometry END
    // TrafficControlMessageV01 END
    control_main.tcm_v01 = control;

    // TrafficControlMessage END
    auto res = worker->encode_geofence_control(control_main);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    auto res_decoded = worker->decode_geofence_control(res.get());

    if(res_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    j2735_v2x_msgs::msg::TrafficControlMessage result = res_decoded.get();
    EXPECT_EQ(result.tcm_v01.id.id, control_main.tcm_v01.id.id);
    EXPECT_EQ(result.tcm_v01.updated, control_main.tcm_v01.updated);
    EXPECT_EQ(control_main, result);

    // TrafficControlMessage END
    auto res2 = worker->encode_geofence_control(res_decoded.get());
    if(res2) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    auto res2_decoded = worker->decode_geofence_control(res2.get());

    if(res2_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
}

TEST(CppMessageTest, testEncodeControlMsgDetail1)
{
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);
    // ControlMessage START
    j2735_v2x_msgs::msg::TrafficControlMessage control_main;
    control_main.choice = j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01;
    // ControlMessageV01 START
    j2735_v2x_msgs::msg::TrafficControlMessageV01 control;
    j2735_v2x_msgs::msg::Id64b id64b;
    for(int i = 0; i < id64b.id.size(); i++){
        id64b.id[i] = 0;
    }
    control.reqid = id64b;
    control.reqseq = 111;
    j2735_v2x_msgs::msg::Id128b id128b;
    for(int i = 0; i < id128b.id.size(); i++){
        id128b.id[i] = 0;
    }
    control.id = id128b;
    control.msgnum = 5;
    control.msgtot = 6;
    control.updated = (unsigned)12345678;

    //==============================================
    // ENABLES OPTIONAL FIELDS IN CONTROL MESSAGE
    bool PACKAGE_BOOL = 1;
    bool PARAM_BOOL = 1;
    bool GEO_BOOL = 1;

    //=======================================
    // TrafficControlPackage START
    j2735_v2x_msgs::msg::TrafficControlPackage package;
    package.label = "avs";
    package.label_exists = true;
    for (auto i = 0; i < 2; i++) package.tcids.push_back(id128b);
    control.package = package;
    control.package_exists = PACKAGE_BOOL;
    // TrafficControlPackage END

    // TrafficControlParams START
    j2735_v2x_msgs::msg::TrafficControlParams params;
    params.regulatory = true;
    j2735_v2x_msgs::msg::TrafficControlVehClass bycicle;
    bycicle.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BICYCLE;
    params.vclasses.push_back(bycicle);
    j2735_v2x_msgs::msg::TrafficControlSchedule schedule;
    schedule.between_exists = true;
    schedule.dow_exists = true;
    schedule.repeat_exists = true;
    schedule.end_exists = true;
    j2735_v2x_msgs::msg::DailySchedule daily_schedule;
    daily_schedule.begin = 1;
    daily_schedule.duration = 2;
    schedule.between.push_back(daily_schedule);
    schedule.start = (unsigned)123456;
    schedule.end = (unsigned)123456;
    j2735_v2x_msgs::msg::DayOfWeek dow;
    dow.dow = {1,1,1,1,1,1,1};
    schedule.dow = dow;
    j2735_v2x_msgs::msg::RepeatParams repeat;
    repeat.offset = (unsigned)1;
    repeat.period = (unsigned)2;
    repeat.span = (unsigned)3;
    schedule.repeat = repeat;
    params.schedule = schedule;
    // TrafficControlDetails START
    j2735_v2x_msgs::msg::TrafficControlDetail detail;
    detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINPLATOONHDWY_CHOICE;
    detail.minplatoonhdwy = 10;

    //boost::array<uint8_t, 2UL> stuff {{0 ,1}}; //
    //detail.latperm = stuff;

    //ROS_WARN_STREAM("latperm og " << (int)detail.latperm.elems[0] << " | " << (int)detail.latperm.elems[1]);
    // TrafficControlDetails END
    params.detail = detail;
    control.params = params;
    control.params_exists = PARAM_BOOL;
    // TrafficControlParams END

    // TrafficControlGeometry START
    j2735_v2x_msgs::msg::TrafficControlGeometry geometry;
    geometry.proj = "this is a sample proj string";
    geometry.datum = "this is a sample datum string";
    geometry.reftime = (unsigned)1213;
    geometry.reflon = 1;
    geometry.reflat = 1;
    geometry.refelv = 1;
    geometry.heading = (unsigned)1;
    j2735_v2x_msgs::msg::PathNode node;
    node.x = 1;
    node.y = 1;
    node.z_exists = true;
    node.z = 1;
    node.width_exists = true;
    node.width = 1;
    geometry.nodes.push_back(node);
    geometry.nodes.push_back(node);
    control.geometry = geometry;
    control.geometry_exists = GEO_BOOL;    
    // TrafficControlGeometry END
    // TrafficControlMessageV01 END
    control_main.tcm_v01 = control;
    // TrafficControlMessage END
    auto res = worker->encode_geofence_control(control_main);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    auto res_decoded = worker->decode_geofence_control(res.get());

    if(res_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    j2735_v2x_msgs::msg::TrafficControlMessage result = res_decoded.get();
    ASSERT_EQ(result.tcm_v01.params.detail.choice, j2735_v2x_msgs::msg::TrafficControlDetail::MINPLATOONHDWY_CHOICE);
    ASSERT_EQ(result.tcm_v01.params.detail.minplatoonhdwy, 10);
}

TEST(CppMessageTest, testEncodeControlMsgDetail2)
{
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);
    // ControlMessage START
    j2735_v2x_msgs::msg::TrafficControlMessage control_main;
    control_main.choice = j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01;
    // ControlMessageV01 START
    j2735_v2x_msgs::msg::TrafficControlMessageV01 control;
    j2735_v2x_msgs::msg::Id64b id64b;
    for(int i = 0; i < id64b.id.size(); i++){
        id64b.id[i] = 0;
    }
    control.reqid = id64b;
    control.reqseq = 111;
    j2735_v2x_msgs::msg::Id128b id128b;
    for(int i = 0; i < id128b.id.size(); i++){
        id128b.id[i] = 0;
    }
    control.id = id128b;
    control.msgnum = 5;
    control.msgtot = 6;
    control.updated = (unsigned)12345678;

    //==============================================
    // ENABLES OPTIONAL FIELDS IN CONTROL MESSAGE
    bool PACKAGE_BOOL = 1;
    bool PARAM_BOOL = 1;
    bool GEO_BOOL = 1;

    //=======================================
    // TrafficControlPackage START
    j2735_v2x_msgs::msg::TrafficControlPackage package;
    package.label = "avs";
    package.label_exists = true;
    for (auto i = 0; i < 2; i++) package.tcids.push_back(id128b);
    control.package = package;
    control.package_exists = PACKAGE_BOOL;
    // TrafficControlPackage END

    // TrafficControlParams START
    j2735_v2x_msgs::msg::TrafficControlParams params;
    params.regulatory = true;
    j2735_v2x_msgs::msg::TrafficControlVehClass bycicle;
    bycicle.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BICYCLE;
    params.vclasses.push_back(bycicle);
    j2735_v2x_msgs::msg::TrafficControlSchedule schedule;
    schedule.between_exists = true;
    schedule.dow_exists = true;
    schedule.repeat_exists = true;
    schedule.end_exists = true;
    j2735_v2x_msgs::msg::DailySchedule daily_schedule;
    daily_schedule.begin = 1;
    daily_schedule.duration = 2;
    schedule.between.push_back(daily_schedule);
    schedule.start = (unsigned)123456;
    schedule.end = (unsigned)123456;
    j2735_v2x_msgs::msg::DayOfWeek dow;
    dow.dow = {1,1,1,1,1,1,1};
    schedule.dow = dow;
    j2735_v2x_msgs::msg::RepeatParams repeat;
    repeat.offset = (unsigned)1;
    repeat.period = (unsigned)2;
    repeat.span = (unsigned)3;
    schedule.repeat = repeat;
    params.schedule = schedule;
    // TrafficControlDetails START
    j2735_v2x_msgs::msg::TrafficControlDetail detail;
    detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXPLATOONSIZE_CHOICE;
    detail.maxplatoonsize = 10;

    // TrafficControlDetails END
    params.detail = detail;
    control.params = params;
    control.params_exists = PARAM_BOOL;
    // TrafficControlParams END
    // TrafficControlMessageV01 END
    control_main.tcm_v01 = control;
    // TrafficControlMessage END
    auto res = worker->encode_geofence_control(control_main);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    auto res_decoded = worker->decode_geofence_control(res.get());

    if(res_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    j2735_v2x_msgs::msg::TrafficControlMessage result = res_decoded.get();
    ASSERT_EQ(result.tcm_v01.params.detail.choice, j2735_v2x_msgs::msg::TrafficControlDetail::MAXPLATOONSIZE_CHOICE);
    ASSERT_EQ(result.tcm_v01.params.detail.maxplatoonsize, 10);
}

TEST(CppMessageTest, testDecodeRequestMsg1)
{
    std::vector<uint8_t> binar_input = {0, 244, 1, 0};
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    auto res = worker->decode_geofence_request(binar_input);
    if(res) EXPECT_TRUE(true);
    else EXPECT_TRUE(false);
}


TEST(CppMessageTest, testDecodeRequestMsg2)
{
    std::vector<uint8_t> binar_input = {0, 244, 37, 32, 0, 0, 0, 0, 0, 0, 0, 0, 44, 0, 0, 0, 0, 0, 146, 134, 180, 
                                        157, 31, 246, 180, 157, 32, 16, 0, 16, 0, 16, 0, 16, 0, 16, 0, 16, 0, 0};

    //  {0, 244, 37, 32, 0, 0, 0, 0, 0, 0, 0, 0, 44, 0, 0, 0, 0, 0, 146, 134, 180, 157, 31, 246, 180, 157, 32, 16, 0, 16, 0, 16, 0, 16, 0, 16, 0, 16, 0, 0};
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    auto res = worker->decode_geofence_request(binar_input);
    // if(res) EXPECT_TRUE(true);
    if (res){
        j2735_v2x_msgs::msg::TrafficControlRequest req = res.get();
        EXPECT_EQ(2344, req.tcr_v01.bounds[0].oldest);
        EXPECT_EQ(0, req.tcr_v01.bounds[0].reflon);
        EXPECT_EQ(1, req.tcr_v01.reqseq);
    }
    else EXPECT_TRUE(false);
}

TEST(CppMessageTest, testEncodeRequestMsg1)
{
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    j2735_v2x_msgs::msg::TrafficControlRequest request;
    request.choice = j2735_v2x_msgs::msg::TrafficControlRequest::RESERVED;
    auto res = worker->encode_geofence_request(request);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
}

TEST(CppMessageTest, testEncodeRequestMsg2)
{
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<cpp_message::Node>(options);

    j2735_v2x_msgs::msg::TrafficControlRequest request;
    request.choice = j2735_v2x_msgs::msg::TrafficControlRequest::TCRV01;

    j2735_v2x_msgs::msg::TrafficControlRequestV01 request1;
    j2735_v2x_msgs::msg::Id64b id64;
    for(int i = 0; i < id64.id.size(); i++){
        id64.id[i] = 0;
    }
    request1.reqid = id64;
    request1.reqseq = 1;
    request1.scale = 0;
    j2735_v2x_msgs::msg::TrafficControlBounds bounds;
    bounds.reflon = 0;
    bounds.reflat = 0;
    bounds.oldest = 2344;
    for(int i = 0; i < bounds.offsets.size(); i++){
        j2735_v2x_msgs::msg::OffsetPoint point;
        point.deltax = 0;
        point.deltay = 0;
        bounds.offsets[i] = point;
    } 
    request1.bounds.push_back(bounds);
    request.tcr_v01 = request1;
    auto res = worker->encode_geofence_request(request);
    if(res) EXPECT_TRUE(true);
    else
    {
        std::cout << "encoding failed!\n";
        EXPECT_TRUE(false);
    }
    auto res_decoded = worker->decode_geofence_request(res.get());
    if(res_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    EXPECT_EQ(request, res_decoded.get());
}
