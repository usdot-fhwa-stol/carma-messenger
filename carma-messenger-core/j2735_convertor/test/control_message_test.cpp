/*
 * Copyright (C) 2020 LEIDOS.
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

#include <gmock/gmock.h>
#include <j2735_convertor/control_message_convertor.h>
#include <ros/ros.h>
#include <ros/time.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;
using ::testing::ElementsAre;

namespace j2735_convertor
{

TEST(ControlMessage, convertControlMessageToCAV)
{
  j2735_msgs::ControlMessage in_msg;
  in_msg.version = "1.2.3.4";
  in_msg.id[0] = 0;
  in_msg.id[1] = 1;
  in_msg.id[2] = 2;
  in_msg.id[3] = 3;
  in_msg.id[4] = 4;
  in_msg.id[5] = 5;
  in_msg.id[6] = 6;
  in_msg.id[7] = 7;
  in_msg.id[8] = 8;
  in_msg.id[9] = 9;
  in_msg.id[10] = 10;
  in_msg.id[11] = 11;
  in_msg.id[12] = 12;
  in_msg.id[13] = 13;
  in_msg.id[14] = 14;
  in_msg.id[15] = 15;
  in_msg.updated = 500;
  j2735_msgs::VType vtype;
  vtype.vehicle_type = j2735_msgs::VType::PASSENGER_CAR;
  in_msg.vtypes.push_back(vtype);
  in_msg.schedule.between_exists = true; // Add flag to schedule to verify it has been modified
  in_msg.regulatory = true;
  in_msg.control_type.control_type = j2735_msgs::ControlType::CLOSED;
  in_msg.control_value.direction = j2735_msgs::ControlValue::REVERSE;
  in_msg.control_value_exists = true;
  in_msg.path_parts = 120;
  in_msg.proj = "my_proj";
  in_msg.datum = "my_datum";
  in_msg.time = 550;
  in_msg.latitude = 450000000;// 45 deg
  in_msg.longitude = 400000000;// 40 deg
  in_msg.altitude = 5000;// 500 m
  in_msg.heading = 300; // 30 deg
  j2735_msgs::Point p;
  p.x = 20;
  p.y = 40;
  p.z = 50;
  p.z_exists = true;
  p.width = 370;
  in_msg.points = {p};

  cav_msgs::ControlMessage out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  ASSERT_STREQ(out_msg.version.c_str(), in_msg.version.c_str());
  ASSERT_EQ(out_msg.id, in_msg.id);
  ASSERT_EQ(out_msg.updated.sec, 0);
  ASSERT_EQ(out_msg.updated.nsec, 500000000);
  ASSERT_EQ(out_msg.vtypes[0].vehicle_type, in_msg.vtypes[0].vehicle_type);
  ASSERT_EQ(out_msg.schedule.between_exists, in_msg.schedule.between_exists);
  ASSERT_EQ(out_msg.regulatory, in_msg.regulatory);
  ASSERT_EQ(out_msg.control_type.control_type, in_msg.control_type.control_type);
  ASSERT_EQ(out_msg.control_value.direction, in_msg.control_value.direction);
  ASSERT_EQ(out_msg.control_value_exists, in_msg.control_value_exists);
  ASSERT_EQ(out_msg.path_parts, in_msg.path_parts);
  ASSERT_STREQ(out_msg.proj.c_str(), in_msg.proj.c_str());
  ASSERT_STREQ(out_msg.datum.c_str(), in_msg.datum.c_str());
  ASSERT_EQ(out_msg.time.sec, 0);
  ASSERT_EQ(out_msg.time.nsec, 550000000);
  ASSERT_NEAR(out_msg.latitude, 45.0, 0.00000001);
  ASSERT_NEAR(out_msg.longitude, 40.0, 0.00000001);
  ASSERT_NEAR(out_msg.altitude, 500-428, 0.00001); // Need to subtract 428 to address offset
  ASSERT_NEAR(out_msg.heading, 30.0, 0.000001);

  constexpr double CM_TO_M = 0.01; 
  ASSERT_EQ(p.z_exists, out_msg.points[0].z_exists);
  ASSERT_NEAR((double)p.x * CM_TO_M, out_msg.points[0].x, 0.000001);
  ASSERT_NEAR((double)p.y * CM_TO_M, out_msg.points[0].y, 0.000001);
  ASSERT_NEAR((double)p.z * CM_TO_M, out_msg.points[0].z, 0.000001);
  ASSERT_NEAR((double)p.width * CM_TO_M, out_msg.points[0].width, 0.000001);
}

TEST(ControlMessage, convertScheduleToCAV)
{
  j2735_msgs::Schedule in_msg;
  in_msg.start = 10001; // ms
  in_msg.end = 40002; // ms
  in_msg.dow = {false, false, true, false, false, false, true};
  in_msg.dow_exists = true;

  j2735_msgs::DaySchedule in_day;
  in_day.start = 60; // min
  in_day.end = 180; // min
  in_day.utcoffset = 500; // min

  in_msg.between = in_day;
  in_msg.between_exists = true;

  j2735_msgs::ScheduleParams params;
  params.interval = 60;
  params.duration = 30;

  in_msg.repeat = params;
  in_msg.repeat_exists = true;

  cav_msgs::Schedule out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  // Evaluate Schedule Fields
  ASSERT_EQ(out_msg.start.sec, 10);
  ASSERT_EQ(out_msg.start.nsec, 1000000);
  ASSERT_EQ(out_msg.end.sec, 40);
  ASSERT_EQ(out_msg.end.nsec, 2000000);

  ASSERT_EQ(out_msg.dow[0], in_msg.dow[0]);
  ASSERT_EQ(out_msg.dow[1], in_msg.dow[1]);
  ASSERT_EQ(out_msg.dow[2], in_msg.dow[2]);
  ASSERT_EQ(out_msg.dow[3], in_msg.dow[3]);
  ASSERT_EQ(out_msg.dow[4], in_msg.dow[4]);
  ASSERT_EQ(out_msg.dow[5], in_msg.dow[5]);
  ASSERT_EQ(out_msg.dow[6], in_msg.dow[6]);

  ASSERT_EQ(out_msg.dow_exists, in_msg.dow_exists);
  ASSERT_EQ(out_msg.between_exists, in_msg.between_exists);
  ASSERT_EQ(out_msg.repeat_exists, in_msg.repeat_exists);

  // Evaluate DaySchedule
  ASSERT_EQ(out_msg.between.start.sec, 3600);
  ASSERT_EQ(out_msg.between.start.nsec, 0);
  ASSERT_EQ(out_msg.between.end.sec, 10800);
  ASSERT_EQ(out_msg.between.end.nsec, 0);
  ASSERT_EQ(out_msg.between.utcoffset.sec, 30000);
  ASSERT_EQ(out_msg.between.utcoffset.nsec, 0);

  // Evaluate ScheduleParams
  ASSERT_EQ(out_msg.repeat.duration.sec, 1800);
  ASSERT_EQ(out_msg.repeat.duration.nsec, 0);
  ASSERT_EQ(out_msg.repeat.interval.sec, 3600);
  ASSERT_EQ(out_msg.repeat.interval.nsec, 0);
}

TEST(ControlMessage, convertDayScheduleToCAV)
{
  j2735_msgs::DaySchedule in_msg;
  in_msg.start = 60; // min
  in_msg.end = 180; // min
  in_msg.utcoffset = 500; // min

  cav_msgs::DaySchedule out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  ASSERT_EQ(out_msg.start.sec, 3600);
  ASSERT_EQ(out_msg.start.nsec, 0);
  ASSERT_EQ(out_msg.end.sec, 10800);
  ASSERT_EQ(out_msg.end.nsec, 0);
  ASSERT_EQ(out_msg.utcoffset.sec, 30000);
  ASSERT_EQ(out_msg.utcoffset.nsec, 0);
}

TEST(ControlMessage, convertScheduleParamsToCAV)
{
  j2735_msgs::ScheduleParams in_msg;
  in_msg.interval = 60; // min
  in_msg.duration = 30; // min

  cav_msgs::ScheduleParams out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  ASSERT_EQ(out_msg.duration.sec, 1800);
  ASSERT_EQ(out_msg.duration.nsec, 0);
  ASSERT_EQ(out_msg.interval.sec, 3600);
  ASSERT_EQ(out_msg.interval.nsec, 0);
}

TEST(ControlMessage, convertPointToCAV)
{
  constexpr double CM_TO_M = 0.01; 
  // Test with z
  j2735_msgs::Point input;
  input.x = 20;
  input.y = 40;
  input.z = 50;
  input.z_exists = true;
  input.width = 370;

  cav_msgs::Point output;
  j2735_convertor::geofence_control::convert(input, output);
  ASSERT_EQ(input.z_exists, output.z_exists);
  ASSERT_NEAR((double)input.x * CM_TO_M, output.x, 0.000001);
  ASSERT_NEAR((double)input.y * CM_TO_M, output.y, 0.000001);
  ASSERT_NEAR((double)input.z * CM_TO_M, output.z, 0.000001);
  ASSERT_NEAR((double)input.width * CM_TO_M, output.width, 0.000001);

  // Test without z
  input.x = 22;
  input.y = 43;
  input.z = 0;
  input.z_exists = false;
  input.width = 370;

  j2735_convertor::geofence_control::convert(input, output);
  ASSERT_EQ(input.z_exists, output.z_exists);
  ASSERT_NEAR((double)input.x * CM_TO_M, output.x, 0.000001);
  ASSERT_NEAR((double)input.y * CM_TO_M, output.y, 0.000001);
  ASSERT_NEAR((double)input.z * CM_TO_M, output.z, 0.000001);
  ASSERT_NEAR((double)input.width * CM_TO_M, output.width, 0.000001);
}

TEST(ControlMessage, convertControlMessageToj2735)
{
  cav_msgs::ControlMessage in_msg;
  in_msg.version = "1.2.3.4";
  in_msg.id[0] = 0;
  in_msg.id[1] = 1;
  in_msg.id[2] = 2;
  in_msg.id[3] = 3;
  in_msg.id[4] = 4;
  in_msg.id[5] = 5;
  in_msg.id[6] = 6;
  in_msg.id[7] = 7;
  in_msg.id[8] = 8;
  in_msg.id[9] = 9;
  in_msg.id[10] = 10;
  in_msg.id[11] = 11;
  in_msg.id[12] = 12;
  in_msg.id[13] = 13;
  in_msg.id[14] = 14;
  in_msg.id[15] = 15;
  in_msg.updated = ros::Time(0, 500000000);
  j2735_msgs::VType vtype;
  vtype.vehicle_type = j2735_msgs::VType::PASSENGER_CAR;
  in_msg.vtypes.push_back(vtype);
  in_msg.schedule.between_exists = true; // Add flag to schedule to verify it has been modified
  in_msg.regulatory = true;
  in_msg.control_type.control_type = j2735_msgs::ControlType::CLOSED;
  in_msg.control_value.direction = j2735_msgs::ControlValue::REVERSE;
  in_msg.control_value_exists = true;
  in_msg.path_parts = 120;
  in_msg.proj = "my_proj";
  in_msg.datum = "my_datum";
  in_msg.time = ros::Time(10, 1000000);
  in_msg.latitude = 45.0;// 45 deg
  in_msg.longitude = 40.0;// 40 deg
  in_msg.altitude = 500.0;// 500 m
  in_msg.heading = 30.0; // 30 deg
  cav_msgs::Point p;
  p.x = 20.3;
  p.y = 40.4;
  p.z = 50.6;
  p.z_exists = true;
  p.width = 370.6;
  in_msg.points = {p};

  j2735_msgs::ControlMessage out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  ASSERT_STREQ(out_msg.version.c_str(), in_msg.version.c_str());
  ASSERT_THAT(out_msg.id, in_msg.id);
  ASSERT_EQ(out_msg.updated, 500);
  ASSERT_EQ(out_msg.vtypes[0].vehicle_type, in_msg.vtypes[0].vehicle_type);
  ASSERT_EQ(out_msg.schedule.between_exists, in_msg.schedule.between_exists);
  ASSERT_EQ(out_msg.regulatory, in_msg.regulatory);
  ASSERT_EQ(out_msg.control_type.control_type, in_msg.control_type.control_type);
  ASSERT_EQ(out_msg.control_value.direction, in_msg.control_value.direction);
  ASSERT_EQ(out_msg.control_value_exists, in_msg.control_value_exists);
  ASSERT_EQ(out_msg.path_parts, in_msg.path_parts);
  ASSERT_STREQ(out_msg.proj.c_str(), in_msg.proj.c_str());
  ASSERT_STREQ(out_msg.datum.c_str(), in_msg.datum.c_str());
  ASSERT_EQ(out_msg.time, 10001);
  ASSERT_EQ(out_msg.latitude, 450000000);
  ASSERT_EQ(out_msg.longitude, 400000000);
  ASSERT_EQ(out_msg.altitude, 5000+4280); // Need to add 428 to address offset
  ASSERT_EQ(out_msg.heading, 300);

  constexpr double M_TO_CM = 100.0; 
  ASSERT_EQ(in_msg.points[0].z_exists, out_msg.points[0].z_exists);
  ASSERT_EQ((int)(in_msg.points[0].x * M_TO_CM), out_msg.points[0].x);
  ASSERT_EQ((int)(in_msg.points[0].y * M_TO_CM), out_msg.points[0].y);
  ASSERT_EQ((int)(in_msg.points[0].z * M_TO_CM), out_msg.points[0].z);
  ASSERT_EQ((int)(in_msg.points[0].width * M_TO_CM), out_msg.points[0].width);
}

TEST(ControlMessage, convertScheduleToj2735)
{
  cav_msgs::Schedule in_msg;
  in_msg.start = ros::Time(10, 1000000);
  in_msg.end = ros::Time(40, 2000000);
  in_msg.dow = {false, false, true, false, false, false, true};
  in_msg.dow_exists = true;

  cav_msgs::DaySchedule in_day;
  in_day.start = ros::Duration(3600);
  in_day.end = ros::Duration(10800);
  in_day.utcoffset = ros::Duration(30000);

  in_msg.between = in_day;
  in_msg.between_exists = true;

  cav_msgs::ScheduleParams params;
  params.interval = ros::Duration(3600);
  params.duration = ros::Duration(1800);

  in_msg.repeat = params;
  in_msg.repeat_exists = true;

  j2735_msgs::Schedule out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  // Evaluate Schedule Fields
  ASSERT_EQ(out_msg.start, 10001);
  ASSERT_EQ(out_msg.end, 40002);

  ASSERT_EQ(out_msg.dow[0], in_msg.dow[0]);
  ASSERT_EQ(out_msg.dow[1], in_msg.dow[1]);
  ASSERT_EQ(out_msg.dow[2], in_msg.dow[2]);
  ASSERT_EQ(out_msg.dow[3], in_msg.dow[3]);
  ASSERT_EQ(out_msg.dow[4], in_msg.dow[4]);
  ASSERT_EQ(out_msg.dow[5], in_msg.dow[5]);
  ASSERT_EQ(out_msg.dow[6], in_msg.dow[6]);

  ASSERT_EQ(out_msg.dow_exists, in_msg.dow_exists);
  ASSERT_EQ(out_msg.between_exists, in_msg.between_exists);
  ASSERT_EQ(out_msg.repeat_exists, in_msg.repeat_exists);

  // Evaluate DaySchedule
  ASSERT_EQ(out_msg.between.start, 60);
  ASSERT_EQ(out_msg.between.end, 180);
  ASSERT_EQ(out_msg.between.utcoffset, 500);

  // Evaluate ScheduleParams
  ASSERT_EQ(out_msg.repeat.duration, 30);
  ASSERT_EQ(out_msg.repeat.interval, 60);
}

TEST(ControlMessage, convertDayScheduleToj2735)
{
  cav_msgs::DaySchedule in_msg;
  in_msg.start = ros::Duration(3600);
  in_msg.end = ros::Duration(10800);
  in_msg.utcoffset = ros::Duration(30000);

  j2735_msgs::DaySchedule out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  ASSERT_EQ(out_msg.start, 60);
  ASSERT_EQ(out_msg.end, 180);
  ASSERT_EQ(out_msg.utcoffset, 500);
}


TEST(ControlMessage, convertScheduleParamsToj2735)
{
  cav_msgs::ScheduleParams in_msg;
  in_msg.interval = ros::Duration(3600);
  in_msg.duration = ros::Duration(1800);

  j2735_msgs::ScheduleParams out_msg;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);

  ASSERT_EQ(out_msg.duration, 30);
  ASSERT_EQ(out_msg.interval, 60);
}

TEST(ControlMessage, convertPointToj2735)
{
  constexpr double M_TO_CM = 100.0; 
  // Test with z
  cav_msgs::Point input;
  input.x = 20.3;
  input.y = 40.4;
  input.z = 50.6;
  input.z_exists = true;
  input.width = 370.6;

  j2735_msgs::Point output;
  j2735_convertor::geofence_control::convert(input, output);
  ASSERT_EQ(input.z_exists, output.z_exists);
  ASSERT_EQ((int)(input.x * M_TO_CM), output.x);
  ASSERT_EQ((int)(input.y * M_TO_CM), output.y);
  ASSERT_EQ((int)(input.z * M_TO_CM), output.z);
  ASSERT_EQ((int)(input.width * M_TO_CM), output.width);

  // Test without z
  input.x = 22;
  input.y = 43;
  input.z = 0;
  input.z_exists = false;
  input.width = 370;

  j2735_convertor::geofence_control::convert(input, output);
  ASSERT_EQ(input.z_exists, output.z_exists);
  ASSERT_EQ((int)(input.x * M_TO_CM), output.x);
  ASSERT_EQ((int)(input.y * M_TO_CM), output.y);
  ASSERT_EQ((int)(input.z * M_TO_CM), output.z);
  ASSERT_EQ((int)(input.width * M_TO_CM), output.width);
}

}  // namespace j2735_convertor
