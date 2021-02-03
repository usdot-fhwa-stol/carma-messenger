/*
 * Copyright (C) 2020-2021 LEIDOS.
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
  constexpr int NUM_NODES = 10;

  /////////////////////////// J2735 TO CAV FUNCTIONS ///////////////////////////////

  j2735_msgs::DailySchedule create_j2735_DailySchedule(bool optional = true)
  {
    j2735_msgs::DailySchedule in_msg;
    in_msg.begin = 60; // min
    in_msg.duration = 180; // min
    return in_msg;
  }

  j2735_msgs::PathNode create_j2735_PathNode(bool optional = true)
  {
    j2735_msgs::PathNode in_msg;
    in_msg.x = 20;
    in_msg.y = 40;

    in_msg.z_exists = optional;
    in_msg.width_exists = optional;
    if(optional)
    {
      in_msg.z = 50;
      in_msg.width = 37;
    }
    return in_msg;
  }

  j2735_msgs::RepeatParams create_j2735_RepeatParams(bool optional = true)
  {
    j2735_msgs::RepeatParams in_msg;
    in_msg.offset = 60; // min
    in_msg.period = 360; // min
    in_msg.span = 180; // min
    return in_msg;
  }

  j2735_msgs::TrafficControlDetail create_j2735_TrafficControlDetail(bool optional = true)
  {
    j2735_msgs::TrafficControlDetail in_msg;
    in_msg.choice = j2735_msgs::TrafficControlDetail::CLOSED_CHOICE;
    in_msg.closed = j2735_msgs::TrafficControlDetail::TAPERLEFT;
    return in_msg;
  }

  j2735_msgs::TrafficControlGeometry create_j2735_TrafficControlGeometry(bool optional = true)
  {
    j2735_msgs::TrafficControlGeometry in_msg;
    in_msg.proj = "Project 1";
    in_msg.datum = "Datum 1";
    in_msg.reftime = 1000000;
    in_msg.reflon = -400000000;
    in_msg.reflat = 800000000;
    in_msg.refelv = 50000;
    in_msg.heading = 1800;

    j2735_msgs::PathNode node;
    for(int i = 0; i < NUM_NODES; i++)
    {
      node = create_j2735_PathNode();
      in_msg.nodes.push_back(node);
    }
    return in_msg;
  }

  j2735_msgs::TrafficControlSchedule create_j2735_TrafficControlSchedule(bool optional = true)
  {
    j2735_msgs::TrafficControlSchedule in_msg;
    in_msg.start = 1000000;
    
    in_msg.end_exists = optional;
    in_msg.dow_exists = optional;
    in_msg.between_exists = optional;
    in_msg.repeat_exists = optional;
    if(optional)
    {
      in_msg.end = 3000000;
      in_msg.dow.dow[0] = j2735_msgs::DayOfWeek::TUE;
      in_msg.dow.dow[1] = j2735_msgs::DayOfWeek::MON;
      in_msg.dow.dow[2] = j2735_msgs::DayOfWeek::WED;
      in_msg.dow.dow[3] = j2735_msgs::DayOfWeek::THU;
      in_msg.dow.dow[4] = j2735_msgs::DayOfWeek::FRI;
      in_msg.dow.dow[5] = j2735_msgs::DayOfWeek::SAT;
      in_msg.dow.dow[6] = j2735_msgs::DayOfWeek::SUN;

      j2735_msgs::DailySchedule ds;
      for(int i = 0; i < NUM_NODES; i++)
      {
        ds = create_j2735_DailySchedule();
        in_msg.between.push_back(ds);
      }

      in_msg.repeat = create_j2735_RepeatParams();
    }
    return in_msg;
  }

  j2735_msgs::TrafficControlParams create_j2735_TrafficControlParams(bool optional = true)
  {
    j2735_msgs::TrafficControlParams in_msg;

    // # vclasses SEQUENCE (SIZE(1..255)) OF TrafficControlVehClass,
    // j2735_msgs/TrafficControlVehClass[] vclasses
    j2735_msgs::TrafficControlVehClass tcvc;
    for(int i = 0; i < NUM_NODES; i++)
    {
      tcvc.vehicle_class = j2735_msgs::TrafficControlVehClass::BUS;
      in_msg.vclasses.push_back(tcvc);
    }
    
    // # schedule TrafficControlSchedule
    // cav_msgs/TrafficControlSchedule schedule
    in_msg.schedule = create_j2735_TrafficControlSchedule();

    // # regulatory BOOLEAN
    // bool regulatory
    in_msg.regulatory = false;

    // # detail TrafficControlDetail
    // cav_msgs/TrafficControlDetail detail
    in_msg.detail = create_j2735_TrafficControlDetail();
    return in_msg;
  }

  j2735_msgs::TrafficControlMessageV01 create_j2735_TrafficControlMessageV01(bool optional = true)
  {
    j2735_msgs::TrafficControlMessageV01 in_msg;
    // # reqid ::= Id64b
    // j2735_msgs/Id64b reqid
    in_msg.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};

    // # reqseq ::= INTEGER (0..255)
    // uint8 reqseq
    in_msg.reqseq = 77;

    // # msgtot INTEGER (0..65535), -- total expected traffic control message responses
    // uint16 msgtot
    in_msg.msgtot = 1;

    // # msgnum INTEGER (0..65535), -- message index for each response out of total responses
    // uint16 msgnum
    in_msg.msgnum = 0;

    // # id Id128b, -- unique traffic control id
    // j2735_msgs/Id128b reqid
    in_msg.id.id = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    // # updated EpochMins
    // time updated
    in_msg.updated = 1000000;

    // # package [0] TrafficControlPackage OPTIONAL, -- related traffic control ids
    // j2735_msgs/TrafficControlPackage package
    // bool package_exists
    in_msg.package_exists = optional;
    in_msg.package.label_exists = optional;

    // # params [1] TrafficControlParams OPTIONAL
    // cav_msgs/TrafficControlParams params
    // bool params_exists
    in_msg.params_exists = optional;

    // # geometry [2] TrafficControlGeometry OPTIONAL
    // cav_msgs/TrafficControlGeometry geometry
    // bool geometry_exists
    in_msg.geometry_exists = optional;

    if(optional)
    {
      in_msg.package.label = "This Is A Label";
      j2735_msgs::Id128b id128b;
      for(int i = 0; i < NUM_NODES; i++)
      {
        id128b.id = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
        in_msg.package.tcids.push_back(id128b);
      }

      in_msg.params = create_j2735_TrafficControlParams();
      in_msg.geometry = create_j2735_TrafficControlGeometry();
    }

    return in_msg;
  }

  j2735_msgs::TrafficControlMessage create_j2735_TrafficControlMessage(bool optional = true)
  {
    j2735_msgs::TrafficControlMessage in_msg;
    in_msg.choice = j2735_msgs::TrafficControlMessage::TCMV01;
    in_msg.tcmV01 = create_j2735_TrafficControlMessageV01();
    return in_msg;
  }

  /////////////////////////// CAV TO J2735 FUNCTIONS //////////////////////////////

  cav_msgs::DailySchedule create_cav_DailySchedule(bool optional = true)
  {
    cav_msgs::DailySchedule in_msg;
    in_msg.begin = ros::Duration(36000);
    in_msg.duration = ros::Duration(3600);
    return in_msg;
  }

  cav_msgs::PathNode create_cav_PathNode(bool optional = true)
  {
    cav_msgs::PathNode in_msg;
    in_msg.x = 20.0;
    in_msg.y = 40.0;

    in_msg.z_exists = optional;
    in_msg.width_exists = optional;
    if(optional)
    {
      in_msg.z = 50.0;
      in_msg.width = 1.0;
    }
    return in_msg;
  }

  cav_msgs::RepeatParams create_cav_RepeatParams(bool optional = true)
  {
    cav_msgs::RepeatParams in_msg;
    in_msg.offset = ros::Duration(3600);
    in_msg.period = ros::Duration(6000);
    in_msg.span = ros::Duration(1800);
    return in_msg;
  }

  cav_msgs::TrafficControlDetail create_cav_TrafficControlDetail(bool optional = true)
  {
    cav_msgs::TrafficControlDetail in_msg;
    in_msg.choice = cav_msgs::TrafficControlDetail::CLOSED_CHOICE;
    in_msg.closed = cav_msgs::TrafficControlDetail::TAPERLEFT;
    return in_msg;
  }

  cav_msgs::TrafficControlGeometry create_cav_TrafficControlGeometry(bool optional = true)
  {
    cav_msgs::TrafficControlGeometry in_msg;
    in_msg.proj = "Project 1";
    in_msg.datum = "Datum 1";
    in_msg.reftime = ros::Time(180000);
    in_msg.reflon = -40.0;
    in_msg.reflat = 80.0;
    in_msg.refelv = 500;
    in_msg.heading = 180;

    cav_msgs::PathNode node;
    for(int i = 0; i < NUM_NODES; i++)
    {
      node = create_cav_PathNode();
      in_msg.nodes.push_back(node);
    }
    return in_msg;
  }

  cav_msgs::TrafficControlSchedule create_cav_TrafficControlSchedule(bool optional = true)
  {
    cav_msgs::TrafficControlSchedule in_msg;
    in_msg.start = ros::Time(180000);
    
    in_msg.end_exists = optional;
    in_msg.dow_exists = optional;
    in_msg.between_exists = optional;
    in_msg.repeat_exists = optional;
    if(optional)
    {
      in_msg.end = ros::Time(180000);
      in_msg.dow.dow[0] = j2735_msgs::DayOfWeek::TUE;
      in_msg.dow.dow[1] = j2735_msgs::DayOfWeek::MON;
      in_msg.dow.dow[2] = j2735_msgs::DayOfWeek::WED;
      in_msg.dow.dow[3] = j2735_msgs::DayOfWeek::THU;
      in_msg.dow.dow[4] = j2735_msgs::DayOfWeek::FRI;
      in_msg.dow.dow[5] = j2735_msgs::DayOfWeek::SAT;
      in_msg.dow.dow[6] = j2735_msgs::DayOfWeek::SUN;

      cav_msgs::DailySchedule ds;
      for(int i = 0; i < NUM_NODES; i++)
      {
        ds = create_cav_DailySchedule();
        in_msg.between.push_back(ds);
      }

      in_msg.repeat = create_cav_RepeatParams();
    }
    return in_msg;
  }

  cav_msgs::TrafficControlParams create_cav_TrafficControlParams(bool optional = true)
  {
    cav_msgs::TrafficControlParams in_msg;

    // # vclasses SEQUENCE (SIZE(1..255)) OF TrafficControlVehClass,
    // j2735_msgs/TrafficControlVehClass[] vclasses
    j2735_msgs::TrafficControlVehClass tcvc;
    for(int i = 0; i < NUM_NODES; i++)
    {
      tcvc.vehicle_class = j2735_msgs::TrafficControlVehClass::BUS;
      in_msg.vclasses.push_back(tcvc);
    }
    
    // # schedule TrafficControlSchedule
    // cav_msgs/TrafficControlSchedule schedule
    in_msg.schedule = create_cav_TrafficControlSchedule();

    // # regulatory BOOLEAN
    // bool regulatory
    in_msg.regulatory = false;

    // # detail TrafficControlDetail
    // cav_msgs/TrafficControlDetail detail
    in_msg.detail = create_cav_TrafficControlDetail();
    return in_msg;
  }

  cav_msgs::TrafficControlMessageV01 create_cav_TrafficControlMessageV01(bool optional = true)
  {
    cav_msgs::TrafficControlMessageV01 in_msg;
    // # reqid ::= Id64b
    // uint8[8] reqid
    in_msg.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};

    // # reqseq ::= INTEGER (0..255)
    // uint8 reqseq
    in_msg.reqseq = 77;

    // # msgtot INTEGER (0..65535), -- total expected traffic control message responses
    // uint16 msgtot
    in_msg.msgtot = 1;

    // # msgnum INTEGER (0..65535), -- message index for each response out of total responses
    // uint16 msgnum
    in_msg.msgnum = 0;

    // # id Id128b, -- unique traffic control id
    // uint8[16] id
    in_msg.id.id = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    // # updated EpochMins
    // time updated
    in_msg.updated = ros::Time(180000);

    // # package [0] TrafficControlPackage OPTIONAL, -- related traffic control ids
    // j2735_msgs/TrafficControlPackage package
    // bool package_exists
    in_msg.package_exists = optional;

    // # params [1] TrafficControlParams OPTIONAL
    // cav_msgs/TrafficControlParams params
    // bool params_exists
    in_msg.params_exists = optional;

    // # geometry [2] TrafficControlGeometry OPTIONAL
    // cav_msgs/TrafficControlGeometry geometry
    // bool geometry_exists
    in_msg.geometry_exists = optional;

    if(optional)
    {
      in_msg.package.label = "This Is A Label";
      j2735_msgs::Id128b id128b;
      for(int i = 0; i < NUM_NODES; i++)
      {
        id128b.id = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
        in_msg.package.tcids.push_back(id128b);
      }

      in_msg.params = create_cav_TrafficControlParams();
      in_msg.geometry = create_cav_TrafficControlGeometry();

    }

    return in_msg;
  }

  cav_msgs::TrafficControlMessage create_cav_TrafficControlMessage(bool optional = true)
  {
    cav_msgs::TrafficControlMessage in_msg;
    in_msg.choice = cav_msgs::TrafficControlMessage::TCMV01;
    in_msg.tcmV01 = create_cav_TrafficControlMessageV01();
    return in_msg;
  }

  ////////////////////////////////// TEST FUNCTIONS ////////////////////////////////

  void test_DailySchedule(
    j2735_msgs::DailySchedule in_msg, 
    cav_msgs::DailySchedule out_msg)
  {
    ASSERT_EQ(out_msg.begin.sec, in_msg.begin * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.begin.nsec, 0);
    ASSERT_EQ(out_msg.duration.sec, in_msg.duration * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.duration.nsec, 0);
  }

  void test_PathNode(
    j2735_msgs::PathNode in_msg, 
    cav_msgs::PathNode out_msg)
  {
    ASSERT_NEAR((double)in_msg.x / units::CM_PER_M, out_msg.x, 0.000001);
    ASSERT_NEAR((double)in_msg.y / units::CM_PER_M, out_msg.y, 0.000001);
    ASSERT_EQ(in_msg.z_exists, out_msg.z_exists);
    ASSERT_EQ(in_msg.width_exists, out_msg.width_exists);

    if(in_msg.z_exists)
    {
      ASSERT_NEAR((double)in_msg.z / units::CM_PER_M, out_msg.z, 0.000001);
    }
    if(in_msg.width_exists)
    {
      ASSERT_NEAR((double)in_msg.width / units::CM_PER_M, out_msg.width, 0.000001);
    }
  }

  void test_RepeatParams(
    j2735_msgs::RepeatParams in_msg, 
    cav_msgs::RepeatParams out_msg)
  {
    ASSERT_EQ(out_msg.offset.sec, in_msg.offset * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.offset.nsec, 0);
    ASSERT_EQ(out_msg.period.sec, in_msg.period * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.period.nsec, 0);
    ASSERT_EQ(out_msg.span.sec, in_msg.span * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.span.nsec, 0);
  }

  void test_TrafficControlDetail(
    j2735_msgs::TrafficControlDetail in_msg, 
    cav_msgs::TrafficControlDetail out_msg)
  {
    ASSERT_EQ(out_msg.choice, in_msg.choice);
    ASSERT_EQ(out_msg.closed, in_msg.closed);
  }

  void test_TrafficControlGeometry(
    j2735_msgs::TrafficControlGeometry in_msg, 
    cav_msgs::TrafficControlGeometry out_msg)
  {
    ASSERT_EQ(out_msg.proj, in_msg.proj);
    ASSERT_EQ(out_msg.datum, in_msg.datum);
    ASSERT_EQ(out_msg.reftime.sec, in_msg.reftime * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.reftime.nsec, 0.);
    ASSERT_NEAR(in_msg.reflon, out_msg.reflon * units::TENTH_MICRO_DEG_PER_DEG, 0.000001);
    ASSERT_NEAR(in_msg.reflat, out_msg.reflat * units::TENTH_MICRO_DEG_PER_DEG, 0.000001);
    ASSERT_NEAR(in_msg.refelv - 4096, out_msg.refelv * units::DECA_M_PER_M, 0.001 );
    ASSERT_NEAR(in_msg.heading, out_msg.heading * units::DECA_S_PER_S, 0.000001);

    for(int i = 0; i < NUM_NODES; i++)
    {
      test_PathNode(in_msg.nodes[i], out_msg.nodes[i]);
    }
  }

  void test_TrafficControlSchedule(
    j2735_msgs::TrafficControlSchedule in_msg, 
    cav_msgs::TrafficControlSchedule out_msg)
  {
    ASSERT_EQ(out_msg.start.sec, in_msg.start * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.start.nsec, 0);

    ASSERT_EQ(out_msg.end_exists, in_msg.end_exists);
    if(in_msg.end_exists)
    {
      ASSERT_EQ(out_msg.end.sec, in_msg.end * units::SEC_PER_MIN);
      ASSERT_EQ(out_msg.end.nsec, 0);
    }
    
    ASSERT_EQ(out_msg.dow_exists, in_msg.dow_exists);
    if(in_msg.dow_exists)
    {
      ASSERT_EQ(out_msg.dow.dow, in_msg.dow.dow);
    }

    ASSERT_EQ(out_msg.between_exists, in_msg.between_exists);
    if(in_msg.between_exists)
    {
      for(int i = 0; i < NUM_NODES; i++)
      {
        test_DailySchedule(in_msg.between[i], out_msg.between[i]);
      }
    }
    
    ASSERT_EQ(out_msg.repeat_exists, in_msg.repeat_exists);
    if(in_msg.repeat_exists)
    {
      test_RepeatParams(in_msg.repeat, out_msg.repeat);
    }
  }
  
  void test_TrafficControlParams(
    j2735_msgs::TrafficControlParams in_msg, 
    cav_msgs::TrafficControlParams out_msg)
  {
    for(int i = 0; i < NUM_NODES; i++)
    {
      ASSERT_EQ(out_msg.vclasses[i].vehicle_class, in_msg.vclasses[i].vehicle_class);
    }

    test_TrafficControlSchedule(in_msg.schedule, out_msg.schedule);

    ASSERT_EQ(out_msg.regulatory, in_msg.regulatory);

  test_TrafficControlDetail(in_msg.detail, out_msg.detail);
  }

  void test_TrafficControlMessageV01(
    j2735_msgs::TrafficControlMessageV01 in_msg, 
    cav_msgs::TrafficControlMessageV01 out_msg)
  {
    ASSERT_EQ(in_msg.reqid.id, out_msg.reqid.id);
    ASSERT_EQ(in_msg.reqseq, out_msg.reqseq);
    ASSERT_EQ(in_msg.msgtot, out_msg.msgtot);
    ASSERT_EQ(in_msg.msgnum, out_msg.msgnum);
    ASSERT_EQ(in_msg.id.id, out_msg.id.id);
    ASSERT_EQ(out_msg.updated.sec, in_msg.updated * units::SEC_PER_MIN);
    ASSERT_EQ(out_msg.updated.nsec, 0);

    ASSERT_EQ(in_msg.package_exists, out_msg.package_exists);
    if(in_msg.package_exists)
    {
      ASSERT_EQ(in_msg.package.label, out_msg.package.label);
      for(int i = 0; i < NUM_NODES; i++)
      {
        ASSERT_EQ(in_msg.package.tcids[i].id, out_msg.package.tcids[i].id);
      }
    }

    ASSERT_EQ(in_msg.params_exists, out_msg.params_exists);
    if(in_msg.params_exists)
    {
      test_TrafficControlParams(in_msg.params, out_msg.params);
    }

    ASSERT_EQ(in_msg.geometry_exists, out_msg.geometry_exists);
    if(in_msg.geometry_exists)
    {
      test_TrafficControlGeometry(in_msg.geometry, out_msg.geometry);
    }
  }
  
  void test_TTrafficControlMessage(
    j2735_msgs::TrafficControlMessage in_msg, 
    cav_msgs::TrafficControlMessage out_msg)
  {
    ASSERT_EQ(out_msg.choice, in_msg.choice);
    test_TrafficControlMessageV01(in_msg.tcmV01, out_msg.tcmV01);
  }

TEST(ControlMessage, convertDailyScheduleToCAV)
{
  // Create variables
  j2735_msgs::DailySchedule in_msg;
  cav_msgs::DailySchedule out_msg;

  // Test
  in_msg = create_j2735_DailySchedule();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_DailySchedule(in_msg, out_msg);
}

TEST(ControlMessage, convertPathNodeToCAV)
{
  // Create variables
  j2735_msgs::PathNode in_msg;
  cav_msgs::PathNode out_msg;

  // Test with all optional elements
  in_msg = create_j2735_PathNode();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_PathNode(in_msg, out_msg);

  // Test without optional elements
  in_msg = create_j2735_PathNode(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_PathNode(in_msg, out_msg);
}

TEST(ControlMessage, convertRepeatParamsToCAV)
{
  // Create variables
  j2735_msgs::RepeatParams in_msg;
  cav_msgs::RepeatParams out_msg;

  // Test
  in_msg = create_j2735_RepeatParams(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_RepeatParams(in_msg, out_msg);
}

TEST(ControlMessage, convertTrafficControlDetailToCAV)
{
  j2735_msgs::TrafficControlDetail in_msg;
  cav_msgs::TrafficControlDetail out_msg;

  // Default Case Test
  in_msg.choice = 63;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // SIGNAL Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::SIGNAL_CHOICE;
  in_msg.signal = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(in_msg.signal, out_msg.signal);

  // STOP Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::STOP_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // YIELD Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::YIELD_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // NOTOWING Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::NOTOWING_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // RESTRICTED Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::RESTRICTED_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // CLOSED Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::CLOSED_CHOICE;
  in_msg.closed = j2735_msgs::TrafficControlDetail::TAPERLEFT;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.closed, in_msg.closed);

  // CHAINS Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::CHAINS_CHOICE;
  in_msg.chains = j2735_msgs::TrafficControlDetail::NO;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.chains, in_msg.chains);

  // DIRECTION Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::DIRECTION_CHOICE;
  in_msg.direction = j2735_msgs::TrafficControlDetail::FORWARD;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.direction, in_msg.direction);

  // LATAFFINITY Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::LATAFFINITY_CHOICE;
  in_msg.lataffinity = j2735_msgs::TrafficControlDetail::LEFT;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.lataffinity, in_msg.lataffinity);

  // LATPERM Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::LATPERM_CHOICE;
  for(int i = 0; i < 2; i++)
  {
    in_msg.latperm[i] = j2735_msgs::TrafficControlDetail::EMERGENCYONLY;
  }
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  for(int i = 0; i < 2; i++)
  {
    ASSERT_EQ(in_msg.latperm[i], out_msg.latperm[i]);
  }

  // PARKING Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::PARKING_CHOICE;
  in_msg.parking = j2735_msgs::TrafficControlDetail::PARALLEL;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.parking, in_msg.parking);

  // MINSPEED Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MINSPEED_CHOICE;
  in_msg.minspeed = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.minspeed, out_msg.minspeed * units::DECA_MPS_PER_MPS, 0.000001);

  // MAXSPEED Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
  in_msg.maxspeed = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxspeed, out_msg.maxspeed * units::DECA_MPS_PER_MPS, 0.000001);

  // MINHDWY Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MINHDWY_CHOICE;
  in_msg.minhdwy = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.minhdwy, out_msg.minhdwy * units::DECA_M_PER_M, 0.000001);

  // MAXVEHMASS Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE;
  in_msg.maxvehmass = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehmass, out_msg.maxvehmass, 0.000001);
  
  // MAXVEHHEIGHT Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE;
  in_msg.maxvehheight = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehheight, out_msg.maxvehheight * units::DECA_M_PER_M, 0.000001);
  
  // MAXVEHWIDTH Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE;
  in_msg.maxvehwidth = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehwidth, out_msg.maxvehwidth * units::DECA_M_PER_M, 0.000001);
  
  // MAXVEHLENGTH Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE;
  in_msg.maxvehlength = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehlength, out_msg.maxvehlength * units::DECA_M_PER_M, 0.000001);
  
  // MAXVEHAXLES Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE;
  in_msg.maxvehaxles = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehaxles, out_msg.maxvehaxles, 0.000001);
  
  // MINVEHOCC Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::MINVEHOCC_CHOICE;
  in_msg.minvehocc = 100;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.minvehocc, out_msg.minvehocc, 0.000001);
  
}

TEST(ControlMessage, convertTrafficControlGeometryToCAV)
{
  // Create variables
  j2735_msgs::TrafficControlGeometry in_msg;
  cav_msgs::TrafficControlGeometry out_msg;

  // Test
  in_msg = create_j2735_TrafficControlGeometry();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlGeometry(in_msg, out_msg);
}

TEST(ControlMessage, convertTrafficControlMessageToCAV)
{
  // Create variables
  j2735_msgs::TrafficControlMessage in_msg;
  cav_msgs::TrafficControlMessage out_msg;

  // Test Default Case
  in_msg.choice = 63;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // Test RESERVED
  in_msg.choice = j2735_msgs::TrafficControlMessage::RESERVED;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // Test TCMV01
  in_msg.choice = j2735_msgs::TrafficControlMessage::TCMV01;
  in_msg.tcmV01 = create_j2735_TrafficControlMessageV01();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  test_TrafficControlMessageV01(in_msg.tcmV01, out_msg.tcmV01);
}

TEST(ControlMessage, convertTrafficControlMessageV01ToCAV)
{
  // Create variables
  j2735_msgs::TrafficControlMessageV01 in_msg;
  cav_msgs::TrafficControlMessageV01 out_msg;

  // Test with all optional elements
  in_msg = create_j2735_TrafficControlMessageV01();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlMessageV01(in_msg, out_msg);

  // Test without optional elements
  in_msg = create_j2735_TrafficControlMessageV01(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlMessageV01(in_msg, out_msg);
}

TEST(ControlMessage, convertTrafficControlParamsToCAV)
{
  // Create variables
  j2735_msgs::TrafficControlParams in_msg;
  cav_msgs::TrafficControlParams out_msg;

  // Test
  in_msg = create_j2735_TrafficControlParams();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlParams(in_msg, out_msg);
}

TEST(ControlMessage, convertTrafficControlScheduleToCAV)
{
  // Create variables
  j2735_msgs::TrafficControlSchedule in_msg;
  cav_msgs::TrafficControlSchedule out_msg;

  // Test with all optional elements
  in_msg = create_j2735_TrafficControlSchedule();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlSchedule(in_msg, out_msg);

  // Test without optional elements
  in_msg = create_j2735_TrafficControlSchedule(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlSchedule(in_msg, out_msg);
}

////////////// Convert CAV messages to J2735 /////////////////////////

TEST(ControlMessage, convertDailyScheduleToJ2735)
{
  // Create variables
  cav_msgs::DailySchedule in_msg;
  j2735_msgs::DailySchedule out_msg;

  // Test
  in_msg = create_cav_DailySchedule();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_DailySchedule(out_msg, in_msg);
}

TEST(ControlMessage, convertPathNodeToJ2735)
{
  // Create variables
  cav_msgs::PathNode in_msg;
  j2735_msgs::PathNode out_msg;

  // Test with all optional elements
  in_msg = create_cav_PathNode();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_PathNode(out_msg, in_msg);

  // Test without optional elements
  in_msg = create_cav_PathNode(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_PathNode(out_msg, in_msg);
}

TEST(ControlMessage, convertRepeatParamsToJ2735)
{
  // Create variables
  cav_msgs::RepeatParams in_msg;
  j2735_msgs::RepeatParams out_msg;

  // Test
  in_msg = create_cav_RepeatParams(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_RepeatParams(out_msg, in_msg);
}

TEST(ControlMessage, convertTrafficControlDetailToJ2735)
{
  cav_msgs::TrafficControlDetail in_msg;
  j2735_msgs::TrafficControlDetail out_msg;

  // Default Case Test
  in_msg.choice = 63;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // SIGNAL Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::SIGNAL_CHOICE;
  in_msg.signal = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(in_msg.signal, out_msg.signal);

  // STOP Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::STOP_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // YIELD Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::YIELD_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // NOTOWING Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::NOTOWING_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // RESTRICTED Test
  in_msg.choice = j2735_msgs::TrafficControlDetail::RESTRICTED_CHOICE;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // CLOSED Test
  in_msg.choice = cav_msgs::TrafficControlDetail::CLOSED_CHOICE;
  in_msg.closed = cav_msgs::TrafficControlDetail::TAPERLEFT;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.closed, in_msg.closed);

  // CHAINS Test
  in_msg.choice = cav_msgs::TrafficControlDetail::CHAINS_CHOICE;
  in_msg.chains = cav_msgs::TrafficControlDetail::NO;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.chains, in_msg.chains);

  // DIRECTION Test
  in_msg.choice = cav_msgs::TrafficControlDetail::DIRECTION_CHOICE;
  in_msg.direction = cav_msgs::TrafficControlDetail::FORWARD;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.direction, in_msg.direction);

  // LATAFFINITY Test
  in_msg.choice = cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE;
  in_msg.lataffinity = cav_msgs::TrafficControlDetail::LEFT;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.lataffinity, in_msg.lataffinity);

  // LATPERM Test
  in_msg.choice = cav_msgs::TrafficControlDetail::LATPERM_CHOICE;
  for(int i = 0; i < 2; i++)
  {
    in_msg.latperm[i] = cav_msgs::TrafficControlDetail::EMERGENCYONLY;
  }
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  for(int i = 0; i < 2; i++)
  {
    ASSERT_EQ(in_msg.latperm[i], out_msg.latperm[i]);
  }

  // PARKING Test
  in_msg.choice = cav_msgs::TrafficControlDetail::PARKING_CHOICE;
  in_msg.parking = cav_msgs::TrafficControlDetail::PARALLEL;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_EQ(out_msg.parking, in_msg.parking);

  // MINSPEED Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MINSPEED_CHOICE;
  in_msg.minspeed = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.minspeed, (float)out_msg.minspeed / units::DECA_MPS_PER_MPS, 0.000001);

  // MAXSPEED Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
  in_msg.maxspeed = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxspeed, (float)out_msg.maxspeed / units::DECA_MPS_PER_MPS, 0.000001);

  // MINHDWY Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MINHDWY_CHOICE;
  in_msg.minhdwy = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.minhdwy, (float)out_msg.minhdwy / units::DECA_M_PER_M, 0.000001);

  // MAXVEHMASS Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE;
  in_msg.maxvehmass = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehmass, out_msg.maxvehmass, 0.000001);
  
  // MAXVEHHEIGHT Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE;
  in_msg.maxvehheight = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehheight, (float)out_msg.maxvehheight / units::DECA_M_PER_M, 0.000001);
  
  // MAXVEHWIDTH Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE;
  in_msg.maxvehwidth = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehwidth, (float)out_msg.maxvehwidth / units::DECA_M_PER_M, 0.000001);
  
  // MAXVEHLENGTH Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE;
  in_msg.maxvehlength = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehlength, (float)out_msg.maxvehlength / units::DECA_M_PER_M, 0.000001);
  
  // MAXVEHAXLES Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE;
  in_msg.maxvehaxles = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.maxvehaxles, out_msg.maxvehaxles, 0.000001);
  
  // MINVEHOCC Test
  in_msg.choice = cav_msgs::TrafficControlDetail::MINVEHOCC_CHOICE;
  in_msg.minvehocc = 10;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  ASSERT_NEAR(in_msg.minvehocc, out_msg.minvehocc, 0.000001);
  
}

TEST(ControlMessage, convertTrafficControlGeometryToJ2735)
{
  // Create variables
  cav_msgs::TrafficControlGeometry in_msg;
  j2735_msgs::TrafficControlGeometry out_msg;

  // Test
  in_msg = create_cav_TrafficControlGeometry();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlGeometry(out_msg, in_msg);
}

TEST(ControlMessage, convertTrafficControlMessageToJ2735)
{
  // Create variables
  cav_msgs::TrafficControlMessage in_msg;
  j2735_msgs::TrafficControlMessage out_msg;

  // Test Default Case
  in_msg.choice = 63;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // Test RESERVED
  in_msg.choice = cav_msgs::TrafficControlMessage::RESERVED;
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);

  // Test TCMV01
  in_msg.choice = cav_msgs::TrafficControlMessage::TCMV01;
  in_msg.tcmV01 = create_cav_TrafficControlMessageV01();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  ASSERT_EQ(out_msg.choice, in_msg.choice);
  test_TrafficControlMessageV01(out_msg.tcmV01, in_msg.tcmV01);
}

TEST(ControlMessage, convertTrafficControlMessageV01ToJ2735)
{
  // Create variables
  cav_msgs::TrafficControlMessageV01 in_msg;
  j2735_msgs::TrafficControlMessageV01 out_msg;

  // Test with all optional elements
  in_msg = create_cav_TrafficControlMessageV01();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlMessageV01(out_msg, in_msg);

  // Test without optional elements
  in_msg = create_cav_TrafficControlMessageV01(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlMessageV01(out_msg, in_msg);
}

TEST(ControlMessage, convertTrafficControlParamsToJ2735)
{
  // Create variables
  cav_msgs::TrafficControlParams in_msg;
  j2735_msgs::TrafficControlParams out_msg;

  // Test
  in_msg = create_cav_TrafficControlParams();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlParams(out_msg, in_msg);
}

TEST(ControlMessage, convertTrafficControlScheduleToJ2735)
{
  // Create variables
  cav_msgs::TrafficControlSchedule in_msg;
  j2735_msgs::TrafficControlSchedule out_msg;

  // Test with all optional elements
  in_msg = create_cav_TrafficControlSchedule();
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlSchedule(out_msg, in_msg);

  // Test without optional elements
  in_msg = create_cav_TrafficControlSchedule(false);
  j2735_convertor::geofence_control::convert(in_msg, out_msg);
  test_TrafficControlSchedule(out_msg, in_msg);
}
}  // namespace j2735_convertor
