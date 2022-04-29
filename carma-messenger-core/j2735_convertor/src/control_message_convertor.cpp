/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <j2735_convertor/control_message_convertor.hpp>
#include <ros/ros.h>
#include <iostream>

namespace j2735_convertor
{
namespace geofence_control
{
////
// Convert j2735_msgs to cav_msgs
////

void convert(const j2735_v2x_msgs::msg::DailySchedule& in_msg, carma_v2x_msgs::msg::DailySchedule& out_msg)
{
  out_msg.begin = ros::Duration(in_msg.begin * units::SEC_PER_MIN, 0);
  out_msg.duration = ros::Duration(in_msg.duration * units::SEC_PER_MIN,0);
}

void convert(const j2735_v2x_msgs::msg::PathNode& in_msg, carma_v2x_msgs::msg::PathNode& out_msg)
{
  out_msg.x = (double)in_msg.x / units::CM_PER_M;
  out_msg.y = (double)in_msg.y / units::CM_PER_M;

  out_msg.z_exists = in_msg.z_exists;
  if(out_msg.z_exists)
  {
    out_msg.z = (double)in_msg.z / units::CM_PER_M;
  }

  out_msg.width_exists = in_msg.width_exists;
  if(out_msg.width_exists)
  {
    out_msg.width = (float)in_msg.width / units::CM_PER_M;
  }
}

void convert(const j2735_v2x_msgs::msg::RepeatParams& in_msg, carma_v2x_msgs::msg::RepeatParams& out_msg)
{
  out_msg.offset = ros::Duration(in_msg.offset * units::SEC_PER_MIN, 0);
  out_msg.period = ros::Duration(in_msg.period * units::SEC_PER_MIN, 0);
  out_msg.span = ros::Duration(in_msg.span * units::SEC_PER_MIN, 0);
}

void convert(const j2735_v2x_msgs::msg::TrafficControlDetail& in_msg, carma_v2x_msgs::msg::TrafficControlDetail& out_msg)
{
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case j2735_v2x_msgs::msg::TrafficControlDetail::SIGNAL_CHOICE : 
      out_msg.signal = in_msg.signal;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::STOP_CHOICE : 
      // Not implemented yet
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::YIELD_CHOICE : 
      // Not implemented yet
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::NOTOWING_CHOICE : 
      // Not implemented yet
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::RESTRICTED_CHOICE : 
      // Not implemented yet
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE : 
      out_msg.closed = in_msg.closed;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::CHAINS_CHOICE : 
      out_msg.chains = in_msg.chains;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE : 
      out_msg.direction = in_msg.direction;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE : 
      out_msg.lataffinity = in_msg.lataffinity;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE : 
      for(int i = 0; i < 2; i++)
      {
        out_msg.latperm[i] = in_msg.latperm[i];
      }
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::PARKING_CHOICE : 
      out_msg.parking = in_msg.parking;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE : 
      out_msg.minspeed = (float)in_msg.minspeed / units::DECA_MPS_PER_MPS;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE : 
      out_msg.maxspeed = (float)in_msg.maxspeed / units::DECA_MPS_PER_MPS;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE : 
      out_msg.minhdwy = (float)in_msg.minhdwy / units::DECA_M_PER_M;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHMASS_CHOICE : 
      out_msg.maxvehmass = (float)in_msg.maxvehmass;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHHEIGHT_CHOICE : 
      out_msg.maxvehheight = (float)in_msg.maxvehheight / units::DECA_M_PER_M;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHWIDTH_CHOICE : 
      out_msg.maxvehwidth = (float)in_msg.maxvehwidth / units::DECA_M_PER_M;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHLENGTH_CHOICE : 
      out_msg.maxvehlength = (float)in_msg.maxvehlength / units::DECA_M_PER_M;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHAXLES_CHOICE : 
      out_msg.maxvehaxles = in_msg.maxvehaxles;
      break;
    case j2735_v2x_msgs::msg::TrafficControlDetail::MINVEHOCC_CHOICE : 
      out_msg.minvehocc = in_msg.minvehocc;
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const j2735_v2x_msgs::msg::TrafficControlGeometry& in_msg, carma_v2x_msgs::msg::TrafficControlGeometry& out_msg)
{
  out_msg.proj = in_msg.proj;
  out_msg.datum = in_msg.datum;

  int32_t sec = 0;
  // Use conditional to avoid issues from casting in std::min
  if ((double)in_msg.reftime* units::SEC_PER_MIN >  (double)std::numeric_limits<int32_t>::max()) 
  {
    sec = std::numeric_limits<int32_t>::max();
  } 
  else 
  {
    sec = in_msg.reftime* units::SEC_PER_MIN;
    
  }

  out_msg.reftime = ros::Time(sec, 0);
  out_msg.reflon = (double)in_msg.reflon / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.reflat = (double)in_msg.reflat / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.refelv = (float)in_msg.refelv / units::DECA_M_PER_M - (float) 409.6; //handle offset
  out_msg.heading = (float)in_msg.heading / units::DECA_S_PER_S;

  for (auto in_node : in_msg.nodes)
  {
    carma_v2x_msgs::msg::PathNode out_node;
    convert(in_node, out_node);
    out_msg.nodes.push_back(out_node);
  }
}

void convert(const j2735_v2x_msgs::msg::TrafficControlMessage& in_msg, carma_v2x_msgs::msg::TrafficControlMessage& out_msg)
{
  // uint8 choice
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case j2735_v2x_msgs::msg::TrafficControlMessage::RESERVED : 
      // Not implemented yet
      break;
    case j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01 : 
      convert(in_msg.tcm_v01, out_msg.tcm_v01);
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const j2735_v2x_msgs::msg::TrafficControlMessageV01& in_msg, carma_v2x_msgs::msg::TrafficControlMessageV01& out_msg)
{
  // # reqid ::= Id64b
  // j2735_msgs/Id64b reqid
  out_msg.reqid = in_msg.reqid;

  // # reqseq ::= INTEGER (0..255)
  // uint8 reqseq
  out_msg.reqseq = in_msg.reqseq;

  // # msgtot INTEGER (0..65535), -- total expected traffic control message responses
  // uint16 msgtot
  out_msg.msgtot = in_msg.msgtot;

  // # msgnum INTEGER (0..65535), -- message index for each response out of total responses
  // uint16 msgnum
  out_msg.msgnum = in_msg.msgnum;

  // # id Id128b, -- unique traffic control id
  // j2735_msgs/Id128b id
  out_msg.id = in_msg.id;

  // # updated EpochMins
  // time updated
  out_msg.updated = ros::Time(in_msg.updated * units::SEC_PER_MIN, 0);

  // # package [0] TrafficControlPackage OPTIONAL, -- related traffic control ids
  // j2735_msgs/TrafficControlPackage package
  // bool package_exists
  out_msg.package_exists = in_msg.package_exists;
  if(out_msg.package_exists)
  {
    out_msg.package = in_msg.package;
  }

  // # params [1] TrafficControlParams OPTIONAL
  // cav_msgs/TrafficControlParams params
  // bool params_exists
  out_msg.params_exists = in_msg.params_exists;
  if(out_msg.params_exists)
  {
    convert(in_msg.params, out_msg.params);
  }

  // # geometry [2] TrafficControlGeometry OPTIONAL
  // cav_msgs/TrafficControlGeometry geometry
  // bool geometry_exists
  out_msg.geometry_exists = in_msg.geometry_exists;
  if(out_msg.geometry_exists)
  {
    convert(in_msg.geometry, out_msg.geometry);
  }
}

void convert(const j2735_v2x_msgs::msg::TrafficControlParams& in_msg, carma_v2x_msgs::msg::TrafficControlParams& out_msg)
{
  // j2735_msgs/TrafficControlVehClass[] vclasses
  out_msg.vclasses = in_msg.vclasses;
  
  // # schedule TrafficControlSchedule
  // carma_v2x_msgs/msg/trafficcontrolschedule s.hppedule
  convert(in_msg.schedule, out_msg.schedule);

  // # regulatory BOOLEAN
  // bool regulatory
  out_msg.regulatory = in_msg.regulatory;

  // # detail TrafficControlDetail
  // cav_msgs/TrafficControlDetail detail
  convert(in_msg.detail, out_msg.detail);
}

void convert(const j2735_v2x_msgs::msg::TrafficControlSchedule& in_msg, carma_v2x_msgs::msg::TrafficControlSchedule& out_msg)
{
  out_msg.start = ros::Time(in_msg.start * units::SEC_PER_MIN, 0);

  out_msg.end_exists = in_msg.end_exists;
  if(out_msg.end_exists)
  {
    int32_t sec = 0;
    // Use conditional to avoid issues from casting in std::min
    if ((double)in_msg.end* units::SEC_PER_MIN >  (double)std::numeric_limits<int32_t>::max()) 
    {
      sec = std::numeric_limits<int32_t>::max();
      
    } 
    else 
    {
      sec = in_msg.end* units::SEC_PER_MIN;
      
    }
    
    out_msg.end = ros::Time(sec, 0);
    
  }

  out_msg.dow_exists = in_msg.dow_exists;
  if(out_msg.dow_exists)
  {
    out_msg.dow = in_msg.dow;
  }

  out_msg.between_exists = in_msg.between_exists;
  if(out_msg.between_exists)
  {
    for (auto in_between : in_msg.between)
    {
      carma_v2x_msgs::msg::DailySchedule out_between;
      convert(in_between, out_between);
      out_msg.between.push_back(out_between);
    }
  }

  out_msg.repeat_exists = in_msg.repeat_exists;
  if(out_msg.repeat_exists)
  {
    convert(in_msg.repeat, out_msg.repeat);
  }
}

////
// Convert cav_msgs to j2735_msgs
////

void convert(const carma_v2x_msgs::msg::DailySchedule& in_msg, j2735_v2x_msgs::msg::DailySchedule& out_msg)
{
  ros::Time begin(in_msg.begin.sec , in_msg.begin.nanosec);
  out_msg.begin = begin.seconds() / units::SEC_PER_MIN;
  ros::Time duration(in_msg.duration.sec  , in_msg.duration.nanosec);
  out_msg.duration = duration.seconds() / units::SEC_PER_MIN;
}

void convert(const carma_v2x_msgs::msg::PathNode& in_msg, j2735_v2x_msgs::msg::PathNode& out_msg)
{
  out_msg.x = (int16_t)(in_msg.x * units::CM_PER_M);
  out_msg.y = (int16_t)(in_msg.y * units::CM_PER_M);

  out_msg.z_exists = in_msg.z_exists;
  if(out_msg.z_exists)
  {
    out_msg.z = (int16_t)(in_msg.z * units::CM_PER_M);
  }

  out_msg.width_exists = in_msg.width_exists;
  if(out_msg.width_exists)
  {
    out_msg.width = (int8_t)(in_msg.width * units::CM_PER_M);
  }
}

void convert(const carma_v2x_msgs::msg::RepeatParams& in_msg, j2735_v2x_msgs::msg::RepeatParams& out_msg)
{
  ros::Time offset(in_msg.offset.sec , in_msg.offset.nanosec);
  out_msg.offset =  offset.seconds() / units::SEC_PER_MIN;
  ros::Time period(in_msg.period.sec  , in_msg.period.nanosec);
  out_msg.period = period.seconds()/ units::SEC_PER_MIN;
  ros::Time span(in_msg.span.sec , in_msg.span.nanosec);
  out_msg.span = span.seconds()/ units::SEC_PER_MIN;
}

void convert(const carma_v2x_msgs::msg::TrafficControlDetail& in_msg, j2735_v2x_msgs::msg::TrafficControlDetail& out_msg)
{
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case carma_v2x_msgs::msg::TrafficControlDetail::SIGNAL_CHOICE : 
      out_msg.signal = in_msg.signal;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::STOP_CHOICE : 
      // Not implemented yet
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::YIELD_CHOICE : 
      // Not implemented yet
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::NOTOWING_CHOICE : 
      // Not implemented yet
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::RESTRICTED_CHOICE : 
      // Not implemented yet
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE : 
      out_msg.closed = in_msg.closed;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::CHAINS_CHOICE : 
      out_msg.chains = in_msg.chains;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE : 
      out_msg.direction = in_msg.direction;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE : 
      out_msg.lataffinity = in_msg.lataffinity;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE : 
      for(int i = 0; i < 2; i++)
      {
        out_msg.latperm[i] = in_msg.latperm[i];
      }
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::PARKING_CHOICE : 
      out_msg.parking = in_msg.parking;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE : 
      out_msg.minspeed = (uint16_t)(in_msg.minspeed * units::DECA_MPS_PER_MPS);
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE : 
      out_msg.maxspeed = (uint16_t)(in_msg.maxspeed * units::DECA_MPS_PER_MPS);
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE : 
      out_msg.minhdwy = (uint16_t)(in_msg.minhdwy * units::DECA_M_PER_M);
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MAXVEHMASS_CHOICE : 
      out_msg.maxvehmass = (uint16_t)in_msg.maxvehmass;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MAXVEHHEIGHT_CHOICE : 
      out_msg.maxvehheight = (uint8_t)(in_msg.maxvehheight * units::DECA_M_PER_M);
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MAXVEHWIDTH_CHOICE : 
      out_msg.maxvehwidth = (uint8_t)(in_msg.maxvehwidth * units::DECA_M_PER_M);
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MAXVEHLENGTH_CHOICE : 
      out_msg.maxvehlength = (uint8_t)(in_msg.maxvehlength * units::DECA_M_PER_M);
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MAXVEHAXLES_CHOICE : 
      out_msg.maxvehaxles = in_msg.maxvehaxles;
      break;
    case carma_v2x_msgs::msg::TrafficControlDetail::MINVEHOCC_CHOICE : 
      out_msg.minvehocc = in_msg.minvehocc;
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const carma_v2x_msgs::msg::TrafficControlGeometry& in_msg, j2735_v2x_msgs::msg::TrafficControlGeometry& out_msg)
{
  out_msg.proj = in_msg.proj;
  out_msg.datum = in_msg.datum;
  ros::Time reftime(in_msg.reftime.sec , in_msg.reftime.nanosec);
  out_msg.reftime = reftime.seconds() / units::SEC_PER_MIN;
  out_msg.reflon = (int32_t)(in_msg.reflon * units::TENTH_MICRO_DEG_PER_DEG);
  out_msg.reflat = (int32_t)(in_msg.reflat * units::TENTH_MICRO_DEG_PER_DEG);
  out_msg.refelv = (int32_t)((in_msg.refelv + 409.6) * units::DECA_M_PER_M); // handle offset
  out_msg.heading = (int32_t)(in_msg.heading * units::DECA_S_PER_S);

  for (auto in_node : in_msg.nodes)
  {
    j2735_v2x_msgs::msg::PathNode out_node;
    convert(in_node, out_node);
    out_msg.nodes.push_back(out_node);
  }
}

void convert(const carma_v2x_msgs::msg::TrafficControlMessage& in_msg, j2735_v2x_msgs::msg::TrafficControlMessage& out_msg)
{
  // uint8 choice
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case carma_v2x_msgs::msg::TrafficControlMessage::RESERVED : 
      // Not implemented yet
      break;
    case carma_v2x_msgs::msg::TrafficControlMessage::TCMV01 : 
      convert(in_msg.tcm_v01, out_msg.tcm_v01);
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const carma_v2x_msgs::msg::TrafficControlMessageV01& in_msg, j2735_v2x_msgs::msg::TrafficControlMessageV01& out_msg)
{
  // # reqid ::= Id64b
  // j2735_msgs/Id64b reqid
  out_msg.reqid = in_msg.reqid;

  // # reqseq ::= INTEGER (0..255)
  // uint8 reqseq
  out_msg.reqseq = in_msg.reqseq;

  // # msgtot INTEGER (0..65535), -- total expected traffic control message responses
  // uint16 msgtot
  out_msg.msgtot = in_msg.msgtot;

  // # msgnum INTEGER (0..65535), -- message index for each response out of total responses
  // uint16 msgnum
  out_msg.msgnum = in_msg.msgnum;

  // # id Id128b, -- unique traffic control id
  // j2735_msgs/Id128b id
  out_msg.id = in_msg.id;

  // # updated EpochMins
  // time updated
  ros::Time updated(in_msg.updated.sec , in_msg.updated.nanosec);
  out_msg.updated = updated.seconds() / units::SEC_PER_MIN;

  // # package [0] TrafficControlPackage OPTIONAL, -- related traffic control ids
  // j2735_msgs/TrafficControlPackage package
  // bool package_exists
  out_msg.package_exists = in_msg.package_exists;
  if(out_msg.package_exists)
  {
    out_msg.package = in_msg.package;
  }

  // # params [1] TrafficControlParams OPTIONAL
  // cav_msgs/TrafficControlParams params
  // bool params_exists
  out_msg.params_exists = in_msg.params_exists;
  if(out_msg.params_exists)
  {
    convert(in_msg.params, out_msg.params);
  }

  // # geometry [2] TrafficControlGeometry OPTIONAL
  // cav_msgs/TrafficControlGeometry geometry
  // bool geometry_exists
  out_msg.geometry_exists = in_msg.geometry_exists;
  if(out_msg.geometry_exists)
  {
    convert(in_msg.geometry, out_msg.geometry);
  }
}

void convert(const carma_v2x_msgs::msg::TrafficControlParams& in_msg, j2735_v2x_msgs::msg::TrafficControlParams& out_msg)
{
  // j2735_msgs/TrafficControlVehClass[] vclasses
  out_msg.vclasses = in_msg.vclasses;
  
  // # schedule TrafficControlSchedule
  // carma_v2x_msgs/msg/trafficcontrolschedule s.hppedule
  convert(in_msg.schedule, out_msg.schedule);

  // # regulatory BOOLEAN
  // bool regulatory
  out_msg.regulatory = in_msg.regulatory;

  // # detail TrafficControlDetail
  // cav_msgs/TrafficControlDetail detail
  convert(in_msg.detail, out_msg.detail);
}

void convert(const carma_v2x_msgs::msg::TrafficControlSchedule& in_msg, j2735_v2x_msgs::msg::TrafficControlSchedule& out_msg)
{
  ros::Time start(in_msg.start.sec, in_msg.start.nanosec);
  out_msg.start = start.seconds()/ units::SEC_PER_MIN;

  out_msg.end_exists = in_msg.end_exists;
  if(out_msg.end_exists)
  {
    ros::Time end(in_msg.end.sec , in_msg.end.nanosec);
    out_msg.end = end.seconds()/ units::SEC_PER_MIN;
  }

  out_msg.dow_exists = in_msg.dow_exists;
  if(out_msg.dow_exists)
  {
    out_msg.dow = in_msg.dow;
  }

  out_msg.between_exists = in_msg.between_exists;
  if(out_msg.between_exists)
  {
    for (auto in_between : in_msg.between)
    {
      j2735_v2x_msgs::msg::DailySchedule out_between;
      convert(in_between, out_between);
      out_msg.between.push_back(out_between);
    }
  }

  out_msg.repeat_exists = in_msg.repeat_exists;
  if(out_msg.repeat_exists)
  {
    convert(in_msg.repeat, out_msg.repeat);
  }
}
}  // namespace geofence_control
}  // namespace j2735_convertor