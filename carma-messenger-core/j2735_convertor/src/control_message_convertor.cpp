/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <j2735_convertor/control_message_convertor.h>
#include <ros/time.h>

namespace j2735_convertor
{
namespace geofence_control
{
////
// Convert j2735_msgs to cav_msgs
////

void convert(const j2735_msgs::ControlMessage& in_msg, cav_msgs::ControlMessage& out_msg)
{
  out_msg.version = in_msg.version;
  out_msg.id = in_msg.id;

  out_msg.updated = out_msg.updated.fromNSec(in_msg.updated * units::NS_PER_MS_INT);

  out_msg.vtypes = in_msg.vtypes;

  convert(in_msg.schedule, out_msg.schedule);

  out_msg.regulatory = in_msg.regulatory;
  out_msg.control_type = in_msg.control_type;
  out_msg.control_value = in_msg.control_value;
  out_msg.control_value_exists = in_msg.control_value_exists;
  out_msg.path_parts = in_msg.path_parts;
  out_msg.proj = in_msg.proj;
  out_msg.datum = in_msg.datum;

  out_msg.time = out_msg.time.fromNSec(in_msg.time * units::NS_PER_MS_INT);

  out_msg.latitude = (double)in_msg.latitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.longitude = (double)in_msg.longitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.altitude =
      ((double)in_msg.altitude / units::DECA_M_PER_M) - 428.0;  // Subtract 428 meters to account for altitude offset
  out_msg.heading = (double)in_msg.heading / units::DECA_DEG_PER_DEG;

  for (auto p : in_msg.points)
  {
    cav_msgs::Point cav_p;
    convert(p, cav_p);
    out_msg.points.emplace_back(cav_p);
  }
}

void convert(const j2735_msgs::Schedule& in_msg, cav_msgs::Schedule& out_msg)
{
  out_msg.start = out_msg.start.fromNSec(in_msg.start * units::NS_PER_MS_INT);
  out_msg.end = out_msg.end.fromNSec(in_msg.end * units::NS_PER_MS_INT);

  out_msg.dow = in_msg.dow;
  out_msg.dow_exists = in_msg.dow_exists;

  convert(in_msg.between, out_msg.between);
  out_msg.between_exists = in_msg.between_exists;

  convert(in_msg.repeat, out_msg.repeat);
  out_msg.repeat_exists = in_msg.repeat_exists;
}

void convert(const j2735_msgs::ScheduleParams& in_msg, cav_msgs::ScheduleParams& out_msg)
{
  out_msg.duration = ros::Duration(in_msg.duration * units::SEC_PER_MIN);
  out_msg.interval = ros::Duration(in_msg.interval * units::SEC_PER_MIN);
}

void convert(const j2735_msgs::DaySchedule& in_msg, cav_msgs::DaySchedule& out_msg)
{
  out_msg.start = ros::Duration(in_msg.start * units::SEC_PER_MIN);
  out_msg.end = ros::Duration(in_msg.end * units::SEC_PER_MIN);
  out_msg.utcoffset = ros::Duration(in_msg.utcoffset * units::SEC_PER_MIN);
}

void convert(const j2735_msgs::Point& in_msg, cav_msgs::Point& out_msg)
{
  out_msg.x = (double)in_msg.x / units::CM_PER_M;
  out_msg.y = (double)in_msg.y / units::CM_PER_M;
  out_msg.z = (double)in_msg.z / units::CM_PER_M;  // Since this conversion is so small there is no need to complicate
                                                   // logic by checking z_exists flag.
  out_msg.width = (double)in_msg.width / units::CM_PER_M;
  out_msg.z_exists = in_msg.z_exists;
}

void convert(const j2735_msgs::DailySchedule& in_msg, cav_msgs::DailySchedule& out_msg)
{
  out_msg.begin = ros::Duration(in_msg.begin * units::SEC_PER_MIN);
  out_msg.duration = ros::Duration(in_msg.duration * units::SEC_PER_MIN);
}

void convert(const j2735_msgs::PathNode& in_msg, cav_msgs::PathNode& out_msg)
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

void convert(const j2735_msgs::RepeatParams& in_msg, cav_msgs::RepeatParams& out_msg)
{
  out_msg.offset = ros::Duration(in_msg.offset * units::SEC_PER_MIN);
  out_msg.period = ros::Duration(in_msg.period * units::SEC_PER_MIN);
  out_msg.span = ros::Duration(in_msg.span * units::SEC_PER_MIN);
}

void convert(const j2735_msgs::TrafficControlDetail& in_msg, cav_msgs::TrafficControlDetail& out_msg)
{
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case j2735_msgs::TrafficControlDetail::SIGNAL_CHOICE : 
      out_msg.signal = in_msg.signal;
      break;
    case j2735_msgs::TrafficControlDetail::STOP_CHOICE : 
      // Not implemented yet
      break;
    case j2735_msgs::TrafficControlDetail::YIELD_CHOICE : 
      // Not implemented yet
      break;
    case j2735_msgs::TrafficControlDetail::NOTOWING_CHOICE : 
      // Not implemented yet
      break;
    case j2735_msgs::TrafficControlDetail::RESTRICTED_CHOICE : 
      // Not implemented yet
      break;
    case j2735_msgs::TrafficControlDetail::CLOSED_CHOICE : 
      out_msg.closed = in_msg.closed;
      break;
    case j2735_msgs::TrafficControlDetail::CHAINS_CHOICE : 
      out_msg.chains = in_msg.chains;
      break;
    case j2735_msgs::TrafficControlDetail::DIRECTION_CHOICE : 
      out_msg.direction = in_msg.direction;
      break;
    case j2735_msgs::TrafficControlDetail::LATAFFINITY_CHOICE : 
      out_msg.lataffinity = in_msg.lataffinity;
      break;
    case j2735_msgs::TrafficControlDetail::LATPERM_CHOICE : 
      for(int i = 0; i < 2; i++)
      {
        out_msg.latperm[i] = in_msg.latperm[i];
      }
      break;
    case j2735_msgs::TrafficControlDetail::PARKING_CHOICE : 
      out_msg.parking = in_msg.parking;
      break;
    case j2735_msgs::TrafficControlDetail::MINSPEED_CHOICE : 
      out_msg.minspeed = (float)in_msg.minspeed / units::DECA_MPS_PER_MPS;
      break;
    case j2735_msgs::TrafficControlDetail::MAXSPEED_CHOICE : 
      out_msg.maxspeed = (float)in_msg.maxspeed / units::DECA_MPS_PER_MPS;
      break;
    case j2735_msgs::TrafficControlDetail::MINHDWY_CHOICE : 
      out_msg.minhdwy = (float)in_msg.minhdwy / units::DECA_M_PER_M;
      break;
    case j2735_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE : 
      out_msg.maxvehmass = (float)in_msg.maxvehmass;
      break;
    case j2735_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE : 
      out_msg.maxvehheight = (float)in_msg.maxvehheight / units::DECA_M_PER_M;
      break;
    case j2735_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE : 
      out_msg.maxvehwidth = (float)in_msg.maxvehwidth / units::DECA_M_PER_M;
      break;
    case j2735_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE : 
      out_msg.maxvehlength = (float)in_msg.maxvehlength / units::DECA_M_PER_M;
      break;
    case j2735_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE : 
      out_msg.maxvehaxles = in_msg.maxvehaxles;
      break;
    case j2735_msgs::TrafficControlDetail::MINVEHOCC_CHOICE : 
      out_msg.minvehocc = in_msg.minvehocc;
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const j2735_msgs::TrafficControlGeometry& in_msg, cav_msgs::TrafficControlGeometry& out_msg)
{
  out_msg.proj = in_msg.proj;
  out_msg.datum = in_msg.datum;
  out_msg.reftime = ros::Time(in_msg.reftime * units::SEC_PER_MIN, 0);
  out_msg.reflon = (double)in_msg.reflon / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.reflat = (double)in_msg.reflat / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.refelv = (float)in_msg.refelv / units::DECA_M_PER_M - (float) 409.6; //handle offset
  out_msg.heading = (float)in_msg.heading / units::DECA_S_PER_S;

  for (auto in_node : in_msg.nodes)
  {
    cav_msgs::PathNode out_node;
    convert(in_node, out_node);
    out_msg.nodes.push_back(out_node);
  }
}

void convert(const j2735_msgs::TrafficControlMessage& in_msg, cav_msgs::TrafficControlMessage& out_msg)
{
  // uint8 choice
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case j2735_msgs::TrafficControlMessage::RESERVED : 
      // Not implemented yet
      break;
    case j2735_msgs::TrafficControlMessage::TCMV01 : 
      convert(in_msg.tcmV01, out_msg.tcmV01);
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const j2735_msgs::TrafficControlMessageV01& in_msg, cav_msgs::TrafficControlMessageV01& out_msg)
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

void convert(const j2735_msgs::TrafficControlParams& in_msg, cav_msgs::TrafficControlParams& out_msg)
{
  // j2735_msgs/TrafficControlVehClass[] vclasses
  out_msg.vclasses = in_msg.vclasses;
  
  // # schedule TrafficControlSchedule
  // cav_msgs/TrafficControlSchedule schedule
  convert(in_msg.schedule, out_msg.schedule);

  // # regulatory BOOLEAN
  // bool regulatory
  out_msg.regulatory = in_msg.regulatory;

  // # detail TrafficControlDetail
  // cav_msgs/TrafficControlDetail detail
  convert(in_msg.detail, out_msg.detail);
}

void convert(const j2735_msgs::TrafficControlSchedule& in_msg, cav_msgs::TrafficControlSchedule& out_msg)
{
  out_msg.start = ros::Time(in_msg.start * units::SEC_PER_MIN, 0);

  out_msg.end_exists = in_msg.end_exists;
  if(out_msg.end_exists)
  {
    out_msg.end = ros::Time(in_msg.end * units::SEC_PER_MIN, 0);
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
      cav_msgs::DailySchedule out_between;
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

void convert(const cav_msgs::ControlMessage& in_msg, j2735_msgs::ControlMessage& out_msg)
{
  out_msg.version = in_msg.version;
  out_msg.id = in_msg.id;

  out_msg.updated = (in_msg.updated.sec * units::MS_PER_S) + (in_msg.updated.nsec / units::NS_PER_MS_INT);

  out_msg.vtypes = in_msg.vtypes;

  convert(in_msg.schedule, out_msg.schedule);

  out_msg.regulatory = in_msg.regulatory;
  out_msg.control_type = in_msg.control_type;
  out_msg.control_value = in_msg.control_value;
  out_msg.control_value_exists = in_msg.control_value_exists;
  out_msg.path_parts = in_msg.path_parts;
  out_msg.proj = in_msg.proj;
  out_msg.datum = in_msg.datum;

  out_msg.time = (in_msg.time.sec * units::MS_PER_S) + (in_msg.time.nsec / units::NS_PER_MS_INT);

  out_msg.latitude = in_msg.latitude * units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.longitude = in_msg.longitude * units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.altitude =
      (uint32_t)((double)in_msg.altitude * units::DECA_M_PER_M) + 4280;  // Add 428 meters to account for altitude offset
  out_msg.heading = (double)in_msg.heading * units::DECA_DEG_PER_DEG;

  for (auto p : in_msg.points)
  {
    j2735_msgs::Point cav_p;
    convert(p, cav_p);
    out_msg.points.emplace_back(cav_p);
  }
}

void convert(const cav_msgs::Schedule& in_msg, j2735_msgs::Schedule& out_msg)
{
  out_msg.start = (in_msg.start.sec * units::MS_PER_S) + (in_msg.start.nsec / units::NS_PER_MS_INT);
  out_msg.end = (in_msg.end.sec * units::MS_PER_S) + (in_msg.end.nsec / units::NS_PER_MS_INT);

  out_msg.dow = in_msg.dow;
  out_msg.dow_exists = in_msg.dow_exists;

  convert(in_msg.between, out_msg.between);
  out_msg.between_exists = in_msg.between_exists;

  convert(in_msg.repeat, out_msg.repeat);
  out_msg.repeat_exists = in_msg.repeat_exists;
}

void convert(const cav_msgs::ScheduleParams& in_msg, j2735_msgs::ScheduleParams& out_msg)
{
  out_msg.duration = in_msg.duration.toSec() / units::SEC_PER_MIN;
  out_msg.interval = in_msg.interval.toSec() / units::SEC_PER_MIN;
}

void convert(const cav_msgs::DaySchedule& in_msg, j2735_msgs::DaySchedule& out_msg)
{
  out_msg.start = in_msg.start.toSec() / units::SEC_PER_MIN;
  out_msg.end = in_msg.end.toSec() / units::SEC_PER_MIN;
  out_msg.utcoffset = in_msg.utcoffset.toSec() / units::SEC_PER_MIN;
}

void convert(const cav_msgs::Point& in_msg, j2735_msgs::Point& out_msg)
{
  out_msg.x = in_msg.x * units::CM_PER_M;
  out_msg.y = in_msg.y * units::CM_PER_M;
  out_msg.z = in_msg.z * units::CM_PER_M;  // Since this conversion is so small there is no need to complicate logic by
                                           // checking z_exists flag.
  out_msg.width = in_msg.width * units::CM_PER_M;
  out_msg.z_exists = in_msg.z_exists;
}

void convert(const cav_msgs::DailySchedule& in_msg, j2735_msgs::DailySchedule& out_msg)
{
  out_msg.begin = in_msg.begin.toSec() / units::SEC_PER_MIN;
  out_msg.duration = in_msg.duration.toSec() / units::SEC_PER_MIN;
}

void convert(const cav_msgs::PathNode& in_msg, j2735_msgs::PathNode& out_msg)
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

void convert(const cav_msgs::RepeatParams& in_msg, j2735_msgs::RepeatParams& out_msg)
{
  out_msg.offset = in_msg.offset.toSec() / units::SEC_PER_MIN;
  out_msg.period = in_msg.period.toSec() / units::SEC_PER_MIN;
  out_msg.span = in_msg.span.toSec() / units::SEC_PER_MIN;
}

void convert(const cav_msgs::TrafficControlDetail& in_msg, j2735_msgs::TrafficControlDetail& out_msg)
{
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case cav_msgs::TrafficControlDetail::SIGNAL_CHOICE : 
      out_msg.signal = in_msg.signal;
      break;
    case cav_msgs::TrafficControlDetail::STOP_CHOICE : 
      // Not implemented yet
      break;
    case cav_msgs::TrafficControlDetail::YIELD_CHOICE : 
      // Not implemented yet
      break;
    case cav_msgs::TrafficControlDetail::NOTOWING_CHOICE : 
      // Not implemented yet
      break;
    case cav_msgs::TrafficControlDetail::RESTRICTED_CHOICE : 
      // Not implemented yet
      break;
    case cav_msgs::TrafficControlDetail::CLOSED_CHOICE : 
      out_msg.closed = in_msg.closed;
      break;
    case cav_msgs::TrafficControlDetail::CHAINS_CHOICE : 
      out_msg.chains = in_msg.chains;
      break;
    case cav_msgs::TrafficControlDetail::DIRECTION_CHOICE : 
      out_msg.direction = in_msg.direction;
      break;
    case cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE : 
      out_msg.lataffinity = in_msg.lataffinity;
      break;
    case cav_msgs::TrafficControlDetail::LATPERM_CHOICE : 
      for(int i = 0; i < 2; i++)
      {
        out_msg.latperm[i] = in_msg.latperm[i];
      }
      break;
    case cav_msgs::TrafficControlDetail::PARKING_CHOICE : 
      out_msg.parking = in_msg.parking;
      break;
    case cav_msgs::TrafficControlDetail::MINSPEED_CHOICE : 
      out_msg.minspeed = (uint16_t)(in_msg.minspeed * units::DECA_MPS_PER_MPS);
      break;
    case cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE : 
      out_msg.maxspeed = (uint16_t)(in_msg.maxspeed * units::DECA_MPS_PER_MPS);
      break;
    case cav_msgs::TrafficControlDetail::MINHDWY_CHOICE : 
      out_msg.minhdwy = (uint16_t)(in_msg.minhdwy * units::DECA_M_PER_M);
      break;
    case cav_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE : 
      out_msg.maxvehmass = (uint16_t)in_msg.maxvehmass;
      break;
    case cav_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE : 
      out_msg.maxvehheight = (uint8_t)(in_msg.maxvehheight * units::DECA_M_PER_M);
      break;
    case cav_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE : 
      out_msg.maxvehwidth = (uint8_t)(in_msg.maxvehwidth * units::DECA_M_PER_M);
      break;
    case cav_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE : 
      out_msg.maxvehlength = (uint8_t)(in_msg.maxvehlength * units::DECA_M_PER_M);
      break;
    case cav_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE : 
      out_msg.maxvehaxles = in_msg.maxvehaxles;
      break;
    case cav_msgs::TrafficControlDetail::MINVEHOCC_CHOICE : 
      out_msg.minvehocc = in_msg.minvehocc;
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const cav_msgs::TrafficControlGeometry& in_msg, j2735_msgs::TrafficControlGeometry& out_msg)
{
  out_msg.proj = in_msg.proj;
  out_msg.datum = in_msg.datum;
  out_msg.reftime = in_msg.reftime.toSec() / units::SEC_PER_MIN;
  out_msg.reflon = (int32_t)(in_msg.reflon * units::TENTH_MICRO_DEG_PER_DEG);
  out_msg.reflat = (int32_t)(in_msg.reflat * units::TENTH_MICRO_DEG_PER_DEG);
  out_msg.refelv = (int32_t)((in_msg.refelv + 409.6) * units::DECA_M_PER_M); // handle offset
  out_msg.heading = (int32_t)(in_msg.heading * units::DECA_S_PER_S);

  for (auto in_node : in_msg.nodes)
  {
    j2735_msgs::PathNode out_node;
    convert(in_node, out_node);
    out_msg.nodes.push_back(out_node);
  }
}

void convert(const cav_msgs::TrafficControlMessage& in_msg, j2735_msgs::TrafficControlMessage& out_msg)
{
  // uint8 choice
  out_msg.choice = in_msg.choice;
  switch(in_msg.choice)
  {
    case cav_msgs::TrafficControlMessage::RESERVED : 
      // Not implemented yet
      break;
    case cav_msgs::TrafficControlMessage::TCMV01 : 
      convert(in_msg.tcmV01, out_msg.tcmV01);
      break;
    default : 
      // Throw Error?
      break;
  }
}

void convert(const cav_msgs::TrafficControlMessageV01& in_msg, j2735_msgs::TrafficControlMessageV01& out_msg)
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
  out_msg.updated = in_msg.updated.toSec() / units::SEC_PER_MIN;

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

void convert(const cav_msgs::TrafficControlParams& in_msg, j2735_msgs::TrafficControlParams& out_msg)
{
  // j2735_msgs/TrafficControlVehClass[] vclasses
  out_msg.vclasses = in_msg.vclasses;
  
  // # schedule TrafficControlSchedule
  // cav_msgs/TrafficControlSchedule schedule
  convert(in_msg.schedule, out_msg.schedule);

  // # regulatory BOOLEAN
  // bool regulatory
  out_msg.regulatory = in_msg.regulatory;

  // # detail TrafficControlDetail
  // cav_msgs/TrafficControlDetail detail
  convert(in_msg.detail, out_msg.detail);
}

void convert(const cav_msgs::TrafficControlSchedule& in_msg, j2735_msgs::TrafficControlSchedule& out_msg)
{
  out_msg.start = in_msg.start.toSec() / units::SEC_PER_MIN;

  out_msg.end_exists = in_msg.end_exists;
  if(out_msg.end_exists)
  {
    out_msg.end = in_msg.end.toSec() / units::SEC_PER_MIN;
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
      j2735_msgs::DailySchedule out_between;
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