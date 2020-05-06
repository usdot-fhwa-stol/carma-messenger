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
}  // namespace geofence_control
}  // namespace j2735_convertor