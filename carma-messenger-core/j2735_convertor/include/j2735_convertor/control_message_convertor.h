#pragma once
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

#include <cstdint>
#include <j2735_msgs/ControlMessage.h>
#include <cav_msgs/ControlMessage.h>
#include "units.h"
#include "value_convertor.h"

namespace j2735_convertor
{
/**
 * @brief Namespace responsible for converting j2735 style Geofence control messages to CARMA usable control messages
 *
 * Handles conversion between ControlMessages in the j2735_msgs and cav_msgs packages.
 * Unit conversions and presence flags are also handled
 */
namespace geofence_control
{
////
// Convert j2735_msgs to cav_msgs
////

/**
 * @brief Convert the contents of a j2735_msgs::ControlMessage into a cav_msgs::ControlMessage
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::ControlMessage& in_msg, cav_msgs::ControlMessage& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::Schedule into a cav_msgs::Schedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::Schedule& in_msg, cav_msgs::Schedule& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::ScheduleParams into a cav_msgs::ScheduleParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::ScheduleParams& in_msg, cav_msgs::ScheduleParams& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::DaySchedule into a cav_msgs::DaySchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::DaySchedule& in_msg, cav_msgs::DaySchedule& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::Point into a cav_msgs::Point
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::Point& in_msg, cav_msgs::Point& out_msg);

////
// Convert cav_msgs to j2735_msgs
////

/**
 * @brief Convert the contents of a cav_msgs::ControlMessage into a j2735_msgs::ControlMessage
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::ControlMessage& in_msg, j2735_msgs::ControlMessage& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::Schedule into a j2735_msgs::Schedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::Schedule& in_msg, j2735_msgs::Schedule& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::ScheduleParams into a j2735_msgs::ScheduleParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::ScheduleParams& in_msg, j2735_msgs::ScheduleParams& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::DaySchedule into a j2735_msgs::DaySchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::DaySchedule& in_msg, j2735_msgs::DaySchedule& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::Point into a j2735_msgs::Point
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::Point& in_msg, j2735_msgs::Point& out_msg);
}  // namespace geofence_control
}  // namespace j2735_convertor