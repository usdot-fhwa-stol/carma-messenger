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
#include <j2735_msgs/TrafficControlMessage.h>
#include <cav_msgs/TrafficControlMessage.h>
#include <cav_msgs/OffsetPoint.h>

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

/**
 * @brief Convert the contents of a j2735_msgs::DailySchedule into a cav_msgs::DailySchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::DailySchedule& in_msg, cav_msgs::DailySchedule& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::OffsetPoint into a cav_msgs::OffsetPoint
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::OffsetPoint& in_msg, cav_msgs::OffsetPoint& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::PathNode into a cav_msgs::PathNode
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::PathNode& in_msg, cav_msgs::PathNode& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::RepeatParams into a cav_msgs::RepeatParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::RepeatParams& in_msg, cav_msgs::RepeatParams& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlBounds into a cav_msgs::TrafficControlBounds
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlBounds& in_msg, cav_msgs::TrafficControlBounds& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlDetail into a cav_msgs::TrafficControlDetail
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlDetail& in_msg, cav_msgs::TrafficControlDetail& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlGeometry into a cav_msgs::TrafficControlGeometry
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlGeometry& in_msg, cav_msgs::TrafficControlGeometry& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlMessage into a cav_msgs::TrafficControlMessage
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlMessage& in_msg, cav_msgs::TrafficControlMessage& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlMessageV01 into a cav_msgs::TrafficControlMessageV01
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlMessageV01& in_msg, cav_msgs::TrafficControlMessageV01& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlParams into a cav_msgs::TrafficControlParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlParams& in_msg, cav_msgs::TrafficControlParams& out_msg);

/**
 * @brief Convert the contents of a j2735_msgs::TrafficControlSchedule into a cav_msgs::TrafficControlSchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_msgs::TrafficControlSchedule& in_msg, cav_msgs::TrafficControlSchedule& out_msg);

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

/**
 * @brief Convert the contents of a cav_msgs::DailySchedule into a j2735_msgs::DailySchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::DailySchedule& in_msg, j2735_msgs::DailySchedule& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::OffsetPoint into a j2735_msgs::OffsetPoint
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::OffsetPoint& in_msg, j2735_msgs::OffsetPoint& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::PathNode into a j2735_msgs::PathNode
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::PathNode& in_msg, j2735_msgs::PathNode& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::RepeatParams into a j2735_msgs::RepeatParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::RepeatParams& in_msg, j2735_msgs::RepeatParams& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlBounds into a j2735_msgs::TrafficControlBounds
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlBounds& in_msg, j2735_msgs::TrafficControlBounds& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlDetail into a j2735_msgs::TrafficControlDetail
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlDetail& in_msg, j2735_msgs::TrafficControlDetail& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlGeometry into a j2735_msgs::TrafficControlGeometry
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlGeometry& in_msg, j2735_msgs::TrafficControlGeometry& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlMessage into a j2735_msgs::TrafficControlMessage
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlMessage& in_msg, j2735_msgs::TrafficControlMessage& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlMessageV01 into a j2735_msgs::TrafficControlMessageV01
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlMessageV01& in_msg, j2735_msgs::TrafficControlMessageV01& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlParams into a j2735_msgs::TrafficControlParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlParams& in_msg, j2735_msgs::TrafficControlParams& out_msg);

/**
 * @brief Convert the contents of a cav_msgs::TrafficControlSchedule into a j2735_msgs::TrafficControlSchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const cav_msgs::TrafficControlSchedule& in_msg, j2735_msgs::TrafficControlSchedule& out_msg);
}  // namespace geofence_control
}  // namespace j2735_convertor