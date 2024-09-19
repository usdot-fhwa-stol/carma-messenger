#pragma once
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

#include <cstdint>
#include <j2735_v2x_msgs/msg/traffic_control_message.hpp>
#include <carma_v2x_msgs/msg/traffic_control_message.hpp>

#include "units.hpp"
#include "value_convertor.hpp"

namespace j2735_convertor
{
/**
 * @brief Namespace responsible for converting j2735 style Geofence control messages to CARMA usable control messages
 *
 * Handles conversion between ControlMessages in the j2735_v2x_msgs and cav_msgs packages.
 * Unit conversions and presence flags are also handled
 */
namespace geofence_control
{
////
// Convert j2735_v2x_msgs to cav_msgs
////

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::DailySchedule into a carma_v2x_msgs::msg::DailySchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::DailySchedule& in_msg, carma_v2x_msgs::msg::DailySchedule& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::PathNode into a carma_v2x_msgs::msg::PathNode
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::PathNode& in_msg, carma_v2x_msgs::msg::PathNode& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::RepeatParams into a carma_v2x_msgs::msg::RepeatParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::RepeatParams& in_msg, carma_v2x_msgs::msg::RepeatParams& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlDetail into a carma_v2x_msgs::msg::TrafficControlDetail
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlDetail& in_msg, carma_v2x_msgs::msg::TrafficControlDetail& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlGeometry into a carma_v2x_msgs::msg::TrafficControlGeometry
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlGeometry& in_msg, carma_v2x_msgs::msg::TrafficControlGeometry& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlMessage into a carma_v2x_msgs::msg::TrafficControlMessage
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlMessage& in_msg, carma_v2x_msgs::msg::TrafficControlMessage& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlMessageV01 into a carma_v2x_msgs::msg::TrafficControlMessageV01
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlMessageV01& in_msg, carma_v2x_msgs::msg::TrafficControlMessageV01& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlParams into a carma_v2x_msgs::msg::TrafficControlParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlParams& in_msg, carma_v2x_msgs::msg::TrafficControlParams& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlSchedule into a carma_v2x_msgs::msg::TrafficControlSchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlSchedule& in_msg, carma_v2x_msgs::msg::TrafficControlSchedule& out_msg);

////
// Convert cav_msgs to j2735_v2x_msgs
////


/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::DailySchedule into a j2735_v2x_msgs::msg::DailySchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::DailySchedule& in_msg, j2735_v2x_msgs::msg::DailySchedule& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::PathNode into a j2735_v2x_msgs::msg::PathNode
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::PathNode& in_msg, j2735_v2x_msgs::msg::PathNode& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::RepeatParams into a j2735_v2x_msgs::msg::RepeatParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::RepeatParams& in_msg, j2735_v2x_msgs::msg::RepeatParams& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlDetail into a j2735_v2x_msgs::msg::TrafficControlDetail
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlDetail& in_msg, j2735_v2x_msgs::msg::TrafficControlDetail& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlGeometry into a j2735_v2x_msgs::msg::TrafficControlGeometry
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlGeometry& in_msg, j2735_v2x_msgs::msg::TrafficControlGeometry& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlMessage into a j2735_v2x_msgs::msg::TrafficControlMessage
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlMessage& in_msg, j2735_v2x_msgs::msg::TrafficControlMessage& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlMessageV01 into a j2735_v2x_msgs::msg::TrafficControlMessageV01
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlMessageV01& in_msg, j2735_v2x_msgs::msg::TrafficControlMessageV01& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlParams into a j2735_v2x_msgs::msg::TrafficControlParams
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlParams& in_msg, j2735_v2x_msgs::msg::TrafficControlParams& out_msg);

/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlSchedule into a j2735_v2x_msgs::msg::TrafficControlSchedule
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlSchedule& in_msg, j2735_v2x_msgs::msg::TrafficControlSchedule& out_msg);
}  // namespace geofence_control
}  // namespace j2735_convertor