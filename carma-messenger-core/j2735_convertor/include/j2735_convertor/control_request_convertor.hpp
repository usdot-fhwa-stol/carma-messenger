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
#include <j2735_v2x_msgs/msg/traffic_control_request.hpp>
#include <carma_v2x_msgs/msg/traffic_control_request.hpp>
#include <j2735_v2x_msgs/msg/traffic_control_request_v01.hpp>
#include <carma_v2x_msgs/msg/traffic_control_request_v01.hpp>
#include <carma_v2x_msgs/msg/offset_point.hpp>
#include "units.hpp"
#include "value_convertor.hpp"

namespace j2735_convertor
{
/**
 * @brief Namespace responsible for converting j2735 style Geofence request messages to CARMA usable control messages
 *
 * Handles conversion between ControlRequest in the j2735_v2x_msgs and cav_msgs packages.
 * Unit conversions and presence flags are also handled
 */
namespace geofence_request
{
////
// Convert j2735_v2x_msgs to cav_msgs
////

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::OffsetPoint into a carma_v2x_msgs::msg::OffsetPoint
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::OffsetPoint& in_msg, carma_v2x_msgs::msg::OffsetPoint& out_msg,const int8_t scale);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlBounds into a carma_v2x_msgs::msg::TrafficControlBounds
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 * @param scale The scaling exponent to apply to the bound position. Units applied as 10^n. This normally comes from a
 * ControlRequest message.
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlBounds& in_msg, carma_v2x_msgs::msg::TrafficControlBounds& out_msg, const int8_t scale);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlRequestV01 into a carma_v2x_msgs::msg::TrafficControlRequestV01
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlRequestV01& in_msg, carma_v2x_msgs::msg::TrafficControlRequestV01& out_msg);

/**
 * @brief Convert the contents of a j2735_v2x_msgs::msg::TrafficControlRequest into a carma_v2x_msgs::msg::TrafficControlRequest
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const j2735_v2x_msgs::msg::TrafficControlRequest& in_msg, carma_v2x_msgs::msg::TrafficControlRequest& out_msg);

////
// Convert cav_msgs to j2735_v2x_msgs
////


/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlBounds into a j2735_v2x_msgs::msg::TrafficControlBounds
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 * @param scale The scaling exponent to apply to the bound position. Units applied as 10^n. This normally comes from a
 * ControlRequest message. The scaling here should be the same as what would appear in the ControlRequest (ie. it is not
 * an inverted scaling)
 * 
 * @throw std::invalid_argument If the provided scale will result in a value that does not fit within the 16 bit int bounds of the out message.
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlBounds& in_msg, j2735_v2x_msgs::msg::TrafficControlBounds& out_msg, const int8_t scale);
/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlRequestV01 into a j2735_v2x_msgs::msg::TrafficControlRequestV01 
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlRequestV01& in_msg, j2735_v2x_msgs::msg::TrafficControlRequestV01& out_msg);
/**
 * @brief Convert the contents of a carma_v2x_msgs::msg::TrafficControlRequest into a j2735_v2x_msgs::msg::TrafficControlRequest
 *
 * @param in_msg The message to be converted
 * @param out_msg The message to store the output
 *
 * NOTE: Scale determined dynamically based on bound size.
 * This function estimates the required scaling based on the following algorithm.
 * If all input bounds are less than 1000, 100, or 10 then use a scale of -1, -2, -3 respectively.
 * This gives higher resolution for smaller bounds while ensuring there is not integer overflow.
 * 
 * If the bounds are larger than 1000, the resolution will cap at 1m accuracy and determine a scaling of
 * 1, 2, or 3 based on if all bounds can be cleanly divided by 10, 100, or 1000 respectively after integer truncation.
 * 
 * If the numbers cannot be scaled and they exceed the bounds of the output 16 bit int, then an exception will be thrown.  
 * 
 *
 * Unit conversions and presence flags are handled
 */
void convert(const carma_v2x_msgs::msg::TrafficControlRequest& in_msg, j2735_v2x_msgs::msg::TrafficControlRequest& out_msg);


}  // namespace geofence_request
}  // namespace j2735_convertor