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

#include <j2735_v2x_msgs/msg/bsm.hpp>
#include <j2735_v2x_msgs/msg/spat.hpp>
#include <j2735_v2x_msgs/msg/map_data.hpp>
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include <carma_v2x_msgs/msg/map_data.hpp>
#include "units.hpp"

namespace j2735_convertor
{
/**
 * @class SPATConvertor
 * @brief Is the class responsible for converting J2735 SPATs to CARMA usable SPATs
 *
 * Handles conversion between Map messages in the j2735_msgs and cav_msgs packages.
 * Unit conversions are handled
 */
class SPATConvertor
{
public:
  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::MapData into a carma_v2x_msgs::msg::MapData
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convert(const j2735_v2x_msgs::msg::SPAT& in_msg, carma_v2x_msgs::msg::SPAT& out_msg);

private:
  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::TimeChangeDetails into a carma_v2x_msgs::msg::TimeChangeDetails
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertTimeChangeDetails(const j2735_v2x_msgs::msg::TimeChangeDetails& in_msg,
                                       carma_v2x_msgs::msg::TimeChangeDetails& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::AdvisorySpeed into a carma_v2x_msgs::msg::AdvisorySpeed
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertAdvisorySpeed(const j2735_v2x_msgs::msg::AdvisorySpeed& in_msg, carma_v2x_msgs::msg::AdvisorySpeed& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::MovementEvent into a carma_v2x_msgs::msg::MovementEvent
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertMovementEvent(const j2735_v2x_msgs::msg::MovementEvent& in_msg, carma_v2x_msgs::msg::MovementEvent& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::MovementState into a carma_v2x_msgs::msg::MovementState
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertMovementState(const j2735_v2x_msgs::msg::MovementState& in_msg, carma_v2x_msgs::msg::MovementState& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::IntersectionState into a carma_v2x_msgs::msg::IntersectionState
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertIntersectionState(const j2735_v2x_msgs::msg::IntersectionState& in_msg,
                                       carma_v2x_msgs::msg::IntersectionState& out_msg);
};
}  // namespace j2735_convertor
