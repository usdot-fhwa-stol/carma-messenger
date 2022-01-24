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
#include <j2735_v2x_msgs/msg/bsm.hpp>
#include <j2735_v2x_msgs/msg/spat.hpp>
#include <j2735_v2x_msgs/msg/map_data.hpp>
#include <carma_v2x_msgs/msg/system_alert.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include <carma_v2x_msgs/msg/map_data.hpp>
#include "units.h"
#include "value_convertor.h"

namespace j2735_convertor
{
/**
 * @class BSMConvertor
 * @brief Is the class responsible for converting J2735 BSMs to CARMA usable BSMs
 *
 * Handles conversion between BSMs in the j2735_msgs and cav_msgs packages.
 * Unit conversions and presence flags are also handled
 */
class BSMConvertor
{
public:
  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::BSM into a carma_v2x_msgs::msg::BSM
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const j2735_v2x_msgs::msg::BSM& in_msg, carma_v2x_msgs::msg::BSM& out_msg);

  /**
   * @brief Convert the contents of a carma_v2x_msgs::msg::BSM into a j2735_v2x_msgs::msg::BSM
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const carma_v2x_msgs::msg::BSM& in_msg, j2735_v2x_msgs::msg::BSM& out_msg);

private:
  ////
  // Convert j2735_msgs to cav_msgs
  ////

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::VehicleSize into a carma_v2x_msgs::msg::VehicleSize
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const j2735_v2x_msgs::msg::VehicleSize& in_msg, carma_v2x_msgs::msg::VehicleSize& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::AccelerationSet4Way into a carma_v2x_msgs::msg::AccelerationSet4Way
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const j2735_v2x_msgs::msg::AccelerationSet4Way& in_msg, carma_v2x_msgs::msg::AccelerationSet4Way& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::PositionalAccuracy into a carma_v2x_msgs::msg::PositionalAccuracy
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const j2735_v2x_msgs::msg::PositionalAccuracy& in_msg, carma_v2x_msgs::msg::PositionalAccuracy& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::BSMCoreData into a carma_v2x_msgs::msg::BSMCoreData
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const j2735_v2x_msgs::msg::BSMCoreData& in_msg, carma_v2x_msgs::msg::BSMCoreData& out_msg);

  ////
  // Convert cav_msgs to j2735_msgs
  ////

  /**
   * @brief Convert the contents of a carma_v2x_msgs::msg::VehicleSize into a j2735_v2x_msgs::msg::VehicleSize
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const carma_v2x_msgs::msg::VehicleSize& in_msg, j2735_v2x_msgs::msg::VehicleSize& out_msg);

  /**
   * @brief Convert the contents of a carma_v2x_msgs::msg::AccelerationSet4Way into a j2735_v2x_msgs::msg::AccelerationSet4Way
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const carma_v2x_msgs::msg::AccelerationSet4Way& in_msg, j2735_v2x_msgs::msg::AccelerationSet4Way& out_msg);

  /**
   * @brief Convert the contents of a carma_v2x_msgs::msg::PositionalAccuracy into a j2735_v2x_msgs::msg::PositionalAccuracy
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const carma_v2x_msgs::msg::PositionalAccuracy& in_msg, j2735_v2x_msgs::msg::PositionalAccuracy& out_msg);

  /**
   * @brief Convert the contents of a carma_v2x_msgs::msg::BSMCoreData into a j2735_v2x_msgs::msg::BSMCoreData
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions and presence flags are handled
   */
  static void convert(const carma_v2x_msgs::msg::BSMCoreData& in_msg, j2735_v2x_msgs::msg::BSMCoreData& out_msg);
};
}  // namespace j2735_convertor