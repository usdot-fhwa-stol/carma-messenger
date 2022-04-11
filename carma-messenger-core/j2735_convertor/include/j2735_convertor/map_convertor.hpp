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
 * @class MAPConvertor
 * @brief Is the class responsible for converting J2735 Maps to CARMA usable Mapss
 *
 * Handles conversion between Map messages in the j2735_msgs and cav_msgs packages.
 * Unit conversions are handled
 * Note: The concept of map Zoom is not accounted for in these conversions
 */
class MapConvertor
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
  static void convert(const j2735_v2x_msgs::msg::MapData& in_msg, carma_v2x_msgs::msg::MapData& out_msg);

private:
  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::OffsetXaxis into a carma_v2x_msgs::msg::OffsetAxis
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertOffsetXaxis(const j2735_v2x_msgs::msg::OffsetXaxis& in_msg, carma_v2x_msgs::msg::OffsetAxis& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::OffsetYaxis into a carma_v2x_msgs::msg::OffsetAxis
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertOffsetYaxis(const j2735_v2x_msgs::msg::OffsetYaxis& in_msg, carma_v2x_msgs::msg::OffsetAxis& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::ComputedLane into a carma_v2x_msgs::msg::ComputedLane
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertComputedLane(const j2735_v2x_msgs::msg::ComputedLane& in_msg, carma_v2x_msgs::msg::ComputedLane& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::NodeOffsetPointXY into a carma_v2x_msgs::msg::NodeOffsetPointXY
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertNodeOffsetPointXY(const j2735_v2x_msgs::msg::NodeOffsetPointXY& in_msg,
                                       carma_v2x_msgs::msg::NodeOffsetPointXY& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::LaneDataAttribute into a carma_v2x_msgs::msg::LaneDataAttribute
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertLaneDataAttribute(const j2735_v2x_msgs::msg::LaneDataAttribute& in_msg,
                                       carma_v2x_msgs::msg::LaneDataAttribute& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::NodeAttributeSetXY into a carma_v2x_msgs::msg::NodeAttributeSetXY
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertNodeAttributeSetXY(const j2735_v2x_msgs::msg::NodeAttributeSetXY& in_msg,
                                        carma_v2x_msgs::msg::NodeAttributeSetXY& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::NodeXY into a carma_v2x_msgs::msg::NodeXY
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertNodeXY(const j2735_v2x_msgs::msg::NodeXY& in_msg, carma_v2x_msgs::msg::NodeXY& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::NodeSetXY into a carma_v2x_msgs::msg::NodeSetXY
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertNodeSetXY(const j2735_v2x_msgs::msg::NodeSetXY& in_msg, carma_v2x_msgs::msg::NodeSetXY& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::NodeListXY into a carma_v2x_msgs::msg::NodeListXY
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertNodeListXY(const j2735_v2x_msgs::msg::NodeListXY& in_msg, carma_v2x_msgs::msg::NodeListXY& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::GenericLane into a carma_v2x_msgs::msg::GenericLane
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertGenericLane(const j2735_v2x_msgs::msg::GenericLane& in_msg, carma_v2x_msgs::msg::GenericLane& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::RegulatorySpeedLimit into a carma_v2x_msgs::msg::RegulatorySpeedLimit
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertRegulatorySpeedLimit(const j2735_v2x_msgs::msg::RegulatorySpeedLimit& in_msg,
                                          carma_v2x_msgs::msg::RegulatorySpeedLimit& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::Position3D into a carma_v2x_msgs::msg::Position3D
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertPosition3D(const j2735_v2x_msgs::msg::Position3D& in_msg, carma_v2x_msgs::msg::Position3D& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::IntersectionGeometry into a carma_v2x_msgs::msg::IntersectionGeometry
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertIntersectionGeometry(const j2735_v2x_msgs::msg::IntersectionGeometry& in_msg,
                                          carma_v2x_msgs::msg::IntersectionGeometry& out_msg);

  /**
   * @brief Convert the contents of a j2735_v2x_msgs::msg::RoadSegment into a carma_v2x_msgs::msg::RoadSegment
   *
   * @param in_msg The message to be converted
   * @param out_msg The message to store the output
   *
   * Unit conversions are handled
   */
  static void convertRoadSegment(const j2735_v2x_msgs::msg::RoadSegment& in_msg, carma_v2x_msgs::msg::RoadSegment& out_msg);
};
}  // namespace j2735_convertor
