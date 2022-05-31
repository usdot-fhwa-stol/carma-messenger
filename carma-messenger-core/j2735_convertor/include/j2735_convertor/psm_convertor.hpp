#pragma once
/*
 * Copyright (C) 2022 LEIDOS.
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
#include <j2735_v2x_msgs/msg/psm.hpp>
#include <j2735_v2x_msgs/msg/spat.hpp>
#include <j2735_v2x_msgs/msg/map_data.hpp>
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <carma_v2x_msgs/msg/spat.hpp>
#include <carma_v2x_msgs/msg/map_data.hpp>
#include "units.hpp"
#include "value_convertor.hpp"

namespace j2735_convertor
{
/**
 * @class PSMConvertor
 * @brief Is the class responsible for converting J2735 PSMs to CARMA usable PSMs
 *
 * Handles conversion between PSMs in the j2735_msgs and cav_msgs packages.
 * Unit conversions and presence flags are also handled
 */
class PSMConvertor
{
  public:
    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::PSM into a carma_v2x_msgs::msg::PSM
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const j2735_v2x_msgs::msg::PSM& in_msg, carma_v2x_msgs::msg::PSM& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PSM into a j2735_v2x_msgs::msg::PSM
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const carma_v2x_msgs::msg::PSM& in_msg, j2735_v2x_msgs::msg::PSM& out_msg);

  private:
    ////
    // Convert j2735_msgs to cav_msgs
    ////

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
    * @brief Convert the contents of a j2735_v2x_msgs::msg::PathPrediction into carma_v2x_msgs::msg::PathPrediction
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const j2735_v2x_msgs::msg::PathPrediction& in_msg, carma_v2x_msgs::msg::PathPrediction& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PathHistory into a j2735_v2x_msgs::msg::PathHistory
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const j2735_v2x_msgs::msg::PathHistory& in_msg, carma_v2x_msgs::msg::PathHistory& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AttachmentRadius into a j2735_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const j2735_v2x_msgs::msg::AttachmentRadius& in_msg, carma_v2x_msgs::msg::AttachmentRadius& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AttachmentRadius into a j2735_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const j2735_v2x_msgs::msg::Heading& in_msg, carma_v2x_msgs::msg::Heading& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AttachmentRadius into a j2735_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const carma_v2x_msgs::msg::Heading& in_msg, j2735_v2x_msgs::msg::Heading& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AttachmentRadius into a j2735_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const j2735_v2x_msgs::msg::Velocity& in_msg, carma_v2x_msgs::msg::Velocity& out_msg);

    ////
    // Convert cav_msgs to j2735_msgs
    ////

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AttachmentRadius into a j2735_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const carma_v2x_msgs::msg::AttachmentRadius& in_msg, j2735_v2x_msgs::msg::AttachmentRadius& out_msg);


    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PathHistory into a j2735_v2x_msgs::msg::PathHistory
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const carma_v2x_msgs::msg::PathHistory& in_msg, j2735_v2x_msgs::msg::PathHistory& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PathHistory into a j2735_v2x_msgs::msg::PathHistory
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const carma_v2x_msgs::msg::PathPrediction& in_msg, j2735_v2x_msgs::msg::PathPrediction& out_msg);

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
    * @brief Convert the contents of a carma_v2x_msgs::msg::PSMCoreData into a j2735_v2x_msgs::msg::PSMCoreData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
     static void convert(const carma_v2x_msgs::msg::Velocity& in_msg, j2735_v2x_msgs::msg::Velocity& out_msg);

  };
}  // namespace j2735_convertor