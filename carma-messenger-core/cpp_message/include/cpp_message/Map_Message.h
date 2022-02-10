#pragma once
/*
 * Copyright (C) 2020-2021 LEIDOS.
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


#include "cpp_message.h"

namespace cpp_message
{
    class Map_Message
    {
        public:
                    /**
            * \brief constructor 
            */
            explicit Map_Message(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging) {

                node_logging_ = node_logging;
            };

            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

            /**
             * @brief Construct a new map message object
             * 
             * @param binary_array Container with binary input.
             *  @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
             */
            boost::optional<j2735_v2x_msgs::msg::MapData>decode_map_message(std::vector<uint8_t>& binary_array);

            /**
             * @brief Construct Generic lane j2735 ROS MAP.msg
             * 
             * @param g_lane Pointer that has decoded GenericLane object to convert into ROS
             * @return Generic Lane j2735 ROS Msg
             */
            j2735_msgs::GenericLane decode_generic_lane(GenericLane_t* g_lane);
    };
}