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
#include <bitset>

namespace cpp_message
{
    class Emergency_Vehicle_Ack
    {
    private:
        // constants
        static const int Emergency_Vehicle_Ack_TEST_ID = 247;

    public:
        /**
         * \brief constructor
         */
        explicit Emergency_Vehicle_Ack(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging)
        {

            node_logging_ = node_logging;
        };

        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

        /**
         * @brief Emergency Vehicle Acknowledgement message decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and an empty optional if decoding fails.
         */
        boost::optional<carma_v2x_msgs::msg::EmergencyVehicleAck> decode_emergency_vehicle_ack_message(std::vector<uint8_t> &binary_array);

        /**
         * @brief helper functions for Emergency Vehicle Acknowledgement message encoding.
         * @param plainMessage contains emergency vehicle acknowledgement ros message.
         * @return encoded byte array, returns ROS warning and an empty optional if encoding fails.
         */
        boost::optional<std::vector<uint8_t>> encode_emergency_vehicle_ack_message(carma_v2x_msgs::msg::EmergencyVehicleAck plainMessage);
    };
}