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
#include "cpp_message.h"

namespace cpp_message
{
    class PSM_Message
    {
        private:

        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

        int PSM_TEST_ID_ = 32;

        // Conversion constants between asn.1 message spec and carma message
        float latitude_conversion_const_ = 0.0000001;
        float longitude_conversion_const_ = 0.0000001;
        float elevation_conversion_const_ = 0.1;

        public:

        /**
        * \brief constructor 
        */
        explicit PSM_Message(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging) {

            node_logging_ = node_logging;
        };

        /**
         * @brief PSM message decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
         */
        boost::optional<j2735_v2x_msgs::msg::PSM>decode_psm_message(std::vector<uint8_t>& binary_array);
        /**
         * @brief Converts AccelerationSet4Way asn1 message to carma_v2x_msg.
         * @param message AccelerationSet4Way in asn.1 format.
         * @return carma_v2x_message version for AccelerationSet4Way. 
         */
        j2735_v2x_msgs::msg::AccelerationSet4Way decode_accel_set_message(AccelerationSet4Way_t& message);
        /**
         * @brief Converts PathHistory asn1 message to carma_v2x_msg.
         * @param message PathHistory in asn.1 format.
         * @return carma_v2x_message version for PathHistory. 
         */
        j2735_v2x_msgs::msg::PathHistory decode_path_history_message(PathHistory_t& message);
        /**
         * @brief Converts PathPrediction asn1 message to carma_v2x_msg.
         * @param message PathPrediction in asn.1 format.
         * @return carma_v2x_message version for PathPrediction. 
         */
        j2735_v2x_msgs::msg::PathPrediction decode_path_prediction_message(PathPrediction_t& message);
        /**
         * @brief Converts PropelledInformation asn1 message to carma_v2x_msg.
         * @param message PropelledInformation in asn.1 format.
         * @return carma_v2x_message version for PropelledInformation. 
         */
        j2735_v2x_msgs::msg::PropelledInformation decode_propulsion_message(PropelledInformation_t& message);
        /**
         * @brief Converts PersonalDeviceUsageState asn1 message to carma_v2x_msg.
         * @param message PersonalDeviceUsageState in asn.1 format.
         * @return carma_v2x_message version for PersonalDeviceUsageState. 
         */
        j2735_v2x_msgs::msg::PersonalDeviceUsageState decode_use_state(PersonalDeviceUsageState_t& message);

        /**
         * @brief helper functions for PSM message encoding.
         * @param plainMessage contains PSM ros message.
         * @return encoded byte array, returns ROS warning and an empty optional if encoding fails. 
         */
        boost::optional<std::vector<uint8_t>> encode_psm_message(const j2735_v2x_msgs::msg::PSM& plainMessage);

    };
}