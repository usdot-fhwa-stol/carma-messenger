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
        static const int latitude_conversion_const_ = 0.0000001;
        static const int longitude_conversion_const_ = 0.0000001;
        static const int elevation_conversion_const_ = 0.1;
        static const int velocity_conversion_const_ = 0.02;
        static const int heading_conversion_const_ = 0.0125;

        public:

        /**
        * \brief constructor 
        */
        explicit PSM_Message(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging) {

            node_logging_ = node_logging;
        };

        /**
         * @brief BSM message decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
         */
        boost::optional<carma_v2x_msgs::msg::PSM>decode_psm_message(std::vector<uint8_t>& binary_array);

        carma_v2x_msgs::msg::AccelerationSet4Way decode_accel_set_message(AccelerationSet4Way_t& message);

        carma_v2x_msgs::msg::PathHistory decode_path_history_message(PathHistory_t& message);

        carma_v2x_msgs::msg::PathPrediction decode_path_prediction_message(PathPrediction_t& message);

        j2735_v2x_msgs::msg::PropelledInformation decode_propulsion_message(PropelledInformation_t& message);

        j2735_v2x_msgs::msg::PersonalDeviceUsageState decode_use_state(PersonalDeviceUsageState_t& message);

        //Encode Optional
        AccelerationSet4Way_t encode_accel_set(carma_v2x_msgs::msg::AccelerationSet4Way& message);


        /**
         * @brief helper functions for BSM message encoding.
         * @param plainMessage contains BSM ros message.
         * @return encoded byte array, returns ROS warning and an empty optional if encoding fails. 
         */
        boost::optional<std::vector<uint8_t>> encode_psm_message(const carma_v2x_msgs::msg::PSM& plainMessage);

    };
}