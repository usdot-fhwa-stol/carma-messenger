#pragma once
/*
 * Copyright (C) 2020-2022 LEIDOS.
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
    class BSM_Message
    {
        private:
        //constants 
        static const int BSM_TEST_ID=20;
        
        public:

        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

        /**
        * \brief constructor 
        */
        explicit BSM_Message(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging) {

            node_logging_ = node_logging;
        };

        /**
         * @brief Converts PathHistory asn1 message to j2735_v2x_msg.
         * @param message PathHistory in asn.1 format.
         * @return carma_v2x_message version for PathHistory. 
         */
        j2735_v2x_msgs::msg::PathHistory decode_path_history_message(PathHistory_t& message);

        /**
         * @brief BSM message decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
         */
        boost::optional<j2735_v2x_msgs::msg::BSM>decode_bsm_message(std::vector<uint8_t>& binary_array);

        /**
         * @brief helper function for BSM message encoding.
         * @param plainMessage contains BSM ros message.
         * @return encoded byte array, returns ROS warning and an empty optional if encoding fails. 
         */
        boost::optional<std::vector<uint8_t>> encode_bsm_message(const j2735_v2x_msgs::msg::BSM& plainMessage);

        /**
         * @brief helper function for decoding a PivotPointDescription_t to a j2735_v2x_msgs::msg::PivotPointDescription
         * @param pivot_point_description The PivotPointDescription_t that will be decoded to a j2735_v2x_msgs::msg::PivotPointDescription
         * @return A j2735_v2x_msg::msg::PivotPointDescription with fields set according to the provided PivotPointDescription_t argument.
         */
        j2735_v2x_msgs::msg::PivotPointDescription decode_pivot_point_description(PivotPointDescription_t& pivot_point_description);

        /**
         * @brief helper function for creating a PivotPointDescription_t message.
         * @param pivot_offset contains the pivot_offset field for the PivotPointDescription_t
         * @param pivot_angle contains the pivot_angle field for the PivotPointDescription_t
         * @param pivoting_allowed contains the pivoting_allowed field for the PivotPointDescription_t
         * @return A PivotPointDescription_t message with fields set according to the provided arguments. 
         */
        PivotPointDescription_t encode_pivot_point_description(long& pivot_offset, long& pivot_angle, bool& pivoting_allowed);
    };
}