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
    class Mobility_Operation
    {
        private:
        //constants
            static const int STRATEGY_MIN_LENGTH=2;
            static const int STRATEGY_MAX_LENGTH=50;
            static const int STRATEGY_PARAMS_MIN_LENGTH=2;
            static const int STRATEGY_PARAMS_MAX_LENGTH=1000;
            static const int MOBILITY_OPERATION_TEST_ID=243;
            std::string STRATEGY_PARAMS_STRING_DEFAULT="[]";
        
        public:
        /**
         * @brief Mobility Operation message decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
         */
        boost::optional<cav_msgs::MobilityOperation>decode_mobility_operation_message(std::vector<uint8_t>& binary_array);
        /**
         * @brief helper functions for Mobility Operation message encoding.
         * @param plainMessage contains mobility operation ros message.
         * @return encoded byte array, returns ROS warning and an empty optional if encoding fails. 
         */
        boost::optional<std::vector<uint8_t>> encode_mobility_operation_message(cav_msgs::MobilityOperation plainMessage);

        /**
         * @brief MobilityOperation Binary to Hex String Conversion Function
         * @param digits Binary input string
         * @return converted binary string into hexadecimal format
        */        
        std::string bintohex(std::string digits);

        /**
         * @brief MobilityOperation Hex String to Binary String Conversion Function
         * @param hex_string Hex string input
         * @return converted hexadecimal string into binary
        */
        std::string hextobin(std::string hex_string);

    };
}