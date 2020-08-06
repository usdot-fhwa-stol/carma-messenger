#pragma once
/*
 * Copyright (C) 2020 LEIDOS.
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
    class Mobility_Path
    {
        private:
        static const int MAX_POINTS_IN_MESSAGE=60; //The maximum number of points which can be included in a mobility message containing a trajectory over DSRC
        //Location Range for x and y
        static const long LOCATION_MIN=-638363700;
        static const long LOCATION_MAX=638363700;
        //Location Range for z
        static const long LOCATION_MIN_Z=-636225200;
        static const long LOCATION_MAX_Z=636225200;
        //Trajectory ranges
        static const int OFFSET_MIN=-500;
        static const int OFFSET_MAX=500;
        static const int OFFSET_UNAVAILABLE=501;

        static const int MOBILITYPATH_TEST_ID=242;
        public:
        /**
         * @brief Mobility Path message decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and empty message if decoding fails. 
         */
        boost::optional<cav_msgs::MobilityPath> decode_mobility_path_message(std::vector<uint8_t>& binary_array);
            /**
         * @brief Mobility Path message encoding function.
         * @param plainMessage Container with MobilityPath ros message.
         * @return decoded ros message, returns ROS warning and empty array if encoding fails. 
         */
        boost::optional<std::vector<uint8_t>> encode_mobility_path_message(cav_msgs::MobilityPath plainMessage);

    };
}