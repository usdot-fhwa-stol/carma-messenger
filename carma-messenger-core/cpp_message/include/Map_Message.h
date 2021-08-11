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
             * @brief Construct a new  map message object
             * 
             * @param binary_array Container with binary input.
             *  @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
             */
            boost::optional<j2735_msgs::MapData>decode_map_message(std::vector<uint8_t>& binary_array);

        
    };
}