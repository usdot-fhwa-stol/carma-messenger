#pragma once
/*
 * Copyright (C) 2023 LEIDOS.
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
    // Template to use when created shared pointer objects for optional data
    template <typename T>
    T *create_store_shared(std::vector<std::shared_ptr<void>> &shared_pointers)
    {
        auto obj_shared = std::make_shared<T>();
        shared_pointers.push_back(obj_shared);
        return obj_shared.get();
    }

    // Template for shared pointers with array elements
    template <typename T>
    T *create_store_shared_array(std::vector<std::shared_ptr<void>> &shared_pointers, int size)
    {
        std::shared_ptr<T[]> array_shared(new T[size]{0});
        shared_pointers.push_back(array_shared);
        return array_shared.get();
    }

    class SDSM_Message
    {
        private:

        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

        int SDSM_TEST_ID_ = 41;

        public:
        
        /**
        * \brief constructor 
        */
        explicit SDSM_Message(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging) {

            node_logging_ = node_logging;
        };

        /**
         * @brief helper function for SDSM message encoding.
         * @param plainMessage contains SDSM ros message.
         * @return encoded byte array, returns ROS warning and an empty optional if encoding fails. 
         */
        std::optional<std::vector<uint8_t>> encode_sdsm_message(const j3224_v2x_msgs::msg::SensorDataSharingMessage& plainMessage);

        /**
         * @brief SDSM decoding function.
         * @param binary_array Container with binary input.
         * @return decoded ros message, returns ROS warning and an empty optional if decoding fails. 
         */
        std::optional<j3224_v2x_msgs::msg::SensorDataSharingMessage>decode_sdsm_message(const std::vector<uint8_t>&binary_array);



    };

}