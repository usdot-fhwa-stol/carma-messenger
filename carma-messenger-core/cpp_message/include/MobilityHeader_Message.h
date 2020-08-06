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
    class Mobility_Header
    {
        private:
        static const int STATIC_ID_MAX_LENGTH=16;
        std::string BSM_ID_DEFAULT="00000000";
        const int BSM_ID_LENGTH=BSM_ID_DEFAULT.length();
        std::string STRING_DEFAULT="[]";
        const int TIMESTAMP_LENGTH=std::to_string(INT64_MAX).length();
        std::string GUID_DEFAULT= "00000000-0000-0000-0000-000000000000";
        const int GUID_LENGTH=GUID_DEFAULT.length();

        public:
        /**
         * @brief This function assigns the decoded MobilityHeader_t message to cav_msg::MobilityHeader data type.
         * @param message_header a pointer to asn1.c decoded Mobility Header message
         * @return  Mobility Header cav_msg for publishing as ros message. 
         * Since this function takes in a decoded message, only if it exists, it can never be NULL.
         */
        cav_msgs::MobilityHeader fromASN1_mobility_header_message(MobilityHeader_t *message_header);
        /**
         * @brief This function assigns MobilityHeader message to asn1.c MobilityHeader_t data type for encoding.
         * @param plainMessage Container with MobilityHeader ros message.
         * @return a pointer to the message stored in asn1.c MobilityHeader format, and an empty message if memory allocation for header message fails. 
         */
        boost::optional<MobilityHeader_t*> toASN1_mobility_header_message(cav_msgs::MobilityHeader plainMessage);
    };
}