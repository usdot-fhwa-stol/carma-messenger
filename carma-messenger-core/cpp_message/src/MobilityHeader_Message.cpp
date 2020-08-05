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

/**
 * CPP File containing Mobility Header Message method implementations
 */
#include "MobilityHeader_Message.h"


namespace cpp_message
{
    boost::optional<cav_msgs::MobilityHeader> Mobility_Header::decode_mobility_header_message(MobilityHeader_t *message_header){
        cav_msgs::MobilityHeader header;
        std::string sender_id, recipient_id, sender_bsm_id, plan_id, timestamp_string;
        uint64_t timestamp;
        auto str_len=message_header->hostStaticId.size;
        if(str_len<=STATIC_ID_MAX_LENGTH && str_len!=0)
        {
            for(auto i=0;i<str_len;i++){
                sender_id +=message_header->hostStaticId.buf[i];
            }
        }
        else
        {
            sender_id=STRING_DEFAULT;
        }
        header.sender_id=sender_id;

        //get recepient id
        str_len=message_header->targetStaticId.size;
        if(str_len<=STATIC_ID_MAX_LENGTH && str_len!=0)
        {
            for(auto i=0;i<str_len;i++){
                recipient_id +=message_header->targetStaticId.buf[i];
            }
        }
        else
        {
            recipient_id=STRING_DEFAULT;
        }

        header.recipient_id=recipient_id;
        
        //get bsm id
        str_len=message_header->hostBSMId.size;
        if(str_len==BSM_ID_LENGTH)
        {
            for(auto i=0;i<str_len;i++){
                sender_bsm_id +=message_header->hostBSMId.buf[i];
            }
        }
        else
        {
            sender_bsm_id=BSM_ID_DEFAULT;
        }
        
        header.sender_bsm_id=sender_bsm_id;

        //get plan id
        str_len=message_header->planId.size;
        if(str_len==GUID_LENGTH)
        {
            for(auto i=0;i<str_len;i++){
                plan_id +=message_header->planId.buf[i];
            }
        }
        else
        {
            plan_id=GUID_DEFAULT;
        }
        header.plan_id=plan_id;
        
        //recover uint64_t timestamp from string
        str_len=message_header->timestamp.size;
        timestamp=0;
        for(auto i=0;i<str_len;i++){
            timestamp*=10;
            timestamp+=int(message_header->timestamp.buf[i])-'0';
        }
        header.timestamp=timestamp;

        return header;
    }

    MobilityHeader_t* Mobility_Header::encode_mobility_header_message(cav_msgs::MobilityHeader plainMessage)
    {
        //return pointer to allocated memory
        MobilityHeader_t* header;
        header=(MobilityHeader_t*)calloc(1,sizeof(MobilityHeader_t));
        //For Header
         //convert host_id string to char array
        auto string_size=plainMessage.sender_id.size();
        uint8_t string_content_hostId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_hostId[i]=plainMessage.sender_id[i];
        }
        header->hostStaticId.buf=string_content_hostId;
        header->hostStaticId.size=string_size;
        //convert target_id string to char array
        string_size=plainMessage.recipient_id.size();
        uint8_t string_content_targetId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_targetId[i]=plainMessage.recipient_id[i];
        }
        header->targetStaticId.buf=string_content_targetId;
        header->targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
        string_size=plainMessage.sender_bsm_id.size();
        uint8_t string_content_BSMId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=plainMessage.sender_bsm_id[i];
        }
        header->hostBSMId.buf=string_content_BSMId;
        header->hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        string_size=plainMessage.plan_id.size();
        uint8_t string_content_planId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_planId[i]=plainMessage.plan_id[i];
        }
        header->planId.buf=string_content_planId;
        header->planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_content_timestamp[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        header->timestamp.buf=string_content_timestamp;
        header->timestamp.size=string_size;

        return header;
    }
}