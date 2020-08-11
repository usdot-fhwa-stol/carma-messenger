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
 * CPP File containing Mobility Operation Message method implementations
 */

#include "MobilityResponse_Message.h"
#include "MobilityHeader_Message.h"

namespace cpp_message
{
    boost::optional<cav_msgs::MobilityResponse> Mobility_Response::decode_mobility_response_message(std::vector<uint8_t>& binary_array)
    {
        cav_msgs::MobilityHeader header;
        cav_msgs::MobilityResponse output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message=nullptr;

        //copy from vector to array         
        auto len=binary_array.size();

        uint8_t buf[len];             
        for(auto i=0;i < len;i++){
            buf[i]=binary_array[i];
        }   
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
        if(rval.code==RC_OK){
            
            Mobility_Header Header_constant;
            std::string sender_id, recipient_id, sender_bsm_id, plan_id;
            uint64_t timestamp;
            //get sender id
            size_t str_len=message->value.choice.TestMessage01.header.hostStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage01.header.hostStaticId.buf[i];
                }
            }
            else sender_id=Header_constant.STRING_DEFAULT;

            header.sender_id=sender_id;

            //get recepient id
            str_len=message->value.choice.TestMessage01.header.targetStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(size_t i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage01.header.targetStaticId.buf[i];
                }
            }
            else recipient_id=Header_constant.STRING_DEFAULT;

            header.recipient_id=recipient_id;
            
            //get bsm id
            str_len=message->value.choice.TestMessage01.header.hostBSMId.size;
            if(str_len==Header_constant.BSM_ID_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_bsm_id +=message->value.choice.TestMessage01.header.hostBSMId.buf[i];
                }
            }
            else sender_bsm_id=Header_constant.BSM_ID_DEFAULT;
            
            header.sender_bsm_id=sender_bsm_id;

            //get plan id
            str_len=message->value.choice.TestMessage01.header.planId.size;
            if(str_len==Header_constant.GUID_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    plan_id +=message->value.choice.TestMessage01.header.planId.buf[i];
                }
            }
            else plan_id=Header_constant.GUID_DEFAULT;

            header.plan_id=plan_id;
            
            //recover uint64_t timestamp from string
            str_len=message->value.choice.TestMessage01.header.timestamp.size;
            timestamp=0;
            for(size_t i=0;i<str_len;i++){
                timestamp*=10;
                timestamp+=int(message->value.choice.TestMessage01.header.timestamp.buf[i])-'0';
            }
            header.timestamp=timestamp;

            output.header=header;

            //get urgency from long to uint16
            long tmp=0;
            tmp=message->value.choice.TestMessage01.body.urgency;
            if(tmp>INT16_MAX){
                tmp=0;
            }
            output.urgency=tmp;
            //get isaccepted bool
            bool isAccepted=message->value.choice.TestMessage01.body.isAccepted;
            output.is_accepted=isAccepted;
            return boost::optional<cav_msgs::MobilityResponse>(output);
        }
        //else return an empty object
        ROS_WARN_STREAM("Decoding mobility response message failed");
        return boost::optional<cav_msgs::MobilityResponse>(output);
    }

    boost::optional<std::vector<uint8_t>> Mobility_Response::encode_mobility_response_message(cav_msgs::MobilityResponse plainMessage)
    {
        //encode result placeholder
        uint8_t buffer[512];
        size_t buffer_size=sizeof(buffer);
        asn_enc_rval_t ec;
        MessageFrame_t* message;
        message=(MessageFrame_t*)calloc(1, sizeof(MessageFrame_t));
        //if mem allocation fails
        if(!message)
        {
            ROS_WARN_STREAM("Cannot allocate mem for MobilityResponse message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        //set message type to TestMessage01
        message->messageId=MOBILITYRESPONSE_TEST_ID; 
        message->value.present=MessageFrame__value_PR_TestMessage01;

        //convert host_id string to char array
        size_t string_size=plainMessage.header.sender_id.size();
        uint8_t string_content_hostId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_hostId[i]=plainMessage.header.sender_id[i];
        }
        message->value.choice.TestMessage01.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage01.header.hostStaticId.size=string_size;
        //convert target_id string to char array
        string_size=plainMessage.header.recipient_id.size();
        uint8_t string_content_targetId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_targetId[i]=plainMessage.header.recipient_id[i];
        }
        message->value.choice.TestMessage01.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage01.header.targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
        string_size=plainMessage.header.sender_bsm_id.size();
        uint8_t string_content_BSMId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=plainMessage.header.sender_bsm_id[i];
        }
        message->value.choice.TestMessage01.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage01.header.hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        string_size=plainMessage.header.plan_id.size();
        uint8_t string_content_planId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_planId[i]=plainMessage.header.plan_id[i];
        }
        message->value.choice.TestMessage01.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage01.header.planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.header.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_content_timestamp[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage01.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage01.header.timestamp.size=string_size;

        //get urgency
        message->value.choice.TestMessage01.body.urgency=plainMessage.urgency; 
        //get isAccepted
        message->value.choice.TestMessage01.body.isAccepted=plainMessage.is_accepted;
        
        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);

        //log a warning if that fails
        if(ec.encoded == -1) {
            ROS_WARN_STREAM("Encoding for Mobility Response Message failed");
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //copy to byte array msg
        auto array_length=ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i=0;i<array_length;i++)b_array[i]=buffer[i];
        
        //for(auto i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);

    }


}