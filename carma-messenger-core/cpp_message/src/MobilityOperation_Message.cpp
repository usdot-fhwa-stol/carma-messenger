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

/**
 * CPP File containing Mobility Operation Message method implementations
 */

#include "MobilityOperation_Message.h"
#include "MobilityHeader_Message.h"
#include <boost/algorithm/string/replace.hpp>

namespace cpp_message
{
    boost::optional<cav_msgs::MobilityOperation> Mobility_Operation::decode_mobility_operation_message(std::vector<uint8_t>& binary_array){
        
        cav_msgs::MobilityHeader header;
        cav_msgs::MobilityOperation output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message=nullptr;
        
        //copy from vector to array         
        size_t len=binary_array.size();    
        
        uint8_t buf[len];             
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
         
        //if decode success
        if(rval.code==RC_OK){
           
            //convert strategy from char array to string (TestMessage03 for MobilityOperation)
            Mobility_Header Header_constant;
            std::string sender_id, recipient_id, sender_bsm_id, plan_id,timestamp_string, strategy,strategy_params;
            uint64_t timestamp;
            //get sender id
            size_t str_len=message->value.choice.TestMessage03.header.hostStaticId.size;
            if(str_len>=Header_constant.STATIC_ID_MIN_LENGTH && str_len<=Header_constant.STATIC_ID_MAX_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage03.header.hostStaticId.buf[i];
                }
            }
            else sender_id=Header_constant.STRING_DEFAULT;

            header.sender_id=sender_id;

            //get recepient id
            str_len=message->value.choice.TestMessage03.header.targetStaticId.size;
            if(str_len>=Header_constant.STATIC_ID_MIN_LENGTH && str_len<=Header_constant.STATIC_ID_MAX_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage03.header.targetStaticId.buf[i];
                }
            }
            else recipient_id=Header_constant.STRING_DEFAULT;

            header.recipient_id=recipient_id;
            
            //get bsm id
            //sender_bsm_id is meant to represent the vehicle BSM id in hex string (Ex: FFFFFFFF)
            str_len=message->value.choice.TestMessage03.header.hostBSMId.size;
            for(size_t i=0;i<str_len;i++){
                sender_bsm_id +=message->value.choice.TestMessage03.header.hostBSMId.buf[i];
            }
            if(str_len<Header_constant.BSM_ID_LENGTH){
                sender_bsm_id=std::string((Header_constant.BSM_ID_LENGTH-str_len),'0').append(sender_bsm_id);
            }
            else if(str_len>Header_constant.BSM_ID_LENGTH){
                ROS_WARN("BSM ID -size greater than limit, changing to default");
                sender_bsm_id=Header_constant.BSM_ID_DEFAULT;
            }
            
            header.sender_bsm_id=sender_bsm_id;

            //get plan id
            str_len=message->value.choice.TestMessage03.header.planId.size;
            if(str_len==Header_constant.GUID_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    plan_id +=message->value.choice.TestMessage03.header.planId.buf[i];
                }
            }
            else plan_id=Header_constant.GUID_DEFAULT;

            header.plan_id=plan_id;
            
            //recover uint64_t timestamp from string
            str_len=message->value.choice.TestMessage03.header.timestamp.size;
            timestamp=0;
            char timestamp_ch[str_len];
            for(size_t i=0;i<str_len;i++){
                timestamp_ch[i]=message->value.choice.TestMessage03.header.timestamp.buf[i];
            }
            timestamp=atoll(timestamp_ch);
            header.timestamp=timestamp;
            output.m_header=header;

            //get strategy
            str_len=message->value.choice.TestMessage03.body.strategy.size;
            if(str_len>=STRATEGY_MIN_LENGTH && str_len<=STRATEGY_MAX_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    strategy +=message->value.choice.TestMessage03.body.strategy.buf[i];
                }
            }
            else strategy=Header_constant.STRING_DEFAULT;
            
            output.strategy=strategy;
            
            //get strategy params
            str_len=message->value.choice.TestMessage03.body.operationParams.size;
            if(str_len>=STRATEGY_PARAMS_MIN_LENGTH && str_len<=STRATEGY_PARAMS_MAX_LENGTH){
                for(size_t i=0;i<str_len;i++){
                    strategy_params +=message->value.choice.TestMessage03.body.operationParams.buf[i];
                }
                boost::replace_all(strategy_params, "&quot;", "\"");
            }
            else strategy_params=STRATEGY_PARAMS_STRING_DEFAULT;
            
            output.strategy_params=strategy_params;

            return boost::optional<cav_msgs::MobilityOperation>(output);
        }
        ROS_WARN_STREAM("mobility operation decoding failed");
        return boost::optional<cav_msgs::MobilityOperation>{};

    }

    boost::optional<std::vector<uint8_t>> Mobility_Operation::encode_mobility_operation_message(cav_msgs::MobilityOperation plainMessage)
    {
        //encode result placeholder
        uint8_t buffer[1472] = {0};
        size_t buffer_size=sizeof(buffer);
        
        asn_enc_rval_t ec;
        std::shared_ptr<MessageFrame_t>message_shared(new MessageFrame_t);
        //if mem allocation fails
        if(!message_shared)
        {
            ROS_WARN_STREAM("Cannot allocate mem for MobilityOperation message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        MessageFrame_t* message=message_shared.get();
        //set message type to TestMessage03
        message->messageId=MOBILITY_OPERATION_TEST_ID;  
        message->value.present=MessageFrame__value_PR_TestMessage03;    

        //convert host_id string to char array
        std::string sender_id=plainMessage.m_header.sender_id;
        Mobility_Header Header;
        size_t string_size=sender_id.size();
        if(string_size<Header.STATIC_ID_MIN_LENGTH || string_size>Header.STATIC_ID_MAX_LENGTH){
            ROS_WARN("Unacceptable host id value, changing to default");
            sender_id=Header.STRING_DEFAULT;
            string_size=Header.STRING_DEFAULT.size();
        }
        uint8_t string_content_hostId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_hostId[i]=sender_id[i];
        }
        message->value.choice.TestMessage03.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage03.header.hostStaticId.size=string_size;
        //convert target_id string to char array
        std::string recipient_id=plainMessage.m_header.recipient_id;
        string_size=recipient_id.size();
        if(string_size<Header.STATIC_ID_MIN_LENGTH || string_size>Header.STATIC_ID_MAX_LENGTH){
            ROS_WARN("Unacceptable recipient id value, changing to default");
            recipient_id=Header.STRING_DEFAULT;
            string_size=Header.STRING_DEFAULT.size();
        }
        uint8_t string_content_targetId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_targetId[i]=recipient_id[i];
        }
        message->value.choice.TestMessage03.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage03.header.targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
         //sender_bsm_id is meant to represent the vehicle BSM id in hex string (Ex: FFFFFFFF)
        std::string sender_bsm_id=plainMessage.m_header.sender_bsm_id;
        string_size=sender_bsm_id.size();
        if(string_size<Header.BSM_ID_LENGTH){
            sender_bsm_id=std::string((Header.BSM_ID_LENGTH-string_size),'0').append(sender_bsm_id);
        }
        else if(string_size>Header.BSM_ID_LENGTH){
            ROS_WARN("BSM ID greater than limit, changing to default");
            sender_bsm_id=Header.BSM_ID_DEFAULT;
        }
        string_size=Header.BSM_ID_LENGTH;
        uint8_t string_content_BSMId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=sender_bsm_id[i];
        }
        message->value.choice.TestMessage03.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage03.header.hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        std::string plan_id=plainMessage.m_header.plan_id;
        string_size=plan_id.size();
        if(string_size!=Header.GUID_LENGTH){
            ROS_WARN("Unacceptable GUID, changing to default");
            plan_id=Header.GUID_DEFAULT;
            string_size=Header.GUID_LENGTH;
        }
        uint8_t string_content_planId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_planId[i]=plan_id[i];
        }
        message->value.choice.TestMessage03.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage03.header.planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.m_header.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        if(string_size<Header.TIMESTAMP_MESSAGE_LENGTH){
            timestamp=std::string((Header.TIMESTAMP_MESSAGE_LENGTH-string_size),'0').append(timestamp);
        }
        else if(string_size>Header.TIMESTAMP_MESSAGE_LENGTH){
            ROS_WARN("Unacceptable timestamp value, changing to default");
            timestamp=std::string(Header.TIMESTAMP_MESSAGE_LENGTH,'0');
        }
        uint8_t string_content_timestamp[Header.TIMESTAMP_MESSAGE_LENGTH];
        for(size_t i=0;i<Header.TIMESTAMP_MESSAGE_LENGTH;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage03.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage03.header.timestamp.size=Header.TIMESTAMP_MESSAGE_LENGTH;

        //convert strategy string to char array
        std::string strategy=plainMessage.strategy;
        string_size=strategy.size();
        if(string_size<STRATEGY_MIN_LENGTH || string_size>STRATEGY_MAX_LENGTH){
            ROS_WARN("Unacceptable strategy_params value, changing to default");
            strategy=Header.STRING_DEFAULT;
            string_size=Header.STRING_DEFAULT.size();
        }        
        uint8_t string_content_strategy[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_strategy[i]=strategy[i];
        }
        message->value.choice.TestMessage03.body.strategy.buf=string_content_strategy;
        message->value.choice.TestMessage03.body.strategy.size=string_size;
        

        //convert parameters string to char array
        std::string strategy_params=plainMessage.strategy_params;
        string_size=strategy_params.size();
        if(string_size<STRATEGY_PARAMS_MIN_LENGTH || string_size>STRATEGY_PARAMS_MAX_LENGTH){
            ROS_WARN("Unacceptable strategy_params value, changing to default");
            strategy_params=STRATEGY_PARAMS_STRING_DEFAULT;
            string_size=STRATEGY_PARAMS_STRING_DEFAULT.size();
        }
        uint8_t string_content_params[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_params[i]=strategy_params[i];
        }
        message->value.choice.TestMessage03.body.operationParams.buf=string_content_params;
        message->value.choice.TestMessage03.body.operationParams.size=string_size;
        
        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
         
        //log a warning if that fails
        if(ec.encoded == -1) {
            ROS_WARN_STREAM("Encoding for Mobility Operation Message failed");
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
                
        //for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }
    
   


}
