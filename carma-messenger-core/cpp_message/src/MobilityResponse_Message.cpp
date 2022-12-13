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

#include "cpp_message/MobilityResponse_Message.h"
#include "cpp_message/MobilityHeader_Message.h"

namespace cpp_message
{
    boost::optional<carma_v2x_msgs::msg::MobilityResponse> Mobility_Response::decode_mobility_response_message(std::vector<uint8_t>& binary_array)
    {
        carma_v2x_msgs::msg::MobilityHeader header;
        carma_v2x_msgs::msg::MobilityResponse output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message=nullptr;

        //copy from vector to array         
        size_t len=binary_array.size();

        uint8_t buf[len];        
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
        if(rval.code==RC_OK){
            
            Mobility_Header Header_constant;
            std::string sender_id, recipient_id, sender_bsm_id, plan_id;
            uint64_t timestamp;
            //get sender id
            size_t str_len=message->value.choice.TestMessage01.header.hostStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len>=Header_constant.STATIC_ID_MIN_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage01.header.hostStaticId.buf[i];
                }
            }
            else sender_id=Header_constant.STRING_DEFAULT;

            header.sender_id=sender_id;

            //get recepient id
            str_len=message->value.choice.TestMessage01.header.targetStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len>=Header_constant.STATIC_ID_MIN_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage01.header.targetStaticId.buf[i];
                }
            }
            else recipient_id=Header_constant.STRING_DEFAULT;

            header.recipient_id=recipient_id;
            
            //get bsm id
            str_len=message->value.choice.TestMessage01.header.hostBSMId.size;
            for(size_t i=0;i<str_len;i++){
                sender_bsm_id +=message->value.choice.TestMessage01.header.hostBSMId.buf[i];
            }

            if(str_len<Header_constant.BSM_ID_LENGTH){
                sender_bsm_id=std::string((Header_constant.BSM_ID_DEFAULT.size()-str_len),'0').append(sender_bsm_id);
            }
            else if(str_len>Header_constant.BSM_ID_LENGTH){
                RCLCPP_WARN(node_logging_->get_logger(),"BSM ID -size greater than limit, changing to default");
                sender_bsm_id=Header_constant.BSM_ID_DEFAULT;
            }
            
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

            output.m_header=header;

            //get urgency from long to uint16
            long tmp=message->value.choice.TestMessage01.body.urgency;
            if(tmp>URGENCY_MAX || tmp<URGENCY_MIN){
                RCLCPP_WARN_STREAM( node_logging_->get_logger(), "Urgency message out of range");
                ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
                return boost::optional<carma_v2x_msgs::msg::MobilityResponse>{};
            }
            output.urgency=tmp;

            //get isaccepted bool
            bool isAccepted=message->value.choice.TestMessage01.body.isAccepted;
            output.is_accepted=isAccepted;

            //get the mobility plan choice
            MobilityPlanType_t planType_choice = message->value.choice.TestMessage01.body.planType;
            output.plan_type.type = planType_choice;
            
            //get the mobility reason choice
            MobilityReason_t reason_choice = *message->value.choice.TestMessage01.body.reason;
            output.reason.reason = reason_choice;
            
            //get the mobility repeat choice
            MobilityRepeat_t repeat_choice = *message->value.choice.TestMessage01.body.repeat;
            output.repeat.repeat = repeat_choice;
            
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<carma_v2x_msgs::msg::MobilityResponse>(output);
        }
        //else return an empty object
        RCLCPP_WARN_STREAM( node_logging_->get_logger(), "Decoding mobility response message failed");
        ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
        return boost::optional<carma_v2x_msgs::msg::MobilityResponse>{};
    }

    boost::optional<std::vector<uint8_t>> Mobility_Response::encode_mobility_response_message(carma_v2x_msgs::msg::MobilityResponse plainMessage)
    {
        //encode result placeholder
        uint8_t buffer[1472];
        size_t buffer_size=sizeof(buffer);
        asn_enc_rval_t ec;
        auto message_shared = std::make_shared<MessageFrame_t>();
        MessageFrame_t* message = message_shared.get();
        //set message type to TestMessage01
        message->messageId=MOBILITY_RESPONSE_TEST_ID; 
        message->value.present=MessageFrame__value_PR_TestMessage01;

        //convert host_id string to char array
        Mobility_Header Header;
        std::string sender_id=plainMessage.m_header.sender_id;
        size_t string_size=sender_id.size();
        if(string_size<Header.STATIC_ID_MIN_LENGTH || string_size>Header.STATIC_ID_MAX_LENGTH){
            RCLCPP_WARN(node_logging_->get_logger(),"Unacceptable host id value, changing to default");
            sender_id=Header.STRING_DEFAULT;
            string_size=Header.STRING_DEFAULT.size();
        }
        uint8_t string_content_hostId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_hostId[i]=sender_id[i];
        }
        message->value.choice.TestMessage01.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage01.header.hostStaticId.size=string_size;
        //convert target_id string to char array
        std::string recipient_id=plainMessage.m_header.recipient_id;
        string_size=recipient_id.size();
        if(string_size<Header.STATIC_ID_MIN_LENGTH || string_size>Header.STATIC_ID_MAX_LENGTH){
            RCLCPP_WARN(node_logging_->get_logger(),"Unacceptable recipient id value, changing to default");
            recipient_id=Header.STRING_DEFAULT;
            string_size=Header.STRING_DEFAULT.size();
        }
        uint8_t string_content_targetId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_targetId[i]=recipient_id[i];
        }
        message->value.choice.TestMessage01.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage01.header.targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
        std::string sender_bsm_id=plainMessage.m_header.sender_bsm_id;
        string_size=sender_bsm_id.size();
        if(string_size<Header.BSM_ID_DEFAULT.size()){
            sender_bsm_id=std::string((Header.BSM_ID_DEFAULT.size()-string_size),'0').append(sender_bsm_id);
        }
        else if(string_size>Header.BSM_ID_DEFAULT.size()){
            RCLCPP_WARN(node_logging_->get_logger(),"Unacceptable BSM ID, changing to default");
            sender_bsm_id=Header.BSM_ID_DEFAULT;
        }
        string_size=Header.BSM_ID_DEFAULT.size();
        uint8_t string_content_BSMId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=sender_bsm_id[i];
        }
        message->value.choice.TestMessage01.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage01.header.hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        std::string plan_id=plainMessage.m_header.plan_id;
        string_size=plan_id.size();
        if(string_size!=Header.GUID_LENGTH){
            RCLCPP_WARN(node_logging_->get_logger(),"Unacceptable GUID, changing to default");
            plan_id=Header.GUID_DEFAULT;
            string_size=Header.GUID_LENGTH;
        }
        uint8_t string_content_planId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_planId[i]=plan_id[i];
        }
        message->value.choice.TestMessage01.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage01.header.planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.m_header.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        if(string_size<Header.TIMESTAMP_MESSAGE_LENGTH){
            timestamp=std::string(Header.TIMESTAMP_MESSAGE_LENGTH-string_size,'0').append(timestamp);
        }
        else if(string_size>Header.TIMESTAMP_MESSAGE_LENGTH){
            RCLCPP_WARN(node_logging_->get_logger(),"Unacceptable timestamp value, changing to default");
            timestamp=std::string(Header.TIMESTAMP_MESSAGE_LENGTH,'0');
        }
        string_size=Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t string_content_timestamp[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage01.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage01.header.timestamp.size=string_size;

        //get urgency
        uint16_t urgency=plainMessage.urgency;
        if(urgency<URGENCY_MIN || urgency> URGENCY_MAX) urgency=URGENCY_UNKNOWN;
        message->value.choice.TestMessage01.body.urgency=urgency; 
        //get isAccepted
        message->value.choice.TestMessage01.body.isAccepted=plainMessage.is_accepted;
        //get plantype
        message->value.choice.TestMessage01.body.planType = plainMessage.plan_type.type;
        //get reason
        long reason_choice = plainMessage.reason.reason;
        message->value.choice.TestMessage01.body.reason = &reason_choice;
        //get repeat
        long repeat_choice = plainMessage.repeat.repeat;
        message->value.choice.TestMessage01.body.repeat = &repeat_choice;

        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);

        //log a warning if that fails
        if(ec.encoded == -1) {
            RCLCPP_WARN_STREAM( node_logging_->get_logger(), "Encoding for Mobility Response Message failed");
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
        
        // for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);

    }


}
