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

#include "Mobility_Operation_Message.h"

namespace Mobility_Operation
{
    void Mobility_Operation_Message::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        //initialize pub/sub
        outbound_binary_message_pub_=nh_->advertise<cav_msgs::ByteArray>("outbound_binary_msg",5);
        inbound_binary_message_sub_=nh_->subscribe("inbound_binary_msg",5, &Mobility_Operation_Message::inbound_binary_callback, this);
        mobility_operation_message_pub_=nh_->advertise<cav_msgs::ByteArray>("incoming_mobility_operation_message_decoded",5);
        mobility_operation_message_sub_=nh_->subscribe<>("outgoing_plain_mobility_operation_message",5, &Mobility_Operation_Message::outbound_mobility_operation_message_callback,this);
    }

    void Mobility_Operation_Message::inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg)
    {   //decode and publish as mobility operation message
        if(msg->messageType=="MobilityOperation")   
        {
            //store message content
            std::vector<uint8_t> array=msg->content;
            //decode the content
            auto output=decode_mobility_operation_message(array);
            if(output)
            {
                mobility_operation_message_pub_.publish(output.get());
            }
            else
            {
                ROS_WARN_STREAM("Cannot decode mobility operation message.");
            }
            
        }

    }

    int Mobility_Operation_Message::run(){
        initialize();
        ros::CARMANodeHandle::spin();
        return 0;
    }

    boost::optional<cav_msgs::MobilityOperation> Mobility_Operation_Message::decode_mobility_operation_message(std::vector<uint8_t>& binary_array){
        
        cav_msgs::MobilityOperation output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message=0;

        //copy from vector to array         
        uint8_t len=binary_array.size();    //error using auto
        uint8_t buf[len];             
        for(uint8_t i=0;i < len;i++){
            buf[i]=binary_array[i];
        }
        //use asn1c lib to decode
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);

        //if decode success
        if(rval.code==RC_OK){
            std::cout<<"rval ok"<<std::endl;
            //convert strategy from char array to string (TestMessage03 for MobilityOperation)
            std::string sender_id, recipient_id, sender_bsm_id, plan_id,timestamp_string, strategy,strategy_params;
            uint64_t timestamp;
            //get sender id
            auto str_len=message->value.choice.TestMessage03.header.hostStaticId.size;
            if(str_len<=STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(auto i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage03.header.hostStaticId.buf[i];
                }
            }
            else
            {
                sender_id=STRING_DEFAULT;
            }
            

            //get recepient id
            str_len=message->value.choice.TestMessage03.header.targetStaticId.size;
            if(str_len<=STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(auto i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage03.header.targetStaticId.buf[i];
                }
            }
            else
            {
                recipient_id=STRING_DEFAULT;
            }

            output.header.recipient_id=recipient_id;
            
            //get bsm id
            str_len=message->value.choice.TestMessage03.header.hostBSMId.size;
            if(str_len==BSM_ID_LENGTH)
            {
                for(auto i=0;i<str_len;i++){
                    sender_bsm_id +=message->value.choice.TestMessage03.header.hostBSMId.buf[i];
                }
            }
            else
            {
                sender_bsm_id=BSM_ID_DEFAULT;
            }
            
            output.header.sender_bsm_id=sender_bsm_id;

            //get plan id
            str_len=message->value.choice.TestMessage03.header.planId.size;
            if(str_len==GUID_LENGTH)
            {
                for(auto i=0;i<str_len;i++){
                    plan_id +=message->value.choice.TestMessage03.header.planId.buf[i];
                }
            }
            else
            {
                plan_id=GUID_DEFAULT;
            }
            output.header.plan_id=plan_id;

            //recover a long value from 8-bit array- timestamp  -TO DO
                timestamp=0;
                for(auto i=0;i<8;i++){
                    timestamp |=message->value.choice.TestMessage03.header.timestamp.buf[i];
                    timestamp=timestamp << 8;
                }
               output.header.timestamp=timestamp;

            //get strategy
            str_len=message->value.choice.TestMessage03.body.strategy.size;
            if(str_len<=STRATEGY_MAX_LENGTH && str_len!=0)
            {
                for(auto i=0;i<str_len;i++){
                    strategy +=message->value.choice.TestMessage03.body.strategy.buf[i];
                }
            }
            else
            {
                strategy=STRING_DEFAULT;
            }
            if(strategy==STRING_DEFAULT){
                strategy="";
            }
            
            output.strategy=strategy;

            //get strategy params
            str_len=message->value.choice.TestMessage03.body.operationParams.size;
            if(str_len <=STRATEGY_MAX_LENGTH && str_len!=0){
                for(auto i=0;i<str_len;i++){
                    strategy_params +=message->value.choice.TestMessage03.body.operationParams.buf[i];
                }
            }
            else
            {
                strategy_params=STRING_DEFAULT;
            }
            if(strategy_params==STRING_DEFAULT){
                strategy_params="";
            }
            output.strategy_params=strategy_params;
            

            return boost::optional<cav_msgs::MobilityOperation>(output);
        }
        return boost::optional<cav_msgs::MobilityOperation>{};

    }


    void Mobility_Operation_Message::outbound_mobility_operation_message_callback(const cav_msgs::MobilityOperation& msg)
    {   //encode and publish as outbound binary message
        //cav_msgs::MobilityOperation mobility_operation_msg(*msg.get());       //check
        auto res= encode_mobility_operation_message(msg);
        if(res)
        {
            //copy to byte array msg
            cav_msgs::ByteArray output;
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_.publish(output);
        }
        else
        {
            ROS_WARN_STREAM("Cannot encode geofence control message.");
        }
    }

    boost::optional<std::vector<uint8_t>> Mobility_Operation_Message::encode_mobility_operation_message(cav_msgs::MobilityOperation plainMessage)
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
            ROS_WARN_STREAM("Cannot allocate mem for MobilityOperation message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //set message type to TestMessage03
        message->messageId=243;  //From J2735 spec file
        message->value.present=MessageFrame__value_PR_TestMessage03;    

        //convert host_id string to char array
        auto string_size=plainMessage.header.sender_id.size();
        uint8_t string_content_hostId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_hostId[i]=plainMessage.header.sender_id[i];
        }
        message->value.choice.TestMessage03.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage03.header.hostStaticId.size=string_size;
        //convert target_id string to char array
        string_size=plainMessage.header.recipient_id.size();
        uint8_t string_content_targetId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_targetId[i]=plainMessage.header.recipient_id[i];
        }
        message->value.choice.TestMessage03.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage03.header.targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
        string_size=plainMessage.header.sender_bsm_id.size();
        uint8_t string_content_BSMId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=plainMessage.header.sender_bsm_id[i];
        }
        message->value.choice.TestMessage03.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage03.header.hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        string_size=plainMessage.header.plan_id.size();
        uint8_t string_content_planId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_planId[i]=plainMessage.header.plan_id[i];
        }
        message->value.choice.TestMessage03.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage03.header.planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.header.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_content_timestamp[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage03.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage03.header.timestamp.size=string_size;

        for(auto i=0;i<string_size;i++)std::cout<<message->value.choice.TestMessage03.header.timestamp.buf[i];
        std::cout<<std::endl;
         //convert strategy string to char array
        string_size=plainMessage.strategy.size();
        uint8_t string_content_strategy[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_strategy[i]=plainMessage.strategy[i];
        }
        message->value.choice.TestMessage03.body.strategy.buf=string_content_strategy;
        message->value.choice.TestMessage03.body.strategy.size=string_size;
        

        //convert parameters string to char array
        string_size=plainMessage.strategy_params.size();
        uint8_t string_content_params[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_params[i]=plainMessage.strategy_params[i];
        }
        message->value.choice.TestMessage03.body.operationParams.buf=string_content_params;
        message->value.choice.TestMessage03.body.operationParams.size=string_size;
        
        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
         
        //log a warning if that fails
        if(ec.encoded == -1) {
            return boost::optional<std::vector<uint8_t>>{};
        }
        ROS_WARN_STREAM("Reaching End");
        //copy to byte array msg
        auto array_length=ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i=0;i<array_length;i++)b_array[i]=buffer[i];
        
        return boost::optional<std::vector<uint8_t>>(b_array);
    }

}