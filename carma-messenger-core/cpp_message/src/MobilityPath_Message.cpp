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
#include "MobilityPath_Message.h"

namespace cpp_message
{
    boost::optional<cav_msgs::MobilityPath> Mobility_Path::decode_mobility_path_message(std::vector<uint8_t>& binary_array)
    {
        cav_msgs::MobilityHeader header;
        cav_msgs::Trajectory trajectory;
        cav_msgs::MobilityPath output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message=0;

        //copy from vector to array         
        auto len=binary_array.size();
  
        uint8_t buf[len];
        for(auto i=0;i<len;i++){
            buf[i]=binary_array[i];
        }
        //use asn1c lib to decode
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);        
        if(rval.code==RC_OK){

            //For Header
            std::string sender_id, recipient_id, sender_bsm_id, plan_id, timestamp_string;
            uint64_t timestamp;
            auto str_len=message->value.choice.TestMessage02.header.hostStaticId.size;
            if(str_len<=STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(auto i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage02.header.hostStaticId.buf[i];
                }
            }
            else
            {
                sender_id=STRING_DEFAULT;
            }
            header.sender_id=sender_id;

            //get recepient id
            str_len=message->value.choice.TestMessage02.header.targetStaticId.size;
            if(str_len<=STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(auto i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage02.header.targetStaticId.buf[i];
                }
            }
            else
            {
                recipient_id=STRING_DEFAULT;
            }

            header.recipient_id=recipient_id;
            
            //get bsm id
            str_len=message->value.choice.TestMessage02.header.hostBSMId.size;
            if(str_len==BSM_ID_LENGTH)
            {
                for(auto i=0;i<str_len;i++){
                    sender_bsm_id +=message->value.choice.TestMessage02.header.hostBSMId.buf[i];
                }
            }
            else
            {
                sender_bsm_id=BSM_ID_DEFAULT;
            }
            
            header.sender_bsm_id=sender_bsm_id;

            //get plan id
            str_len=message->value.choice.TestMessage02.header.planId.size;
            if(str_len==GUID_LENGTH)
            {
                for(auto i=0;i<str_len;i++){
                    plan_id +=message->value.choice.TestMessage02.header.planId.buf[i];
                }
            }
            else
            {
                plan_id=GUID_DEFAULT;
            }
            header.plan_id=plan_id;
            
            //recover uint64_t timestamp from string
            str_len=message->value.choice.TestMessage02.header.timestamp.size;
            timestamp=0;
            for(auto i=0;i<str_len;i++){
                timestamp*=10;
                timestamp+=int(message->value.choice.TestMessage02.header.timestamp.buf[i])-'0';
            }
               header.timestamp=timestamp;

            output.header=header;
            
            //Trajectory
            //contains location and offset-decode to store in trajectory values
            cav_msgs::LocationECEF location;
            long tmp=message->value.choice.TestMessage02.body.location.ecefX;
            if(tmp>LOCATION_MAX || tmp<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefX is out of range");
                //return boost::optional<cav_msgs::MobilityPath>{};
            }
            location.ecef_x=tmp;

            tmp=message->value.choice.TestMessage02.body.location.ecefY;
            if(tmp>LOCATION_MAX || tmp<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefY is out of range");
                //return boost::optional<cav_msgs::MobilityPath>{};
            }
            location.ecef_y=tmp;

            tmp=message->value.choice.TestMessage02.body.location.ecefZ;
            if(tmp>LOCATION_MAX_Z || tmp<LOCATION_MIN_Z){
                ROS_WARN_STREAM("Location ecefZ is out of range");
                //return boost::optional<cav_msgs::MobilityPath>{};
            }
            location.ecef_z=tmp;
            
            //convert location timestamp from string in asn1 to uint64 for ros message
            str_len=message->value.choice.TestMessage02.body.location.timestamp.size;
            uint64_t location_timestamp=0;
            for(auto i=0;i<str_len;i++){
                location_timestamp*=10;
                location_timestamp+=int(message->value.choice.TestMessage02.body.location.timestamp.buf[i])-'0';
            }
            location.timestamp=location_timestamp;

            trajectory.location=location;
            
            //Trajectory-offset
            auto offset_count=message->value.choice.TestMessage02.body.trajectory.list.count;
            
            if(offset_count>MAX_POINTS_IN_MESSAGE){
            ROS_WARN_STREAM("offset count greater than 60.");
            return boost::optional<cav_msgs::MobilityPath>{};
            } 

            for(auto i=0;i<offset_count;i++){
                cav_msgs::LocationOffsetECEF Offsets;
                Offsets.offset_x=message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetX;
                if(Offsets.offset_x<OFFSET_MIN || Offsets.offset_x>OFFSET_MAX)Offsets.offset_x=OFFSET_UNAVAILABLE;
       
                Offsets.offset_y=message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetY;
                if(Offsets.offset_y<OFFSET_MIN || Offsets.offset_y>OFFSET_MAX)Offsets.offset_y=OFFSET_UNAVAILABLE;

                Offsets.offset_z=message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetZ;
                if(Offsets.offset_z<OFFSET_MIN || Offsets.offset_z>OFFSET_MAX)Offsets.offset_z=OFFSET_UNAVAILABLE;
                trajectory.offsets.push_back(Offsets);
            }

            output.trajectory=trajectory;
            

            return boost::optional<cav_msgs::MobilityPath>(output);
        }
        return boost::optional<cav_msgs::MobilityPath>{};
    }
    
    boost::optional<std::vector<uint8_t>> Mobility_Path::encode_mobility_path_message(cav_msgs::MobilityPath plainMessage)
    {
        
        uint8_t buffer[512];
        size_t buffer_size=sizeof(buffer);
        asn_enc_rval_t ec;
        MessageFrame_t* message;
        message=(MessageFrame_t*)calloc(1,sizeof(MessageFrame_t));
        //if mem allocation fails
        if(!message)
        {
            ROS_WARN_STREAM("Cannot allocate mem for MobilityPath message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        //set message type to TestMessage02
        message->messageId=242;
        message->value.present=MessageFrame__value_PR_TestMessage02;

        //For Header
         //convert host_id string to char array
        auto string_size=plainMessage.header.sender_id.size();
        uint8_t string_content_hostId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_hostId[i]=plainMessage.header.sender_id[i];
        }
        message->value.choice.TestMessage02.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage02.header.hostStaticId.size=string_size;
        //convert target_id string to char array
        string_size=plainMessage.header.recipient_id.size();
        uint8_t string_content_targetId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_targetId[i]=plainMessage.header.recipient_id[i];
        }
        message->value.choice.TestMessage02.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage02.header.targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
        string_size=plainMessage.header.sender_bsm_id.size();
        uint8_t string_content_BSMId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=plainMessage.header.sender_bsm_id[i];
        }
        message->value.choice.TestMessage02.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage02.header.hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        string_size=plainMessage.header.plan_id.size();
        uint8_t string_content_planId[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_planId[i]=plainMessage.header.plan_id[i];
        }
        message->value.choice.TestMessage02.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage02.header.planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.header.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_content_timestamp[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage02.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage02.header.timestamp.size=string_size;
        
        //location
        cav_msgs::LocationECEF starting_location;
        long location_val;
        location_val=plainMessage.trajectory.location.ecef_x;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefX is out of range");
            //return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefX=location_val;

        location_val=plainMessage.trajectory.location.ecef_y;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefY is out of range");
            //return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefY=location_val;

        location_val=plainMessage.trajectory.location.ecef_z;
        if(location_val> LOCATION_MAX_Z || location_val<LOCATION_MIN_Z){
            ROS_WARN_STREAM("Location ecefX is out of range");
            //return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefZ=location_val;

        //get location timestamp and convert to char array
        time=plainMessage.trajectory.location.timestamp;
        timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_location_timestamp[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage02.body.location.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage02.body.location.timestamp.size=string_size;

        //trajectory
        auto offset_count=plainMessage.trajectory.offsets.size();
        
        if(offset_count>MAX_POINTS_IN_MESSAGE){
            ROS_WARN_STREAM("offset count greater than 60.");
            return boost::optional<std::vector<uint8_t>>{};
        }

        MobilityLocationOffsets* offsets_list;
        offsets_list=(MobilityLocationOffsets*)calloc(1,sizeof(MobilityLocationOffsets));

        MobilityECEFOffset* Offsets;    
        for(int i=0;i<offset_count;i++){
            Offsets=(MobilityECEFOffset*)calloc(1,sizeof(MobilityECEFOffset));
            Offsets->offsetX=plainMessage.trajectory.offsets[i].offset_x;
            Offsets->offsetY=plainMessage.trajectory.offsets[i].offset_y;
            Offsets->offsetZ=plainMessage.trajectory.offsets[i].offset_z;
            asn_sequence_add(&offsets_list->list,Offsets);
        }

        message->value.choice.TestMessage02.body.trajectory=*offsets_list;
        
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame,0, message, buffer, buffer_size);
        //log a warning if it fails
        if(ec.encoded==-1){
            return boost::optional<std::vector<uint8_t>>{}; 
        }

        //copy to byte array msg
        auto array_length=ec.encoded/8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) b_array[i] = buffer[i];
        return boost::optional<std::vector<uint8_t>>(b_array);
    }
}