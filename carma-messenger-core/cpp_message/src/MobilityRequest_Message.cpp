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
 * CPP File containing Mobility Request Message method implementations
 */

#include "MobilityRequest_Message.h"
#include "MobilityHeader_Message.h"

namespace cpp_message
{
    boost::optional<cav_msgs::MobilityRequest> Mobility_Request::decode_mobility_request_message(std::vector<uint8_t>& binary_array)
    {
        cav_msgs::MobilityHeader header;
        cav_msgs::MobilityRequest output;

        //decode results - stored in binary array
        asn_dec_rval_t rval;
        MessageFrame_t* message=nullptr;

        //copy from vector to array
        auto len=binary_array.size();

        uint8_t buf[len];
        for(auto i=0;i<len;i++){
            buf[i]=binary_array[i];
        }
        
        //use asn1c lib to decode
        rval=uper_decode(0, &asn_DEF_MessageFrame, (void **) &message,buf, len,0,0);
        if(rval.code==RC_OK){
            Mobility_Header Header_constant;
            std::string sender_id, recipient_id, sender_bsm_id, plan_id;
            uint64_t timestamp;
            //get sender id
            size_t str_len=message->value.choice.TestMessage00.header.hostStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage00.header.hostStaticId.buf[i];
                }
            }
            else sender_id=Header_constant.STRING_DEFAULT;

            header.sender_id=sender_id;

            //get recepient id
            str_len=message->value.choice.TestMessage00.header.targetStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len!=0)
            {
                for(size_t i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage00.header.targetStaticId.buf[i];
                }
            }
            else recipient_id=Header_constant.STRING_DEFAULT;

            header.recipient_id=recipient_id;
            
            //get bsm id
            str_len=message->value.choice.TestMessage00.header.hostBSMId.size;
            if(str_len==Header_constant.BSM_ID_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_bsm_id +=message->value.choice.TestMessage00.header.hostBSMId.buf[i];
                }
            }
            else sender_bsm_id=Header_constant.BSM_ID_DEFAULT;
            
            header.sender_bsm_id=sender_bsm_id;

            //get plan id
            str_len=message->value.choice.TestMessage00.header.planId.size;
            if(str_len==Header_constant.GUID_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    plan_id +=message->value.choice.TestMessage00.header.planId.buf[i];
                }
            }
            else plan_id=Header_constant.GUID_DEFAULT;

            header.plan_id=plan_id;
            
            //recover uint64_t timestamp from string
            str_len=message->value.choice.TestMessage00.header.timestamp.size;
            timestamp=0;
            for(size_t i=0;i<str_len;i++){
                timestamp*=10;
                timestamp+=int(message->value.choice.TestMessage00.header.timestamp.buf[i])-'0';
            }
            header.timestamp=timestamp;

            output.header=header;

            //strategy
            std::string strategy;
            str_len=message->value.choice.TestMessage00.body.strategy.size;
            if(str_len<=STRATEGY_MAX_LENGTH && str_len!=0)
            {
                for(size_t i=0;i<str_len;i++){
                    strategy +=message->value.choice.TestMessage00.body.strategy.buf[i];
                }
            }
            else strategy="";

            if(strategy==STRING_DEFAULT){
                strategy="";
            }

            //plan type   
            MobilityPlanType_t type;
            type=message->value.choice.TestMessage00.body.planType;
            enum plan {UNKNOWN=0,CHANGE_LANE_LEFT=1,CHANGE_LANE_RIGHT=2,JOIN_PLATOON_AT_REAR=3,PLATOON_FOLLOWER_JOIN=4};
            //plan plan_type;
            uint8_t plan_type;
            switch(type){
                case 0:  
                    plan_type=UNKNOWN;
                    break;
                case 1:
                    plan_type=CHANGE_LANE_LEFT;
                    break;
                case 2:
                    plan_type=CHANGE_LANE_RIGHT;
                    break;
                case 3:
                    plan_type=JOIN_PLATOON_AT_REAR;
                    break;
                case 4:
                    plan_type=PLATOON_FOLLOWER_JOIN;
                    break;
                default:plan_type=UNKNOWN;
           }
            output.plan_type.type=plan_type;

            //urgency
            long tmp=message->value.choice.TestMessage00.body.urgency;
            if(tmp>URGENCY_MIN){
                if(tmp>URGENCY_MAX){
                    tmp=URGENCY_MAX;
                }
            }
            else tmp=URGENCY_MIN;
            output.urgency=tmp;

            //location
            cav_msgs::LocationECEF location;
            long tmp_l=message->value.choice.TestMessage00.body.location.ecefX;
            if(tmp_l>LOCATION_MAX || tmp_l<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefX is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            location.ecef_x=tmp_l;

            tmp_l=message->value.choice.TestMessage00.body.location.ecefY;
            if(tmp_l>LOCATION_MAX || tmp_l<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefY is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            location.ecef_y=tmp_l;

            tmp=message->value.choice.TestMessage00.body.location.ecefZ;
            if(tmp>LOCATION_MAX_Z || tmp<LOCATION_MIN_Z){
                ROS_WARN_STREAM("Location ecefZ is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            location.ecef_z=tmp_l;

            //recover uint64_t timestamp from string
            str_len=message->value.choice.TestMessage00.body.location.timestamp.size;
            timestamp=0;
            for(auto i=0;i<str_len;i++){
                timestamp*=10;
                timestamp+=int(message->value.choice.TestMessage00.body.location.timestamp.buf[i])-'0';
            }
            location.timestamp=timestamp;

            output.location=location;    

            //strategy params
            std::string strategy_params;
            str_len=message->value.choice.TestMessage00.body.strategyParams.size;
            for(auto i=0;i<str_len;i++){
                strategy_params +=message->value.choice.TestMessage00.body.strategyParams.buf[i];
            }
            if(strategy_params==STRING_DEFAULT){
                strategy_params="";
            }
            output.strategy_params=strategy_params;   

            //Trajectory
            cav_msgs::Trajectory trajectory;
            cav_msgs::LocationECEF trajectory_start;
            long tmp_t=message->value.choice.TestMessage00.body.trajectoryStart->ecefX;
            if(tmp_t>LOCATION_MAX || tmp_t<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefX is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            trajectory_start.ecef_x=tmp;

            tmp_t=message->value.choice.TestMessage00.body.trajectoryStart->ecefY;
            if(tmp_t>LOCATION_MAX || tmp_t<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefY is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            trajectory_start.ecef_y=tmp_t;

            tmp_t=message->value.choice.TestMessage00.body.trajectoryStart->ecefZ;
            if(tmp_t>LOCATION_MAX_Z || tmp_t<LOCATION_MIN_Z){
                ROS_WARN_STREAM("Location ecefZ is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            trajectory_start.ecef_z=tmp_t;

            //convert location timestamp from string in asn1 to uint64 for ros message
            str_len=message->value.choice.TestMessage00.body.trajectoryStart->timestamp.size;
            uint64_t location_timestamp=0;
            for(size_t i=0;i<str_len;i++){
                location_timestamp*=10;
                location_timestamp+=int(message->value.choice.TestMessage00.body.trajectoryStart->timestamp.buf[i])-'0';
            }
            trajectory_start.timestamp=location_timestamp;

            trajectory.location=trajectory_start;

            //Trajectory-offset
            auto offset_count=message->value.choice.TestMessage00.body.trajectory->list.count;
            
            if(offset_count>MAX_POINTS_IN_MESSAGE){
            ROS_WARN_STREAM("offset count greater than 60.");
            return boost::optional<cav_msgs::MobilityRequest>{};
            } 

            for(size_t i=0;i<offset_count;i++){
                cav_msgs::LocationOffsetECEF Offsets;
                Offsets.offset_x=message->value.choice.TestMessage00.body.trajectory->list.array[i]->offsetX;
                if(Offsets.offset_x<OFFSET_MIN || Offsets.offset_x>OFFSET_MAX)Offsets.offset_x=OFFSET_UNAVAILABLE;
       
                Offsets.offset_y=message->value.choice.TestMessage00.body.trajectory->list.array[i]->offsetY;
                if(Offsets.offset_y<OFFSET_MIN || Offsets.offset_y>OFFSET_MAX)Offsets.offset_y=OFFSET_UNAVAILABLE;

                Offsets.offset_z=message->value.choice.TestMessage00.body.trajectory->list.array[i]->offsetZ;
                if(Offsets.offset_z<OFFSET_MIN || Offsets.offset_z>OFFSET_MAX)Offsets.offset_z=OFFSET_UNAVAILABLE;
                trajectory.offsets.push_back(Offsets);
            }

            output.trajectory=trajectory;
            // //expiration time
            // str_len=message->value.choice.TestMessage00.body.expiration->size;
            // std::cout<<"Reaching here"<<std::endl;
            // uint64_t expiration=0;
            // for(auto i=0;i<str_len;i++){
            //     expiration*=10;
            //     expiration+=int(message->value.choice.TestMessage00.body.expiration->buf[i])-'0';
            // }
            // output.expiration=expiration;

            return boost::optional<cav_msgs::MobilityRequest>(output);
        }
        ROS_WARN_STREAM("Decoding mobility request failed");
        return boost::optional<cav_msgs::MobilityRequest>{};
    }

    boost::optional<std::vector<uint8_t>> Mobility_Request::encode_mobility_request_message(cav_msgs::MobilityRequest plainMessage)
    {
        uint8_t buffer[512];
        size_t buffer_size=sizeof(buffer);
        asn_enc_rval_t ec;
        MessageFrame_t* message;
        message=(MessageFrame_t*)calloc(1,sizeof(MessageFrame_t));
        //if mem allocation fails
        if(!message)
        {
            ROS_WARN_STREAM("Cannot allocate mem for MobilityRequest message encoding");
            return boost::optional<std::vector<uint8_t>>{};            
        }
        
        message->messageId=MOBILITY_REQUEST_MESSAGE_ID_;
        message->value.present=MessageFrame__value_PR_TestMessage00;

        //For Header
        //convert host_id string to char array
        size_t string_size=plainMessage.header.sender_id.size();
        uint8_t string_content_hostId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_hostId[i]=plainMessage.header.sender_id[i];
        }
        message->value.choice.TestMessage00.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage00.header.hostStaticId.size=string_size;
        //convert target_id string to char array
        string_size=plainMessage.header.recipient_id.size();
        uint8_t string_content_targetId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_targetId[i]=plainMessage.header.recipient_id[i];
        }
        message->value.choice.TestMessage00.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage00.header.targetStaticId.size=string_size;
        
         //convert bsm_id string to char array
        string_size=plainMessage.header.sender_bsm_id.size();
        uint8_t string_content_BSMId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=plainMessage.header.sender_bsm_id[i];
        }
        message->value.choice.TestMessage00.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage00.header.hostBSMId.size=string_size;
        
         //convert plan_id string to char array
        string_size=plainMessage.header.plan_id.size();
        uint8_t string_content_planId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_planId[i]=plainMessage.header.plan_id[i];
        }
        message->value.choice.TestMessage00.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage00.header.planId.size=string_size;
        //get timestamp and convert to char array
        uint64_t time=plainMessage.header.timestamp;
        std::string timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_content_timestamp[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage00.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage00.header.timestamp.size=string_size;
        //strategy
        string_size=plainMessage.strategy.size();
        uint8_t string_content_strategy[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_strategy[i]=plainMessage.strategy[i];
        }
        message->value.choice.TestMessage00.body.strategy.buf=string_content_strategy;
        message->value.choice.TestMessage00.body.strategy.size=string_size;

        //plantype
        MobilityPlanType_t type;
            enum plan {UNKNOWN=0,CHANGE_LANE_LEFT=1,CHANGE_LANE_RIGHT=2,JOIN_PLATOON_AT_REAR=3,PLATOON_FOLLOWER_JOIN=4};
            //plan plan_type;
            uint8_t plan_type=plainMessage.plan_type.type;
            switch(plan_type){
                case 0:  
                    type=UNKNOWN;
                    break;
                case 1:
                    type=CHANGE_LANE_LEFT;
                    break;
                case 2:
                    type=CHANGE_LANE_RIGHT;
                    break;
                case 3:
                    type=JOIN_PLATOON_AT_REAR;
                    break;
                case 4:
                    type=PLATOON_FOLLOWER_JOIN;
                    break;
                default:type=UNKNOWN;
           }
            
        message->value.choice.TestMessage00.body.planType=type;

        //urgency
        message->value.choice.TestMessage00.body.urgency=plainMessage.urgency;

        //location
        cav_msgs::LocationECEF starting_location;
        long location_val;
        location_val=plainMessage.trajectory.location.ecef_x;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage00.body.location.ecefX=location_val;

        location_val=plainMessage.trajectory.location.ecef_y;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefY is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage00.body.location.ecefY=location_val;

        location_val=plainMessage.trajectory.location.ecef_z;
        if(location_val> LOCATION_MAX_Z || location_val<LOCATION_MIN_Z){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage00.body.location.ecefZ=location_val;

        time=plainMessage.location.timestamp;
        timestamp=std::to_string(time);
        string_size=timestamp.size();
        string_content_timestamp[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage00.body.location.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage00.body.location.timestamp.size=string_size;

        //strategyParams
        string_size=plainMessage.strategy_params.size();
        uint8_t string_content_params[string_size];
        for(int i=0;i<string_size;i++)
        {
            string_content_params[i]=plainMessage.strategy_params[i];
        }
        message->value.choice.TestMessage00.body.strategyParams.buf=string_content_params;
        message->value.choice.TestMessage00.body.strategyParams.size=string_size;
        
        //Trajectory
            //trajectoryStart
        MobilityLocation* trajectory_location;
        trajectory_location=(MobilityLocation*)calloc(1,sizeof(MobilityLocation));
        long trajectory_start;
        trajectory_start=plainMessage.trajectory.location.ecef_x;
        if(trajectory_start> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        trajectory_location->ecefX=trajectory_start;
        
        trajectory_start=plainMessage.trajectory.location.ecef_y;
        if(trajectory_start> LOCATION_MAX || trajectory_start<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefY is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        trajectory_location->ecefY=trajectory_start;

        trajectory_start=plainMessage.trajectory.location.ecef_z;
        if(trajectory_start> LOCATION_MAX_Z || trajectory_start<LOCATION_MIN_Z){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        trajectory_location->ecefZ=trajectory_start;

        //get location timestamp and convert to char array
        time=plainMessage.trajectory.location.timestamp;
        timestamp=std::to_string(time);
        string_size=timestamp.size();
        uint8_t string_location_timestamp[string_size];
        for(auto i=0;i<string_size;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        trajectory_location->timestamp.buf=string_content_timestamp;
        trajectory_location->timestamp.size=string_size;
        message->value.choice.TestMessage00.body.trajectoryStart=trajectory_location;

            //trajectory offsets
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
        
        message->value.choice.TestMessage00.body.trajectory=offsets_list;

        // //expiration
        // uint64_t expiration_time=plainMessage.expiration;
        // std::string expiration_string=std::to_string(expiration_time);
        // size_t expiration_string_size=expiration_string.size();
        // uint8_t expiration_array[expiration_string_size];  
        // for(size_t i=0;i<expiration_string_size ;i++)
        // {
        //     expiration_array[i]=expiration_string[i];
        // }
        // message->value.choice.TestMessage00.body.expiration->size=expiration_string_size;
        // message->value.choice.TestMessage00.body.expiration->buf=expiration_array;

        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame,0, message, buffer, buffer_size);
        if(ec.encoded==-1){
            return boost::optional<std::vector<uint8_t>>{}; 
        }
        //copy to byte array msg
        auto array_length=ec.encoded/8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) 
        {
            b_array[i] = buffer[i];
            //std::cout<<int(b_array[i])<<",";
        }
        return boost::optional<std::vector<uint8_t>>(b_array);

    }
}
