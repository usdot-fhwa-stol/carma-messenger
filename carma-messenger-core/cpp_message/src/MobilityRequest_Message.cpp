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
 * CPP File containing Mobility Request Message method implementations
 */

#include "MobilityRequest_Message.h"
#include "MobilityHeader_Message.h"
#include <algorithm>

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
        size_t len=binary_array.size();

        uint8_t buf[len];
        std::copy(binary_array.begin(),binary_array.end(),buf);
        
        //use asn1c lib to decode
        rval=uper_decode(0, &asn_DEF_MessageFrame, (void **) &message,buf, len,0,0);
        if(rval.code==RC_OK){
            Mobility_Header Header_constant;
            std::string sender_id, recipient_id, sender_bsm_id, plan_id;
            uint64_t timestamp;
            //get sender id
            size_t str_len=message->value.choice.TestMessage00.header.hostStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len>=Header_constant.STATIC_ID_MIN_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    sender_id +=message->value.choice.TestMessage00.header.hostStaticId.buf[i];
                }
            }
            else sender_id=Header_constant.STRING_DEFAULT;

            header.sender_id=sender_id;

            //get recepient id
            str_len=message->value.choice.TestMessage00.header.targetStaticId.size;
            if(str_len<=Header_constant.STATIC_ID_MAX_LENGTH && str_len>=Header_constant.STATIC_ID_MIN_LENGTH)
            {
                for(size_t i=0;i<str_len;i++){
                    recipient_id +=message->value.choice.TestMessage00.header.targetStaticId.buf[i];
                }
            }
            else recipient_id=Header_constant.STRING_DEFAULT;

            header.recipient_id=recipient_id;
            
            //get bsm id
            str_len=message->value.choice.TestMessage00.header.hostBSMId.size;
            for(size_t i=0;i<str_len;i++){
                sender_bsm_id +=message->value.choice.TestMessage00.header.hostBSMId.buf[i];
            }

            if(str_len<Header_constant.BSM_ID_LENGTH){
                sender_bsm_id=std::string((Header_constant.BSM_ID_LENGTH-str_len),'0').append(sender_bsm_id);
            }
            else if (str_len>Header_constant.BSM_ID_LENGTH){
                ROS_WARN("BSM ID -size greater than limit, changing to default");
                sender_bsm_id=Header_constant.BSM_ID_DEFAULT;
            }
            
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
            char timestamp_ch[str_len];
            for(size_t i=0;i<str_len;i++){
                timestamp_ch[i]=message->value.choice.TestMessage00.header.timestamp.buf[i];
            }
            timestamp=atoll(timestamp_ch);
            header.timestamp=timestamp;

            output.m_header=header;

            //strategy
            std::string strategy;
            str_len=message->value.choice.TestMessage00.body.strategy.size;
            if(str_len<=STRATEGY_MAX_LENGTH && str_len!=0)
            {
                for(size_t i=0;i<str_len;i++){
                    strategy +=message->value.choice.TestMessage00.body.strategy.buf[i];
                }
            }
            else strategy=Header_constant.STRING_DEFAULT;

            output.strategy=strategy;
            //plan type   
            MobilityPlanType_t type;
            type=message->value.choice.TestMessage00.body.planType;

            output.plan_type.type=message->value.choice.TestMessage00.body.planType;
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

            tmp_l=message->value.choice.TestMessage00.body.location.ecefZ;
            if(tmp>LOCATION_MAX_Z || tmp<LOCATION_MIN_Z){
                ROS_WARN_STREAM("Location ecefZ is out of range");
                return boost::optional<cav_msgs::MobilityRequest>{};
            }
            location.ecef_z=tmp_l;

            //recover uint64_t timestamp from string
            str_len=message->value.choice.TestMessage00.body.location.timestamp.size;
            uint64_t location_timestamp=0;
            char location_timestamp_ch[str_len];
            for(size_t i=0;i<str_len;i++){
                location_timestamp_ch[i]=message->value.choice.TestMessage00.body.location.timestamp.buf[i];
            }
            location_timestamp=atoll(location_timestamp_ch);
            location.timestamp=location_timestamp;

            output.location=location;    

            //strategy params
            std::string strategy_params;
            str_len=message->value.choice.TestMessage00.body.strategyParams.size;
            for(size_t i=0;i<str_len;i++){
                strategy_params +=message->value.choice.TestMessage00.body.strategyParams.buf[i];
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
            trajectory_start.ecef_x=tmp_t;

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
            uint64_t trajectory_timestamp=0;
            char trajectory_timestamp_ch[str_len];
            for(size_t i=0;i<str_len;i++){
                trajectory_timestamp_ch[i]=message->value.choice.TestMessage00.body.trajectoryStart->timestamp.buf[i];
            }
            trajectory_timestamp=atoll(trajectory_timestamp_ch);
            trajectory_start.timestamp=trajectory_timestamp;

            trajectory.location=trajectory_start;

            //Trajectory-offset
            int offset_count=message->value.choice.TestMessage00.body.trajectory->list.count;

            if(offset_count>MAX_POINTS_IN_MESSAGE){
            ROS_WARN_STREAM("offset count greater than 60.");
            return boost::optional<cav_msgs::MobilityRequest>{};
            } 

            for(size_t i=0;i<offset_count;i++){
                cav_msgs::LocationOffsetECEF Offsets;
                Offsets.offset_x=message->value.choice.TestMessage00.body.trajectory->list.array[i]->offsetX;
                if(Offsets.offset_x<OFFSET_MIN || Offsets.offset_x>OFFSET_MAX)  Offsets.offset_x=OFFSET_UNAVAILABLE;

                Offsets.offset_y=message->value.choice.TestMessage00.body.trajectory->list.array[i]->offsetY;
                if(Offsets.offset_y<OFFSET_MIN || Offsets.offset_y>OFFSET_MAX)  Offsets.offset_y=OFFSET_UNAVAILABLE;

                Offsets.offset_z=message->value.choice.TestMessage00.body.trajectory->list.array[i]->offsetZ;
                if(Offsets.offset_z<OFFSET_MIN || Offsets.offset_z>OFFSET_MAX)  Offsets.offset_z=OFFSET_UNAVAILABLE;

                trajectory.offsets.push_back(Offsets);
            }

            output.trajectory=trajectory;
            // //expiration time
            str_len=message->value.choice.TestMessage00.body.expiration->size;
            uint64_t expiration=0;
            char expiration_ch[str_len];
            for(size_t i=0;i<str_len;i++){
                expiration_ch[i]=message->value.choice.TestMessage00.body.expiration->buf[i];
            }
            expiration=atoll(expiration_ch);
            output.expiration=expiration;

            return boost::optional<cav_msgs::MobilityRequest>(output);
        }
        return boost::optional<cav_msgs::MobilityRequest>{};
    }

    boost::optional<std::vector<uint8_t>> Mobility_Request::encode_mobility_request_message(cav_msgs::MobilityRequest plainMessage)
    {
        uint8_t buffer[1472];
        size_t buffer_size=sizeof(buffer);
        asn_enc_rval_t ec;
        std::shared_ptr<MessageFrame_t>message_shared(new MessageFrame_t);
        //if mem allocation fails
        if(!message_shared)
        {
            ROS_WARN_STREAM("Cannot allocate mem for MobilityRequest message encoding");
            return boost::optional<std::vector<uint8_t>>{};            
        }
        MessageFrame_t* message=message_shared.get();
        message->messageId=MOBILITY_REQUEST_TEST_ID_;
        message->value.present=MessageFrame__value_PR_TestMessage00;

        //For Header
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
        message->value.choice.TestMessage00.header.hostStaticId.buf=string_content_hostId;
        message->value.choice.TestMessage00.header.hostStaticId.size=string_size;
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
        message->value.choice.TestMessage00.header.targetStaticId.buf=string_content_targetId;
        message->value.choice.TestMessage00.header.targetStaticId.size=string_size;
        
        //convert bsm_id string to char array
        std::string sender_bsm_id=plainMessage.m_header.sender_bsm_id;
        string_size=sender_bsm_id.size();
        if(string_size<Header.BSM_ID_LENGTH){
            sender_bsm_id=std::string((Header.BSM_ID_LENGTH-string_size),'0').append(sender_bsm_id);
        }
        else if(string_size>Header.BSM_ID_LENGTH){
            ROS_WARN("Unacceptable BSM ID, changing to default");
            sender_bsm_id=Header.BSM_ID_DEFAULT;
        }
        string_size=Header.BSM_ID_LENGTH;
        uint8_t string_content_BSMId[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_BSMId[i]=sender_bsm_id[i];
        }
        message->value.choice.TestMessage00.header.hostBSMId.buf=string_content_BSMId;
        message->value.choice.TestMessage00.header.hostBSMId.size=string_size;
        
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
        message->value.choice.TestMessage00.header.planId.buf=string_content_planId;
        message->value.choice.TestMessage00.header.planId.size=string_size;
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
        string_size=Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t string_content_timestamp[Header.TIMESTAMP_MESSAGE_LENGTH];
        for(size_t i=0;i<Header.TIMESTAMP_MESSAGE_LENGTH;i++)
        {
            string_content_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage00.header.timestamp.buf=string_content_timestamp;
        message->value.choice.TestMessage00.header.timestamp.size=string_size;
        //strategy
        std::string strategy=plainMessage.strategy;
        string_size=strategy.size();
        if(string_size<STRATEGY_MIN_LENGTH || string_size>STRATEGY_MAX_LENGTH){
            ROS_WARN("Unacceptable strategy value, changing to default");
            strategy=Header.STRING_DEFAULT;
            string_size=Header.STRING_DEFAULT.size();
        }        
        uint8_t string_content_strategy[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_content_strategy[i]=strategy[i];
        }
        message->value.choice.TestMessage00.body.strategy.buf=string_content_strategy;
        message->value.choice.TestMessage00.body.strategy.size=string_size;

        //plantype
            
        message->value.choice.TestMessage00.body.planType=plainMessage.plan_type.type;

        //urgency
        uint16_t urgency=plainMessage.urgency;
        if(urgency<URGENCY_MIN || urgency> URGENCY_MAX) urgency=cav_msgs::MobilityRequest::_plan_type_type::UNKNOWN;
        message->value.choice.TestMessage00.body.urgency=urgency;

        //location
        long location_val;
        location_val=plainMessage.location.ecef_x;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage00.body.location.ecefX=location_val;

        location_val=plainMessage.location.ecef_y;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefY is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage00.body.location.ecefY=location_val;

        location_val=plainMessage.location.ecef_z;
        if(location_val> LOCATION_MAX_Z || location_val<LOCATION_MIN_Z){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage00.body.location.ecefZ=location_val;

        uint64_t location_time=plainMessage.location.timestamp;
        std::string location_timestamp=std::to_string(location_time);
        size_t location_timestamp_string_size=location_timestamp.size();
        //append 0's if size less than required
        if(location_timestamp_string_size<Header.TIMESTAMP_MESSAGE_LENGTH){
            location_timestamp=std::string((Header.TIMESTAMP_MESSAGE_LENGTH-location_timestamp_string_size),'0').append(location_timestamp);
        }
        else if(location_timestamp_string_size>Header.TIMESTAMP_MESSAGE_LENGTH){
            ROS_WARN("Unacceptable location timestamp value, changing to default");
            location_timestamp=std::string(Header.TIMESTAMP_MESSAGE_LENGTH,'0');
        }
        location_timestamp_string_size=Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t string_content_location_timestamp[location_timestamp_string_size];
        for(size_t i=0;i<Header.TIMESTAMP_MESSAGE_LENGTH;i++)
        {
            string_content_location_timestamp[i]=location_timestamp[i];
        }
        message->value.choice.TestMessage00.body.location.timestamp.buf=string_content_location_timestamp;
        message->value.choice.TestMessage00.body.location.timestamp.size=location_timestamp_string_size;

        //strategyParams
        std::string params_string=plainMessage.strategy_params;
        size_t params_string_size=params_string.size();
        if(params_string_size<STRATEGY_PARAMS_MIN_LENGTH || params_string_size>STRATEGY_PARAMS_MAX_LENGTH){
            ROS_WARN("Unacceptable strategy_params value, changing to default");
            params_string=Header.STRING_DEFAULT;
            params_string_size=Header.STRING_DEFAULT.size();
        }
        uint8_t string_content_params[params_string_size];
        for(size_t i=0;i<params_string_size;i++)
        {
            string_content_params[i]=params_string[i];
        }
        message->value.choice.TestMessage00.body.strategyParams.buf=string_content_params;
        message->value.choice.TestMessage00.body.strategyParams.size=params_string_size;
        
        //Trajectory
            //trajectoryStart
        std::shared_ptr<MobilityLocation>trajectory_location_shared(new MobilityLocation);
        if(!trajectory_location_shared)
        {
            ROS_WARN_STREAM("Cannot allocate mem for trajectory.location encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        MobilityLocation* trajectory_location=trajectory_location_shared.get();
        long trajectory_start;
        trajectory_start=plainMessage.trajectory.location.ecef_x;
        if(trajectory_start> LOCATION_MAX || trajectory_start<LOCATION_MIN){
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
        uint64_t trajectory_time=plainMessage.trajectory.location.timestamp;
        std::string trajectory_timestamp=std::to_string(trajectory_time);
        size_t trajectory_timestamp_string_size=trajectory_timestamp.size();
        //append 0's if size is lower than required
        if(trajectory_timestamp_string_size<Header.TIMESTAMP_MESSAGE_LENGTH){
            trajectory_timestamp=std::string((Header.TIMESTAMP_MESSAGE_LENGTH-trajectory_timestamp_string_size),'0').append(timestamp);
        }
        else if(trajectory_timestamp_string_size>Header.TIMESTAMP_MESSAGE_LENGTH){
            ROS_WARN("Unacceptable trajectory timestamp value, changing to default");
            trajectory_timestamp=std::string(Header.TIMESTAMP_MESSAGE_LENGTH,'0');
        }
        trajectory_timestamp_string_size=Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t string_trajectory_timestamp[trajectory_timestamp_string_size];
        for(size_t i=0;i<Header.TIMESTAMP_MESSAGE_LENGTH;i++)
        {
            string_trajectory_timestamp[i]=trajectory_timestamp[i];
        }
        trajectory_location->timestamp.buf=string_trajectory_timestamp;
        trajectory_location->timestamp.size=trajectory_timestamp_string_size;
        message->value.choice.TestMessage00.body.trajectoryStart=trajectory_location;
            //trajectory offsets
        size_t offset_count=plainMessage.trajectory.offsets.size();
        
        if(offset_count>MAX_POINTS_IN_MESSAGE){
            ROS_WARN_STREAM("offset count greater than 60.");
            return boost::optional<std::vector<uint8_t>>{};
        }

        std::shared_ptr <MobilityLocationOffsets>offsets_list_shared ((MobilityLocationOffsets*)calloc(1,sizeof(MobilityLocationOffsets)),free);
        if(!offsets_list_shared)
        {
            ROS_WARN_STREAM("Cannot allocate mem for offsets list encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        MobilityLocationOffsets* offsets_list=offsets_list_shared.get();    

        for(size_t i=0;i<offset_count;i++){
            MobilityECEFOffset* Offsets=new MobilityECEFOffset;
            if(!Offsets)
            {
                ROS_WARN_STREAM("Cannot allocate mem for Offsets encoding");
                return boost::optional<std::vector<uint8_t>>{};
            }
            Offsets->offsetX=plainMessage.trajectory.offsets[i].offset_x;
            Offsets->offsetY=plainMessage.trajectory.offsets[i].offset_y;
            Offsets->offsetZ=plainMessage.trajectory.offsets[i].offset_z;
            asn_sequence_add(&offsets_list->list,Offsets);
            Offset_ptrs.push_back(Offsets);
        }

        message->value.choice.TestMessage00.body.trajectory=offsets_list;

        // //expiration
        uint64_t expiration_message=plainMessage.expiration;
        std::string expiration_string=std::to_string(expiration_message);
        size_t expiration_string_size=expiration_string.size();
        if(expiration_string_size<Header.TIMESTAMP_MESSAGE_LENGTH){
            expiration_string=std::string((Header.TIMESTAMP_MESSAGE_LENGTH-expiration_string_size),'0').append(expiration_string);
        }
        else if(string_size>Header.TIMESTAMP_MESSAGE_LENGTH){
            ROS_WARN("Unacceptable expiration time value, changing to default");
            expiration_string=std::string(Header.TIMESTAMP_MESSAGE_LENGTH,'0');
        }
        expiration_string_size=Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t expiration_array[expiration_string_size];  
        for(size_t i=0;i<expiration_string_size ;i++)
        {
            expiration_array[i]=expiration_string[i];
        }

        std::shared_ptr<MobilityTimestamp_t>expiration_time_shared(new MobilityTimestamp_t);
        if(!expiration_time_shared)
        {
            ROS_WARN_STREAM("Cannot allocate mem for expiration message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        MobilityTimestamp_t* expiration_time=expiration_time_shared.get();
        expiration_time->size=Header.TIMESTAMP_MESSAGE_LENGTH;
        expiration_time->buf=expiration_array;
        message->value.choice.TestMessage00.body.expiration=expiration_time;


        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame,0, message, buffer, buffer_size);
        if(ec.encoded==-1){
            return boost::optional<std::vector<uint8_t>>{}; 
        }     
        //copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i = 0; i < array_length; i++) 
        {
            b_array[i] = buffer[i];
        }

        return boost::optional<std::vector<uint8_t>>(b_array);

    }

    Mobility_Request::~Mobility_Request(){
        if(!Offset_ptrs.empty()){
            for(size_t i=0;i<Offset_ptrs.size();i++){
                delete Offset_ptrs[i];
            }
        }
    }

}
