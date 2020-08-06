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
 * CPP File containing Mobility Path Message method implementations
 */
#include "MobilityPath_Message.h"
#include "MobilityHeader_Message.h"

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
        size_t len=binary_array.size();
  
        uint8_t buf[len];
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);        
        if(rval.code==RC_OK){

            //For Header
            MobilityHeader_t* header;
            header=(MobilityHeader*)calloc(1,sizeof(Mobility_Header));

            header->hostStaticId=message->value.choice.TestMessage02.header.hostStaticId;
            header->targetStaticId=message->value.choice.TestMessage02.header.targetStaticId;
            header->hostBSMId=message->value.choice.TestMessage02.header.hostBSMId;
            header->planId=message->value.choice.TestMessage02.header.planId;
            header->timestamp=message->value.choice.TestMessage02.header.timestamp;

            Mobility_Header decode_header;
            cav_msgs::MobilityHeader header_ros=decode_header.fromASN1_mobility_header_message(header);
            output.header=header_ros;

            //Trajectory
            cav_msgs::LocationECEF location;
            long tmp=message->value.choice.TestMessage02.body.location.ecefX;
            if(tmp>LOCATION_MAX || tmp<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefX is out of range");
                return boost::optional<cav_msgs::MobilityPath>{};
            }
            location.ecef_x=tmp;

            tmp=message->value.choice.TestMessage02.body.location.ecefY;
            if(tmp>LOCATION_MAX || tmp<LOCATION_MIN){
                ROS_WARN_STREAM("Location ecefY is out of range");
                return boost::optional<cav_msgs::MobilityPath>{};
            }
            location.ecef_y=tmp;

            tmp=message->value.choice.TestMessage02.body.location.ecefZ;
            if(tmp>LOCATION_MAX_Z || tmp<LOCATION_MIN_Z){
                ROS_WARN_STREAM("Location ecefZ is out of range");
                return boost::optional<cav_msgs::MobilityPath>{};
            }
            location.ecef_z=tmp;
            
            //convert location timestamp from string in asn1 to uint64 for ros message
            size_t str_len=message->value.choice.TestMessage02.body.location.timestamp.size;
            uint64_t location_timestamp=0;
            for(size_t i=0;i<str_len;i++){
                location_timestamp*=10;
                location_timestamp+=int(message->value.choice.TestMessage02.body.location.timestamp.buf[i])-'0';
            }
            location.timestamp=location_timestamp;

            trajectory.location=location;
            
            //Trajectory-offset
            int offset_count=message->value.choice.TestMessage02.body.trajectory.list.count;
            
            if(offset_count>MAX_POINTS_IN_MESSAGE){
                ROS_WARN_STREAM("offset count greater than 60.");
                return boost::optional<cav_msgs::MobilityPath>{};
            } 

            for(int i=0;i<offset_count;i++){
                cav_msgs::LocationOffsetECEF Offsets;
                
                Offsets.offset_x=message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetX;
                if(Offsets.offset_x<OFFSET_MIN || Offsets.offset_x>OFFSET_MAX)
                {
                    Offsets.offset_x=OFFSET_UNAVAILABLE;
                }
                
                Offsets.offset_y=message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetY;
                if(Offsets.offset_y<OFFSET_MIN || Offsets.offset_y>OFFSET_MAX)
                {
                    Offsets.offset_y=OFFSET_UNAVAILABLE;
                }
                
                Offsets.offset_z=message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetZ;                
                if(Offsets.offset_z<OFFSET_MIN || Offsets.offset_z>OFFSET_MAX)
                {
                    Offsets.offset_z=OFFSET_UNAVAILABLE;
                }
                
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
        
        message->messageId=MOBILITYPATH_TEST_ID;
        message->value.present=MessageFrame__value_PR_TestMessage02;

        //send plainMessage to message header encode and get pointer
        cav_msgs::MobilityHeader Header;
        Header=plainMessage.header;
        Mobility_Header encode_header;
        MobilityHeader_t* header=(encode_header.toASN1_mobility_header_message(Header)).get();
        if(!header)
        {
            ROS_WARN_STREAM("Could not convert mobility header to asn1 format");
            return boost::optional<std::vector<uint8_t>>{};

        }
        message->value.choice.TestMessage02.header.hostStaticId.buf=header->hostStaticId.buf;
        message->value.choice.TestMessage02.header.hostStaticId.size=header->hostStaticId.size;

        message->value.choice.TestMessage02.header.targetStaticId.buf=header->targetStaticId.buf;
        message->value.choice.TestMessage02.header.targetStaticId.size=header->targetStaticId.size;

        message->value.choice.TestMessage02.header.hostBSMId.buf=header->hostBSMId.buf;
        message->value.choice.TestMessage02.header.hostBSMId.size=header->hostBSMId.size;

        message->value.choice.TestMessage02.header.planId.buf=header->planId.buf;
        message->value.choice.TestMessage02.header.planId.size=header->planId.size;
        
        message->value.choice.TestMessage02.header.timestamp.buf=header->timestamp.buf;
        message->value.choice.TestMessage02.header.timestamp.size=header->timestamp.size;

        
        //location
        cav_msgs::LocationECEF starting_location;
        long location_val;
        location_val=plainMessage.trajectory.location.ecef_x;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefX=location_val;

        location_val=plainMessage.trajectory.location.ecef_y;
        if(location_val> LOCATION_MAX || location_val<LOCATION_MIN){
            ROS_WARN_STREAM("Location ecefY is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefY=location_val;

        location_val=plainMessage.trajectory.location.ecef_z;
        if(location_val> LOCATION_MAX_Z || location_val<LOCATION_MIN_Z){
            ROS_WARN_STREAM("Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefZ=location_val;

        //get location timestamp and convert to char array
        uint64_t time=plainMessage.trajectory.location.timestamp;
        std::string timestamp=std::to_string(time);
        size_t string_size=timestamp.size();
        uint8_t string_location_timestamp[string_size];
        for(size_t i=0;i<string_size;i++)
        {
            string_location_timestamp[i]=timestamp[i];
        }
        message->value.choice.TestMessage02.body.location.timestamp.buf=string_location_timestamp;
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
        for(size_t i=0;i<offset_count;i++){
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
        size_t array_length=ec.encoded/8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i = 0; i < array_length; i++) b_array[i] = buffer[i];
        
        return boost::optional<std::vector<uint8_t>>(b_array);
    }
}