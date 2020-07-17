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
 * CPP File containing Message method implementations
 */

#include "cpp_message.h"

namespace cpp_message
{

    void Message::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        // initialize pub/sub
        outbound_binary_message_pub_ = nh_->advertise<cav_msgs::ByteArray>("outbound_binary_msg", 5);
        inbound_binary_message_sub_ = nh_->subscribe("inbound_binary_msg", 5, &Message::inbound_binary_callback, this);
        outbound_geofence_request_message_sub_ = nh_->subscribe("outgoing_j2735_geofence_request", 5, &Message::outbound_control_request_callback, this);
        inbound_geofence_request_message_pub_ = nh_->advertise<j2735_msgs::ControlRequest>("incoming_j2735_geofence_request", 5);
        outbound_geofence_control_message_sub_ = nh_->subscribe("outgoing_j2735_geofence_control", 5, &Message::outbound_control_message_callback, this);
        inbound_geofence_control_message_pub_ = nh_->advertise<j2735_msgs::ControlMessage>("incoming_j2735_geofence_control", 5);
    }

    void Message::inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg)
    {
        // only handle ControlRequest for now
        if(msg->messageType == "TrafficControlRequest") {
            std::vector<uint8_t> array = msg->content;
            auto output = decode_geofence_request(array);
            if(output)
            {
                inbound_geofence_request_message_pub_.publish(output.get());
            } else
            {
                ROS_WARN_STREAM("Cannot decode geofence request message.");
            }
        }

            // handle ControlMessage
        else if(msg->messageType == "TrafficControlMessage") {
            std::vector<uint8_t> array = msg->content;
            auto output = decode_geofence_control(array);
            if(output)
            {
                inbound_geofence_control_message_pub_.publish(output.get());
            } else
            {
                ROS_WARN_STREAM("Cannot decode geofence control message.");
            }
        }
    }

    void Message::outbound_control_request_callback(const j2735_msgs::ControlRequestConstPtr& msg)
    {
        /* TODO: by Saina
        j2735_msgs::ControlRequest request_msg(*msg.get());
        auto res = encode_geofence_request(request_msg);
        if(res) {
            // copy to byte array msg
            cav_msgs::ByteArray output;
            output.content = res.get();
            // publish result
            outbound_binary_message_pub_.publish(output);
        } else
        {
            ROS_WARN_STREAM("Cannot encode geofence request message.");
        }
        */
    }

    void Message::outbound_control_message_callback(const j2735_msgs::ControlMessageConstPtr& msg)
    {
        j2735_msgs::ControlMessage control_msg(*msg.get());
        auto res = encode_geofence_control(control_msg);
        if(res) {
            // copy to byte array msg
            cav_msgs::ByteArray output;
            output.content = res.get();
            // publish result
            outbound_binary_message_pub_.publish(output);
        } else
        {
            ROS_WARN_STREAM("Cannot encode geofence control message.");
        }
    }

    int Message::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
        return 0;
    }

    boost::optional<j2735_msgs::ControlRequest> Message::decode_geofence_request(std::vector<uint8_t>& binary_array)
    {
        /* TODO by Saina
        j2735_msgs::ControlRequest output;
        // decode results
        asn_dec_rval_t rval;
        MessageFrame_t* message = 0;
        // copy from vector to array
        auto len = binary_array.size();
        uint8_t buf[len];
        for(auto i = 0; i < len; i++) {
            buf[i] = binary_array[i];
        }
        
        // use asn1c lib to decode
        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

        // if decode successed
        if(rval.code == RC_OK) {
            // convert version from char array to string
            std::string version;
            auto str_len = message->value.choice.TestMessage04.body.version.size;
            for(auto i = 0; i < str_len; i++)
            {
                version += message->value.choice.TestMessage04.body.version.buf[i];
            }
            output.version = version;
            // copy scale field
            output.scale = message->value.choice.TestMessage04.body.scale;
            // copy bounds
            auto bounds_count = message->value.choice.TestMessage04.body.bounds.list.count;
            for(auto i = 0; i < bounds_count; i++) {
                j2735_msgs::ControlBounds bound;
                // copy lat/lon
                bound.latitude = message->value.choice.TestMessage04.body.bounds.list.array[i]->lat;
                bound.longitude = message->value.choice.TestMessage04.body.bounds.list.array[i]->lon;
                // copy offset array to boost vector
                auto count = message->value.choice.TestMessage04.body.bounds.list.array[i]->offsets.list.count;
                for(auto j = 0; j < 3; j++) {
                    bound.offsets[j] = *message->value.choice.TestMessage04.body.bounds.list.array[i]->offsets.list.array[j];
                }
                // recover a long value from 8-bit array
                uint64_t long_bits = 0;
                auto bits_array_size = message->value.choice.TestMessage04.body.bounds.list.array[i]->oldest.size;
                for(auto j = 0; j < bits_array_size; j++) {
                    long_bits |= message->value.choice.TestMessage04.body.bounds.list.array[i]->oldest.buf[j];
                    long_bits = long_bits << 8;
                }
                bound.oldest = long_bits;
                output.bounds.push_back(bound);
            }
            return boost::optional<j2735_msgs::ControlRequest>(output);
        }
        */
        return boost::optional<j2735_msgs::ControlRequest>{};
    }

    boost::optional<j2735_msgs::ControlMessage> Message::decode_geofence_control(std::vector<uint8_t>& binary_array)
    {
        j2735_msgs::ControlMessage output;
        // decode results
        asn_dec_rval_t rval;
        MessageFrame_t* message = 0;
        // copy from vector to array
        auto len = binary_array.size();
        uint8_t buf[len];
        for(auto i = 0; i < len; i++) {
            buf[i] = binary_array[i];
        }
        
        // use asn1c lib to decode
        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

        // if decode successed
        if(rval.code == RC_OK) {
            // convert version from char array to string
            std::string version;
            auto str_len = message->value.choice.TestMessage05.body.version.size;
            for(auto i = 0; i < str_len; i++)
            {
                version += message->value.choice.TestMessage05.body.version.buf[i];
            }
            output.version = version;
            
            // decode id
            uint8_t id[16];
            auto id_len = message->value.choice.TestMessage05.body.id.size;
            for(auto i = 0; i < id_len; i++)
            {
                 output.id[i] = message->value.choice.TestMessage05.body.id.buf[i];
            }

            // recover a long value from 8-bit array
            uint64_t tmp_update=0;
            for (auto i=0; i<8; i++){
                tmp_update |= message->value.choice.TestMessage05.body.updated.buf[i];
                tmp_update = tmp_update << 8;
            }
            output.updated = tmp_update;
           
            // decode vtype list
            auto vtype_count = message->value.choice.TestMessage05.body.vtypes.list.size;
            for (auto i = 0; i < vtype_count; i++){
                j2735_msgs::VType v_type;
                v_type.vehicle_type = (long) message->value.choice.TestMessage05.body.vtypes.list.array[i];
                output.vtypes.push_back(v_type);
            }

            // recover schedule parameters
            j2735_msgs::Schedule schedule_tmp;


            // recover schedule start and end
            uint64_t schedule_start = 0;
            for (auto i=0; i<8; i++){
                schedule_start |= message->value.choice.TestMessage05.body.schedule.start.buf[i];
                schedule_start = schedule_start << 8;
            }
            schedule_tmp.start = schedule_start;

            uint64_t schedule_end = 0;
            for (auto i=0; i<8; i++){
                schedule_end |= message->value.choice.TestMessage05.body.schedule.end.buf[i];
                schedule_end = schedule_end << 8;
            }
            schedule_tmp.end = schedule_end;

            // recover the schedule dow array (optional)
            bool schedule_dow_exist = false;
            auto dow_count = message->value.choice.TestMessage05.body.schedule.dow->list.count;
            if (dow_count > 0){
                schedule_dow_exist = true;
                for (auto i=0; i<dow_count; i++){
                    schedule_tmp.dow[i] = *message->value.choice.TestMessage05.body.schedule.dow->list.array[i];
                }
            }
            schedule_tmp.dow_exists = schedule_dow_exist;

            // decode schedule between parameters
            bool schedule_between_exist = false;
            if (message->value.choice.TestMessage05.body.schedule.between){
                schedule_between_exist = true;
                
                j2735_msgs::DaySchedule schedule_between;

                schedule_between.start = message->value.choice.TestMessage05.body.schedule.between->start;
                schedule_between.end = message->value.choice.TestMessage05.body.schedule.between->end;
                schedule_between.utcoffset = message->value.choice.TestMessage05.body.schedule.between->utcoffset;  
                schedule_tmp.between = schedule_between;
            }
            schedule_tmp.between_exists = schedule_between_exist;

            
            bool repeat_exist = false;
                            
            // decode schedule repeat
            if (message->value.choice.TestMessage05.body.schedule.repeat){
                repeat_exist = true;
                j2735_msgs::ScheduleParams schedule_repeat;
                uint64_t repeat_interval = 0;
                uint64_t repeat_duration = 0;

                for (auto i=0; i<8; i++){
                    repeat_interval |= message->value.choice.TestMessage05.body.schedule.repeat->interval;//.buf[i];
                    repeat_interval = repeat_interval << 8;
                }

                for (auto i=0; i<8; i++){
                    repeat_duration |= message->value.choice.TestMessage05.body.schedule.repeat->duration;//.buf[i];
                    repeat_duration = repeat_duration << 8;
                }
                schedule_repeat.interval = repeat_interval;
                schedule_repeat.duration = repeat_duration;
                schedule_tmp.repeat = schedule_repeat;
            }
            
            schedule_tmp.repeat_exists = repeat_exist;
            output.schedule = schedule_tmp;

            // copy regulatory
            output.regulatory = message->value.choice.TestMessage05.body.regulatory;

            // copy control type
            j2735_msgs::ControlType ctrl_type;
            ctrl_type.control_type = message->value.choice.TestMessage05.body.controltype;
            output.control_type = ctrl_type;

            
            // copy control value (optional)
            if (message->value.choice.TestMessage05.body.controlvalue){
                output.control_value_exists = true;
                j2735_msgs::ControlValue ctrl_value;
                ctrl_value.value = message->value.choice.TestMessage05.body.controlvalue->choice.value;
                ctrl_value.direction = message->value.choice.TestMessage05.body.controlvalue->choice.direction;
                ctrl_value.lataffinity = message->value.choice.TestMessage05.body.controlvalue->choice.lataffinity;
                ctrl_value.perm = message->value.choice.TestMessage05.body.controlvalue->choice.perm;
                ctrl_value.prkingallowd = message->value.choice.TestMessage05.body.controlvalue->choice.prkingallowd;
                output.control_value = ctrl_value;

            }
            else output.control_value_exists = false;

            // copy path parts
            output.path_parts = message->value.choice.TestMessage05.body.pathParts;

            // convert proj from 8-bit array to string
            std::string proj;
            auto proj_len = message->value.choice.TestMessage05.body.proj.size;
            for(auto i = 0; i < proj_len; i++)
            {
                proj += message->value.choice.TestMessage05.body.proj.buf[i];
            }
            output.proj = proj;

            // convert datum from 8-bit array to string
            std::string datum;
            auto datum_len = message->value.choice.TestMessage05.body.datum.size;
            for(auto i = 0; i < datum_len; i++)
            {
                datum += message->value.choice.TestMessage05.body.datum.buf[i];
            }
            output.datum = datum;

            output.latitude = message->value.choice.TestMessage05.body.lat;
            output.longitude = message->value.choice.TestMessage05.body.lon;
            output.altitude = message->value.choice.TestMessage05.body.alt;
            output.heading = message->value.choice.TestMessage05.body.heading;

            // recover long int from 8-bit array
            uint64_t tmp_time = 0;
            for (auto i=0; i<8; i++){
                tmp_time |= message->value.choice.TestMessage05.body.time.buf[i];
                tmp_time = tmp_time << 8;
            }
            output.time = tmp_time;

            // recover points list
            auto points_count = message->value.choice.TestMessage05.body.points.list.count;
            for(auto i = 0; i < points_count; i++) {
                j2735_msgs::Point point;
                point.x = message->value.choice.TestMessage05.body.points.list.array[i]->x;
                point.y = message->value.choice.TestMessage05.body.points.list.array[i]->y;
                if (message->value.choice.TestMessage05.body.points.list.array[i]->z)
                    point.z = *message->value.choice.TestMessage05.body.points.list.array[i]->z;
                point.width = message->value.choice.TestMessage05.body.points.list.array[i]->width;
                output.points.push_back(point);
            }
            
            return boost::optional<j2735_msgs::ControlMessage>(output);
        }
        return boost::optional<j2735_msgs::ControlMessage>{};
    }

    boost::optional<std::vector<uint8_t>> Message::encode_geofence_request(j2735_msgs::ControlRequest request_msg)
    {
        /* TODO By Saina
        // encode result placeholder
        uint8_t buffer[512];
	    size_t buffer_size = sizeof(buffer);
	    asn_enc_rval_t ec;
	    MessageFrame_t* message;
	    message = (MessageFrame_t*)calloc(1, sizeof(MessageFrame_t));
        // if mem allocation fails
	    if (!message)
        {
		    ROS_WARN_STREAM("Cannot allocate mem for ControlRequest message encoding");
            return boost::optional<std::vector<uint8_t>>{};
	    }

	    //set message type to TestMessage04
	    message->messageId = 244;
        message->value.present = MessageFrame__value_PR_TestMessage04;        
        //convert version string to char array
        auto string_size = request_msg.version.size();
        uint8_t strategy_string_content[string_size];
        for(auto i = 0; i < string_size; i++)
        {
            strategy_string_content[i] = request_msg.version[i];
        }
        message->value.choice.TestMessage04.body.version.buf = strategy_string_content;
        message->value.choice.TestMessage04.body.version.size = string_size;
        // copy scale
        message->value.choice.TestMessage04.body.scale = request_msg.scale;
        // copy bounds
        auto count = request_msg.bounds.size();
        ControlRequest::ControlRequest__bounds* bounds_list;
        bounds_list = (ControlRequest::ControlRequest__bounds*)calloc(1, sizeof(ControlRequest::ControlRequest__bounds));
        for(auto i = 0; i < count; i++) {
            // construct control bounds
            ControlBounds_t* bounds_p;
            bounds_p = (ControlBounds_t*) calloc(1, sizeof(ControlBounds_t));
            bounds_p->lat = request_msg.bounds[i].latitude;
            bounds_p->lon = request_msg.bounds[i].longitude;
            // copy offsets from array to C list struct
            ControlBounds::ControlBounds__offsets* offsets;
            offsets = (ControlBounds::ControlBounds__offsets*)calloc(1, sizeof(ControlBounds::ControlBounds__offsets));
            auto offset_count = request_msg.bounds[i].offsets.size();
            for(auto j = 0; j < 3; j++) {
                int16_t temp = request_msg.bounds[i].offsets[j];
                asn_sequence_add(&offsets->list, &temp);
            }
            bounds_p->offsets = *offsets;
            //convert a long value to an 8-bit array of length 8
            uint8_t oldest_val[8];
            for(auto k = 7; k >= 0; k--) {
                // TODO this line needs to be tested
                oldest_val[7 - k] = request_msg.bounds[i].oldest >> (k * 8);
            }
            bounds_p->oldest.size = 8;
            bounds_p->oldest.buf = oldest_val;
            asn_sequence_add(&bounds_list->list, bounds_p);
        }
        message->value.choice.TestMessage04.body.bounds = *bounds_list;

	    // encode message
	    ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
        // log a warning if fails
        if(ec.encoded == -1) {
            return boost::optional<std::vector<uint8_t>>{};
        }
        // copy to byte array msg
        auto array_length = ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) b_array[i] = buffer[i];
        return boost::optional<std::vector<uint8_t>>(b_array);
        */
        return boost::optional<std::vector<uint8_t>>({});
    }

    boost::optional<std::vector<uint8_t>> Message::encode_geofence_control(j2735_msgs::ControlMessage control_msg)
    {
        // encode result placeholder
        uint8_t buffer[512];
	    size_t buffer_size = sizeof(buffer);
	    asn_enc_rval_t ec;
	    MessageFrame_t* message;
	    message = (MessageFrame_t*)calloc(1, sizeof(MessageFrame_t));
        // if mem allocation fails
	    if (!message)
        {
		    ROS_WARN_STREAM("Cannot allocate mem for ControlMessage message encoding");
            return boost::optional<std::vector<uint8_t>>{};
	    }

	    //set message type to TestMessage05
	    message->messageId = 245;
        message->value.present = MessageFrame__value_PR_TestMessage05;        
        //convert version string to char array
        auto string_size = control_msg.version.size();
        uint8_t version_content[string_size];
        for(auto i = 0; i < string_size; i++)
        {
            version_content[i] = control_msg.version[i];
        }
        message->value.choice.TestMessage05.body.version.buf = version_content;
        message->value.choice.TestMessage05.body.version.size = string_size;

        //convert id string to integer array
        uint8_t id_content[16];
        for(auto i = 0; i < 16; i++)
        {
            id_content[i] = control_msg.id[i];
        }

        message->value.choice.TestMessage05.body.id.buf = id_content;
        message->value.choice.TestMessage05.body.id.size = 16;

        // convert updated long value to an 8-bit array of length 8
        uint8_t updated_val[8];
        for(auto k = 7; k >= 0; k--) {
            updated_val[7 - k] = control_msg.updated >> (k * 8);
        }
        message->value.choice.TestMessage05.body.updated.buf = updated_val;
        message->value.choice.TestMessage05.body.updated.size = 8;

        // copy VTypes

        auto vtype_count = control_msg.vtypes.size();
        ControlMessage::ControlMessage__vtypes* vtype_list;
        vtype_list = (ControlMessage::ControlMessage__vtypes*)calloc(1, sizeof(ControlMessage::ControlMessage__vtypes));
        for(auto i = 0; i < vtype_count; i++) {
            // construct VType
            VType_t* vtype_p;
            vtype_p = (VType_t*) calloc(1, sizeof(VType_t));
            *vtype_p = control_msg.vtypes[i].vehicle_type;
            asn_sequence_add(&vtype_list->list, vtype_p);
        }
        message->value.choice.TestMessage05.body.vtypes = *vtype_list;
        
        // encode Schedule fields

        // convert schedule start and end time to 8 bit array
        Schedule_t* schedule_p;
        schedule_p = (Schedule_t*) calloc(1, sizeof(Schedule_t));
        uint8_t start_val[8];
        uint8_t end_val[8];
        for(auto k = 7; k >= 0; k--) {
            start_val[7 - k] = control_msg.schedule.start >> (k * 8);
            end_val[7 - k] = control_msg.schedule.end >> (k * 8);
        }
        schedule_p->start.size = 8;
        schedule_p->start.buf = start_val;
        schedule_p->end.size = 8;
        schedule_p->end.buf = end_val;

        // copy schedule day of week
        if (control_msg.schedule.dow_exists){
            Schedule::Schedule__dow* dow;
            dow = (Schedule::Schedule__dow*)calloc(1, sizeof(Schedule::Schedule__dow*));
            for(auto i = 0; i < 7; i++) {
                bool temp = control_msg.schedule.dow[i];
                asn_sequence_add(&dow->list, &temp);
            }
            schedule_p->dow = dow;
        }
        // copy schedule between
        if (control_msg.schedule.between_exists){
            DaySchedule_t* between_p;
            between_p = (DaySchedule_t*) calloc(1, sizeof(DaySchedule_t));
            between_p->start = control_msg.schedule.between.start;
            between_p->end = control_msg.schedule.between.end;
            between_p->utcoffset = control_msg.schedule.between.utcoffset;

            schedule_p->between = between_p;
        }

        // copy schedule repear
        if (control_msg.schedule.repeat_exists){
            ScheduleParams_t* repeat_p;
            repeat_p = (ScheduleParams_t*) calloc(1, sizeof(ScheduleParams_t));
            repeat_p->interval = control_msg.schedule.repeat.interval;
            repeat_p->duration = control_msg.schedule.repeat.duration;
            schedule_p->repeat = repeat_p;
        }

        message->value.choice.TestMessage05.body.schedule = *schedule_p;

        // copy regulatory
        message->value.choice.TestMessage05.body.regulatory = control_msg.regulatory;

        // copy control type
        message->value.choice.TestMessage05.body.controltype = control_msg.control_type.control_type;

        // copy control value
        if (control_msg.control_value_exists){
            ControlValue_t* ctrl_value_p;
            ctrl_value_p = (ControlValue_t*) calloc(1, sizeof(ControlValue_t));
            //TODO: Is there a better way to check which field is populated?
            if (sizeof(control_msg.control_value.value)>0.0){
                ctrl_value_p->present = ControlValue_PR_value;
                ctrl_value_p->choice.value = control_msg.control_value.value;
            }
            else if (sizeof(control_msg.control_value.direction)>0.0){
                ctrl_value_p->present = ControlValue_PR_direction;
                ctrl_value_p->choice.direction = control_msg.control_value.direction;
            }
            else if (sizeof(control_msg.control_value.lataffinity)>0.0){
                ctrl_value_p->present = ControlValue_PR_lataffinity;
                ctrl_value_p->choice.lataffinity = control_msg.control_value.lataffinity;
            }
            else if (sizeof(control_msg.control_value.perm)>0.0){
                ctrl_value_p->present = ControlValue_PR_perm;
                ctrl_value_p->choice.perm = control_msg.control_value.perm;
            }
            else if (sizeof(control_msg.control_value.prkingallowd)>0.0){
                ctrl_value_p->present = ControlValue_PR_prkingallowd;
                ctrl_value_p->choice.prkingallowd = control_msg.control_value.prkingallowd;
            }            
            message->value.choice.TestMessage05.body.controlvalue = ctrl_value_p;
        }
        // copy pathParts
        message->value.choice.TestMessage05.body.pathParts = control_msg.path_parts;
        
        //convert proj string to char array
        auto proj_size = control_msg.proj.size();
        uint8_t proj_content[proj_size];
        for(auto i = 0; i < proj_size; i++)
        {
            proj_content[i] = control_msg.proj[i];
        }
        message->value.choice.TestMessage05.body.proj.buf = proj_content;
        message->value.choice.TestMessage05.body.proj.size = proj_size;

        //convert datum string to char array
        auto datum_size = control_msg.datum.size();
        uint8_t datum_content[datum_size];
        for(auto i = 0; i < datum_size; i++)
        {
            datum_content[i] = control_msg.datum[i];
        }
        message->value.choice.TestMessage05.body.datum.buf = datum_content;
        message->value.choice.TestMessage05.body.datum.size = datum_size;

        // convert time long value to an 8-bit array of length 8
        uint8_t time_val[8];
        for(auto k = 7; k >= 0; k--) {
            time_val[7 - k] = control_msg.time >> (k * 8);
        }
        message->value.choice.TestMessage05.body.time.buf = updated_val;
        message->value.choice.TestMessage05.body.time.size = 8;


        // copy longitude
        message->value.choice.TestMessage05.body.lon = control_msg.longitude;

        // copy latitude
        message->value.choice.TestMessage05.body.lat = control_msg.latitude;

        // copy altitude
        message->value.choice.TestMessage05.body.alt = control_msg.altitude;

        // copy heading
        message->value.choice.TestMessage05.body.heading = control_msg.heading;

        // copy Points
        auto points_count = control_msg.points.size();
        ControlMessage::ControlMessage__points* points_list;
        points_list = (ControlMessage::ControlMessage__points*)calloc(1, sizeof(ControlMessage::ControlMessage__points));
        for(auto i = 0; i < points_count; i++) {
            // construct Point
            Point_t* point_p;
            point_p = (Point_t*) calloc(1, sizeof(Point_t));
            point_p->x = control_msg.points[i].x;
            point_p->y = control_msg.points[i].y;
            point_p->width = control_msg.points[i].width;
            if (control_msg.points[i].z_exists){
                long zp = control_msg.points[i].z;
                point_p->z = &zp;
            }  
            asn_sequence_add(&points_list->list, point_p);
        }
        message->value.choice.TestMessage05.body.points = *points_list;
	    // encode message
	    ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
        // log a warning if fails
        if(ec.encoded == -1) {
            return boost::optional<std::vector<uint8_t>>{};
        }
        // copy to byte array msg
        auto array_length = ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) b_array[i] = buffer[i];
        // for(auto i = 0; i < array_length; i++) std::cout<< b_array[i]<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }

}
