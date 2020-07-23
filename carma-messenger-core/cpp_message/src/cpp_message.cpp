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
        inbound_geofence_control_message_pub_ = nh_->advertise<j2735_msgs::TrafficControlMessage>("incoming_j2735_geofence_control", 5);
    }

    void Message::inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg)
    {
        // only handle TrafficControlRequest for now
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

            // handle TrafficControlMessage
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

    void Message::outbound_control_message_callback(const j2735_msgs::TrafficControlMessageConstPtr& msg)
    {
        j2735_msgs::TrafficControlMessage control_msg(*msg.get());
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


    boost::optional<j2735_msgs::TrafficControlMessage> Message::decode_geofence_control(std::vector<uint8_t>& binary_array)
    {
        j2735_msgs::TrafficControlMessage output;
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

        // if decode succeed
        if(rval.code == RC_OK) {
            if (message->value.choice.TestMessage05.body.choice.reserved)
            {
                output.choice = j2735_msgs::TrafficControlMessage::RESERVED;
            }
            else if (message->value.choice.TestMessage05.body.choice.tcmV01.reqid.size)
            {
                output.choice = j2735_msgs::TrafficControlMessage::TCMV01;
                output.tcmV01 = decode_geofence_control_v01(message->value.choice.TestMessage05.body.choice.tcmV01);
            }
            return output;            
        }
        return boost::optional<j2735_msgs::TrafficControlMessage>{};
    }

    j2735_msgs::TrafficControlMessageV01 Message::decode_geofence_control_v01(const TrafficControlMessageV01_t& message)
    {
        j2735_msgs::TrafficControlMessageV01 output;

        // decode reqid
        output.reqid = decode_id64b(message.reqid);
        
        // decode reqseq 
        output.reqseq = message.reqseq;

        // decode msgtot
        output.msgtot = message.msgtot;

        // decode msgnum
        output.msgnum = message.msgnum;

        // decode id
        output.id = decode_id128b(message.id);
                
        // decode updated
        // recover a long value from 8-bit array
        uint64_t tmp_update=0;
        for (auto i=0; i<8; i++){
            tmp_update |= message.updated.buf[i];
            tmp_update = tmp_update << 8;
        }
        output.updated = tmp_update;

        // decode package optional
        output.package_exists = false;
        if (message.package->label->size)
        {
            output.package_exists = true;
            output.package = decode_geofence_control_package(*message.package);
        }

        // decode params optional
        output.params_exists = false;
        if (message.params->detail.choice.signal.size)
        {
            output.params_exists = true;
            output.params = decode_geofence_control_params(*message.params);        
        }

        // decode geometry optional
        output.geometry_exists = false;
        if (message.geometry->nodes.list.size)
        {
            output.geometry_exists = true;
            output.geometry = decode_geofence_control_geometry(*message.geometry);
        }
        return output;
    }

    j2735_msgs::Id64b Message::decode_id64b (const Id64b_t& message)
    {
        j2735_msgs::Id64b output;
        
        // Type uint8[8]
        auto id_len = message.size;
        for(auto i = 0; i < id_len; i++)
        {
            output.id[i] = message.buf[i];
        }
        return output;
    }
    
    j2735_msgs::Id128b Message::decode_id128b (const Id128b_t& message)
    {
        j2735_msgs::Id128b output;
        // Type uint8[16]
        auto id_len = message.size;
        for(auto i = 0; i < id_len; i++)
        {
            output.id[i] = message.buf[i];
        }
        return output;
    }

    j2735_msgs::TrafficControlPackage Message::decode_geofence_control_package (const TrafficControlPackage_t& message)
    {
        j2735_msgs::TrafficControlPackage output;

        // convert label from 8-bit array to string optional
        std::string label;
        auto label_len = message.label->size;
        for(auto i = 0; i < label_len; i++)
            label += message.label->buf[i];

        output.label = label;
        output.label_exists = label_len > 0;

        // convert tcids from list of Id128b
        auto tcids_len = message.tcids.list.size;
        for (auto i = 0; i < tcids_len; i++)
            output.tcids.push_back(decode_id128b(*message.tcids.list.array[i]));

        return output;
    }

    j2735_msgs::TrafficControlParams Message::decode_geofence_control_params (const TrafficControlParams_t& message)
    {
        j2735_msgs::TrafficControlParams output;

        // convert vlasses
        auto vclasses_len = message.vclasses.list.size;
        for (auto i = 0; i < vclasses_len; i ++)
        {
            output.vclasses.push_back(decode_geofence_control_veh_class(*message.vclasses.list.array[i]));
        }
        
        // convert schedule
        output.schedule = decode_geofence_control_schedule(message.schedule);
        
        // regulatory
        output.regulatory = message.regulatory;

        // convert traffic control detail
        output.detail = decode_geofence_control_detail(message.detail);

        return output;
    }

    j2735_msgs::TrafficControlVehClass Message::decode_geofence_control_veh_class (const TrafficControlVehClass_t& message)
    {
        j2735_msgs::TrafficControlVehClass output;

        output.vehicle_class = message;    

        return output;
    }

    j2735_msgs::TrafficControlSchedule Message::decode_geofence_control_schedule (const TrafficControlSchedule_t& message)
    {
        j2735_msgs::TrafficControlSchedule output;
        
        // long int from 8-bit array for "start"
        uint64_t tmp_time = 0;
        for (auto i=0; i<8; i++){
            tmp_time |= message.start.buf[i];
            tmp_time = tmp_time << 8;
        }
        output.start = tmp_time;

        // long int from 8-bit array for "end" (optional)
        tmp_time = 0;
        for (auto i=0; i<8; i++){
            tmp_time |= message.end->buf[i];
            tmp_time = tmp_time << 8;
        }
        output.end = tmp_time;
        output.end_exists = output.end != 153722867280912; // default value, which is same as not having it

        // recover the dow array (optional)
        output.dow_exists = message.dow->size > 0;
        output.dow = decode_day_of_week(*message.dow);
        
        // recover the dailyschedule between (optional)
        auto between_len = message.between->list.size;
        output.between_exists = between_len > 0;
        for (auto i = 0; i < between_len; i ++)
        {
            output.between.push_back(decode_daily_schedule(*message.between->list.array[i]));
        }

        // recover the repeat parameter (optional)
        output.repeat_exists = message.repeat->span > 0; // span is number of minuts schedule is active
                                                           // therefore it gives reasonable idea that the parameter is there
        output.repeat = decode_repeat_params(*message.repeat);

        return output;
    }

    j2735_msgs::DayOfWeek Message::decode_day_of_week(const DSRC_DayOfWeek_t& message)
    {
        j2735_msgs::DayOfWeek output;
        
        auto dow_size= message.size;
        for (auto i = 0; i < dow_size; i++)
        {
            output.dow[i] = message.buf[i];
        }
        return output;
    } 

    j2735_msgs::DailySchedule Message::decode_daily_schedule(const DailySchedule_t& message)
    {
        j2735_msgs::DailySchedule output;
        
        output.begin = message.begin;
        output.duration = message.duration;

        return output;
    } 

    j2735_msgs::RepeatParams Message::decode_repeat_params (const RepeatParams_t& message)
    {
        j2735_msgs::RepeatParams output;

        output.offset = message.offset;
        output.period = message.period; 
        output.span = message.span;   

        return output;
    }

    j2735_msgs::TrafficControlDetail Message::decode_geofence_control_detail (const TrafficControlDetail_t& message)
    {
        j2735_msgs::TrafficControlDetail output;

        // TODO figure out how choice fits in here

        // signal OCTET STRING SIZE(0..63),
        auto signal_size = message.choice.signal.size;
        for (auto i = 0; i < signal_size; i ++)
            output.signal.push_back(message.choice.signal.buf[i]);

        // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
        output.closed = message.choice.closed;

        // 	chains ENUMERATED {no, permitted, required},
        output.chains = message.choice.chains;

        // 	direction ENUMERATED {forward, reverse},
        output.direction = message.choice.direction;

        // 	lataffinity ENUMERATED {left, right},
        output.lataffinity = message.choice.lataffinity;

        // 	latperm SEQUENCE (SIZE(2)) OF ENUMERATED {none, permitted, passing-only, emergency-only},
        auto latperm_size = message.choice.latperm.list.size;
        for(auto i = 0; i < latperm_size; i++)
            output.latperm[i] = *message.choice.latperm.list.array[i];

        // 	parking ENUMERATED {no, parallel, angled},
        output.parking = message.choice.parking;

        // 	minspeed INTEGER (0..1023), -- tenths of m/s
        output.minspeed = message.choice.minspeed;

        // 	maxspeed INTEGER (0..1023), -- tenths of m/s
        output.maxspeed = message.choice.maxspeed;

        // 	minhdwy INTEGER (0..2047), -- tenths of meters
        output.minhdwy = message.choice.minhdwy;

        // 	maxvehmass INTEGER (0..65535), -- kg
        output.maxvehmass = message.choice.maxvehmass;

        // 	maxvehheight INTEGER (0..127), -- tenths of meters
        output.maxvehheight = message.choice.maxvehheight;

        // 	maxvehwidth INTEGER (0..127), -- tenths of meters
        output.maxvehwidth = message.choice.maxvehwidth;

        // 	maxvehlength INTEGER (0..1023), -- tenths of meters
        output.maxvehlength = message.choice.maxvehlength;

        // 	maxvehaxles INTEGER (2..15),
        output.maxvehaxles = message.choice.maxvehaxles;

        // 	minvehocc INTEGER (1..15), 
        output.minvehocc = message.choice.minvehocc;

        return output;
    }

    j2735_msgs::TrafficControlGeometry Message::decode_geofence_control_geometry (const TrafficControlGeometry_t& message)
    {
        j2735_msgs::TrafficControlGeometry output;

        // proj
        std::string proj;
        auto proj_len = message.proj.size;
        for(auto i = 0; i < proj_len; i++)
        {
            proj += message.proj.buf[i];
        }
        output.proj = proj;

        // datum
        std::string datum;
        auto datum_len = message.datum.size;
        for(auto i = 0; i < datum_len; i++)
        {
            datum += message.datum.buf[i];
        }
        output.datum = datum;

        // convert reftime
        uint64_t reftime = 0;
        for (auto i=0; i<8; i++){
            reftime |= message.reftime.buf[i];
            reftime = reftime << 8;
        }
        output.reftime = reftime;

        // reflon
        output.reflon = message.reflon;
        
        // reflat
        output.reflat = message.reflat;

        // refelv
        output.refelv = message.refelv;

        // heading
        output.heading = message.heading;

        // nodes
        auto nodes_len = message.nodes.list.size;
        for (auto i = 0; i < nodes_len; i ++)
        {
            output.nodes.push_back(decode_path_node(*message.nodes.list.array[i]));
        }
        return output;
    }

    j2735_msgs::PathNode Message::decode_path_node (const PathNode_t& message)
    {
        j2735_msgs::PathNode output;

        output.x = message.x;
        output.y = message.y; 
        output.z = *message.z;
        
        output.z_exists = false;
        if ( *message.z <= 32767 || *message.z >= -32768)
            output.z_exists = true;

        output.width_exists = false;
        if ( *message.width <= 127 || *message.width >= -128)
            output.width_exists = true;
           
        return output;
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
        std::vector<uint8_t> b_array(10);
        return boost::optional<std::vector<uint8_t>>(b_array);
    }
    
    boost::optional<std::vector<uint8_t>> Message::encode_geofence_control(j2735_msgs::TrafficControlMessage control_msg)
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
		    ROS_WARN_STREAM("Cannot allocate mem for TrafficControlMessage message encoding");
            return boost::optional<std::vector<uint8_t>>{};
	    }

	    //set message type to TestMessage05
	    message->messageId = 245;
        message->value.present = MessageFrame__value_PR_TestMessage05;        
    
        switch(control_msg.choice)
        {
            case j2735_msgs::TrafficControlMessage::RESERVED:
            // do nothing as it is NULL
            break;
            case j2735_msgs::TrafficControlMessage::TCMV01:
            message->value.choice.TestMessage05.body.choice.tcmV01 = *encode_geofence_control_v01(control_msg.tcmV01);
            break;
            default:
            break;
        }

        // encode message
	    ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
        // log a warning if fails
        if(ec.encoded == -1) {
            ROS_WARN_STREAM("Encoding Control Message failed!");
            return boost::optional<std::vector<uint8_t>>{};
        }
        // copy to byte array msg
        auto array_length = ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) b_array[i] = buffer[i];
        // for(auto i = 0; i < array_length; i++) std::cout<< b_array[i]<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }

    TrafficControlMessageV01_t* Message::encode_geofence_control_v01(const j2735_msgs::TrafficControlMessageV01& msg)
    {
        TrafficControlMessageV01_t* output;
        output = (TrafficControlMessageV01_t*) calloc(1, sizeof(TrafficControlMessageV01_t));
        
        // encode reqid
        output->reqid = *encode_id64b(msg.reqid);
        
        // encode reqseq 
        output->reqseq = msg.reqseq;

        // encode msgtot
        output->msgtot = msg.msgtot;

        // encode msgnum
        output->msgnum = msg.msgnum;

        // encode id
        output->id = *encode_id128b(msg.id);
    
        // encode updated
        // recover an 8-bit array from a long value 
        uint8_t updated_val[8];
        for(auto k = 7; k >= 0; k--) {
            updated_val[7 - k] = msg.updated >> (k * 8);
        }
        output->updated.buf = updated_val;
        output->updated.size = 8;

        // encode package optional
        if (msg.package_exists)
        {
            output->package = encode_geofence_control_package(msg.package);
        }
        ROS_WARN_STREAM("REACHED PACKAGE ENDING");
        // encode params optional
        if (msg.params_exists)
        {
            output->params = encode_geofence_control_params(msg.params);        
        }
        ROS_WARN_STREAM("REACHED PARAMS ENDING");

        // encode geometry optional
        if (msg.geometry_exists)
        {
            output->geometry = encode_geofence_control_geometry(msg.geometry);
        }
        ROS_WARN_STREAM("REACHED GEOMETRY ENDING");

        return output;
    }

    Id64b_t* Message::encode_id64b (const j2735_msgs::Id64b& msg)
    {
        Id64b_t* output;
        output = (Id64b_t*) calloc(1, sizeof(Id64b_t));
        
        // Type uint8[8]
        uint8_t val[8];
        for(auto i = 0; i < msg.id.size(); i++)
        {
            val[i] = msg.id[i];
        }
        output->buf = val;
        output->size = msg.id.size();

        return output;
    }
    
    Id128b_t* Message::encode_id128b (const j2735_msgs::Id128b& msg)
    {
        Id128b_t* output;
        output = (Id128b_t*) calloc(1, sizeof(Id128b_t));
        // Type uint8[16]
        uint8_t val[16];
        for(auto i = 0; i < msg.id.size(); i++)
        {
            val[i] = msg.id[i];
        }
        output->buf = val;
        output->size = msg.id.size();

        return output;
    }

    TrafficControlPackage_t* Message::encode_geofence_control_package (const j2735_msgs::TrafficControlPackage& msg)
    {
        TrafficControlPackage_t* output;
        output = (TrafficControlPackage_t*) calloc(1, sizeof(TrafficControlPackage_t));
        
        //convert label string to char array (optional)
        if (msg.label_exists)
        {
            auto string_size = msg.label.size();
            uint8_t label_content[string_size];
            for(auto i = 0; i < string_size; i++)
            {
                label_content[i] = msg.label[i];
            }
            output->label = (IA5String_t*) calloc(1, sizeof(IA5String_t));
            output->label->buf = label_content;
            output->label->size = string_size;
        }
        
        // convert tcids from list of Id128b
        auto tcids_len = msg.tcids.size();
        TrafficControlPackage::TrafficControlPackage__tcids* tcids;
        tcids = (TrafficControlPackage::TrafficControlPackage__tcids*)calloc(1, sizeof(TrafficControlPackage::TrafficControlPackage__tcids));
        for (auto i = 0; i < tcids_len; i++)
            asn_sequence_add(&tcids->list, encode_id128b(msg.tcids[i]));
        output->tcids = *tcids;

        return output;
    }

    TrafficControlParams_t* Message::encode_geofence_control_params (const j2735_msgs::TrafficControlParams& msg)
    {
        TrafficControlParams_t* output;
        output = (TrafficControlParams_t*) calloc(1, sizeof(TrafficControlParams_t));

        // convert vlasses
        auto vclasses_size = msg.vclasses.size();
        TrafficControlParams::TrafficControlParams__vclasses* vclasses_list;
        vclasses_list = (TrafficControlParams::TrafficControlParams__vclasses*)calloc(1, sizeof(TrafficControlParams::TrafficControlParams__vclasses));
        
        for (auto i = 0; i < vclasses_size; i ++)
        {
            asn_sequence_add(&vclasses_list->list, encode_geofence_control_veh_class(msg.vclasses[i]));
        }
        output->vclasses = *vclasses_list;

        // convert schedule
        output->schedule = *encode_geofence_control_schedule(msg.schedule);
        ROS_WARN_STREAM("REACHED CHECKPOINT");
        
        // regulatory
        output->regulatory = msg.regulatory;

        // convert traffic control detail
        output->detail = *encode_geofence_control_detail(msg.detail);
        ROS_WARN_STREAM("REACHED CHECKPOINT NEXT");
        return output;
    }

    TrafficControlVehClass_t* Message::encode_geofence_control_veh_class (const j2735_msgs::TrafficControlVehClass& msg)
    {
        TrafficControlVehClass_t* output;
        output = (TrafficControlVehClass_t*) calloc(1, sizeof(TrafficControlVehClass_t));
        
        *output = msg.vehicle_class;
        return output;
    }

    TrafficControlSchedule_t* Message::encode_geofence_control_schedule (const j2735_msgs::TrafficControlSchedule& msg)
    {
        TrafficControlSchedule_t* output;
        output = (TrafficControlSchedule_t*) calloc(1, sizeof(TrafficControlSchedule_t));
        // 8-bit array from long int for "start"
        uint8_t start_val[8];
        for(auto k = 7; k >= 0; k--) {
            start_val[7 - k] = msg.start >> (k * 8);
        }
        output->start.buf= start_val;
        output->start.size = 8;
        // long int from 8-bit array for "end" (optional)
        if (msg.end_exists)
        {
            uint8_t end_val[8];
            for(auto k = 7; k >= 0; k--) {
                end_val[7 - k] = msg.end >> (k * 8);
            }
            output->end = ((EpochMins_t*) calloc(1, sizeof(EpochMins_t)));
            output->end->buf = end_val;
            output->end->size = 8;
        }
        // recover the dow array (optional)
        if (msg.dow_exists)
        {
            output->dow = encode_day_of_week(msg.dow);
        }
        // recover the dailyschedule between (optional)
        if (msg.between_exists)
        {
            auto between_len = msg.between.size();
            TrafficControlSchedule::TrafficControlSchedule__between* between_list;
            between_list = (TrafficControlSchedule::TrafficControlSchedule__between*) calloc(1, sizeof(TrafficControlSchedule::TrafficControlSchedule__between));
            
            for (auto i = 0; i < between_len; i ++)
            {
                asn_sequence_add(&between_list->list, encode_daily_schedule(msg.between[i]));
            }
            output->between = between_list;
        }

        // recover the repeat parameter (optional)
        if (msg.repeat_exists)
        {
            output->repeat = encode_repeat_params(msg.repeat);
        }

        return output;
    }

    DSRC_DayOfWeek_t* Message::encode_day_of_week(const j2735_msgs::DayOfWeek& msg)
    {
        DSRC_DayOfWeek_t* output;
        output = (DSRC_DayOfWeek_t*) calloc(1, sizeof(DSRC_DayOfWeek_t));
        
        output->size = 8;
        uint8_t dow_val[8];
        for (auto i = 0; i < msg.dow.size(); i++)
        {
            dow_val[i] = msg.dow[i];
        }
        output->buf = dow_val;

        return output;
    } 

    DailySchedule_t* Message::encode_daily_schedule(const j2735_msgs::DailySchedule& msg)
    {
        DailySchedule_t* output;
        output = (DailySchedule_t*) calloc(1, sizeof(DailySchedule_t));
        
        output->begin = msg.begin;
        output->duration = msg.duration;

        return output;
    } 

    RepeatParams_t* Message::encode_repeat_params (const j2735_msgs::RepeatParams& msg)
    {
        RepeatParams_t* output;
        output = (RepeatParams_t*) calloc(1, sizeof(RepeatParams_t));

        output->offset = msg.offset;
        output->period = msg.period; 
        output->span = msg.span;   

        return output;
    }

    TrafficControlDetail_t* Message::encode_geofence_control_detail (const j2735_msgs::TrafficControlDetail& msg)
    {
        TrafficControlDetail_t* output;
        output = (TrafficControlDetail_t*) calloc(1, sizeof(TrafficControlDetail_t));

        // TODO figure out how choice fits in here

        // signal OCTET STRING SIZE(0..63),
        auto signal_size = msg.signal.size();
        uint8_t signal_val[signal_size];
        for (auto i = 0; i < signal_size; i ++)
            signal_val[i] = msg.signal[i];
        output->choice.signal.buf = signal_val;
        output->choice.signal.size = signal_size;
        ROS_WARN_STREAM("REACHED CHECKPOINT A");
        // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
        output->choice.closed = msg.closed;

        // 	chains ENUMERATED {no, permitted, required},
        output->choice.chains = msg.chains;
        // 	direction ENUMERATED {forward, reverse},
        output->choice.direction = msg.direction;

        // 	lataffinity ENUMERATED {left, right},
        output->choice.lataffinity = msg.lataffinity;

        // 	latperm SEQUENCE (SIZE(2)) OF ENUMERATED {none, permitted, passing-only, emergency-only},
        // TODO: review if this conversion is correct
        auto latperm_size = msg.latperm.size();
        long latperm_val[latperm_size];
        for(auto i = 0; i < latperm_size; i++)
        {
            ROS_WARN_STREAM("REACHED CHECKPOINT B " << msg.latperm.at(0));
            //latperm_val[i] = msg.latperm[i];
        }

        TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm* latperm_p;
        latperm_p = (TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm*) calloc(1, sizeof(TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm));
        output->choice.latperm = *latperm_p;
        *output->choice.latperm.list.array = latperm_val;
        output->choice.latperm.list.size = latperm_size;
        ROS_WARN_STREAM("REACHED CHECKPOINT B ");
        ROS_WARN_STREAM("REACHED CHECKPOINT B " << output->choice.latperm.list.array[0]);
        // 	parking ENUMERATED {no, parallel, angled},
        output->choice.parking = msg.parking;

        // 	minspeed INTEGER (0..1023), -- tenths of m/s
        output->choice.minspeed = msg.minspeed;

        // 	maxspeed INTEGER (0..1023), -- tenths of m/s
        output->choice.maxspeed = msg.maxspeed;

        // 	minhdwy INTEGER (0..2047), -- tenths of meters
        output->choice.minhdwy = msg.minhdwy;

        // 	maxvehmass INTEGER (0..65535), -- kg
        output->choice.maxvehmass = msg.maxvehmass;

        // 	maxvehheight INTEGER (0..127), -- tenths of meters
        output->choice.maxvehheight = msg.maxvehheight;

        // 	maxvehwidth INTEGER (0..127), -- tenths of meters
        output->choice.maxvehwidth = msg.maxvehwidth;

        // 	maxvehlength INTEGER (0..1023), -- tenths of meters
        output->choice.maxvehlength = msg.maxvehlength;

        // 	maxvehaxles INTEGER (2..15),
        output->choice.maxvehaxles = msg.maxvehaxles;

        // 	minvehocc INTEGER (1..15), 
        output->choice.minvehocc = msg.minvehocc;
        ROS_WARN_STREAM("REACHED CHECKPOINT C");
        return output;
    }

    TrafficControlGeometry_t* Message::encode_geofence_control_geometry (const j2735_msgs::TrafficControlGeometry& msg)
    {
        TrafficControlGeometry_t* output;
        output = (TrafficControlGeometry_t*) calloc(1, sizeof(TrafficControlGeometry_t));

        // convert proj string to char array
        auto proj_size = msg.proj.size();
        uint8_t proj_content[proj_size];
        for(auto i = 0; i < proj_size; i++)
        {
            proj_content[i] = msg.proj[i];
        }
        output->proj.buf = proj_content;
        output->proj.size = proj_size;

        // convert datum string to char array
        auto datum_size = msg.datum.size();
        uint8_t datum_content[datum_size];
        for(auto i = 0; i < datum_size; i++)
        {
            datum_content[i] = msg.datum[i];
        }
        output->datum.buf = datum_content;
        output->datum.size = datum_size;

        // encode reftime
        // recover an 8-bit array from a long value 
        uint8_t reftime_val[8];
        for(auto k = 7; k >= 0; k--) {
            reftime_val[7 - k] = msg.reftime >> (k * 8);
        }
        output->reftime.buf = reftime_val;
        output->reftime.size = 8;

        // reflon
        output->reflon = msg.reflon;
        
        // reflat
        output->reflat = msg.reflat;

        // refelv
        output->refelv = msg.refelv;

        // heading
        output->heading = msg.heading;

        // nodes
        auto nodes_len = msg.nodes.size();
        TrafficControlGeometry::TrafficControlGeometry__nodes* nodes_list;
        nodes_list = (TrafficControlGeometry::TrafficControlGeometry__nodes*) calloc(1, sizeof(TrafficControlGeometry::TrafficControlGeometry__nodes));
        for (auto i = 0; i < nodes_len; i ++)
        {
            asn_sequence_add(&nodes_list->list, encode_path_node(msg.nodes[i]));
        }
        output->nodes = *nodes_list;

        return output;
    }

    PathNode_t* Message::encode_path_node (const j2735_msgs::PathNode& msg)
    {
        PathNode_t* output;
        output = (PathNode_t*) calloc(1, sizeof(PathNode_t));

        output->x = msg.x;
        output->y = msg.y;

        // optional fields
        if (msg.z_exists) *output->z = msg.z;
        if (msg.width_exists) *output->width = msg.width;
           
        return output;
    }

} // cpp_message namespace
