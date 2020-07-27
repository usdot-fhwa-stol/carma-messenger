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
#include <mutex>   
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
        return;
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
            if (message->value.choice.TestMessage05.body.present == TrafficControlMessage_PR_reserved)
            {
                output.choice = j2735_msgs::TrafficControlMessage::RESERVED;
            }
            else if (message->value.choice.TestMessage05.body.present == TrafficControlMessage_PR_tcmV01)
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
        std::cout << "checking insideseq " << (int)output.reqseq << std::endl; 
        // decode msgtot
        output.msgtot = message.msgtot;
        std::cout << "checking insidetot " << output.msgtot << std::endl; 
        // decode msgnum
        output.msgnum = message.msgnum;
        std::cout << "checking insidenum " << output.msgnum << std::endl; 
        // decode id
        output.id = decode_id128b(message.id);
        std::cout << "checking inside id" << output.id << std::endl; 
        // decode updated
        // recover a long value from 8-bit array
        uint64_t tmp_update=0;
        auto update_bits_size = message.updated.size;
        for (auto i=0; i< update_bits_size; i++){
            tmp_update |= message.updated.buf[i];
            if (i !=7) tmp_update = tmp_update << 8;
        }
        output.updated = tmp_update;
        std::cout << "checking inside updated" << (int)tmp_update << std::endl; 
        // decode package optional
        output.package_exists = false;
        if (message.package)
        {
            output.package_exists = true;

            output.package = decode_geofence_control_package(*message.package);
        }

        // decode params optional
        output.params_exists = false;
        if (message.params)
        {
            ROS_WARN_STREAM("HERE! params ");
            
            output.params_exists = true;
            output.params = decode_geofence_control_params(*message.params);        
        }
        ROS_WARN_STREAM("HERE!2");

        // decode geometry optional
        output.geometry_exists = false;
        if (message.geometry)
        {
            output.geometry_exists = true;
            output.geometry = decode_geofence_control_geometry(*message.geometry);
        }
        ROS_WARN_STREAM("HERE!3");

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
        size_t tcids_len = message.tcids.list.count;
        ROS_WARN_STREAM("HERE! packages 1 " << tcids_len);

        for (auto i = 0; i < tcids_len; i++)
        {
            ROS_WARN_STREAM("HERE! packages" << decode_id128b(*message.tcids.list.array[i]));
            output.tcids.push_back(decode_id128b(*message.tcids.list.array[i]));
        }
            

        return output;
    }

    j2735_msgs::TrafficControlParams Message::decode_geofence_control_params (const TrafficControlParams_t& message)
    {
        j2735_msgs::TrafficControlParams output;

        // convert vlasses
        auto vclasses_len = message.vclasses.list.count;
        for (auto i = 0; i < vclasses_len; i ++)
        {
            output.vclasses.push_back(decode_geofence_control_veh_class(*message.vclasses.list.array[i]));
            ROS_WARN_STREAM(">>" << decode_geofence_control_veh_class(*message.vclasses.list.array[i]));        
        }
        
        // convert schedule
        output.schedule = decode_geofence_control_schedule(message.schedule);
        ROS_WARN_STREAM(">>" << output.schedule);
        
        // regulatory
        output.regulatory = message.regulatory;
        ROS_WARN_STREAM(">>" << (int)output.regulatory);
        // convert traffic control detail
        output.detail = decode_geofence_control_detail(message.detail);
        ROS_WARN_STREAM(">>" << output.detail);

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
        uint64_t tmp_start = 0;
        for (auto i=0; i<8; i++){
            tmp_start |= message.start.buf[i];
            if (i != 7) tmp_start = tmp_start << 8;
        }
        output.start = tmp_start;
        
        // long int from 8-bit array for "end" (optional)
        uint64_t tmp_end = 0;
        for (auto i=0; i<8; i++){
            tmp_end |= message.end->buf[i];
            if (i != 7) tmp_end = tmp_end << 8;
        }
        output.end = tmp_end;
        output.end_exists = output.end != 153722867280912; // default value, which is same as not having it

        // recover the dow array (optional)
        output.dow_exists = message.dow->size > 0;
        output.dow = decode_day_of_week(*message.dow);
        
        // recover the dailyschedule between (optional)
        auto between_len = message.between->list.count;
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
        
        size_t dow_size= message.size;
        ROS_WARN_STREAM("=================decode dow size: " << (uint8_t)dow_size);
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

        switch (message.present)
        {
            case TrafficControlDetail_PR_signal:
            {
                // signal OCTET STRING SIZE(0..63),
                output.choice = j2735_msgs::TrafficControlDetail::SIGNAL_CHOICE;
                auto signal_size = message.choice.signal.size;
                for (auto i = 0; i < signal_size; i ++)
                output.signal.push_back(message.choice.signal.buf[i]);    
                break;
            }
            case TrafficControlDetail_PR_stop:
                output.choice = j2735_msgs::TrafficControlDetail::STOP_CHOICE;
                break;
            case TrafficControlDetail_PR_yield:
                output.choice = j2735_msgs::TrafficControlDetail::YIELD_CHOICE;
                break;
            case TrafficControlDetail_PR_notowing:
                output.choice = j2735_msgs::TrafficControlDetail::NOTOWING_CHOICE;
                break;
            case TrafficControlDetail_PR_restricted:
                output.choice = j2735_msgs::TrafficControlDetail::RESTRICTED_CHOICE;
                break;
            case TrafficControlDetail_PR_closed:
                // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
                output.closed = message.choice.closed;
                output.choice = j2735_msgs::TrafficControlDetail::CLOSED_CHOICE;
                break;
            case TrafficControlDetail_PR_chains:
                // 	chains ENUMERATED {no, permitted, required},
                output.chains = message.choice.chains;
                output.choice = j2735_msgs::TrafficControlDetail::CHAINS_CHOICE;
                break;
            case TrafficControlDetail_PR_direction:
                // 	direction ENUMERATED {forward, reverse},
                output.direction = message.choice.direction;
                output.choice = j2735_msgs::TrafficControlDetail::DIRECTION_CHOICE;
                break;
            case TrafficControlDetail_PR_lataffinity:
                // 	lataffinity ENUMERATED {left, right},
                output.lataffinity = message.choice.lataffinity;
                output.choice = j2735_msgs::TrafficControlDetail::LATAFFINITY_CHOICE;
                break;
            case TrafficControlDetail_PR_latperm:
            {
                // 	latperm SEQUENCE (SIZE(2)) OF ENUMERATED {none, permitted, passing-only, emergency-only},
                auto latperm_size = message.choice.latperm.list.count;
                for(auto i = 0; i < latperm_size; i++)
                    output.latperm[i] = *message.choice.latperm.list.array[i];
                output.choice = j2735_msgs::TrafficControlDetail::LATPERM_CHOICE;
                break;
            }
            case TrafficControlDetail_PR_parking:
                // 	parking ENUMERATED {no, parallel, angled},
                output.parking = message.choice.parking;
                output.choice = j2735_msgs::TrafficControlDetail::PARKING_CHOICE;
                break;
            case TrafficControlDetail_PR_minspeed:
                // 	minspeed INTEGER (0..1023), -- tenths of m/s
                output.minspeed = message.choice.minspeed;
                output.choice = j2735_msgs::TrafficControlDetail::MINSPEED_CHOICE;
                break;
            case TrafficControlDetail_PR_maxspeed:
                // 	maxspeed INTEGER (0..1023), -- tenths of m/s
                output.maxspeed = message.choice.maxspeed;
                output.choice = j2735_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
                break;
            case TrafficControlDetail_PR_minhdwy:
                // 	minhdwy INTEGER (0..2047), -- tenths of meters
                output.minhdwy = message.choice.minhdwy;
                output.choice = j2735_msgs::TrafficControlDetail::MINHDWY_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehmass:
                // 	maxvehmass INTEGER (0..65535), -- kg
                output.maxvehmass = message.choice.maxvehmass;
                output.choice = j2735_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehheight:
                // 	maxvehheight INTEGER (0..127), -- tenths of meters
                output.maxvehheight = message.choice.maxvehheight;
                output.choice = j2735_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehwidth:
                // 	maxvehwidth INTEGER (0..127), -- tenths of meters
                output.maxvehwidth = message.choice.maxvehwidth;
                output.choice = j2735_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehlength:
                // 	maxvehlength INTEGER (0..1023), -- tenths of meters
                output.maxvehlength = message.choice.maxvehlength;
                output.choice = j2735_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehaxles:
                // 	maxvehaxles INTEGER (2..15),
                output.maxvehaxles = message.choice.maxvehaxles;
                output.choice = j2735_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE;
                break;
            case TrafficControlDetail_PR_minvehocc:
                // 	minvehocc INTEGER (1..15), 
                output.minvehocc = message.choice.minvehocc;
                output.choice = j2735_msgs::TrafficControlDetail::MINVEHOCC_CHOICE;
                break;
            default:
                break;
        }
        
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
        std:: cout << std::endl;
        uint64_t reftime = 0;
        for (auto i=0; i<8; i++){
            reftime |= message.reftime.buf[i];
            ROS_WARN_STREAM("decodign " << message.reftime.buf[i]);
            if (i != 7) reftime = reftime << 8;
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
        auto nodes_len = message.nodes.list.count;
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
        
        std::vector<uint8_t> b_array(10);
        return boost::optional<std::vector<uint8_t>>(b_array);
    }
    
    // ==================================== START =============================================

    boost::optional<std::vector<uint8_t>> Message::encode_geofence_control_combined(j2735_msgs::TrafficControlMessage control_msg)
    {
        // encode result placeholder
        uint8_t buffer[512];
        void * buffer_new;
	    size_t buffer_size = sizeof(buffer);
	    asn_enc_rval_t ec;
	    MessageFrame_t* message;
        size_t s1 = sizeof(TrafficControlMessageV01_t);
        size_t s2 = sizeof(TrafficControlGeometry_t);
        size_t s3 = sizeof(TrafficControlParams_t);
        size_t s4 = sizeof(TrafficControlPackage_t);
        std::cout << "message frame: " << s1 << std::endl;
        std::cout << "geometry: " << s2 << std::endl;
        std::cout << "params: " << s3 << std::endl;        
        std::cout << "package: " << s4 << std::endl;
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
        //======================== CONTROL MESSAGE START =====================
        if (control_msg.choice == j2735_msgs::TrafficControlMessage::RESERVED)
        {
            message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_reserved;
        }
        else if (control_msg.choice == j2735_msgs::TrafficControlMessage::TCMV01)
        {
            message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_tcmV01;
            // ========== TCMV01 START =============
            TrafficControlMessageV01_t* output_v01;
            output_v01 = (TrafficControlMessageV01_t*) calloc(1, sizeof(TrafficControlMessageV01_t));
            j2735_msgs::TrafficControlMessageV01 msg_v01;
            msg_v01 = control_msg.tcmV01;
            // encode reqid
            Id64b_t* output_64b;
            j2735_msgs::Id64b msg_64b;
            msg_64b = msg_v01.reqid;
            output_64b = (Id64b_t*) calloc(1, sizeof(Id64b_t));
            // Type uint8[8]
            uint8_t val_64b[8];
            for(auto i = 0; i < msg_64b.id.size(); i++)
            {
                val_64b[i] = msg_64b.id[i];
            }
            output_64b->buf = val_64b;
            output_64b->size = msg_64b.id.size();
            output_v01->reqid = *output_64b;
            // encode reqseq 
            output_v01->reqseq = msg_v01.reqseq;
            // encode msgtot
            output_v01->msgtot = msg_v01.msgtot;
            // encode msgnum
            output_v01->msgnum = msg_v01.msgnum;
            // encode id
            Id128b_t* output_128b;
            j2735_msgs::Id128b msg_128b;
            msg_128b = msg_v01.id;
            output_128b = (Id128b_t*) calloc(1, sizeof(Id128b_t));
            
            // Type uint8[8]
            uint8_t val_128b[16];
            for(auto i = 0; i < msg_128b.id.size(); i++)
            {
                val_128b[i] = msg_128b.id[i];
            }
            output_128b->buf = val_128b;
            output_128b->size = msg_128b.id.size();
            output_v01->id = *output_128b;

            // encode updated
            // recover an 8-bit array from a long value 
            uint8_t updated_val[8];
            for(auto k = 7; k >= 0; k--) {
                updated_val[7 - k] = msg_v01.updated >> (k * 8);
                ROS_WARN_STREAM("weird " << (uint8_t)updated_val[7-k]);
            }
            output_v01->updated.buf = updated_val;
            output_v01->updated.size = 8;

            // encode package optional
            if (msg_v01.package_exists)
            {
                //===================PACKAGE START==================
                TrafficControlPackage_t* output_package;
                output_package = (TrafficControlPackage_t*) calloc(1, sizeof(TrafficControlPackage_t));
                
                j2735_msgs::TrafficControlPackage msg_package;
                msg_package = msg_v01.package;
                //convert label string to char array (optional)
                std::cout << "here " << output_package->label << std::endl;
                
                if (msg_package.label_exists)
                {
                    auto label_size = msg_package.label.size();

                    uint8_t label_content[63] = {0};
                    for(auto i = 0; i < label_size; i++)
                    {
                        label_content[i] = (char)msg_package.label[i];
                        //std::cout<< label_content[i] << std::endl;
                    }
                    IA5String_t* label_p;
                    label_p = (IA5String_t*) calloc(1, sizeof(IA5String_t));
                    label_p->buf = label_content;
                    label_p->size = (size_t)label_size;
                    output_package->label = label_p;
                }

                // convert tcids from list of Id128b
                auto tcids_len = msg_package.tcids.size();
                TrafficControlPackage::TrafficControlPackage__tcids* tcids;
                tcids = (TrafficControlPackage::TrafficControlPackage__tcids*)calloc(1, sizeof(TrafficControlPackage::TrafficControlPackage__tcids));
                for (auto i = 0; i < tcids_len; i++)
                {
                    Id128b_t* output_128b;
                    j2735_msgs::Id128b msg_128b;
                    msg_128b = msg_package.tcids[i];
                    output_128b = (Id128b_t*) calloc(1, sizeof(Id128b_t));
                    
                    // Type uint8[16]
                    uint8_t val[16];
                    for(auto i = 0; i < msg_128b.id.size(); i++)
                    {
                        val[i] = msg_128b.id[i];
                    }
                    output_128b->buf = val;
                    output_128b->size = msg_128b.id.size();
                    asn_sequence_add(&tcids->list, output_128b);
                }
                
                output_package->tcids = *tcids;
                ROS_WARN_STREAM( "size " << output_package->tcids.list.count);
                ROS_WARN_STREAM( "size " << tcids_len);

                
                // ================= PACKAGE END ==========================
                output_v01->package = output_package;
                
                size_t ss = sizeof(*output_package);
                std::cout << "real package: " << ss;
            }
            // encode params optional
            if (msg_v01.params_exists)
            {
                // ===================== PARAMS START =====================
                TrafficControlParams_t* output_params;
                output_params = (TrafficControlParams_t*) calloc(1, sizeof(TrafficControlParams_t));
                j2735_msgs::TrafficControlParams msg_params;
                msg_params = msg_v01.params;
                // convert vlasses
                auto vclasses_size = msg_params.vclasses.size();
                TrafficControlParams::TrafficControlParams__vclasses* vclasses_list;
                vclasses_list = (TrafficControlParams::TrafficControlParams__vclasses*)calloc(1, sizeof(TrafficControlParams::TrafficControlParams__vclasses));
                
                for (auto i = 0; i < vclasses_size; i ++)
                {
                    asn_sequence_add(&vclasses_list->list, encode_geofence_control_veh_class(msg_params.vclasses[i]));
                }
                output_params->vclasses = *vclasses_list;

                // convert schedule
                // ======================= SCHEDULE START ===================================
                TrafficControlSchedule_t* output_schedule;
                output_schedule = (TrafficControlSchedule_t*) calloc(1, sizeof(TrafficControlSchedule_t));
                j2735_msgs::TrafficControlSchedule msg_schedule;
                msg_schedule = msg_params.schedule;
                // 8-bit array from long int for "start"
                uint8_t start_val[8];
                for(auto k = 7; k >= 0; k--) {
                    start_val[7 - k] = msg_schedule.start >> (k * 8);
                }
                output_schedule->start.buf= start_val;
                output_schedule->start.size = 8;
                ROS_WARN_STREAM("================s" <<(long)msg_schedule.start );

                // long int from 8-bit array for "end" (optional)
                if (msg_schedule.end_exists)
                {
                    uint8_t end_val[8];
                    for(auto k = 7; k >= 0; k--) {
                        end_val[7 - k] = msg_schedule.end >> (k * 8);
                    }
                    ROS_WARN_STREAM("===============e" <<(long)msg_schedule.end );
                    EpochMins_t* end_p;
                    end_p = ((EpochMins_t*) calloc(1, sizeof(EpochMins_t)));
                    end_p->buf = end_val;
                    end_p->size = 8;
                    output_schedule->end = end_p;
                }
                // recover the dow array (optional)
                if (msg_schedule.dow_exists)
                {
                    output_schedule->dow = encode_day_of_week(msg_schedule.dow);
                    ROS_WARN_STREAM("=========" << msg_schedule.dow);
                }
                // recover the dailyschedule between (optional)
                if (msg_schedule.between_exists)
                {
                    auto between_len = msg_schedule.between.size();
                    TrafficControlSchedule::TrafficControlSchedule__between* between_list;
                    between_list = (TrafficControlSchedule::TrafficControlSchedule__between*) calloc(1, sizeof(TrafficControlSchedule::TrafficControlSchedule__between));
                    
                    for (auto i = 0; i < between_len; i ++)
                    {
                        asn_sequence_add(&between_list->list, encode_daily_schedule(msg_schedule.between[i]));
                    }
                    output_schedule->between = between_list;
                }

                // recover the repeat parameter (optional)
                if (msg_schedule.repeat_exists)
                {
                    output_schedule->repeat = encode_repeat_params(msg_schedule.repeat);
                }
                // ======================= SCHEDULE END =============================
                output_params->schedule = *output_schedule;

                // regulatory
                output_params->regulatory = msg_params.regulatory;

                // convert traffic control detail
                // ===================== DETAIL START ============================
                TrafficControlDetail_t* output_detail;
                output_detail = (TrafficControlDetail_t*) calloc(1, sizeof(TrafficControlDetail_t));
                j2735_msgs::TrafficControlDetail msg_detail;
                msg_detail = msg_params.detail;
                switch(msg_detail.choice)
                {
                    case j2735_msgs::TrafficControlDetail::SIGNAL_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_signal;
                        // signal OCTET STRING SIZE(0..63),
                        auto signal_size = msg_detail.signal.size();
                        uint8_t signal_val[signal_size];
                        for (auto i = 0; i < signal_size; i ++)
                            signal_val[i] = msg_detail.signal[i];
                        output_detail->choice.signal.buf = signal_val;
                        output_detail->choice.signal.size = signal_size;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::STOP_CHOICE:
                        output_detail->present = TrafficControlDetail_PR_stop;
                    break;
                    
                    case j2735_msgs::TrafficControlDetail::YIELD_CHOICE:
                        output_detail->present = TrafficControlDetail_PR_yield;
                    break;

                    case j2735_msgs::TrafficControlDetail::NOTOWING_CHOICE:
                        output_detail->present = TrafficControlDetail_PR_notowing;
                    break;

                    case j2735_msgs::TrafficControlDetail::RESTRICTED_CHOICE:
                        output_detail->present = TrafficControlDetail_PR_restricted;
                    break;

                    case j2735_msgs::TrafficControlDetail::CLOSED_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_closed;
                        // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
                        output_detail->choice.closed = msg_detail.closed;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::CHAINS_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_chains;
                        // 	chains ENUMERATED {no, permitted, required},
                        output_detail->choice.chains = msg_detail.chains;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::DIRECTION_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_direction;
                        // 	direction ENUMERATED {forward, reverse},
                        output_detail->choice.direction = msg_detail.direction;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::LATAFFINITY_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_lataffinity;
                        // 	lataffinity ENUMERATED {left, right},
                        output_detail->choice.lataffinity = msg_detail.lataffinity;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::LATPERM_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_latperm;
                        // 	latperm SEQUENCE (SIZE(2)) OF ENUMERATED {none, permitted, passing-only, emergency-only},
                        auto latperm_size = msg_detail.latperm.size();
                        TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm* latperm_p;
                        latperm_p = (TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm*) calloc(1, sizeof(TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm));
                        
                        for(auto i = 0; i < latperm_size; i++)
                        {
                            long* item_p;
                            item_p = (long*) calloc(1, sizeof(long));
                            *item_p = msg_detail.latperm[i];
                            asn_sequence_add(&latperm_p->list, item_p);
                        }
                        output_detail->choice.latperm = *latperm_p;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::PARKING_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_parking;
                        // 	parking ENUMERATED {no, parallel, angled},
                        output_detail->choice.parking = msg_detail.parking;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MINSPEED_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_minspeed;
                        // 	minspeed INTEGER (0..1023), -- tenths of m/s
                        output_detail->choice.minspeed = msg_detail.minspeed;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MAXSPEED_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_maxspeed;
                        // 	maxspeed INTEGER (0..1023), -- tenths of m/s
                        output_detail->choice.maxspeed = msg_detail.maxspeed;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MINHDWY_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_minhdwy;
                        // 	minhdwy INTEGER (0..2047), -- tenths of meters
                        output_detail->choice.minhdwy = msg_detail.minhdwy;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_maxvehmass;
                        // 	maxvehmass INTEGER (0..65535), -- kg
                        output_detail->choice.maxvehmass = msg_detail.maxvehmass;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_maxvehheight;
                        // 	maxvehheight INTEGER (0..127), -- tenths of meters
                        output_detail->choice.maxvehheight = msg_detail.maxvehheight;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_maxvehwidth;
                        // 	maxvehwidth INTEGER (0..127), -- tenths of meters
                        output_detail->choice.maxvehwidth = msg_detail.maxvehwidth;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_maxvehlength;
                        // 	maxvehlength INTEGER (0..1023), -- tenths of meters
                        output_detail->choice.maxvehlength = msg_detail.maxvehlength;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_maxvehaxles;
                        // 	maxvehaxles INTEGER (2..15),
                        output_detail->choice.maxvehaxles = msg_detail.maxvehaxles;
                    break;
                    }
                    case j2735_msgs::TrafficControlDetail::MINVEHOCC_CHOICE:
                    {
                        output_detail->present = TrafficControlDetail_PR_minvehocc;
                        // 	minvehocc INTEGER (1..15), 
                        output_detail->choice.minvehocc = msg_detail.minvehocc;
                    break;
                    }
                    default:
                        output_detail->present = TrafficControlDetail_PR_NOTHING;
                    break;
                }

                // ===================== DETAIL END =====================
                output_params->detail = *output_detail;
                ROS_WARN_STREAM("REACHED CHECKPOINT NEXT");
                // ===================== PARAMS END =====================
                output_v01->params = output_params;
                size_t ss = sizeof(*output_params);
                std::cout << "params real: " << ss;
            }
            ROS_WARN_STREAM("REACHED PARAMS ENDING");

            // encode geometry optional
            if (msg_v01.geometry_exists)
            {
                // ====================== GEOMETRY START ==========================
                //output_v01->geometry = encode_geofence_control_geometry(msg_v01.geometry);
                TrafficControlGeometry_t* output_geometry;
                output_geometry = (TrafficControlGeometry_t*) calloc(1, sizeof(TrafficControlGeometry_t));
                j2735_msgs::TrafficControlGeometry msg_geometry;
                msg_geometry = msg_v01.geometry;
                // convert proj string to char array
                /// DEBUG START
                size_t proj_size = msg_geometry.proj.size();
                uint8_t proj_content[63] = {0}; // 63 needs to be specified here
                for(auto i = 0; i < proj_size; i++)
                {
                    proj_content[i] = msg_geometry.proj[i];
                }

                output_geometry->proj.buf = proj_content;
                output_geometry->proj.size = proj_size;

                // convert datum string to char array
                size_t datum_size = msg_geometry.datum.size();
                uint8_t datum_content[63] = {0}; // 63 needs to be specified here
                for(auto i = 0; i < datum_size; i++)
                {
                    datum_content[i] = msg_geometry.datum[i]; 
                }

                output_geometry->datum.buf = datum_content;
                output_geometry->datum.size = datum_size;

                /// DEBUGGG
                
                // encode reftime
                // recover an 8-bit array from a long value 
                //uint8_t reftime_val[8];
                //for(auto k = 7; k >= 0; k--) {
                //    reftime_val[7 - k] = msg_geometry.reftime >> (k * 8);
                //}
                //output_geometry->reftime.buf = reftime_val;
                //output_geometry->reftime.size = 8;
                
                uint8_t reftime_val[8];
                for(auto k = 7; k >= 0; k--) {
                    reftime_val[7 - k] = msg_geometry.reftime >> (k * 8);
                    ROS_WARN_STREAM("reftime" << reftime_val[7 - k]);
                }

                EpochMins_t* reftime_p;
                reftime_p = ((EpochMins_t*) calloc(1, sizeof(EpochMins_t)));
                reftime_p->buf = reftime_val;
                reftime_p->size = 8;
                output_geometry->reftime = *reftime_p;
                
                // reflon
                output_geometry->reflon = msg_geometry.reflon;
                ROS_WARN_STREAM("reflon" << (long)output_geometry->reflon);
                
                // reflat
                output_geometry->reflat = msg_geometry.reflat;
                ROS_WARN_STREAM("reflat" << (long)output_geometry->reflat);

                // refelv
                //uint16_t refelv_corrected = (unsigned) msg_geometry.refelv + 4096; // corrected refelv
                output_geometry->refelv = msg_geometry.refelv;
                ROS_WARN_STREAM("redev" << output_geometry->refelv);

                // heading
                output_geometry->heading = msg_geometry.heading;
                ROS_WARN_STREAM("jeading" << (long)output_geometry->heading);
                
                // nodes
                auto nodes_len = msg_geometry.nodes.size();
                TrafficControlGeometry::TrafficControlGeometry__nodes* nodes_list;
                nodes_list = (TrafficControlGeometry::TrafficControlGeometry__nodes*) calloc(1, sizeof(TrafficControlGeometry::TrafficControlGeometry__nodes));
                
                for (auto i = 0; i < nodes_len; i ++)
                {
                    //=============== NODE START ===========================
                    PathNode_t* output_node;
                    output_node = (PathNode_t*) calloc(1, sizeof(PathNode_t));
                    j2735_msgs::PathNode msg_node;
                    msg_node = msg_geometry.nodes[i];
                    output_node->x = msg_node.x;
                    output_node->y = msg_node.y;
                    std::cout << "x:" << msg_node.x;
                    // optional fields
                    if (msg_node.z_exists) 
                    {
                        long z_temp = msg_node.z;
                        output_node->z = &z_temp;
                    }

                    if (msg_node.width_exists) 
                    {
                        long width_temp = msg_node.width;
                        output_node->width = &width_temp;
                    }
                    //================== NODE END =============================
                    asn_sequence_add(&nodes_list->list, output_node);
                }
                output_geometry->nodes = *nodes_list;
                ROS_WARN_STREAM("sze" << (long)output_geometry->nodes.list.count);
                
                //// DEBUG END
                // ======================== GEOMETRY END =========================
                output_v01->geometry = output_geometry;
                size_t ss = sizeof(*output_geometry);
                std::cout << "sdsdsd: " << ss;
            }
            ROS_WARN_STREAM("REACHED GEOMETRY ENDING");
            //============================TCMV01 END=====================
            message->value.choice.TestMessage05.body.choice.tcmV01 = *output_v01;
        }
        else
        {
            message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_NOTHING;
        }


        // ===================== CONTROL MESSAGE end =====================
        // encode message
        ssize_t ssize_res;
        ssize_res = uper_encode_to_new_buffer(&asn_DEF_MessageFrame, 0, message, &buffer_new);
        if (ssize_res =! -1)
        {
            ROS_WARN_STREAM("BYTES" << (int)ssize_res);
        }
        else
        {
            ROS_WARN_STREAM("SSIT_T LOST");
        }
        
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
        for(auto i = 0; i < array_length; i++) std::cout<< (int)b_array[i]<< ", ";
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
                message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_reserved;
            break;
            case j2735_msgs::TrafficControlMessage::TCMV01:
                message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_tcmV01;
                message->value.choice.TestMessage05.body.choice.tcmV01 = *encode_geofence_control_v01(control_msg.tcmV01);
            break;
            default:
                message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_NOTHING;
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
            // such chain init and assignments are needed to properly encode
            IA5String_t* label_p;
            label_p = (IA5String_t*) calloc(1, sizeof(IA5String_t));
            label_p->buf = label_content;
            label_p->size = string_size;
            output->label = label_p;
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
            EpochMins_t* end_p;
            end_p = ((EpochMins_t*) calloc(1, sizeof(EpochMins_t)));
            end_p->buf = end_val;
            end_p->size = 8;
            output->end = end_p;
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
        
        uint8_t dow_val[8]; 
        for (auto i = 0; i < 7; i++)
        {
            dow_val[i] = msg.dow[i];
            ROS_WARN_STREAM("dow enc" << (uint8_t)dow_val[i]);

        }
        output->buf = dow_val;
        output->size = 8;

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

        switch(msg.choice)
        {
            case j2735_msgs::TrafficControlDetail::SIGNAL_CHOICE:
            {
                output->present = TrafficControlDetail_PR_signal;
                // signal OCTET STRING SIZE(0..63),
                auto signal_size = msg.signal.size();
                uint8_t signal_val[signal_size];
                for (auto i = 0; i < signal_size; i ++)
                    signal_val[i] = msg.signal[i];
                output->choice.signal.buf = signal_val;
                output->choice.signal.size = signal_size;
            break;
            }
            case j2735_msgs::TrafficControlDetail::STOP_CHOICE:
                output->present = TrafficControlDetail_PR_stop;
            break;
            
            case j2735_msgs::TrafficControlDetail::YIELD_CHOICE:
                output->present = TrafficControlDetail_PR_yield;
            break;

            case j2735_msgs::TrafficControlDetail::NOTOWING_CHOICE:
                output->present = TrafficControlDetail_PR_notowing;
            break;

            case j2735_msgs::TrafficControlDetail::RESTRICTED_CHOICE:
                output->present = TrafficControlDetail_PR_restricted;
            break;

            case j2735_msgs::TrafficControlDetail::CLOSED_CHOICE:
            {
                output->present = TrafficControlDetail_PR_closed;
                // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
                output->choice.closed = msg.closed;
            break;
            }
            case j2735_msgs::TrafficControlDetail::CHAINS_CHOICE:
            {
                output->present = TrafficControlDetail_PR_chains;
                // 	chains ENUMERATED {no, permitted, required},
                output->choice.chains = msg.chains;
            break;
            }
            case j2735_msgs::TrafficControlDetail::DIRECTION_CHOICE:
            {
                output->present = TrafficControlDetail_PR_direction;
                // 	direction ENUMERATED {forward, reverse},
                output->choice.direction = msg.direction;
            break;
            }
            case j2735_msgs::TrafficControlDetail::LATAFFINITY_CHOICE:
            {
                output->present = TrafficControlDetail_PR_lataffinity;
                // 	lataffinity ENUMERATED {left, right},
                output->choice.lataffinity = msg.lataffinity;
            break;
            }
            case j2735_msgs::TrafficControlDetail::LATPERM_CHOICE:
            {
                output->present = TrafficControlDetail_PR_latperm;
                // 	latperm SEQUENCE (SIZE(2)) OF ENUMERATED {none, permitted, passing-only, emergency-only},
                auto latperm_size = msg.latperm.size();
                TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm* latperm_p;
                latperm_p = (TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm*) calloc(1, sizeof(TrafficControlDetail::TrafficControlDetail_u::TrafficControlDetail__latperm));
                
                for(auto i = 0; i < latperm_size; i++)
                {
                    long* item_p;
                    item_p = (long*) calloc(1, sizeof(long));
                    *item_p = msg.latperm[i];
                    asn_sequence_add(&latperm_p->list, item_p);
                }
                output->choice.latperm = *latperm_p;
            break;
            }
            case j2735_msgs::TrafficControlDetail::PARKING_CHOICE:
            {
                output->present = TrafficControlDetail_PR_parking;
                // 	parking ENUMERATED {no, parallel, angled},
                output->choice.parking = msg.parking;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MINSPEED_CHOICE:
            {
                output->present = TrafficControlDetail_PR_minspeed;
                // 	minspeed INTEGER (0..1023), -- tenths of m/s
                output->choice.minspeed = msg.minspeed;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MAXSPEED_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxspeed;
                // 	maxspeed INTEGER (0..1023), -- tenths of m/s
                output->choice.maxspeed = msg.maxspeed;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MINHDWY_CHOICE:
            {
                output->present = TrafficControlDetail_PR_minhdwy;
                // 	minhdwy INTEGER (0..2047), -- tenths of meters
                output->choice.minhdwy = msg.minhdwy;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MAXVEHMASS_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehmass;
                // 	maxvehmass INTEGER (0..65535), -- kg
                output->choice.maxvehmass = msg.maxvehmass;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MAXVEHHEIGHT_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehheight;
                // 	maxvehheight INTEGER (0..127), -- tenths of meters
                output->choice.maxvehheight = msg.maxvehheight;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MAXVEHWIDTH_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehwidth;
                // 	maxvehwidth INTEGER (0..127), -- tenths of meters
                output->choice.maxvehwidth = msg.maxvehwidth;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MAXVEHLENGTH_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehlength;
                // 	maxvehlength INTEGER (0..1023), -- tenths of meters
                output->choice.maxvehlength = msg.maxvehlength;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MAXVEHAXLES_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehaxles;
                // 	maxvehaxles INTEGER (2..15),
                output->choice.maxvehaxles = msg.maxvehaxles;
            break;
            }
            case j2735_msgs::TrafficControlDetail::MINVEHOCC_CHOICE:
            {
                output->present = TrafficControlDetail_PR_minvehocc;
                // 	minvehocc INTEGER (1..15), 
                output->choice.minvehocc = msg.minvehocc;
            break;
            }
            default:
                output->present = TrafficControlDetail_PR_NOTHING;
            break;
        }

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
        EpochMins_t* reftime_p;
        reftime_p = (EpochMins_t*) calloc(1, sizeof(EpochMins_t));
        reftime_p->buf = reftime_val;
        reftime_p->size = 8;
        output->reftime = *reftime_p;

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
        if (msg.z_exists) 
        {
            long* z_p;
            z_p = (long*) calloc(1, sizeof(long));
            *z_p = msg.z;
            output->z = z_p;
        }

        if (msg.width_exists) 
        {
            long* width_p;
            width_p = (long*) calloc(1, sizeof(long));
            *width_p = msg.z;
            output->width = width_p;
        }
           
        return output;
    }

} // cpp_message namespace
