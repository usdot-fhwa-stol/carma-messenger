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
        inbound_geofence_request_message_pub_ = nh_->advertise<j2735_msgs::TrafficControlRequest>("incoming_j2735_geofence_request", 5);
        outbound_geofence_control_message_sub_ = nh_->subscribe("outgoing_j2735_geofence_control", 5, &Message::outbound_control_message_callback, this);
        inbound_geofence_control_message_pub_ = nh_->advertise<j2735_msgs::ControlMessage>("incoming_j2735_geofence_control", 5);
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

    void Message::outbound_control_request_callback(const j2735_msgs::TrafficControlRequestConstPtr& msg)
    {

        j2735_msgs::TrafficControlRequest request_msg(*msg.get());
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

    boost::optional<j2735_msgs::TrafficControlRequest> Message::decode_geofence_request(std::vector<uint8_t>& binary_array)
    {
        j2735_msgs::TrafficControlRequest output;
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
            if (message->value.choice.TestMessage04.body.present = TrafficControlRequest_PR_reserved){
                
                output.choice = j2735_msgs::TrafficControlRequest::RESERVED;
            }
            else if (message->value.choice.TestMessage04.body.present = TrafficControlRequest_PR_tcrV01){

                output.choice = j2735_msgs::TrafficControlRequest::TCRV01;
                j2735_msgs::TrafficControlRequestV01 tcrV01;

                // decode id
                uint8_t id[8];
                auto id_len = message->value.choice.TestMessage04.body.choice.tcrV01.reqid.size;
                for(auto i = 0; i < id_len; i++)
                {
                    tcrV01.reqid.id[i] = message->value.choice.TestMessage04.body.choice.tcrV01.reqid.buf[i];
                }

                tcrV01.reqseq = message->value.choice.TestMessage04.body.choice.tcrV01.reqseq;

                tcrV01.scale = message->value.choice.TestMessage04.body.choice.tcrV01.scale;

                // copy bounds
                auto bounds_count = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.count;
                for(auto i = 0; i < bounds_count; i++) {
                j2735_msgs::TrafficControlBounds bound;
                
                // recover a long value from 8-bit array
                uint64_t long_bits = 0;
                auto bits_array_size = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->oldest.size;
                for(auto j = 0; j < bits_array_size; j++) {
                    long_bits |= message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->oldest.buf[j];
                    long_bits = long_bits << 8;
                }
                bound.oldest = long_bits;
                // copy lat/lon
                bound.reflon = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->reflon;
                bound.reflon = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->reflon;
                // copy offset array to boost vector
                auto count = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->offsets.list.count;
                for(auto j = 0; j < count; j++) {
                    j2735_msgs::OffsetPoint offset;
                    offset.deltax = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->offsets.list.array[j]->deltax;
                    offset.deltay = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->offsets.list.array[j]->deltay;
                    bound.offsets[j] = offset;
                }
                
                tcrV01.bounds.push_back(bound);
            }

            output.tcrV01 = tcrV01;
            }

            return boost::optional<j2735_msgs::TrafficControlRequest>(output);
        }
        return boost::optional<j2735_msgs::TrafficControlRequest>{};
    }

    boost::optional<j2735_msgs::ControlMessage> Message::decode_geofence_control(std::vector<uint8_t>& binary_array)
    {
        return boost::optional<j2735_msgs::ControlMessage>{};
    }

    boost::optional<std::vector<uint8_t>> Message::encode_geofence_request(j2735_msgs::TrafficControlRequest request_msg)
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
		    ROS_WARN_STREAM("Cannot allocate mem for TrafficControlRequest message encoding");
            return boost::optional<std::vector<uint8_t>>{};
	    }
        //set message type to TestMessage04
	    message->messageId = 244;
        message->value.present = MessageFrame__value_PR_TestMessage04;

        // Check and copy TrafficControlRequest choice
        if (request_msg.choice == j2735_msgs::TrafficControlRequest::RESERVED){
            message->value.choice.TestMessage04.body.present = TrafficControlRequest_PR_reserved;
        }
        else if (request_msg.choice == j2735_msgs::TrafficControlRequest::TCRV01) {
            message->value.choice.TestMessage04.body.present = TrafficControlRequest_PR_tcrV01;
        
            // create 
            TrafficControlRequestV01_t* tcr;
            tcr = (TrafficControlRequestV01_t*)calloc(1, sizeof(TrafficControlRequestV01_t));
            
            //convert id string to integer array
            Id64b_t* id64;
            id64 = (Id64b_t*)calloc(1, sizeof(Id64b_t));

            uint8_t id_content[8];
            for(auto i = 0; i < 8; i++)
            {
                id_content[i] = request_msg.tcrV01.reqid.id[i];
            }

            tcr->reqid.size = 8;
            tcr->reqid.buf = id_content;
            // copy reqseq
            tcr->reqseq = request_msg.tcrV01.reqseq;

            // copy scale
            tcr->scale = request_msg.tcrV01.scale;
            
            // copy bounds
            auto count = request_msg.tcrV01.bounds.size();
            TrafficControlRequestV01::TrafficControlRequestV01__bounds* bounds_list;
            bounds_list = (TrafficControlRequestV01::TrafficControlRequestV01__bounds*)calloc(1, sizeof(TrafficControlRequestV01::TrafficControlRequestV01__bounds));
            
            for(auto i = 0; i < count; i++) {
                // construct control bounds
                TrafficControlBounds_t* bounds_p;
                bounds_p = (TrafficControlBounds_t*) calloc(1, sizeof(TrafficControlBounds_t));
                bounds_p->reflat = request_msg.tcrV01.bounds[i].reflat;
                bounds_p->reflon = request_msg.tcrV01.bounds[i].reflon;
                // copy offsets from array to C list struct
                TrafficControlBounds::TrafficControlBounds__offsets* offsets;
                offsets = (TrafficControlBounds::TrafficControlBounds__offsets*)calloc(1, sizeof(TrafficControlBounds::TrafficControlBounds__offsets));
                auto offset_count = request_msg.tcrV01.bounds[i].offsets.size();
                for(auto j = 0; j < offset_count; j++) {
                    OffsetPoint_t* offset_p;
                    offset_p = (OffsetPoint_t*) calloc(1, sizeof(OffsetPoint_t));
                    offset_p->deltax = request_msg.tcrV01.bounds[i].offsets[j].deltax;
                    offset_p->deltay = request_msg.tcrV01.bounds[i].offsets[j].deltay;
                    asn_sequence_add(&offsets->list, offset_p);
                }
                bounds_p->offsets = *offsets;
                //convert a long value to an 8-bit array of length 8
                uint8_t oldest_val[8];
                for(auto k = 7; k >= 0; k--) {
                    oldest_val[7 - k] = request_msg.tcrV01.bounds[i].oldest >> (k * 8);
                }
                bounds_p->oldest.size = 8;
                bounds_p->oldest.buf = oldest_val;
                asn_sequence_add(&bounds_list->list, bounds_p);
        }

        tcr->bounds = *bounds_list;

        message->value.choice.TestMessage04.body.choice.tcrV01 = *tcr;
        }

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
        // for(auto i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";//For unit test purposes
        return boost::optional<std::vector<uint8_t>>(b_array);
    }

    boost::optional<std::vector<uint8_t>> Message::encode_geofence_control(j2735_msgs::ControlMessage control_msg)
    {
        return boost::optional<std::vector<uint8_t>>{};
    }

}
