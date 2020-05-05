/*
 * Copyright (C) 2018-2020 LEIDOS.
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
        outbound_binary_message_pub_ = nh_->advertise<cav_msgs::ByteArray>("outbound_binary_msg", 5);
        inbound_binary_message_sub_ = nh_->subscribe("inbound_binary_msg", 5, &Message::inbound_binary_callback, this);
        outbound_geofence_request_message_sub_ = nh_->subscribe("outbound_control_request", 5, &Message::outbound_control_request_callback, this);
        inbound_geofence_request_message_pub_ = nh_->advertise<j2735_msgs::ControlRequest>("inbound_control_request", 5);
    }

    void Message::inbound_binary_callback(const cav_msgs::ByteArrayConstPtr& msg)
    {
        if(msg->messageType == "ControlRequest") {
            j2735_msgs::ControlRequest output;
            asn_dec_rval_t rval;
	        MessageFrame_t* message = 0;
	        std::vector<uint8_t> array = msg->content;
            auto len = array.size();
            uint8_t buf[len];
            for(auto i = 0; i < len; i++) {
	            buf[i] = array[i];
	        }
	        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);
            if(rval.code == RC_OK) {
                
                std::string version;
                auto str_len = message->value.choice.TestMessage04.body.version.size;
                for(auto i = 0; i < str_len; i++)
                {
                    version += message->value.choice.TestMessage04.body.version.buf[i];
                }
                output.version = version;
                output.scale = message->value.choice.TestMessage04.body.scale;
                auto bounds_count = message->value.choice.TestMessage04.body.bounds.list.size;
                for(auto i = 0; i < bounds_count; i++) {
                    j2735_msgs::ControlBounds bounds;
                    bounds.latitude = message->value.choice.TestMessage04.body.bounds.list.array[i]->lat;
                    bounds.longitude = message->value.choice.TestMessage04.body.bounds.list.array[i]->lon;
                    auto count = message->value.choice.TestMessage04.body.bounds.list.array[i]->offsets.list.size;
                    for(auto j = 0; j < count; j++) {
                        bounds.offsets[j] = *message->value.choice.TestMessage04.body.bounds.list.array[i]->offsets.list.array[j];
                    }
                    long temp = 0;
                    for(auto j = 0; j < 8; j++) {
                        temp |= message->value.choice.TestMessage04.body.bounds.list.array[i]->oldest.buf[j];
                        temp = temp << 8;
                    }
                    bounds.oldest = temp;
                    output.bounds.push_back(bounds);
                }
                
                inbound_geofence_request_message_pub_.publish(output);
            } else {
                ROS_WARN_STREAM("Cannot decode ControlRequest message");
            }
            
        }
    }

    // Encode Control Request
    void Message::outbound_control_request_callback(const j2735_msgs::ControlRequestConstPtr& msg)
    {
        uint8_t buffer[512];
	    size_t buffer_size = sizeof(buffer);
	    asn_enc_rval_t ec;
	    MessageFrame_t* message;
	    message = (MessageFrame_t*)calloc(1, sizeof(MessageFrame_t));
	    if (!message)
        {
		    ROS_WARN_STREAM("Cannot allocate mem for ControlRequest message encoding");
            return;
	    }

	    //set default value of testmessage04
	    message->messageId = 244;
        message->value.present = MessageFrame__value_PR_TestMessage04;
        
        //set version string
        auto string_size = msg->version.size();
        uint8_t strategy_string_content[string_size];
        for(auto i = 0; i < string_size; i++)
        {
            strategy_string_content[i] = msg->version[i];
        }
        message->value.choice.TestMessage04.body.version.buf = strategy_string_content;
        message->value.choice.TestMessage04.body.version.size = string_size;
        message->value.choice.TestMessage04.body.scale = msg->scale;

        auto count = msg->bounds.size();
        if(count > 0)
        {
            for(auto i = 0; i < count; i++) {
                ControlBounds_t* bounds_p;
                bounds_p = (ControlBounds_t*) calloc(1, sizeof(ControlBounds_t));
                bounds_p->lat = msg->bounds[i].latitude;
                bounds_p->lon = msg->bounds[i].longitude;
                auto offset_count = msg->bounds[i].offsets.size();
                if(offset_count > 0) {
                    for(auto j = 0; j < offset_count; j++) {
                        int16_t temp = msg->bounds[i].offsets[j];
                        asn_sequence_add(&bounds_p->offsets.list, &temp);
                    }
                }
                uint8_t oldest_val[8];
                for(auto k = 7; k >= 0; k--) {
                    // TODO this line needs to be tested
                    oldest_val[7 - k] = msg->bounds[i].oldest >> (k * 8);
                }
                bounds_p->oldest.size = 8;
                bounds_p->oldest.buf = oldest_val;
                asn_sequence_add(&message->value.choice.TestMessage04.body.bounds.list, bounds_p);   
            }
        }

	    //encode message
	    ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
        if(ec.encoded == -1) {
            ROS_WARN_STREAM("Cannot encode ControlRequest message");
            return;
        }

        //copy to byte array msg
        cav_msgs::ByteArray output;
        auto array_length = ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) b_array[i] = buffer[i];
        output.content = b_array;
        outbound_binary_message_pub_.publish(output);
    }

    int Message::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
        return 0;
    }

}
