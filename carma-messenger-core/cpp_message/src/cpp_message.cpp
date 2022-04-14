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
 * CPP File containing Message method implementations
 */

#include "cpp_message/cpp_message.h"
#include "cpp_message/MobilityOperation_Message.h"
#include "cpp_message/MobilityResponse_Message.h"
#include "cpp_message/MobilityPath_Message.h"
#include "cpp_message/MobilityRequest_Message.h"
#include "cpp_message/BSM_Message.h"
#include "cpp_message/SPAT_Message.h"
#include "cpp_message/Map_Message.h"
#include "cpp_message/PSM_Message.h"


namespace cpp_message
{
    namespace std_ph = std::placeholders;

    Node::Node(const rclcpp::NodeOptions &options): carma_ros2_utils::CarmaLifecycleNode(options)
    {
        
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        outbound_binary_message_pub_ = create_publisher<carma_driver_msgs::msg::ByteArray>("outbound_binary_msg", 5);
        inbound_binary_message_sub_ = create_subscription<carma_driver_msgs::msg::ByteArray>("inbound_binary_msg", 5, std::bind(&Node::inbound_binary_callback, this, std_ph::_1));
        outbound_geofence_request_message_sub_ = create_subscription<j2735_v2x_msgs::msg::TrafficControlRequest>("outgoing_j2735_geofence_request", 5, std::bind(&Node::outbound_control_request_callback, this, std_ph::_1));
        inbound_geofence_request_message_pub_ = create_publisher<j2735_v2x_msgs::msg::TrafficControlRequest>("incoming_j2735_geofence_request", 5);
        outbound_geofence_control_message_sub_ = create_subscription<j2735_v2x_msgs::msg::TrafficControlMessage>("outgoing_j2735_geofence_control", 5, std::bind(&Node::outbound_control_message_callback, this, std_ph::_1));
        
        inbound_geofence_control_message_pub_ = create_publisher<j2735_v2x_msgs::msg::TrafficControlMessage>("incoming_j2735_geofence_control", 5);
        mobility_operation_message_pub_=create_publisher<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation",5);

        mobility_operation_message_sub_=create_subscription<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 5, std::bind(&Node::outbound_mobility_operation_message_callback, this, std_ph::_1));
        mobility_response_message_pub_=create_publisher<carma_v2x_msgs::msg::MobilityResponse>("incoming_mobility_response",5);
        mobility_response_message_sub_=create_subscription<carma_v2x_msgs::msg::MobilityResponse>("outgoing_mobility_response",5, std::bind(&Node::outbound_mobility_response_message_callback, this, std_ph::_1));
        mobility_path_message_pub_=create_publisher<carma_v2x_msgs::msg::MobilityPath>("incoming_mobility_path",5);
        mobility_path_message_sub_=create_subscription<carma_v2x_msgs::msg::MobilityPath>("outgoing_mobility_path",5, std::bind(&Node::outbound_mobility_path_message_callback, this, std_ph::_1));
        mobility_request_message_pub_=create_publisher<carma_v2x_msgs::msg::MobilityRequest>("incoming_mobility_request",5);
        mobility_request_message_sub_=create_subscription<carma_v2x_msgs::msg::MobilityRequest>("outgoing_mobility_request",5, std::bind(&Node::outbound_mobility_request_message_callback, this, std_ph::_1));
        bsm_message_pub_=create_publisher<j2735_v2x_msgs::msg::BSM>("incoming_j2735_bsm",5);
        bsm_message_sub_=create_subscription<j2735_v2x_msgs::msg::BSM>("outgoing_j2735_bsm",5, std::bind(&Node::outbound_bsm_message_callback, this, std_ph::_1));
        spat_message_pub_ = create_publisher<j2735_v2x_msgs::msg::SPAT>("incoming_j2735_spat", 5);
        map_message_pub_ = create_publisher<j2735_v2x_msgs::msg::MapData>("incoming_j2735_map", 5);
        psm_message_pub_ = create_publisher<j2735_v2x_msgs::msg::PSM>("incoming_psm", 5);
        psm_message_sub_ = create_subscription<j2735_v2x_msgs::msg::PSM>("outgoing_psm", 5, std::bind(&Node::outbound_psm_message_callback, this, std_ph::_1));

        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;
    }


    void Node::inbound_binary_callback(carma_driver_msgs::msg::ByteArray::UniquePtr msg)
    {
        // only handle TrafficControlRequest for now
        if(msg->message_type == "TrafficControlRequest") {
            std::vector<uint8_t> array = msg->content;
            auto output = decode_geofence_request(array);
            if(output)
            {
                inbound_geofence_request_message_pub_->publish(output.get());
            } else
            {
                RCLCPP_WARN_STREAM(get_logger(), "Cannot decode geofence request message.");
            }
        }

            // handle TrafficControlMessage
        else if(msg->message_type == "TrafficControlMessage") {
            std::vector<uint8_t> array = msg->content;
            auto output = decode_geofence_control(array);
            if(output)
            {
                inbound_geofence_control_message_pub_->publish(output.get());
            } else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode geofence control message.");
            }
        }
        
        else if(msg->message_type=="MobilityOperation")   
        {
            std::vector<uint8_t> array=msg->content;
            Mobility_Operation decode(this->get_node_logging_interface());
            auto output=decode.decode_mobility_operation_message(array);
            if(output)
            {
                mobility_operation_message_pub_->publish(output.get());
            }
            else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode Mobility Operation message");
            }

        }

        else if(msg->message_type=="MobilityResponse")
        {
            std::vector<uint8_t> array=msg->content;
            Mobility_Response decode(this->get_node_logging_interface());
            auto output=decode.decode_mobility_response_message(array);
            if(output)
            {
                mobility_response_message_pub_->publish(output.get());
            }
            else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode Mobility Response message");
            }
            
        }
        else if(msg->message_type=="MobilityPath")   
        {
            std::vector<uint8_t> array=msg->content;
            Mobility_Path decode(this->get_node_logging_interface());
            auto output=decode.decode_mobility_path_message(array);
            if(output)
            {
                mobility_path_message_pub_->publish(output.get());
            }
            else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode Mobility Path message");
            }
             
        }
        else if(msg->message_type=="MobilityRequest")   
        {
            std::vector<uint8_t> array=msg->content;
            Mobility_Request decode(this->get_node_logging_interface());
            auto output=decode.decode_mobility_request_message(array);
            if(output)
            {
                mobility_request_message_pub_->publish(output.get());
            }
            else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode Mobility Request message");
            }
             
        }
        else if(msg->message_type=="BSM")   
        {
            std::vector<uint8_t> array=msg->content;
            BSM_Message decode(this->get_node_logging_interface());
            auto output=decode.decode_bsm_message(array);
            if(output)
            {
                bsm_message_pub_->publish(output.get());
            }
            else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode BSM message");
            }
             
        }
        else if(msg->message_type=="SPAT")
        {
            std::vector<uint8_t> array=msg->content;
            SPAT_Message decode;
            auto output = decode.decode_spat_message(array);
            if(output)
            {
                spat_message_pub_->publish(output.get());
            }
            else{
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode SPAT message");
            }
        }
        else if(msg->message_type=="MAP")
        {
            std::vector<uint8_t> array=msg->content;
            Map_Message decode(this->get_node_logging_interface());
            auto output = decode.decode_map_message(array);
            if(output)
            {
                map_message_pub_->publish(output.get());
            }
            else
            {
                RCLCPP_WARN_STREAM( get_logger(), "Cannot decode MapData Message");
            }
        }
        else if(msg->message_type=="PSM"){
            std::vector<uint8_t> array = msg->content;
            PSM_Message decode(this->get_node_logging_interface());
            auto output = decode.decode_psm_message(array);
            if(output)
            {
                psm_message_pub_->publish(output.get());
            }
            else{
                RCLCPP_WARN_STREAM(get_logger(), "Cannot decode PSM Message");
            }
        }
    }

    void Node::outbound_control_request_callback(j2735_v2x_msgs::msg::TrafficControlRequest::UniquePtr msg)
    {

        j2735_v2x_msgs::msg::TrafficControlRequest request_msg(*msg.get());
        auto res = encode_geofence_request(request_msg);
        if(res) {
            // copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="TrafficControlRequest";
            output.content = res.get();            
            // publish result
            outbound_binary_message_pub_->publish(output);
        } else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode geofence request message.");
        }

    }

    void Node::outbound_control_message_callback(j2735_v2x_msgs::msg::TrafficControlMessage::UniquePtr msg)
    {
        j2735_v2x_msgs::msg::TrafficControlMessage control_msg(*msg.get());
        auto res = encode_geofence_control(control_msg);
        if(res) {
            // copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="TrafficControlMessage";
            output.content = res.get();
            // publish result
            outbound_binary_message_pub_->publish(output);
        } else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode geofence control message.");
        }
    }

    void Node::outbound_mobility_operation_message_callback(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg)
    {//encode and publish as outbound binary message
        Mobility_Operation encode(this->get_node_logging_interface());
        auto res=encode.encode_mobility_operation_message(*msg.get());
        if(res)
        {
            //copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="MobilityOperation";
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_->publish(output);
        }
        else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode mobility operation message.");
        }
    }

    void Node::outbound_mobility_response_message_callback(carma_v2x_msgs::msg::MobilityResponse::UniquePtr msg)
        {//encode and publish as outbound binary message
        Mobility_Response encode(this->get_node_logging_interface());
        auto res=encode.encode_mobility_response_message(*msg.get());
        if(res)
        {
            //copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="MobilityResponse";
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_->publish(output);
        }
        else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode mobility response message.");
        }
    }
    void Node::outbound_mobility_path_message_callback(carma_v2x_msgs::msg::MobilityPath::UniquePtr msg)
    {//encode and publish as outbound binary message
        Mobility_Path encode(this->get_node_logging_interface());
        auto res=encode.encode_mobility_path_message(*msg.get());
        if(res)
        {
            //copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="MobilityPath";
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_->publish(output);
        }
        else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode mobility path message.");
        }
    }
    void Node::outbound_mobility_request_message_callback(carma_v2x_msgs::msg::MobilityRequest::UniquePtr msg)
    {//encode and publish as outbound binary message
        Mobility_Request encode(this->get_node_logging_interface());
        auto res=encode.encode_mobility_request_message(*msg.get());
        if(res)
        {
            //copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="MobilityRequest";
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_->publish(output);
        }
        else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode mobility request message.");
        }
    }
    void Node::outbound_bsm_message_callback(j2735_v2x_msgs::msg::BSM::UniquePtr msg)
    {//encode and publish as outbound binary message
        BSM_Message encode(this->get_node_logging_interface());
        auto res=encode.encode_bsm_message(*msg.get());
        if(res)
        {
            //copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="BSM";
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_->publish(output);
        }
        else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode BSM message.");
        }
    }

    void Node::outbound_psm_message_callback(j2735_v2x_msgs::msg::PSM::UniquePtr msg)
    {//encode and publish as outbound binary message
        PSM_Message encode(this->get_node_logging_interface());
        auto res = encode.encode_psm_message(*msg.get());
        if(res)
        {
            //copy to byte array msg
            carma_driver_msgs::msg::ByteArray output;
            output.header.frame_id="0";
            output.header.stamp=this->now();
            output.message_type="PSM";
            output.content=res.get();
            //publish result
            outbound_binary_message_pub_->publish(output);
        }
        else
        {
            RCLCPP_WARN_STREAM( get_logger(), "Cannot encode PSM message.");
        }

    }

    boost::optional<j2735_v2x_msgs::msg::TrafficControlMessage> Node::decode_geofence_control(std::vector<uint8_t>& binary_array)
    {
        j2735_v2x_msgs::msg::TrafficControlMessage output;
        // decode results
        asn_dec_rval_t rval;
        MessageFrame_t* message = 0;
        // copy from vector to array
        auto len = binary_array.size();
        uint8_t buf[len] = {0};
        for(auto i = 0; i < len; i++) {
            buf[i] = binary_array[i];
        }
        
        // use asn1c lib to decode
        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

        // if decode succeed
        if(rval.code == RC_OK) {
            if (message->value.choice.TestMessage05.body.present == TrafficControlMessage_PR_reserved)
            {
                output.choice = j2735_v2x_msgs::msg::TrafficControlMessage::RESERVED;
            }
            else if (message->value.choice.TestMessage05.body.present == TrafficControlMessage_PR_tcmV01)
            {
                output.choice = j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01;
                output.tcm_v01 = decode_geofence_control_v01(message->value.choice.TestMessage05.body.choice.tcmV01);
            }
            return output;
        }
        return boost::optional<j2735_v2x_msgs::msg::TrafficControlMessage>{};
    }

    j2735_v2x_msgs::msg::TrafficControlMessageV01 Node::decode_geofence_control_v01(const TrafficControlMessageV01_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlMessageV01 output;

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
        auto update_bits_size = message.updated.size;
        for (auto i=0; i< update_bits_size; i++){
            tmp_update |= message.updated.buf[i];
            if (i != update_bits_size - 1) tmp_update = tmp_update << 8;
        }
        output.updated = tmp_update;

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
            output.params_exists = true;
            output.params = decode_geofence_control_params(*message.params);        
        }

        // decode geometry optional
        output.geometry_exists = false;
        if (message.geometry)
        {
            output.geometry_exists = true;
            output.geometry = decode_geofence_control_geometry(*message.geometry);
        }

        return output;
    }

    j2735_v2x_msgs::msg::Id64b Node::decode_id64b (const Id64b_t& message)
    {
        j2735_v2x_msgs::msg::Id64b output;
        
        // Type uint8[8] size can vary due to encoder optimization
        auto id_len = message.size;
        for(auto i = 0; i < id_len; i++)
        {
            output.id[i] = message.buf[i];
        }
        return output;
    }
    
    j2735_v2x_msgs::msg::Id128b Node::decode_id128b (const Id128b_t& message)
    {
        j2735_v2x_msgs::msg::Id128b output;
        // Type uint8[16]  size can vary due to encoder optimization
        auto id_len = message.size;
        for(auto i = 0; i < id_len; i++)
        {
            output.id[i] = message.buf[i];
        }
        return output;
    }

    j2735_v2x_msgs::msg::TrafficControlPackage Node::decode_geofence_control_package (const TrafficControlPackage_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlPackage output;

        // convert label from 8-bit array to string optional
        std::string label;
        auto label_len = message.label->size;
        for(auto i = 0; i < label_len; i++)
            label += message.label->buf[i];

        output.label = label;
        output.label_exists = label_len > 0;

        // convert tcids from list of Id128b
        size_t tcids_len = message.tcids.list.count;

        for (auto i = 0; i < tcids_len; i++)
        {
            output.tcids.push_back(decode_id128b(*message.tcids.list.array[i]));
        }
            

        return output;
    }

    j2735_v2x_msgs::msg::TrafficControlParams Node::decode_geofence_control_params (const TrafficControlParams_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlParams output;

        // convert vlasses
        auto vclasses_len = message.vclasses.list.count;
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

    j2735_v2x_msgs::msg::TrafficControlVehClass Node::decode_geofence_control_veh_class (const TrafficControlVehClass_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlVehClass output;

        output.vehicle_class = message;    

        return output;
    }

    j2735_v2x_msgs::msg::TrafficControlSchedule Node::decode_geofence_control_schedule (const TrafficControlSchedule_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlSchedule output;
        
        // long int from 8-bit array for "start"  size can vary due to encoder optimization
        uint64_t tmp_start = 0;
        for (auto i=0; i<message.start.size; i++){
            tmp_start |= message.start.buf[i];
            if (i != message.start.size - 1) tmp_start = tmp_start << 8;
        }
        output.start = tmp_start;
        
        // long int from 8-bit array for "end" (optional)  size can vary due to encoder optimization
        if (message.end)
        {
            uint64_t tmp_end = 0;
            for (auto i=0; i<message.end->size; i++){
                tmp_end |= message.end->buf[i];
                if (i != message.end->size - 1) tmp_end = tmp_end << 8;
            }
            output.end = tmp_end;
            output.end_exists = output.end != 153722867280912; // default value, which is same as not having it
        }

        // recover the dow array (optional)
        if (message.dow)
        {
            output.dow_exists = true;
            output.dow = decode_day_of_week(*message.dow);
        }
        
        // recover the dailyschedule between (optional)
        if (message.between)
        {
            auto between_len = message.between->list.count;
            output.between_exists = between_len > 0;
            for (auto i = 0; i < between_len; i ++)
            {
                output.between.push_back(decode_daily_schedule(*message.between->list.array[i]));
            }
        }
        
        // recover the repeat parameter (optional)
        if (message.repeat)
        {
            output.repeat_exists = true;
            output.repeat = decode_repeat_params(*message.repeat);
        }
        
        return output;
    }

    j2735_v2x_msgs::msg::DayOfWeek Node::decode_day_of_week(const DSRC_DayOfWeek_t& message)
    {
        j2735_v2x_msgs::msg::DayOfWeek output;
        
        uint8_t tmp_binary=0;
        if (message.size > 0) // size is default 1 as 8 bits are sufficient for bit-wise encoding of 7 days 
        {
            tmp_binary = message.buf[0] >> 1; // 7 days in a week, while there are 8 entries
            for (int j = output.dow.size() - 1; j >= 0; j --) // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow
            {
                output.dow[j] = tmp_binary % 2;
                tmp_binary = tmp_binary >> 1;
            }
        }
        return output;
    } 

    j2735_v2x_msgs::msg::DailySchedule Node::decode_daily_schedule(const DailySchedule_t& message)
    {
        j2735_v2x_msgs::msg::DailySchedule output;
        
        output.begin = message.begin;
        output.duration = message.duration;

        return output;
    } 

    j2735_v2x_msgs::msg::RepeatParams Node::decode_repeat_params (const RepeatParams_t& message)
    {
        j2735_v2x_msgs::msg::RepeatParams output;

        output.offset = message.offset;
        output.period = message.period; 
        output.span = message.span;   

        return output;
    }

    j2735_v2x_msgs::msg::TrafficControlDetail Node::decode_geofence_control_detail (const TrafficControlDetail_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlDetail output;

        switch (message.present)
        {
            case TrafficControlDetail_PR_signal:
            {
                // signal OCTET STRING SIZE(0..63),
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::SIGNAL_CHOICE;
                auto signal_size = message.choice.signal.size;
                for (auto i = 0; i < signal_size; i ++)
                output.signal.push_back(message.choice.signal.buf[i]);    
                break;
            }
            case TrafficControlDetail_PR_stop:
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::STOP_CHOICE;
                break;
            case TrafficControlDetail_PR_yield:
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::YIELD_CHOICE;
                break;
            case TrafficControlDetail_PR_notowing:
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::NOTOWING_CHOICE;
                break;
            case TrafficControlDetail_PR_restricted:
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::RESTRICTED_CHOICE;
                break;
            case TrafficControlDetail_PR_closed:
                // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
                output.closed = message.choice.closed;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE;
                break;
            case TrafficControlDetail_PR_chains:
                // 	chains ENUMERATED {no, permitted, required},
                output.chains = message.choice.chains;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::CHAINS_CHOICE;
                break;
            case TrafficControlDetail_PR_direction:
                // 	direction ENUMERATED {forward, reverse},
                output.direction = message.choice.direction;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE;
                break;
            case TrafficControlDetail_PR_lataffinity:
                // 	lataffinity ENUMERATED {left, right},
                output.lataffinity = message.choice.lataffinity;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE;
                break;
            case TrafficControlDetail_PR_latperm:
            {
                // 	latperm SEQUENCE (SIZE(2)) OF ENUMERATED {none, permitted, passing-only, emergency-only},
                auto latperm_size = message.choice.latperm.list.count;
                for(auto i = 0; i < latperm_size; i++)
                    output.latperm[i] = *message.choice.latperm.list.array[i];
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE;
                break;
            }
            case TrafficControlDetail_PR_parking:
                // 	parking ENUMERATED {no, parallel, angled},
                output.parking = message.choice.parking;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::PARKING_CHOICE;
                break;
            case TrafficControlDetail_PR_minspeed:
                // 	minspeed INTEGER (0..1023), -- tenths of m/s
                output.minspeed = message.choice.minspeed;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE;
                break;
            case TrafficControlDetail_PR_maxspeed:
                // 	maxspeed INTEGER (0..1023), -- tenths of m/s
                output.maxspeed = message.choice.maxspeed;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
                break;
            case TrafficControlDetail_PR_minhdwy:
                // 	minhdwy INTEGER (0..2047), -- tenths of meters
                output.minhdwy = message.choice.minhdwy;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehmass:
                // 	maxvehmass INTEGER (0..65535), -- kg
                output.maxvehmass = message.choice.maxvehmass;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHMASS_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehheight:
                // 	maxvehheight INTEGER (0..127), -- tenths of meters
                output.maxvehheight = message.choice.maxvehheight;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHHEIGHT_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehwidth:
                // 	maxvehwidth INTEGER (0..127), -- tenths of meters
                output.maxvehwidth = message.choice.maxvehwidth;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHWIDTH_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehlength:
                // 	maxvehlength INTEGER (0..1023), -- tenths of meters
                output.maxvehlength = message.choice.maxvehlength;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHLENGTH_CHOICE;
                break;
            case TrafficControlDetail_PR_maxvehaxles:
                // 	maxvehaxles INTEGER (2..15),
                output.maxvehaxles = message.choice.maxvehaxles;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHAXLES_CHOICE;
                break;
            case TrafficControlDetail_PR_minvehocc:
                // 	minvehocc INTEGER (1..15), 
                output.minvehocc = message.choice.minvehocc;
                output.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINVEHOCC_CHOICE;
                break;
            default:
                break;
        }
        
        return output;
    }

    j2735_v2x_msgs::msg::TrafficControlGeometry Node::decode_geofence_control_geometry (const TrafficControlGeometry_t& message)
    {
        j2735_v2x_msgs::msg::TrafficControlGeometry output;

        // proj
        std::string proj;
        auto proj_len = message.proj.size;
        for(auto i = 0; i < proj_len; i++)
        {
            proj += (char)message.proj.buf[i];
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
        for (auto i=0; i<message.reftime.size; i++){
            reftime |= message.reftime.buf[i];
            //reftime |= message.reftime.buf[i];
            if (i != message.reftime.size - 1) reftime = reftime << 8;
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

    j2735_v2x_msgs::msg::PathNode Node::decode_path_node (const PathNode_t& message)
    {
        j2735_v2x_msgs::msg::PathNode output;

        output.x = message.x;
        output.y = message.y; 
        
        output.z_exists = false;
        if ( message.z && (*message.z <= 32767 || *message.z >= -32768))
        {
            output.z_exists = true;
            output.z = *message.z;
        }

        output.width_exists = false;
        if ( message.width && (*message.width <= 127 || *message.width >= -128))
        {
            output.width_exists = true;
            output.width = *message.width;
        }
           
        return output;
    }

    boost::optional<j2735_v2x_msgs::msg::TrafficControlRequest> Node::decode_geofence_request(std::vector<uint8_t>& binary_array)
    {
        j2735_v2x_msgs::msg::TrafficControlRequest output;
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
            if (message->value.choice.TestMessage04.body.present == TrafficControlRequest_PR_reserved){
                
                output.choice = j2735_v2x_msgs::msg::TrafficControlRequest::RESERVED;
            }
            else if (message->value.choice.TestMessage04.body.present == TrafficControlRequest_PR_tcrV01){

                output.choice = j2735_v2x_msgs::msg::TrafficControlRequest::TCRV01;
                j2735_v2x_msgs::msg::TrafficControlRequestV01 tcrV01;

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
                j2735_v2x_msgs::msg::TrafficControlBounds bound;
                
                // recover a long value from 8-bit array
                uint64_t long_bits = 0;
                auto bits_array_size = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->oldest.size;
                for(auto j = 0; j < bits_array_size; j++) {
                    long_bits = long_bits << 8;
                    long_bits |= message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->oldest.buf[j];
                }
                
                bound.oldest = long_bits;
                // copy lat/lon
                bound.reflon = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->reflon;
                bound.reflat = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->reflat;
                // copy offset array to boost vector
                auto count = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->offsets.list.count;
                for(auto j = 0; j < count; j++) {
                    j2735_v2x_msgs::msg::OffsetPoint offset;
                    offset.deltax = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->offsets.list.array[j]->deltax;
                    offset.deltay = message->value.choice.TestMessage04.body.choice.tcrV01.bounds.list.array[i]->offsets.list.array[j]->deltay;
                    bound.offsets[j] = offset;
                }
                
                tcrV01.bounds.push_back(bound);
            }

            output.tcr_v01 = tcrV01;
            }

            return boost::optional<j2735_v2x_msgs::msg::TrafficControlRequest>(output);
        }
        return boost::optional<j2735_v2x_msgs::msg::TrafficControlRequest>{};
    }
    
    // Futher separating into helper convertor function fails the asn1c encoder.
    // As the problem seem to arise from nested calloc-ed functions, left the function in monolithic
    // structure, while refactoring what is possible (those that don't further call calloc functions)
    boost::optional<std::vector<uint8_t>> Node::encode_geofence_control(j2735_v2x_msgs::msg::TrafficControlMessage control_msg)
    {
        // encode result placeholder
        uint8_t buffer[512] = {0};
        void * buffer_new;
	    size_t buffer_size = sizeof(buffer);
	    asn_enc_rval_t ec;
	    MessageFrame_t* message;
	    message = (MessageFrame_t*)calloc(1, sizeof(MessageFrame_t));
        // if mem allocation fails
	    if (!message)
        {
            return boost::optional<std::vector<uint8_t>>{};
	    }

	    //set message type to TestMessage05
	    message->messageId = 245;
        message->value.present = MessageFrame__value_PR_TestMessage05;        
        //======================== CONTROL MESSAGE START =====================
        if (control_msg.choice == j2735_v2x_msgs::msg::TrafficControlMessage::RESERVED)
        {
            message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_reserved;
        }
        else if (control_msg.choice == j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01)
        {
            message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_tcmV01;
            // ======================== TCMV01 START =============================
            TrafficControlMessageV01_t* output_v01;
            output_v01 = (TrafficControlMessageV01_t*) calloc(1, sizeof(TrafficControlMessageV01_t));
            j2735_v2x_msgs::msg::TrafficControlMessageV01 msg_v01;
            msg_v01 = control_msg.tcm_v01;
            // encode reqid
            Id64b_t* output_64b;
            j2735_v2x_msgs::msg::Id64b msg_64b;
            msg_64b = msg_v01.reqid;
            output_64b = (Id64b_t*) calloc(1, sizeof(Id64b_t));
            // Type uint8[8]
            uint8_t val_64b[8] = {0};
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
            j2735_v2x_msgs::msg::Id128b msg_128b;
            msg_128b = msg_v01.id;
            output_128b = (Id128b_t*) calloc(1, sizeof(Id128b_t));
            
            // Type uint8[16]
            uint8_t val_128b[16] = {0};
            for(auto i = 0; i < msg_128b.id.size(); i++)
            {
                val_128b[i] = msg_128b.id[i];
            }
            output_128b->buf = val_128b;
            output_128b->size = msg_128b.id.size();
            output_v01->id = *output_128b;

            // encode updated
            // recover an 8-bit array from a long value 
            uint8_t updated_val[8] = {0};
            for(int k = 7; k >= 0; k--) {
                updated_val[7 - k] = msg_v01.updated >> (k * 8);
            }
            output_v01->updated.buf = updated_val;
            output_v01->updated.size = 8;

            // encode package optional
            if (msg_v01.package_exists)
            {
                //===================PACKAGE START==================
                TrafficControlPackage_t* output_package;
                output_package = (TrafficControlPackage_t*) calloc(1, sizeof(TrafficControlPackage_t));
                
                j2735_v2x_msgs::msg::TrafficControlPackage msg_package;
                msg_package = msg_v01.package;
                //convert label string to char array (optional)
                
                if (msg_package.label_exists)
                {
                    size_t label_size = msg_package.label.size();
                    uint8_t* label_content;
                    label_content = (uint8_t*) calloc(1, sizeof(uint8_t));
                    for(auto i = 0; i < label_size; i++)
                    {
                        label_content[i] = (char)msg_package.label[i];
                    }
                    IA5String_t* label_p;
                    label_p = (IA5String_t*) calloc(1, sizeof(IA5String_t));
                    label_p->buf = label_content;
                    label_p->size = label_size;
                    output_package->label = label_p;
                }

                // convert tcids from list of Id128b
                auto tcids_len = msg_package.tcids.size();
                TrafficControlPackage::TrafficControlPackage__tcids* tcids;
                tcids = (TrafficControlPackage::TrafficControlPackage__tcids*)calloc(1, sizeof(TrafficControlPackage::TrafficControlPackage__tcids));
                for (auto i = 0; i < tcids_len; i++)
                {
                    Id128b_t* output_128b;
                    j2735_v2x_msgs::msg::Id128b msg_128b;
                    msg_128b = msg_package.tcids[i];
                    output_128b = (Id128b_t*) calloc(1, sizeof(Id128b_t));
                    
                    // Type uint8[16]
                    uint8_t val[16] = {0};
                    for(auto i = 0; i < msg_128b.id.size(); i++)
                    {
                        val[i] = msg_128b.id[i];
                    }
                    output_128b->buf = val;
                    output_128b->size = msg_128b.id.size();
                    asn_sequence_add(&tcids->list, output_128b);
                }
                
                output_package->tcids = *tcids;
                
                // ================= PACKAGE END ==========================
                output_v01->package = output_package;
            }
            
            // encode params optional
            if (msg_v01.params_exists)
            {
                // ===================== TCMV01 - PARAMS START =====================
                TrafficControlParams_t* output_params;
                output_params = (TrafficControlParams_t*) calloc(1, sizeof(TrafficControlParams_t));
                j2735_v2x_msgs::msg::TrafficControlParams msg_params;
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

                // ======================= TCMV01 - PARAMS - SCHEDULE START ===================================
                TrafficControlSchedule_t* output_schedule;
                output_schedule = (TrafficControlSchedule_t*) calloc(1, sizeof(TrafficControlSchedule_t));
                j2735_v2x_msgs::msg::TrafficControlSchedule msg_schedule;
                msg_schedule = msg_params.schedule;
                // 8-bit array from long int for "start"
                uint8_t start_val[8] = {0};
                for(int k = 7; k >= 0; k--) {
                    start_val[7 - k] = msg_schedule.start >> (k * 8);
                }
                EpochMins_t* start_p;
                start_p = ((EpochMins_t*) calloc(1, sizeof(EpochMins_t)));
                start_p->buf = start_val;
                start_p->size = 8;
                output_schedule->start = *start_p;

                // long int from 8-bit array for "end" (optional)
                if (msg_schedule.end_exists)
                {
                    uint8_t end_val[8] = {0};
                    for(int k = 7; k >= 0; k--) {
                        end_val[7 - k] = msg_schedule.end >> (k * 8);
                    }
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
                // ======================= TCMV01 - PARAMS - SCHEDULE END =============================
                output_params->schedule = *output_schedule;

                // regulatory
                output_params->regulatory = msg_params.regulatory;

                // detail
                output_params->detail = *encode_geofence_control_detail(msg_params.detail);
                // ===================== TCMV01 - PARAMS END =====================
                output_v01->params = output_params;
            }

            // encode geometry optional
            if (msg_v01.geometry_exists)
            {
                // ====================== TCMV01 - GEOMETRY START ==========================
                //output_v01->geometry = encode_geofence_control_geometry(msg_v01.geometry);
                TrafficControlGeometry_t* output_geometry;
                output_geometry = (TrafficControlGeometry_t*) calloc(1, sizeof(TrafficControlGeometry_t));
                j2735_v2x_msgs::msg::TrafficControlGeometry msg_geometry;
                msg_geometry = msg_v01.geometry;
                // convert proj string to char array
                size_t proj_size = msg_geometry.proj.size();
                uint8_t* proj_content;
                proj_content = (uint8_t*) calloc(proj_size, sizeof(uint8_t));
                for(auto i = 0; i < proj_size; i++)
                {
                    proj_content[i] = msg_geometry.proj[i];
                }
                output_geometry->proj.buf = proj_content;
                output_geometry->proj.size = proj_size;

                // convert datum string to char array
                size_t datum_size = msg_geometry.datum.size();
                uint8_t* datum_content;
                datum_content = (uint8_t*) calloc(datum_size, sizeof(uint8_t));
                for(auto i = 0; i < datum_size; i++)
                {
                    datum_content[i] = msg_geometry.datum[i]; 
                }
                output_geometry->datum.buf = datum_content;
                output_geometry->datum.size = datum_size;
                
                uint8_t reftime_val[8] = {0};
                for(int k = 7; k >= 0; k--) {
                    reftime_val[7 - k] = msg_geometry.reftime >> (k * 8);
                }

                EpochMins_t* reftime_p;
                reftime_p = ((EpochMins_t*) calloc(1, sizeof(EpochMins_t)));
                reftime_p->buf = reftime_val;
                reftime_p->size = 8;
                output_geometry->reftime = *reftime_p;

                // reflon
                output_geometry->reflon = msg_geometry.reflon;
                
                // reflat
                output_geometry->reflat = msg_geometry.reflat;

                // refelv
                output_geometry->refelv = msg_geometry.refelv;

                // heading
                output_geometry->heading = msg_geometry.heading;
                
                // nodes
                auto nodes_len = msg_geometry.nodes.size();
                TrafficControlGeometry::TrafficControlGeometry__nodes* nodes_list;
                nodes_list = (TrafficControlGeometry::TrafficControlGeometry__nodes*) calloc(1, sizeof(TrafficControlGeometry::TrafficControlGeometry__nodes));
                
                for (auto i = 0; i < nodes_len; i ++)
                {
                    //=============== TCMV01 - GEOMETRY - NODE START ===========================
                    PathNode_t* output_node;
                    output_node = (PathNode_t*) calloc(1, sizeof(PathNode_t));
                    j2735_v2x_msgs::msg::PathNode msg_node;
                    msg_node = msg_geometry.nodes[i];
                    output_node->x = msg_node.x;
                    output_node->y = msg_node.y;
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
                    //================== TCMV01 - GEOMETRY - NODE END =============================
                    asn_sequence_add(&nodes_list->list, output_node);
                }
                output_geometry->nodes = *nodes_list;
                
                //// DEBUG END
                // ======================== GEOMETRY END =========================
                output_v01->geometry = output_geometry;
            }
            //============================TCMV01 END=====================
            message->value.choice.TestMessage05.body.choice.tcmV01 = *output_v01;
        }
        else
        {
            message->value.choice.TestMessage05.body.present = TrafficControlMessage_PR_NOTHING;
        }

        // ===================== CONTROL MESSAGE end =====================
        // encode message
        void *buffer_void = NULL;
        asn_per_constraints_s *constraints = NULL;
        ssize_t ec_new = uper_encode_to_new_buffer(&asn_DEF_MessageFrame, constraints, message, &buffer_void);
	    ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
        // log a warning if fails
        if(ec.encoded == -1) {
            return boost::optional<std::vector<uint8_t>>{};
        }
        // copy to byte array msg
        auto array_length = ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(auto i = 0; i < array_length; i++) b_array[i] = buffer[i];
        //for(auto i = 0; i < array_length; i++) std::cout<< (int)b_array[i]<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }

    Id64b_t* Node::encode_id64b (const j2735_v2x_msgs::msg::Id64b& msg)
    {
        Id64b_t* output;
        output = (Id64b_t*) calloc(1, sizeof(Id64b_t));
        
        // Type uint8[8]
        uint8_t val[8] = {0};
        for(auto i = 0; i < msg.id.size(); i++)
        {
            val[i] = msg.id[i];
        }
        output->buf = val;
        output->size = msg.id.size();
        return output;
    }
    
    Id128b_t* Node::encode_id128b (const j2735_v2x_msgs::msg::Id128b& msg)
    {
        Id128b_t* output;
        output = (Id128b_t*) calloc(1, sizeof(Id128b_t));
        // Type uint8[16]
        uint8_t val[16] = {0};
        for(auto i = 0; i < msg.id.size(); i++)
        {
            val[i] = msg.id[i];
        }
        output->buf = val;
        output->size = msg.id.size();

        return output;
    }

    TrafficControlVehClass_t* Node::encode_geofence_control_veh_class (const j2735_v2x_msgs::msg::TrafficControlVehClass& msg)
    {
        TrafficControlVehClass_t* output;
        output = (TrafficControlVehClass_t*) calloc(1, sizeof(TrafficControlVehClass_t));
        
        *output = msg.vehicle_class;
        return output;
    }

    TrafficControlDetail_t* Node::encode_geofence_control_detail(const j2735_v2x_msgs::msg::TrafficControlDetail& msg)
    {
        TrafficControlDetail_t* output;
        output = (TrafficControlDetail_t*) calloc(1, sizeof(TrafficControlDetail_t));
        switch(msg.choice)
        {
            case j2735_v2x_msgs::msg::TrafficControlDetail::SIGNAL_CHOICE:
            {
                output->present = TrafficControlDetail_PR_signal;
                // signal OCTET STRING SIZE(0..63),
                auto signal_size = msg.signal.size();
                uint8_t* signal_content;
                signal_content = (uint8_t*) calloc(signal_size, sizeof(uint8_t));
                for (auto i = 0; i < signal_size; i ++)
                    signal_content[i] = msg.signal[i];
                output->choice.signal.buf = signal_content;
                output->choice.signal.size = signal_size;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::STOP_CHOICE:
                output->present = TrafficControlDetail_PR_stop;
            break;
            
            case j2735_v2x_msgs::msg::TrafficControlDetail::YIELD_CHOICE:
                output->present = TrafficControlDetail_PR_yield;
            break;

            case j2735_v2x_msgs::msg::TrafficControlDetail::NOTOWING_CHOICE:
                output->present = TrafficControlDetail_PR_notowing;
            break;

            case j2735_v2x_msgs::msg::TrafficControlDetail::RESTRICTED_CHOICE:
                output->present = TrafficControlDetail_PR_restricted;
            break;

            case j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE:
            {
                output->present = TrafficControlDetail_PR_closed;
                // closed ENUMERATED {open, closed, taperleft, taperright, openleft, openright}
                output->choice.closed = msg.closed;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::CHAINS_CHOICE:
            {
                output->present = TrafficControlDetail_PR_chains;
                // 	chains ENUMERATED {no, permitted, required},
                output->choice.chains = msg.chains;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE:
            {
                output->present = TrafficControlDetail_PR_direction;
                // 	direction ENUMERATED {forward, reverse},
                output->choice.direction = msg.direction;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::LATAFFINITY_CHOICE:
            {
                output->present = TrafficControlDetail_PR_lataffinity;
                // 	lataffinity ENUMERATED {left, right},
                output->choice.lataffinity = msg.lataffinity;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::LATPERM_CHOICE:
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
            case j2735_v2x_msgs::msg::TrafficControlDetail::PARKING_CHOICE:
            {
                output->present = TrafficControlDetail_PR_parking;
                // 	parking ENUMERATED {no, parallel, angled},
                output->choice.parking = msg.parking;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE:
            {
                output->present = TrafficControlDetail_PR_minspeed;
                // 	minspeed INTEGER (0..1023), -- tenths of m/s
                output->choice.minspeed = msg.minspeed;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxspeed;
                // 	maxspeed INTEGER (0..1023), -- tenths of m/s
                output->choice.maxspeed = msg.maxspeed;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE:
            {
                output->present = TrafficControlDetail_PR_minhdwy;
                // 	minhdwy INTEGER (0..2047), -- tenths of meters
                output->choice.minhdwy = msg.minhdwy;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHMASS_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehmass;
                // 	maxvehmass INTEGER (0..65535), -- kg
                output->choice.maxvehmass = msg.maxvehmass;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHHEIGHT_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehheight;
                // 	maxvehheight INTEGER (0..127), -- tenths of meters
                output->choice.maxvehheight = msg.maxvehheight;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHWIDTH_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehwidth;
                // 	maxvehwidth INTEGER (0..127), -- tenths of meters
                output->choice.maxvehwidth = msg.maxvehwidth;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHLENGTH_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehlength;
                // 	maxvehlength INTEGER (0..1023), -- tenths of meters
                output->choice.maxvehlength = msg.maxvehlength;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHAXLES_CHOICE:
            {
                output->present = TrafficControlDetail_PR_maxvehaxles;
                // 	maxvehaxles INTEGER (2..15),
                output->choice.maxvehaxles = msg.maxvehaxles;
            break;
            }
            case j2735_v2x_msgs::msg::TrafficControlDetail::MINVEHOCC_CHOICE:
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
    
    DSRC_DayOfWeek_t* Node::encode_day_of_week(const j2735_v2x_msgs::msg::DayOfWeek& msg)
    {
        DSRC_DayOfWeek_t* output;
        output = (DSRC_DayOfWeek_t*) calloc(1, sizeof(DSRC_DayOfWeek_t));
        
        uint8_t* dow_val;
        dow_val = (uint8_t*)calloc(1, sizeof(uint8_t)); // 8 bits are sufficient for bit-wise encoding for 7 days
        uint8_t tmp_binary;
        for (auto i = 0; i < msg.dow.size(); i++)
        {
            tmp_binary |= msg.dow[i];
            tmp_binary = tmp_binary << 1;
        }
        output->buf = dow_val;
        output->size = msg.dow.size();

        return output;
    } 

    DailySchedule_t* Node::encode_daily_schedule(const j2735_v2x_msgs::msg::DailySchedule& msg)
    {
        DailySchedule_t* output;
        output = (DailySchedule_t*) calloc(1, sizeof(DailySchedule_t));
        
        output->begin = msg.begin;
        output->duration = msg.duration;

        return output;
    } 

    RepeatParams_t* Node::encode_repeat_params (const j2735_v2x_msgs::msg::RepeatParams& msg)
    {
        RepeatParams_t* output;
        output = (RepeatParams_t*) calloc(1, sizeof(RepeatParams_t));

        output->offset = msg.offset;
        output->period = msg.period; 
        output->span = msg.span;   

        return output;
    }

    PathNode_t* Node::encode_path_node (const j2735_v2x_msgs::msg::PathNode& msg)
    {
        PathNode_t* output;
        output = (PathNode_t*) calloc(1, sizeof(PathNode_t));

        output->x = msg.x;
        output->y = msg.y;
        // optional fields
        if (msg.z_exists) 
        {
            long z_temp = msg.z;
            output->z = &z_temp;
        }

        if (msg.width_exists) 
        {
            long width_temp = msg.width;
            output->width = &width_temp;
        }
           
        return output;
    }

    boost::optional<std::vector<uint8_t>> Node::encode_geofence_request(j2735_v2x_msgs::msg::TrafficControlRequest request_msg)
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
		    RCLCPP_WARN_STREAM( get_logger(), "Cannot allocate mem for TrafficControlRequest message encoding");
            return boost::optional<std::vector<uint8_t>>{};
	    }
        //set message type to TestMessage04
	    message->messageId = 244;
        message->value.present = MessageFrame__value_PR_TestMessage04;

        // Check and copy TrafficControlRequest choice
        if (request_msg.choice == j2735_v2x_msgs::msg::TrafficControlRequest::RESERVED){
            message->value.choice.TestMessage04.body.present = TrafficControlRequest_PR_reserved;
        }
        else if (request_msg.choice == j2735_v2x_msgs::msg::TrafficControlRequest::TCRV01) {
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
                id_content[i] = request_msg.tcr_v01.reqid.id[i];
            }

            tcr->reqid.size = 8;
            tcr->reqid.buf = id_content;
            // copy reqseq
            tcr->reqseq = request_msg.tcr_v01.reqseq;

            // copy scale
            tcr->scale = request_msg.tcr_v01.scale;
            
            // copy bounds
            auto count = request_msg.tcr_v01.bounds.size();
            TrafficControlRequestV01::TrafficControlRequestV01__bounds* bounds_list;
            bounds_list = (TrafficControlRequestV01::TrafficControlRequestV01__bounds*)calloc(1, sizeof(TrafficControlRequestV01::TrafficControlRequestV01__bounds));
            
            for(auto i = 0; i < count; i++) {
                // construct control bounds
                TrafficControlBounds_t* bounds_p;
                bounds_p = (TrafficControlBounds_t*) calloc(1, sizeof(TrafficControlBounds_t));
                bounds_p->reflat = request_msg.tcr_v01.bounds[i].reflat;
                bounds_p->reflon = request_msg.tcr_v01.bounds[i].reflon;
                // copy offsets from array to C list struct
                TrafficControlBounds::TrafficControlBounds__offsets* offsets;
                offsets = (TrafficControlBounds::TrafficControlBounds__offsets*)calloc(1, sizeof(TrafficControlBounds::TrafficControlBounds__offsets));
                auto offset_count = request_msg.tcr_v01.bounds[i].offsets.size();
                for(auto j = 0; j < offset_count; j++) {
                    OffsetPoint_t* offset_p;
                    offset_p = (OffsetPoint_t*) calloc(1, sizeof(OffsetPoint_t));
                    offset_p->deltax = request_msg.tcr_v01.bounds[i].offsets[j].deltax;
                    offset_p->deltay = request_msg.tcr_v01.bounds[i].offsets[j].deltay;
                    asn_sequence_add(&offsets->list, offset_p);
                }
                bounds_p->offsets = *offsets;
                //convert a long value to an 8-bit array of length 8
                uint8_t oldest_val[8];
                for(int k = 7; k >= 0; k--) {
                    oldest_val[7-k] = request_msg.tcr_v01.bounds[i].oldest >> (k * 8);
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

} // cpp_message namespace

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(cpp_message::Node)
