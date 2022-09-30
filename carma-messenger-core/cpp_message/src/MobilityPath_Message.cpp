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
 * CPP File containing Mobility Path Message method implementations
 */
#include "cpp_message/MobilityPath_Message.h"
#include "cpp_message/MobilityHeader_Message.h"

namespace cpp_message
{
    template <typename T>
    T *create_store_shared(std::vector<std::shared_ptr<void>> &shared_pointers)
    {
        auto obj_shared = std::make_shared<T>();
        shared_pointers.push_back(obj_shared);
        return obj_shared.get();
    }

    boost::optional<carma_v2x_msgs::msg::MobilityPath> Mobility_Path::decode_mobility_path_message(std::vector<uint8_t> &binary_array)
    {
        std::vector<std::shared_ptr<void>> shared_ptrs; // keep references to the objects until the encoding is complete
        carma_v2x_msgs::msg::MobilityHeader header;
        carma_v2x_msgs::msg::Trajectory trajectory;
        carma_v2x_msgs::msg::MobilityPath output;
        // decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t *message = nullptr;

        // copy from vector to array
        size_t len = binary_array.size();

        uint8_t buf[len];
        std::copy(binary_array.begin(), binary_array.end(), buf);
        // use asn1c lib to decode
        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **)&message, buf, len, 0, 0);
        if (rval.code != RC_OK)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoding mobility path message failed");
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<carma_v2x_msgs::msg::MobilityPath>{};
        }
        Mobility_Header Header_constant;
        std::string sender_id, recipient_id, sender_bsm_id, plan_id;
        uint64_t timestamp;
        // get sender id
        size_t str_len = message->value.choice.TestMessage02.header.hostStaticId.size;
        if (str_len <= Header_constant.STATIC_ID_MAX_LENGTH && str_len >= Header_constant.STATIC_ID_MIN_LENGTH)
        {
            for (size_t i = 0; i < str_len; i++)
            {
                sender_id += message->value.choice.TestMessage02.header.hostStaticId.buf[i];
            }
        }
        else
            sender_id = Header_constant.STRING_DEFAULT;

        header.sender_id = sender_id;

        // get recepient id
        str_len = message->value.choice.TestMessage02.header.targetStaticId.size;
        if (str_len <= Header_constant.STATIC_ID_MAX_LENGTH && str_len >= Header_constant.STATIC_ID_MIN_LENGTH)
        {
            for (size_t i = 0; i < str_len; i++)
            {
                recipient_id += message->value.choice.TestMessage02.header.targetStaticId.buf[i];
            }
        }
        else
            recipient_id = Header_constant.STRING_DEFAULT;

        header.recipient_id = recipient_id;

        // get bsm id
        // sender_bsm_id is meant to represent the vehicle BSM id in hex string (Ex: FFFFFFFF)
        str_len = message->value.choice.TestMessage02.header.hostBSMId.size;
        for (size_t i = 0; i < str_len; i++)
        {
            sender_bsm_id += message->value.choice.TestMessage02.header.hostBSMId.buf[i];
        }

        if (str_len < Header_constant.BSM_ID_LENGTH)
        {
            sender_bsm_id = std::string((Header_constant.BSM_ID_LENGTH - str_len), '0').append(sender_bsm_id);
        }
        else if (str_len > Header_constant.BSM_ID_LENGTH)
        {
            // RCLCPP_WARN(get_logger(),"BSM ID -size greater than limit, changing to default");
            sender_bsm_id = Header_constant.BSM_ID_DEFAULT;
        }
        header.sender_bsm_id = sender_bsm_id;

        // get plan id
        str_len = message->value.choice.TestMessage02.header.planId.size;
        if (str_len == Header_constant.GUID_LENGTH)
        {
            for (size_t i = 0; i < str_len; i++)
            {
                plan_id += message->value.choice.TestMessage02.header.planId.buf[i];
            }
        }
        else
            plan_id = Header_constant.GUID_DEFAULT;

        header.plan_id = plan_id;

        // recover uint64_t timestamp from string
        str_len = message->value.choice.TestMessage02.header.timestamp.size;
        timestamp = 0;
        char timestamp_ch[str_len + 1];
        for (size_t i = 0; i < str_len; i++)
        {
            timestamp_ch[i] = message->value.choice.TestMessage02.header.timestamp.buf[i];
        }
        timestamp_ch[str_len] = 0; // String needs a null terminator
        timestamp = atoll(timestamp_ch);
        header.timestamp = timestamp;
        output.m_header = header;
        // Trajectory
        carma_v2x_msgs::msg::LocationECEF location;
        long tmp = message->value.choice.TestMessage02.body.location.ecefX;
        if (tmp > LOCATION_MAX || tmp < LOCATION_MIN)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Location ecefX is out of range");
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<carma_v2x_msgs::msg::MobilityPath>{};
        }
        location.ecef_x = tmp;

        tmp = message->value.choice.TestMessage02.body.location.ecefY;
        if (tmp > LOCATION_MAX || tmp < LOCATION_MIN)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Location ecefY is out of range");
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<carma_v2x_msgs::msg::MobilityPath>{};
        }
        location.ecef_y = tmp;

        tmp = message->value.choice.TestMessage02.body.location.ecefZ;
        if (tmp > LOCATION_MAX_Z || tmp < LOCATION_MIN_Z)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Location ecefZ is out of range");
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<carma_v2x_msgs::msg::MobilityPath>{};
        }
        location.ecef_z = tmp;

        // convert location timestamp from string in asn1 to uint64 for ros message
        str_len = message->value.choice.TestMessage02.body.location.timestamp.size;
        uint64_t location_timestamp = 0;
        char location_timestamp_ch[str_len + 1];
        for (size_t i = 0; i < str_len; i++)
        {
            location_timestamp_ch[i] = message->value.choice.TestMessage02.body.location.timestamp.buf[i];
        }
        location_timestamp_ch[str_len] = 0; // String needs a null terminator
        location_timestamp = atoll(location_timestamp_ch);
        location.timestamp = location_timestamp;

        trajectory.location = location;

        // Trajectory-offset
        int offset_count = message->value.choice.TestMessage02.body.trajectory.list.count;

        if (offset_count > MAX_POINTS_IN_MESSAGE)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "offset count greater than 60.");
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<carma_v2x_msgs::msg::MobilityPath>{};
        }

        for (int i = 0; i < offset_count; i++)
        {
            carma_v2x_msgs::msg::LocationOffsetECEF Offsets;

            Offsets.offset_x = message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetX;
            if (Offsets.offset_x < OFFSET_MIN || Offsets.offset_x > OFFSET_MAX)
            {
                Offsets.offset_x = OFFSET_UNAVAILABLE;
            }

            Offsets.offset_y = message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetY;
            if (Offsets.offset_y < OFFSET_MIN || Offsets.offset_y > OFFSET_MAX)
            {
                Offsets.offset_y = OFFSET_UNAVAILABLE;
            }

            Offsets.offset_z = message->value.choice.TestMessage02.body.trajectory.list.array[i]->offsetZ;
            if (Offsets.offset_z < OFFSET_MIN || Offsets.offset_z > OFFSET_MAX)
            {
                Offsets.offset_z = OFFSET_UNAVAILABLE;
            }

            trajectory.offsets.push_back(Offsets);
        }

        output.trajectory = trajectory;

        ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
        return boost::optional<carma_v2x_msgs::msg::MobilityPath>(output);
    }

    boost::optional<std::vector<uint8_t>> Mobility_Path::encode_mobility_path_message(carma_v2x_msgs::msg::MobilityPath plainMessage)
    {
        std::vector<std::shared_ptr<void>> shared_ptrs; // keep references to the objects until the encoding is complete
        uint8_t buffer[1472];
        size_t buffer_size = sizeof(buffer);
        asn_enc_rval_t ec;
        MessageFrame_t message_val;
        MessageFrame_t *message = &message_val;
        // MessageFrame_t *message = *MessageFrame_t;

        message->messageId = MOBILITYPATH_TEST_ID;
        message->value.present = MessageFrame__value_PR_TestMessage02;

        Mobility_Header Header;
        // convert host_id string to char array
        std::string sender_id = plainMessage.m_header.sender_id;
        size_t string_size = sender_id.size();
        if (string_size < Header.STATIC_ID_MIN_LENGTH || string_size > Header.STATIC_ID_MAX_LENGTH)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Unacceptable host id value, changing to default");
            sender_id = Header.STRING_DEFAULT;
            string_size = Header.STRING_DEFAULT.size();
        }

        uint8_t string_content_hostId[string_size];
        for (size_t i = 0; i < string_size; i++)
        {
            string_content_hostId[i] = sender_id[i];
        }
        message->value.choice.TestMessage02.header.hostStaticId.buf = string_content_hostId;
        message->value.choice.TestMessage02.header.hostStaticId.size = string_size;
        // convert target_id string to char array
        std::string recipient_id = plainMessage.m_header.recipient_id;
        string_size = recipient_id.size();
        if (string_size < Header.STATIC_ID_MIN_LENGTH || string_size > Header.STATIC_ID_MAX_LENGTH)
        {
            // RCLCPP_WARN(get_logger(),"Unacceptable recipient id value, changing to default");
            recipient_id = Header.STRING_DEFAULT;
            string_size = Header.STRING_DEFAULT.size();
        }

        uint8_t string_content_targetId[string_size];
        for (size_t i = 0; i < string_size; i++)
        {
            string_content_targetId[i] = recipient_id[i];
        }
        message->value.choice.TestMessage02.header.targetStaticId.buf = string_content_targetId;
        message->value.choice.TestMessage02.header.targetStaticId.size = string_size;

        // convert bsm_id string to char array
        // sender_bsm_id is meant to represent the vehicle BSM id in hex string (Ex: FFFFFFFF)
        std::string sender_bsm_id = plainMessage.m_header.sender_bsm_id;
        string_size = sender_bsm_id.size();
        if (string_size < Header.BSM_ID_LENGTH)
        {
            sender_bsm_id = std::string((Header.BSM_ID_LENGTH - string_size), '0').append(sender_bsm_id);
        }
        else if (string_size > Header.BSM_ID_LENGTH)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Unacceptable bsm id, changing to default");
            sender_bsm_id = Header.BSM_ID_DEFAULT;
        }
        string_size = Header.BSM_ID_LENGTH;
        RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "String_Size " << string_size);

        uint8_t string_content_BSMId[string_size];
        for (size_t i = 0; i < string_size; i++)
        {
            string_content_BSMId[i] = sender_bsm_id[i];
        }
        message->value.choice.TestMessage02.header.hostBSMId.buf = string_content_BSMId;
        message->value.choice.TestMessage02.header.hostBSMId.size = string_size;

        // convert plan_id string to char array
        std::string plan_id = plainMessage.m_header.plan_id;
        string_size = plainMessage.m_header.plan_id.size();
        if (string_size != Header.GUID_LENGTH)
        {
            // RCLCPP_WARN(get_logger(),"Unacceptable GUID, changing to default");
            plan_id = Header.GUID_DEFAULT;
            string_size = Header.GUID_LENGTH;
        }
        uint8_t string_content_planId[string_size];
        for (size_t i = 0; i < string_size; i++)
        {
            string_content_planId[i] = plan_id[i];
        }
        message->value.choice.TestMessage02.header.planId.buf = string_content_planId;
        message->value.choice.TestMessage02.header.planId.size = string_size;
        // get timestamp and convert to char array
        uint64_t time = plainMessage.m_header.timestamp;
        std::string timestamp = std::to_string(time);
        string_size = timestamp.size();
        if (string_size < Header.TIMESTAMP_MESSAGE_LENGTH)
        {
            timestamp = std::string((Header.TIMESTAMP_MESSAGE_LENGTH - string_size), '0').append(timestamp);
        }
        else if (string_size > Header.TIMESTAMP_MESSAGE_LENGTH)
        {
            // RCLCPP_WARN(get_logger(),"Unacceptable timestamp, changing to default");
            timestamp = std::string(Header.TIMESTAMP_MESSAGE_LENGTH, '0');
        }
        string_size = Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t string_content_timestamp[string_size];
        for (size_t i = 0; i < string_size; i++)
        {
            string_content_timestamp[i] = timestamp[i];
        }
        message->value.choice.TestMessage02.header.timestamp.buf = string_content_timestamp;
        message->value.choice.TestMessage02.header.timestamp.size = string_size;

        // location
        carma_v2x_msgs::msg::LocationECEF starting_location;
        long location_val;
        location_val = plainMessage.trajectory.location.ecef_x;
        if (location_val > LOCATION_MAX || location_val < LOCATION_MIN)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefX = location_val;

        location_val = plainMessage.trajectory.location.ecef_y;
        if (location_val > LOCATION_MAX || location_val < LOCATION_MIN)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Location ecefY is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefY = location_val;

        location_val = plainMessage.trajectory.location.ecef_z;
        if (location_val > LOCATION_MAX_Z || location_val < LOCATION_MIN_Z)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Location ecefX is out of range");
            return boost::optional<std::vector<uint8_t>>{};
        }
        message->value.choice.TestMessage02.body.location.ecefZ = location_val;

        // get location timestamp and convert to char array
        time = plainMessage.trajectory.location.timestamp;
        timestamp = std::to_string(time);
        string_size = timestamp.size();
        if (string_size < Header.TIMESTAMP_MESSAGE_LENGTH)
        {
            timestamp = std::string(Header.TIMESTAMP_MESSAGE_LENGTH - string_size, '0').append(timestamp);
        }
        else if (string_size > Header.TIMESTAMP_MESSAGE_LENGTH)
        {
            // RCLCPP_WARN(get_logger(),"Unacceptable timestamp, changing to default");
            timestamp = std::string(Header.TIMESTAMP_MESSAGE_LENGTH, '0');
        }
        string_size = Header.TIMESTAMP_MESSAGE_LENGTH;
        uint8_t string_location_timestamp[string_size];
        for (size_t i = 0; i < string_size; i++)
        {
            string_location_timestamp[i] = timestamp[i];
        }
        message->value.choice.TestMessage02.body.location.timestamp.buf = string_location_timestamp;
        message->value.choice.TestMessage02.body.location.timestamp.size = string_size;

        // trajectory
        size_t offset_count = plainMessage.trajectory.offsets.size();

        if (offset_count > MAX_POINTS_IN_MESSAGE)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "offset count greater than 60.");
            return boost::optional<std::vector<uint8_t>>{};
        }

        MobilityLocationOffsets offsets_list = {0};
        for (size_t i = 0; i < offset_count; i++)
        {
            MobilityECEFOffset* Offsets = (MobilityECEFOffset*) calloc(1, sizeof(MobilityECEFOffset));
            Offsets->offsetX = plainMessage.trajectory.offsets[i].offset_x;
            Offsets->offsetY = plainMessage.trajectory.offsets[i].offset_y;
            Offsets->offsetZ = plainMessage.trajectory.offsets[i].offset_z;
            asn_sequence_add(&offsets_list.list, Offsets);
        }

        message->value.choice.TestMessage02.body.trajectory = offsets_list;

        ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, message, buffer, buffer_size);
        // log a warning if it fails
        if (ec.encoded == -1)
        {
            return boost::optional<std::vector<uint8_t>>{};
        }

        // copy to byte array msg
        size_t array_length = (ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for (size_t i = 0; i < array_length; i++)
            b_array[i] = buffer[i];

        ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_MobilityLocationOffsets, &offsets_list);
        // ASN_STRUCT_FREE_CONTENTS_ONLY(asn_DEF_MessageFrame, message);
        return boost::optional<std::vector<uint8_t>>(b_array);
    }
}
