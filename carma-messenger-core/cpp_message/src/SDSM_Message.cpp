/*
 * Copyright (C) 2023 LEIDOS.
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
 * CPP File containing SDSM Message method implementations
 */

#include "cpp_message/SDSM_Message.h"

namespace cpp_message
{
    // Compare to line 1288 of BSM_Message.cpp
    std::optional<std::vector<uint8_t>> SDSM_Message::encode_sdsm_message(const j3224_v2x_msgs::msg::SensorDataSharingMessage& plainMessage)
    {
        // Uncomment below (and one line at the end of the function) to print message to file

        // FILE *fp;
        // fp = fopen("encoded-sdsm-output.txt", "w");
        // fprintf(fp, "encode_sdsm_message is called\n");

        // ^Resulting file can be found in VSCode explorer > build_ros2/cpp_message/encoded-sdsm-output.txt

        // Define shared_ptrs to access optional message data
        std::vector<std::shared_ptr<void>> shared_ptrs;

        // Setup for encoded message
        uint8_t buffer[544];
        size_t buffer_size = sizeof(buffer);
        asn_enc_rval_t ec;
        MessageFrame_t *message = create_store_shared<MessageFrame_t>(shared_ptrs);

        // SDSM setup, ID = 41
        message->messageId = SDSM_TEST_ID_;
        message->value.present = MessageFrame__value_PR_SensorDataSharingMessage;


        // Comments above messages describe their access naming schemes from generated headers/j3224_v2x_msgs
        // MsgName | generatedMessage - j3224_v2x_msg
        // A * indicates that message has optional data and requires pointer management

        //// SensorDataSharingMessage
        // MsgCount | msgCnt - msg_cnt
        if(plainMessage.msg_cnt.msg_cnt > j2735_v2x_msgs::msg::MsgCount::MSG_COUNT_MAX){
            RCLCPP_WARN(node_logging_->get_logger(), "Encoding msg count value is greater than max, setting to max");
            message->value.choice.SensorDataSharingMessage.msgCnt = j2735_v2x_msgs::msg::MsgCount::MSG_COUNT_MAX;
        }
        else{
            message->value.choice.SensorDataSharingMessage.msgCnt = plainMessage.msg_cnt.msg_cnt;
        }


        // TemporaryID | sourceID - source_id
        uint8_t temp_id_content[4] ={0};
        for(int i = 0; i < 4; i++){
            temp_id_content[i] = (char) plainMessage.source_id.id[i];
        }
        TemporaryID_t temp_id;
        temp_id.buf = temp_id_content;
        temp_id.size = 4;
        message->value.choice.SensorDataSharingMessage.sourceID = temp_id;


        // EquipmentType | equipmentType - equipment_type
        if(!plainMessage.equipment_type.equipment_type || plainMessage.equipment_type.equipment_type == j3224_v2x_msgs::msg::EquipmentType::UNKNOWN){
            message->value.choice.SensorDataSharingMessage.equipmentType = j3224_v2x_msgs::msg::EquipmentType::UNKNOWN;
        }
        else{
            message->value.choice.SensorDataSharingMessage.equipmentType = plainMessage.equipment_type.equipment_type;
        }


        // DDateTime | sDSMTimeStamp - sdsm_time_stamp
        auto temp_time_stamp = create_store_shared<DDateTime_t>(shared_ptrs);

        // DDateTime | DYear | *year - year
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::YEAR){
            auto year_ptr = create_store_shared<DYear_t>(shared_ptrs);

            uint16_t year = plainMessage.sdsm_time_stamp.year.year;
            if(year > j2735_v2x_msgs::msg::DYear::MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded year value greater than max, setting to max");
                year = j2735_v2x_msgs::msg::DYear::MAX;   
            }
            *year_ptr = year;

            temp_time_stamp->year = year_ptr;
        }

        // DDateTime | DMonth | *month - month
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::MONTH){
            auto month_ptr = create_store_shared<DMonth_t>(shared_ptrs);

            uint16_t month = plainMessage.sdsm_time_stamp.month.month;
            if(month > j2735_v2x_msgs::msg::DMonth::MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded month value greater than max, setting to max");
                month = j2735_v2x_msgs::msg::DMonth::MAX;   
            }
            *month_ptr = month;

            temp_time_stamp->month = month_ptr;
        }

        // DDateTime | DDay | *day - day
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::DAY){
            auto day_ptr = create_store_shared<DDay_t>(shared_ptrs);

            uint16_t day = plainMessage.sdsm_time_stamp.day.day;
            if(day > j2735_v2x_msgs::msg::DDay::MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded day value greater than max, setting to max");
                day = j2735_v2x_msgs::msg::DDay::MAX;   
            }
            *day_ptr = day;

            temp_time_stamp->day = day_ptr;
        }

        // DDateTime | DHour | *hour - hour
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::HOUR){
            auto hour_ptr = create_store_shared<DHour_t>(shared_ptrs);

            uint16_t hour = plainMessage.sdsm_time_stamp.hour.hour;
            if(hour > j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded hour value greater than max, setting to max");
                hour = j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX;   
            }
            *hour_ptr = hour;

            temp_time_stamp->hour = hour_ptr;
        }

        // DDateTime | DMinute | *minute - minute
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::MINUTE){
            auto minute_ptr = create_store_shared<DMinute_t>(shared_ptrs);

            uint16_t minute = plainMessage.sdsm_time_stamp.minute.minute;
            if(minute > j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded minute value greater than max, setting to max");
                minute = j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX;   
            }
            *minute_ptr = minute;

            temp_time_stamp->minute = minute_ptr;
        }

        // DDateTime | DSecond | *second - second
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::SECOND){
            auto second_ptr = create_store_shared<DSecond_t>(shared_ptrs);

            uint16_t second = plainMessage.sdsm_time_stamp.second.millisecond;
            if(second > j2735_v2x_msgs::msg::DSecond::RESERVED_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded second value greater than max, setting to max");
                second = j2735_v2x_msgs::msg::DSecond::RESERVED_MAX;   
            }
            *second_ptr = second;

            temp_time_stamp->second = second_ptr;
        }

        // DDateTime | DOffset | *offset - offset
        if(plainMessage.sdsm_time_stamp.presence_vector & j2735_v2x_msgs::msg::DDateTime::OFFSET){
            auto offset_ptr = create_store_shared<DOffset_t>(shared_ptrs);

            long offset = plainMessage.sdsm_time_stamp.offset.offset_minute;
            if(offset > j2735_v2x_msgs::msg::DOffset::MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded millisecond value greater than max, setting to max");
                offset = j2735_v2x_msgs::msg::DOffset::MAX;   
            }
            else if(offset < j2735_v2x_msgs::msg::DOffset::MIN){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoded millisecond value less than min, setting to min");
                offset = j2735_v2x_msgs::msg::DOffset::MIN;   
            }
            *offset_ptr = offset;

            temp_time_stamp->offset = offset_ptr;
        }
        // Add finalized time configuration to encoding message
        message->value.choice.SensorDataSharingMessage.sDSMTimeStamp = *temp_time_stamp;


        // Position3D | lat - ref_pos.latitude
        if(!plainMessage.ref_pos.latitude || plainMessage.ref_pos.latitude == j2735_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE){
            message->value.choice.SensorDataSharingMessage.refPos.lat = j2735_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE;
        }
        else{
            long temp_lat = plainMessage.ref_pos.latitude;
            if(temp_lat < j2735_v2x_msgs::msg::Position3D::LATITUDE_MIN){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding latitude value less than min, setting to min");
                temp_lat = j2735_v2x_msgs::msg::Position3D::LATITUDE_MIN;
            }
            else if(temp_lat > j2735_v2x_msgs::msg::Position3D::LATITUDE_MAX){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding latitude value greater than max, setting to max");
                temp_lat = j2735_v2x_msgs::msg::Position3D::LATITUDE_MAX;
            }
            message->value.choice.SensorDataSharingMessage.refPos.lat = temp_lat;  
        }
        
        // Position 3D | Long - ref_pos.longitude
        if(!plainMessage.ref_pos.longitude || plainMessage.ref_pos.longitude == j2735_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE){
            message->value.choice.SensorDataSharingMessage.refPos.Long = j2735_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE;
        }
        else{
            long temp_long = plainMessage.ref_pos.longitude;
            if(temp_long < j2735_v2x_msgs::msg::Position3D::LONGITUDE_MIN){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding longitude value less than min, setting to min");
                temp_long = j2735_v2x_msgs::msg::Position3D::LONGITUDE_MIN;
            }
            else if(temp_long > j2735_v2x_msgs::msg::Position3D::LONGITUDE_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding longitude value greater than max, setting to max");
                temp_long = j2735_v2x_msgs::msg::Position3D::LONGITUDE_MAX;
            }
            
            message->value.choice.SensorDataSharingMessage.refPos.Long = temp_long;
        }
        
        // Position3D | *elevation - ref_pos.elevation
        auto elevation_ptr = create_store_shared<DSRC_Elevation_t>(shared_ptrs);

        if(!plainMessage.ref_pos.elevation_exists || plainMessage.ref_pos.elevation == j2735_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE){
            *elevation_ptr = j2735_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE;
        }
        else{
            long temp_elevation = plainMessage.ref_pos.elevation;
            if(temp_elevation < j2735_v2x_msgs::msg::Position3D::ELEVATION_MIN){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding elevation value less than min, setting to min");
                temp_elevation = j2735_v2x_msgs::msg::Position3D::ELEVATION_MIN;
            }
            else if (temp_elevation > j2735_v2x_msgs::msg::Position3D::ELEVATION_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding elevation value greater than max, setting to max");
                temp_elevation = j2735_v2x_msgs::msg::Position3D::ELEVATION_MAX;
            }
            *elevation_ptr = temp_elevation;
            
        }
        message->value.choice.SensorDataSharingMessage.refPos.elevation = elevation_ptr;


        // PositionalAccuracy | semiMajor - semi_minor
        if(plainMessage.ref_pos_xy_conf.semi_major == j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE){
            message->value.choice.SensorDataSharingMessage.refPosXYConf.semiMajor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
        }
        else{
            uint8_t temp_semimajor = plainMessage.ref_pos_xy_conf.semi_major;
            if(temp_semimajor > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding semi major axis accuracy value greater than max, setting to max");
                temp_semimajor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX;
            }

            message->value.choice.SensorDataSharingMessage.refPosXYConf.semiMajor = temp_semimajor;
        }

        // PositionalAccuracy | semiMinor - semi_minor
        if(plainMessage.ref_pos_xy_conf.semi_minor == j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE){
            message->value.choice.SensorDataSharingMessage.refPosXYConf.semiMinor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
        }
        else{
            uint8_t temp_semiminor = plainMessage.ref_pos_xy_conf.semi_minor;
            if(temp_semiminor > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding semi minor axis accuracy value greater than max, setting to max");
                temp_semiminor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX;
            }

            message->value.choice.SensorDataSharingMessage.refPosXYConf.semiMinor = temp_semiminor;
        }

        // PositionalAccuracy | orientation - orientation
        if(plainMessage.ref_pos_xy_conf.orientation == j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE){
            message->value.choice.SensorDataSharingMessage.refPosXYConf.orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE;
        }
        else{
            uint16_t temp_orientation = plainMessage.ref_pos_xy_conf.orientation;
            if(temp_orientation > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX){
                RCLCPP_WARN(node_logging_->get_logger(),"Encoding orientation accuracy value greater than max, setting to max");
                temp_orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX;
            }

            message->value.choice.SensorDataSharingMessage.refPosXYConf.orientation = temp_orientation;
        }


        // ElevationConfidence | *refPosElConf - ref_pos_el_conf
        if(plainMessage.presence_vector & j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF){
            auto elevation_conf = create_store_shared<ElevationConfidence_t>(shared_ptrs);
            
            if(!plainMessage.ref_pos_el_conf.confidence || plainMessage.ref_pos_el_conf.confidence == j2735_v2x_msgs::msg::ElevationConfidence::UNAVAILABLE){
                *elevation_conf = j2735_v2x_msgs::msg::ElevationConfidence::UNAVAILABLE;
            }
            else{
                *elevation_conf = plainMessage.ref_pos_el_conf.confidence;
            }
            message->value.choice.SensorDataSharingMessage.refPosElConf = elevation_conf;
        }


        //// DetectedObjectList
        auto detected_object_list = create_store_shared<DetectedObjectList_t>(shared_ptrs);

        // Create an "in_object" for every element in the detected object list
        for(auto in_object : plainMessage.objects.detected_object_data){

            // Contains the data to be encoded from common/optional data
            auto encode_object = create_store_shared<DetectedObjectData_t>(shared_ptrs);

            //// DetectedObjectCommonData | detObjCommon - detected_object_common_data
            auto encode_obj_com = create_store_shared<DetectedObjectCommonData_t>(shared_ptrs);


            // ObjectType | objType - obj_type.object_type
            if(!in_object.detected_object_common_data.obj_type.object_type || in_object.detected_object_common_data.obj_type.object_type == j3224_v2x_msgs::msg::ObjectType::UNKNOWN){
                encode_obj_com->objType = j3224_v2x_msgs::msg::ObjectType::UNKNOWN;
            }
            else{
                encode_obj_com->objType = in_object.detected_object_common_data.obj_type.object_type;
            }


            // ClassificationConfidence | objTypeCfd - obj_type_conf.classification_confidence
            uint8_t temp_type_conf = in_object.detected_object_common_data.obj_type_cfd.classification_confidence;

            if(temp_type_conf > j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding object type confidence value greater than max, setting to max");
                temp_type_conf = j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE;
            }
            encode_obj_com->objTypeCfd  = temp_type_conf;


            // ObjectID | objectID - detected_id.object_id
            uint16_t temp_obj_id = in_object.detected_object_common_data.detected_id.object_id;

            if(temp_obj_id > j3224_v2x_msgs::msg::ObjectID::MAX_OBJECT_ID){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding object id value greater than max, setting to max");
                temp_obj_id = j3224_v2x_msgs::msg::ObjectID::MAX_OBJECT_ID;
            }
            encode_obj_com->objectID  = temp_obj_id;


            // MeasurementTimeOffset | measurementTime - measurement_time.measurement_time_offset
            long temp_m_time = in_object.detected_object_common_data.measurement_time.measurement_time_offset;

            if(temp_m_time < j3224_v2x_msgs::msg::MeasurementTimeOffset::MIN_MEASUREMENT_TIME_OFFSET){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding measurement time offset value less than min, setting to min");
                temp_m_time = j3224_v2x_msgs::msg::MeasurementTimeOffset::MIN_MEASUREMENT_TIME_OFFSET;
            }
            else if(temp_m_time > j3224_v2x_msgs::msg::MeasurementTimeOffset::MAX_MEASUREMENT_TIME_OFFSET){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding measurement time offset value greater than max, setting to max");
                temp_m_time = j3224_v2x_msgs::msg::MeasurementTimeOffset::MAX_MEASUREMENT_TIME_OFFSET;
            }
            encode_obj_com->measurementTime  = temp_m_time;


            // TimeConfidence | timeConfidence - time_confidence.confidence
            if(!in_object.detected_object_common_data.time_confidence.confidence || in_object.detected_object_common_data.time_confidence.confidence == j2735_v2x_msgs::msg::TimeConfidence::UNAVAILABLE){
                encode_obj_com->timeConfidence = j2735_v2x_msgs::msg::TimeConfidence::UNAVAILABLE;
            }
            else{
                encode_obj_com->timeConfidence = in_object.detected_object_common_data.time_confidence.confidence;
            }


            // PositionOffsetXYZ | pos.offsetX - pos.offset_x.object_dist
            long temp_offset_x = in_object.detected_object_common_data.pos.offset_x.object_distance;

            if(temp_offset_x < j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding x offset value less than min, setting to min");
                temp_offset_x = j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE;
            }
            else if(temp_offset_x > j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding x offset value greater than max, setting to max");
                temp_offset_x = j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE;
            }
            encode_obj_com->pos.offsetX  = temp_offset_x;

            // PositionOffsetXYZ | pos.offsetY - pos.offset_y.object_dist
            long temp_offset_y = in_object.detected_object_common_data.pos.offset_y.object_distance;

            if(temp_offset_y < j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding y offset value less than min, setting to min");
                temp_offset_y= j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE;
            }
            else if(temp_offset_y > j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE){

                RCLCPP_WARN(node_logging_->get_logger(),"Encoding y offset value greater than max, setting to max");
                temp_offset_y = j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE;
            }
            encode_obj_com->pos.offsetY  = temp_offset_y;

            // PositionOffsetXYZ | *pos.offsetZ - pos.offset_z.object_dist
            if(in_object.detected_object_common_data.pos.presence_vector & j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z){
                auto offset_z_ptr = create_store_shared<ObjectDistance_t>(shared_ptrs);

                long temp_offset_z = in_object.detected_object_common_data.pos.offset_z.object_distance;
                if(temp_offset_z < j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE){

                    RCLCPP_WARN(node_logging_->get_logger(),"Encoding z offset value less than min, setting to min");
                    temp_offset_z = j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE;
                }
                else if(temp_offset_z > j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE){

                    RCLCPP_WARN(node_logging_->get_logger(),"Encoding z offset value greater than max, setting to max");
                    temp_offset_z= j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE;
                }
                *offset_z_ptr = temp_offset_z;

                encode_obj_com->pos.offsetZ = offset_z_ptr;
            }


            // PositionConfidenceSet | posConfidence.pos - pos_confidence.pos.confidence
            if(!in_object.detected_object_common_data.pos_confidence.pos.confidence || in_object.detected_object_common_data.pos_confidence.pos.confidence == j2735_v2x_msgs::msg::PositionConfidence::UNAVAILABLE){
                encode_obj_com->posConfidence.pos = j2735_v2x_msgs::msg::PositionConfidence::UNAVAILABLE;
            }
            else{
                encode_obj_com->posConfidence.pos = in_object.detected_object_common_data.pos_confidence.pos.confidence;
            }

            // PositionConfidenceSet | posConfidence.elevation - pos_confidence.elevation.confidence
            if(in_object.detected_object_common_data.pos_confidence.elevation.confidence == j2735_v2x_msgs::msg::ElevationConfidence::UNAVAILABLE){
                encode_obj_com->posConfidence.elevation = j2735_v2x_msgs::msg::ElevationConfidence::UNAVAILABLE;
            }
            else{
                encode_obj_com->posConfidence.elevation = in_object.detected_object_common_data.pos_confidence.elevation.confidence;
            }


            // Speed | speed - speed
            if(in_object.detected_object_common_data.speed.speed == j2735_v2x_msgs::msg::Speed::UNAVAILABLE){
                encode_obj_com->speed = j2735_v2x_msgs::msg::Speed::UNAVAILABLE;
            }
            else{
                uint16_t temp_speed = in_object.detected_object_common_data.speed.speed;
                if(temp_speed  > j2735_v2x_msgs::msg::Speed::MAX){

                    RCLCPP_WARN(node_logging_->get_logger(),"Encoding speed value greater than max, setting to max");
                    temp_speed = j2735_v2x_msgs::msg::Speed::MAX;
                }
                encode_obj_com->speed = temp_speed;
            }


            // SpeedConfidence | speedConfidence - speed_confidence.speed_confidence
            if(!in_object.detected_object_common_data.speed_confidence.speed_confidence || in_object.detected_object_common_data.speed_confidence.speed_confidence == j2735_v2x_msgs::msg::SpeedConfidence::UNAVAILABLE){
                encode_obj_com->speedConfidence = j2735_v2x_msgs::msg::SpeedConfidence::UNAVAILABLE;
            }
            else{
                encode_obj_com->speedConfidence = in_object.detected_object_common_data.speed_confidence.speed_confidence;
            }


            // Heading | heading - heading
            if(in_object.detected_object_common_data.heading.heading == j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE){
                encode_obj_com->heading = j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE;
            }
            else{
                uint16_t temp_heading = in_object.detected_object_common_data.heading.heading;
                if(temp_heading > j2735_v2x_msgs::msg::Heading::HEADING_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(),"Encoding heading value greater than max/unavailable, setting to max");
                    temp_heading = j2735_v2x_msgs::msg::Heading::HEADING_MAX;
                }
                encode_obj_com->heading = temp_heading;
            }
            

            // HeadingConfidence | headingConf heading_conf.confidence
            if(!in_object.detected_object_common_data.heading_conf.confidence || in_object.detected_object_common_data.heading_conf.confidence == j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE){
                encode_obj_com->headingConf = j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE;
            }
            else{
                encode_obj_com->headingConf = in_object.detected_object_common_data.heading_conf.confidence;
            }


            // Speed | speedZ - speed_z.speed
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z){
                auto temp_speed_z = create_store_shared<Speed_t>(shared_ptrs);

                if(in_object.detected_object_common_data.speed_z.speed == j2735_v2x_msgs::msg::Speed::UNAVAILABLE){
                    *temp_speed_z = j2735_v2x_msgs::msg::Speed::UNAVAILABLE;
                }
                else{
                    *temp_speed_z = in_object.detected_object_common_data.speed_z.speed;
                    if(*temp_speed_z > j2735_v2x_msgs::msg::Speed::UNAVAILABLE){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding speed z value greater than max/unavailable, setting to max");
                        *temp_speed_z = j2735_v2x_msgs::msg::Speed::MAX;
                    }
                }
                encode_obj_com->speedZ = temp_speed_z;
            }


            // SpeedConfidence | speedConfidenceZ - speed_confidence_z
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z){
                auto temp_spd_conf_z = create_store_shared<SpeedConfidence_t>(shared_ptrs);

                if(!in_object.detected_object_common_data.speed_confidence_z.speed_confidence || in_object.detected_object_common_data.speed_confidence_z.speed_confidence == j2735_v2x_msgs::msg::SpeedConfidence::UNAVAILABLE){
                    *temp_spd_conf_z = j2735_v2x_msgs::msg::SpeedConfidence::UNAVAILABLE;
                }
                else{
                    *temp_spd_conf_z = in_object.detected_object_common_data.speed_confidence_z.speed_confidence;
                }
                encode_obj_com->speedConfidenceZ = temp_spd_conf_z;
            }


            // accel4way - accel_4_way
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY){
                auto temp_accel_4way = create_store_shared<AccelerationSet4Way_t>(shared_ptrs);
                // Long - longitudinal
                if(!in_object.detected_object_common_data.accel_4_way.longitudinal || in_object.detected_object_common_data.accel_4_way.longitudinal == j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE){
                    temp_accel_4way->Long = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE;
                }
                else{
                    long temp_long = in_object.detected_object_common_data.accel_4_way.longitudinal;
                    if(temp_long < j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding longitudinal accel value less than min, setting to min");
                        temp_long = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN;
                    }
                    else if(temp_long > j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding longitudinal accel value greater than max, setting to max");
                        temp_long = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX;
                    }
                    temp_accel_4way->Long = temp_long;
                }

                // lat - lateral
                if(!in_object.detected_object_common_data.accel_4_way.lateral || in_object.detected_object_common_data.accel_4_way.lateral == j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE){
                    temp_accel_4way->lat = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE;
                }
                else{
                    long temp_lat = in_object.detected_object_common_data.accel_4_way.lateral;
                    if(temp_lat < j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding lateral accel value less than min, setting to min");
                        temp_lat = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN;
                    }
                    else if(temp_lat > j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding lateral accel value greater than max, setting to max");
                        temp_lat = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX;
                    }
                    temp_accel_4way->lat = temp_lat;
                }

                // vert - vert
                if(!in_object.detected_object_common_data.accel_4_way.vert || in_object.detected_object_common_data.accel_4_way.vert == j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE){
                    temp_accel_4way->vert = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE;
                }
                else{
                    long temp_vert = in_object.detected_object_common_data.accel_4_way.vert;
                    if(temp_vert < j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MIN){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding vert accel value less than min, setting to min");
                        temp_vert = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MIN;
                    }
                    else if(temp_vert > j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding vert accel value greater than max, setting to max");
                        temp_vert = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX;
                    }
                    temp_accel_4way->vert = temp_vert;
                }

                // yaw - yaw_Rate
                if(!in_object.detected_object_common_data.accel_4_way.yaw_rate || in_object.detected_object_common_data.accel_4_way.yaw_rate == j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_UNAVAILABLE){
                    temp_accel_4way->yaw = j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_UNAVAILABLE;
                }
                else{
                    long temp_yaw = in_object.detected_object_common_data.accel_4_way.yaw_rate;
                    if(temp_yaw < j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding yaw value less than min, setting to min");
                        temp_yaw = j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN;
                    }
                    else if(temp_yaw > j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX){

                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding yaw value greater than max, setting to max");
                        temp_yaw = j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX;
                    }
                    temp_accel_4way->yaw = temp_yaw;
                }

                encode_obj_com->accel4way = temp_accel_4way;
            }


            // AccelerationConfidence | accCfdX - acc_cfd_x
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X){
                auto temp_acc_cfd_x = create_store_shared<AccelerationConfidence_t>(shared_ptrs);

                *temp_acc_cfd_x = in_object.detected_object_common_data.acc_cfd_x.acceleration_confidence;
                encode_obj_com->accCfdX = temp_acc_cfd_x;
            }


            // AccelerationConfidence | accCfdY - acc_cfd_y
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y){
                auto temp_acc_cfd_y = create_store_shared<AccelerationConfidence_t>(shared_ptrs);

                *temp_acc_cfd_y = in_object.detected_object_common_data.acc_cfd_y.acceleration_confidence;
                encode_obj_com->accCfdY = temp_acc_cfd_y;
            }


            // AccelerationConfidence | accCfdZ - acc_cfd_z
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z){
                auto temp_acc_cfd_z = create_store_shared<AccelerationConfidence_t>(shared_ptrs);

                *temp_acc_cfd_z = in_object.detected_object_common_data.acc_cfd_z.acceleration_confidence;
                encode_obj_com->accCfdZ = temp_acc_cfd_z;
            }


            // YawRateConfidence | accCfdYaw - acc_cfd_yaw
            if(in_object.detected_object_common_data.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW){
                auto temp_acc_cfd_yaw = create_store_shared<YawRateConfidence_t>(shared_ptrs);

                *temp_acc_cfd_yaw = in_object.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence;
                encode_obj_com->accCfdYaw = temp_acc_cfd_yaw;
            }

            encode_object->detObjCommon = *encode_obj_com;

            
            //// DetectedObjectOptionalData

            // detObjOptData - detected_object_optional_data
            if(in_object.presence_vector & j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA){
                // Optional data
                auto encode_obj_opt = create_store_shared<DetectedObjectOptionalData_t>(shared_ptrs);


                // DetectedVehicleData | detVeh - det_veh
                if(in_object.detected_object_optional_data.choice == j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH){
                    encode_obj_opt->present = DetectedObjectOptionalData_PR_detVeh;
                    auto encode_det_veh = create_store_shared<DetectedVehicleData_t>(shared_ptrs);

                    // ExteriorLights | lights - lights
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS){

                        auto temp_lights = create_store_shared<ExteriorLights_t>(shared_ptrs);

                        std::string lights_str = std::to_string(in_object.detected_object_optional_data.det_veh.lights.exterior_lights);
                        size_t light_size = lights_str.size();

                        auto light_arr = create_store_shared_array<uint8_t>(shared_ptrs, light_size);
                        for(size_t copy_itr = 0; copy_itr < light_size; copy_itr++){
                            light_arr[copy_itr] = lights_str[copy_itr] - '0';
                        }

                        temp_lights->size = light_size;
                        temp_lights->buf = light_arr;

                        encode_det_veh->lights = temp_lights;
                    }


                    // Attitude | vehAttitude - veh_attitude
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE){
                        auto temp_attitude = create_store_shared<Attitude_t>(shared_ptrs);

                        // pitch - pitch.pitch_detected
                        long temp_pitch_det = in_object.detected_object_optional_data.det_veh.veh_attitude.pitch.pitch_detected;
                        if(temp_pitch_det < j3224_v2x_msgs::msg::PitchDetected::MIN_PITCH_DETECTED){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attitude pitch detected value less than min, setting to min");
                            temp_pitch_det = j3224_v2x_msgs::msg::PitchDetected::MIN_PITCH_DETECTED;
                        }
                        else if(temp_pitch_det > j3224_v2x_msgs::msg::PitchDetected::MAX_PITCH_DETECTED){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attitude pitch detected value greater than max, setting to max");
                            temp_pitch_det = j3224_v2x_msgs::msg::PitchDetected::MAX_PITCH_DETECTED;
                        }
                        temp_attitude->pitch = temp_pitch_det;

                        // roll - roll.roll_detected
                        long temp_roll_det = in_object.detected_object_optional_data.det_veh.veh_attitude.roll.roll_detected;
                        if(temp_roll_det < j3224_v2x_msgs::msg::RollDetected::MIN_ROLL_DETECTED){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attitude roll detected value less than min, setting to min");
                            temp_roll_det = j3224_v2x_msgs::msg::RollDetected::MIN_ROLL_DETECTED;
                        }
                        else if(temp_roll_det > j3224_v2x_msgs::msg::RollDetected::MAX_ROLL_DETECTED){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attitude roll detected value greater than max, setting to max");
                            temp_roll_det = j3224_v2x_msgs::msg::RollDetected::MAX_ROLL_DETECTED;
                        }
                        temp_attitude->roll = temp_roll_det;

                        // yaw - yaw.yaw_detected
                        long temp_yaw_det = in_object.detected_object_optional_data.det_veh.veh_attitude.yaw.yaw_detected;
                        if(temp_yaw_det < j3224_v2x_msgs::msg::YawDetected::MIN_YAW_DETECTED){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attitude yaw detected value less than min, setting to min");
                            temp_yaw_det = j3224_v2x_msgs::msg::YawDetected::MIN_YAW_DETECTED;
                        }
                        else if(temp_yaw_det > j3224_v2x_msgs::msg::YawDetected::MAX_YAW_DETECTED){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attitude yaw detected value greater than max, setting to max");
                            temp_yaw_det = j3224_v2x_msgs::msg::YawDetected::MAX_YAW_DETECTED;
                        }
                        temp_attitude->yaw = temp_yaw_det;

                        encode_det_veh->vehAttitude = temp_attitude;
                    }


                    // AttitudeConfidence | vehAttitudeConfidence - veh_attitude_confidence
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE){
                        auto temp_attitude_conf = create_store_shared<AttitudeConfidence_t>(shared_ptrs);

                        // pitch - pitch.pitch_detected
                        uint8_t temp_pitch_conf = in_object.detected_object_optional_data.det_veh.veh_attitude_confidence.pitch_confidence.confidence;
                        temp_attitude_conf->pitchConfidence = temp_pitch_conf;

                        // roll - roll.roll_detected
                        uint8_t temp_roll_conf = in_object.detected_object_optional_data.det_veh.veh_attitude_confidence.roll_confidence.confidence;
                        temp_attitude_conf->rollConfidence = temp_roll_conf;

                        // yaw - yaw.yaw_detected
                        uint8_t temp_yaw_conf = in_object.detected_object_optional_data.det_veh.veh_attitude_confidence.yaw_confidence.confidence;

                        temp_attitude_conf->yawConfidence = temp_yaw_conf;
                        encode_det_veh->vehAttitudeConfidence = temp_attitude_conf;

                    }


                    // AngularVelocity | vehAngVel - veh_ang_vel
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL){
                        auto temp_ang_vel = create_store_shared<AngularVelocity_t>(shared_ptrs);

                        // pitchRate - pitch_rate.pitch_rate
                        if(!in_object.detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate || in_object.detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate == j3224_v2x_msgs::msg::PitchRate::UNAVAILABLE){
                            temp_ang_vel->pitchRate = j3224_v2x_msgs::msg::PitchRate::UNAVAILABLE;
                        }
                        else{
                            long temp_pitch_rate = in_object.detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate;
                            if(temp_pitch_rate < j3224_v2x_msgs::msg::PitchRate::MIN_PITCH_RATE){

                                RCLCPP_WARN(node_logging_->get_logger(),"Encoding pitch rate detected value less than min, setting to min");
                                temp_pitch_rate = j3224_v2x_msgs::msg::PitchRate::MIN_PITCH_RATE;
                            }
                            else if(temp_pitch_rate > j3224_v2x_msgs::msg::PitchRate::MAX_PITCH_RATE){

                                RCLCPP_WARN(node_logging_->get_logger(),"Encoding pitch rate detected value greater than max, setting to max");
                                temp_pitch_rate = j3224_v2x_msgs::msg::PitchRate::MAX_PITCH_RATE;
                            }
                            temp_ang_vel->pitchRate = temp_pitch_rate;
                        }

                        // rollRate - roll_rate.roll_rate
                        if(!in_object.detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate || in_object.detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate == j3224_v2x_msgs::msg::RollRate::UNAVAILABLE){
                            temp_ang_vel->rollRate = j3224_v2x_msgs::msg::RollRate::UNAVAILABLE;
                        }
                        else{
                            long temp_roll_rate = in_object.detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate;
                            if(temp_roll_rate < j3224_v2x_msgs::msg::RollRate::MIN_ROLL_RATE){

                                RCLCPP_WARN(node_logging_->get_logger(),"Encoding roll rate detected value less than min, setting to min");
                                temp_roll_rate = j3224_v2x_msgs::msg::RollRate::MIN_ROLL_RATE;
                            }
                            else if(temp_roll_rate > j3224_v2x_msgs::msg::RollRate::MAX_ROLL_RATE){

                                RCLCPP_WARN(node_logging_->get_logger(),"Encoding roll rate detected value greater than max, setting to max");
                                temp_roll_rate = j3224_v2x_msgs::msg::RollRate::MAX_ROLL_RATE;
                            }
                            temp_ang_vel->rollRate = temp_roll_rate;
                        }

                        encode_det_veh->vehAngVel = temp_ang_vel;

                    }


                    // AngularVelocityConfidence | *vehAngVelConfidence - veh_ang_vel_confidence
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE){
                        auto temp_ang_vel_conf = create_store_shared<AngularVelocityConfidence_t>(shared_ptrs);

                        // pitchRateConfidence - pitch_rate_confidence
                        if(in_object.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector & j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_PITCH_RATE_CONFIDENCE){
                            auto temp_pitch_rate_conf = create_store_shared<PitchRateConfidence_t>(shared_ptrs);

                            *temp_pitch_rate_conf = in_object.detected_object_optional_data.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence;
                            temp_ang_vel_conf->pitchRateConfidence = temp_pitch_rate_conf;
                        }

                        // rollRateConfidence - *roll_rate_confidence
                        if(in_object.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector & j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_ROLL_RATE_CONFIDENCE){
                            auto temp_roll_rate_conf = create_store_shared<RollRateConfidence_t>(shared_ptrs);

                            *temp_roll_rate_conf = in_object.detected_object_optional_data.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence;
                            temp_ang_vel_conf->rollRateConfidence = temp_roll_rate_conf;
                        }

                        encode_det_veh->vehAngVelConfidence = temp_ang_vel_conf;
                    }


                    // VehicleSize | size - size
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE){
                        auto temp_veh_size = create_store_shared<VehicleSize_t>(shared_ptrs);

                        // width - vehicle_width
                        uint16_t temp_veh_width = in_object.detected_object_optional_data.det_veh.size.vehicle_width;
                        if(temp_veh_width > j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding vehicle width value greater than max, setting to max");
                            temp_veh_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX;
                        }
                        temp_veh_size->width = temp_veh_width;

                        // length - vehicle_length
                        uint16_t temp_veh_length = in_object.detected_object_optional_data.det_veh.size.vehicle_length;
                        if(temp_veh_length > j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding vehicle length value greater than max, setting to max");
                            temp_veh_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX;
                        }
                        temp_veh_size->length = temp_veh_length;
                    
                        encode_det_veh->size = temp_veh_size;
                    }


                    // VehicleHeight | height - height.vehicle_height
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT){
                        auto temp_veh_height = create_store_shared<VehicleHeight_t>(shared_ptrs);
                        
                        *temp_veh_height = in_object.detected_object_optional_data.det_veh.height.vehicle_height;
                        if(*temp_veh_height > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){

                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding vehicle height value greater than max, setting to max");
                            *temp_veh_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                        }
                        encode_det_veh->height = temp_veh_height;
                    }


                    // VehicleSizeConfidence | vehicleSizeConfidence - vehicle_size_confidence
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE){
                        auto temp_veh_size_conf = create_store_shared<VehicleSizeConfidence_t>(shared_ptrs);

                        // vehicleWidthConfidence - vehicle_width_confidence
                        temp_veh_size_conf->vehicleWidthConfidence = in_object.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence;
                        // vehicleLengthConfidence - vehicle_length_confidence
                        temp_veh_size_conf->vehicleLengthConfidence = in_object.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence;

                        // vehicleHeightConfidence - vehicle_height_confidence
                        if(in_object.detected_object_optional_data.det_veh.vehicle_size_confidence.presence_vector & j3224_v2x_msgs::msg::VehicleSizeConfidence::HAS_VEHICLE_HEIGHT_CONFIDENCE){
                            auto temp_veh_height_conf = create_store_shared<SizeValueConfidence_t>(shared_ptrs);

                            *temp_veh_height_conf = in_object.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence;
                            temp_veh_size_conf->vehicleHeightConfidence = temp_veh_height_conf;
                        }

                        encode_det_veh->vehicleSizeConfidence = temp_veh_size_conf;
                    }


                    // BasicVehicleClass | vehicleClass - vehicle_class
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS){
                        auto temp_veh_class = create_store_shared<BasicVehicleClass_t>(shared_ptrs);

                        *temp_veh_class = in_object.detected_object_optional_data.det_veh.vehicle_class.basic_vehicle_class;
                        encode_det_veh->vehicleClass = temp_veh_class;
                    }


                    // ClassificationConfidence | classConf - class_conf
                    if(in_object.detected_object_optional_data.det_veh.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF){
                        auto temp_class_conf = create_store_shared<ClassificationConfidence_t>(shared_ptrs);

                        *temp_class_conf = in_object.detected_object_optional_data.det_veh.class_conf.classification_confidence;
                        if(*temp_class_conf > j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE){
                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding classification confidence value greater than max, setting to max");
                            *temp_class_conf = j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE;
                        }
                        encode_det_veh->classConf = temp_class_conf;
                    }

                    encode_obj_opt->choice.detVeh = *encode_det_veh;

                }


                // DetectedVRUData | detVRU - det_vru
                if(in_object.detected_object_optional_data.choice == j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU){
                    encode_obj_opt->present = DetectedObjectOptionalData_PR_detVRU;
                    auto encode_det_vru = create_store_shared<DetectedVRUData_t>(shared_ptrs);


                    // PersonalDeviceUserType | basicType - basic_type
                    if(in_object.detected_object_optional_data.det_vru.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE){
                        auto temp_basic_type = create_store_shared<PersonalDeviceUserType_t>(shared_ptrs);

                        *temp_basic_type = in_object.detected_object_optional_data.det_vru.basic_type.type;
                        encode_det_vru->basicType = temp_basic_type;
                    }

                    // PropelledInformation | propulsion - propulsion
                    if(in_object.detected_object_optional_data.det_vru.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION){
                        auto temp_propulsion = create_store_shared<PropelledInformation_t>(shared_ptrs);

                        // human - human
                        if(in_object.detected_object_optional_data.det_vru.propulsion.choice == j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN){
                            temp_propulsion->present = PropelledInformation_PR_human;
                            
                            temp_propulsion->choice.human = in_object.detected_object_optional_data.det_vru.propulsion.human.type;

                            }
                        // animal - animal
                        else if(in_object.detected_object_optional_data.det_vru.propulsion.choice == j2735_v2x_msgs::msg::PropelledInformation::CHOICE_ANIMAL){
                            temp_propulsion->present = PropelledInformation_PR_animal;
                            
                            temp_propulsion->choice.animal = in_object.detected_object_optional_data.det_vru.propulsion.animal.type;

                            }
                        // motor - motor
                        else if(in_object.detected_object_optional_data.det_vru.propulsion.choice == j2735_v2x_msgs::msg::PropelledInformation::CHOICE_MOTOR){
                            temp_propulsion->present = PropelledInformation_PR_motor;
                            
                            temp_propulsion->choice.motor = in_object.detected_object_optional_data.det_vru.propulsion.motor.type;

                            }

                        encode_det_vru->propulsion = temp_propulsion;
                    }


                    // Attachment | attachment - attachment
                    if(in_object.detected_object_optional_data.det_vru.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT){
                        auto temp_attachment = create_store_shared<Attachment_t>(shared_ptrs);

                        *temp_attachment = in_object.detected_object_optional_data.det_vru.attachment.type;
                        encode_det_vru->attachment = temp_attachment;
                    }


                    // AttachmentRadius | radius - attachment_radius
                    if(in_object.detected_object_optional_data.det_vru.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS){
                        auto temp_radius = create_store_shared<AttachmentRadius_t>(shared_ptrs);

                        *temp_radius = in_object.detected_object_optional_data.det_vru.radius.attachment_radius;
                        if(*temp_radius > j2735_v2x_msgs::msg::AttachmentRadius::ATTACHMENT_RADIUS_MAX){
                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding attachment radius value greater than max, setting to max");
                            *temp_radius = j2735_v2x_msgs::msg::AttachmentRadius::ATTACHMENT_RADIUS_MAX;
                        }
                        encode_det_vru->radius = temp_radius;
                    }

                    encode_obj_opt->choice.detVRU = *encode_det_vru;

                }


                // DetectedObstacleData | detObst - det_obst
                if(in_object.detected_object_optional_data.choice == j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST){
                    encode_obj_opt->present = DetectedObjectOptionalData_PR_detObst;
                    auto encode_det_obst = create_store_shared<DetectedObstacleData_t>(shared_ptrs);

                    auto temp_obst_size = create_store_shared<ObstacleSize_t>(shared_ptrs);

                    // width - width
                    uint16_t temp_obst_width = in_object.detected_object_optional_data.det_obst.obst_size.width.size_value;
                    if(temp_obst_width > j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE){
                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding obst width value greater than max, setting to max");
                        temp_obst_width = j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE;
                    }
                    temp_obst_size->width = temp_obst_width;

                    // length - length
                    uint16_t temp_obst_length = in_object.detected_object_optional_data.det_obst.obst_size.length.size_value;
                    if(temp_obst_length > j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE){
                        RCLCPP_WARN(node_logging_->get_logger(),"Encoding obst length value greater than max, setting to max");
                        temp_obst_length = j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE;
                    }
                    temp_obst_size->length = temp_obst_length;

                    // height - height
                    if(in_object.detected_object_optional_data.det_obst.obst_size.presence_vector & j3224_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT){
                        auto temp_obst_height = create_store_shared<SizeValue_t>(shared_ptrs);

                        *temp_obst_height = in_object.detected_object_optional_data.det_obst.obst_size.height.size_value;
                        if(*temp_obst_height > j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE){
                            RCLCPP_WARN(node_logging_->get_logger(),"Encoding obst height value greater than max, setting to max");
                            *temp_obst_height = j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE;
                        }
                        temp_obst_size->height = temp_obst_height;
                    }

                    encode_det_obst->obstSize = *temp_obst_size;


                    // obstSizeConfidence - obst_size_confidence
                    auto temp_obst_size_conf = create_store_shared<ObstacleSizeConfidence_t>(shared_ptrs);

                    // widthConfidence - width_confidence
                    uint8_t temp_obst_width_conf = in_object.detected_object_optional_data.det_obst.obst_size_confidence.width_confidence.size_value_confidence;
                    temp_obst_size_conf->widthConfidence = temp_obst_width_conf;

                    // lengthConfidence - length_confidence
                    uint8_t temp_obst_length_conf = in_object.detected_object_optional_data.det_obst.obst_size_confidence.length_confidence.size_value_confidence;
                    temp_obst_size_conf->lengthConfidence = temp_obst_length_conf;

                    // heightConfidence - height_confidence
                    if(in_object.detected_object_optional_data.det_obst.obst_size_confidence.presence_vector & j3224_v2x_msgs::msg::ObstacleSizeConfidence::HAS_HEIGHT_CONFIDENCE){
                        auto temp_obst_height_conf = create_store_shared<SizeValueConfidence_t>(shared_ptrs);

                        *temp_obst_height_conf = in_object.detected_object_optional_data.det_obst.obst_size_confidence.height_confidence.size_value_confidence;
                        temp_obst_size_conf->heightConfidence = temp_obst_height_conf;
                    }
                    encode_det_obst->obstSizeConfidence = *temp_obst_size_conf;

                    encode_obj_opt->choice.detObst = *encode_det_obst;

                }

                // Add the temporary optional object data to the final encode object
                encode_object->detObjOptData = encode_obj_opt;

            }

            // Add a finalized object to the detected object list
            asn_sequence_add(&detected_object_list->list, encode_object);
        }
        message->value.choice.SensorDataSharingMessage.objects = *detected_object_list;



        // Encode message
        ec = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
        
        // Uncomment the line below to save the message to the encoded-sdsm-output.txt file
        // asn_fprint(fp, &asn_DEF_MessageFrame, message);
    
        // log a warning if encoding fails
        if(ec.encoded == -1) {
            RCLCPP_WARN( node_logging_->get_logger(), "Encoding for SDSM Message failed");
            std::cout << "Failed: " << ec.failed_type->name << std::endl;
            return std::optional<std::vector<uint8_t>>{};
        }
        // copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
                
        //for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return std::optional<std::vector<uint8_t>>(b_array);

    }



    // SDSM decoding
    std::optional<j3224_v2x_msgs::msg::SensorDataSharingMessage> SDSM_Message::decode_sdsm_message(const std::vector<uint8_t>&binary_array){
        
        j3224_v2x_msgs::msg::SensorDataSharingMessage output;
        // decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message = nullptr;
        
        // copy from vector to array         
        size_t len = binary_array.size();    
        std::vector<uint8_t> buf;

        std::copy(binary_array.cbegin(), binary_array.cend(), std::back_inserter(buf));
        
        // use asn1c lib to decode
        rval = uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf.data(), len, 0, 0);

        if(rval.code == RC_OK)
        {
            // Incoming SDSM in ASN.1 C-struct format
            SensorDataSharingMessage_t sdsm_core = message->value.choice.SensorDataSharingMessage;
            
            // MessageCount
            if(sdsm_core.msgCnt < j2735_v2x_msgs::msg::MsgCount::MSG_COUNT_MAX){
                output.msg_cnt.msg_cnt = sdsm_core.msgCnt;
            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM MsgCount above max");
            }

            // SourceID
            if(sdsm_core.sourceID.size > 0){
                auto id_len = sdsm_core.sourceID.size;
                std::vector<std::reference_wrapper<std::uint8_t>> id_arr{sdsm_core.sourceID.buf, std::next(sdsm_core.sourceID.buf, id_len)};

                for(const auto & id_ind : id_arr){
                    output.source_id.id.push_back(id_ind);
                }

            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM SourceID does not exist");
            }

            // EquipmentType
            if(sdsm_core.equipmentType){
                output.equipment_type.equipment_type = sdsm_core.equipmentType;
            }
            else{
                output.equipment_type.equipment_type = j3224_v2x_msgs::msg::EquipmentType::UNKNOWN;
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM EquipmentType does not exist, set to unknown");
            }

            // SDSMTimeStamp
            if(sdsm_core.sDSMTimeStamp.year){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
                long year = *sdsm_core.sDSMTimeStamp.year;
                if(year > j2735_v2x_msgs::msg::DYear::MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM year above max, setting to max");
                    year = j2735_v2x_msgs::msg::DYear::MAX;
                }
                output.sdsm_time_stamp.year.year = year;
            }
            if(sdsm_core.sDSMTimeStamp.month){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
                long month = *sdsm_core.sDSMTimeStamp.month;
                if(month > j2735_v2x_msgs::msg::DMonth::MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM month above max, setting to max");
                    month = j2735_v2x_msgs::msg::DMonth::MAX;
                }
                output.sdsm_time_stamp.month.month = month;
            }
            if(sdsm_core.sDSMTimeStamp.day){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
                long day = *sdsm_core.sDSMTimeStamp.day;
                if(day > j2735_v2x_msgs::msg::DDay::MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM day above max, setting to max");
                    day = j2735_v2x_msgs::msg::DDay::MAX;
                }
                output.sdsm_time_stamp.day.day = day;
            }
            if(sdsm_core.sDSMTimeStamp.hour){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
                long hour = *sdsm_core.sDSMTimeStamp.hour;
                if(hour > j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM hour above max, setting to max");
                    hour = j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX;
                }
                output.sdsm_time_stamp.hour.hour = hour;
            }
            if(sdsm_core.sDSMTimeStamp.minute){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
                long minute = *sdsm_core.sDSMTimeStamp.minute;
                if(minute > j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM minute above max, setting to max");
                    minute = j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX;
                }
                output.sdsm_time_stamp.minute.minute = minute;
            }
            // Note: DSecond has a NON_LEAP_MIN/MAX and LEAP_MIN/MAX, depending on the system we may want to consider
            //       how to handle these thresholds in the case where leap second values are generated
            if(sdsm_core.sDSMTimeStamp.second){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
                long second = *sdsm_core.sDSMTimeStamp.second;
                if(second > j2735_v2x_msgs::msg::DSecond::MILLISEC_WITHIN_MINUTE_NON_LEAP_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM second above max, setting to max");
                    second = j2735_v2x_msgs::msg::DSecond::MILLISEC_WITHIN_MINUTE_NON_LEAP_MAX;
                }
                output.sdsm_time_stamp.second.millisecond = second;
            }
            if(sdsm_core.sDSMTimeStamp.offset){
                output.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
                long offset = *sdsm_core.sDSMTimeStamp.offset;
                if(offset > j2735_v2x_msgs::msg::DOffset::MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM offset above max, setting to max");
                    offset = j2735_v2x_msgs::msg::DOffset::MAX;
                }
                output.sdsm_time_stamp.offset.offset_minute = offset;
            }

            // Position3D - lat
            if (sdsm_core.refPos.lat && sdsm_core.refPos.lat != j2735_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE){
                double lat = sdsm_core.refPos.lat;
                if(lat > j2735_v2x_msgs::msg::Position3D::LATITUDE_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM refPos.lat above max, setting to max");
                    lat = j2735_v2x_msgs::msg::Position3D::LATITUDE_MAX;
                }
                else if(lat < j2735_v2x_msgs::msg::Position3D::LATITUDE_MIN){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM refPos.lat below min, setting to min");
                    lat = j2735_v2x_msgs::msg::Position3D::LATITUDE_MIN;
                }
                output.ref_pos.latitude = lat;
            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing refPos.lat, set to unavailable");
                output.ref_pos.latitude = j2735_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE;
            }
            // Position3D - Long
            if (sdsm_core.refPos.Long && sdsm_core.refPos.Long != j2735_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE){
                double longitude = sdsm_core.refPos.Long;
                if(longitude > j2735_v2x_msgs::msg::Position3D::LONGITUDE_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM refPos.Long above max, setting to max");
                    longitude = j2735_v2x_msgs::msg::Position3D::LONGITUDE_MAX;
                }
                else if(longitude < j2735_v2x_msgs::msg::Position3D::LONGITUDE_MIN){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM refPos.Long below min, setting to min");
                    longitude = j2735_v2x_msgs::msg::Position3D::LONGITUDE_MIN;
                }
                output.ref_pos.longitude = longitude;
            }
            else{
                 RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing refPos.Long, set to unavailable");
                output.ref_pos.longitude = j2735_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE;
            }
            // Position3D - *elevation
            if(sdsm_core.refPos.elevation && *sdsm_core.refPos.elevation != j2735_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE){
                double elev = *sdsm_core.refPos.elevation;
                output.ref_pos.elevation_exists = true;
                if(elev > j2735_v2x_msgs::msg::Position3D::ELEVATION_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM refPos.elevation above max, setting to max");
                    elev = j2735_v2x_msgs::msg::Position3D::ELEVATION_MAX;
                }
                else if(elev < j2735_v2x_msgs::msg::Position3D::ELEVATION_MIN){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM refPos.elevation below min, setting to min");
                    elev = j2735_v2x_msgs::msg::Position3D::ELEVATION_MIN;
                }
                output.ref_pos.elevation = elev;
            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing refPos.elevation, set to unavailable");
                output.ref_pos.elevation = j2735_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE;
            }
            // PositionalAccuracy - semiMajor
            if(sdsm_core.refPosXYConf.semiMajor && sdsm_core.refPosXYConf.semiMajor != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE){
                long semi_major = sdsm_core.refPosXYConf.semiMajor;
                if(semi_major > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM semiMajor above max, setting to max");
                    semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX;
                }
                else if(semi_major < j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM semiMajor below min, setting to min");
                    semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                }
                output.ref_pos_xy_conf.semi_major = semi_major;
            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing semiMajor, set to unavailable");
                output.ref_pos_xy_conf.semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
            }
            // PositionalAccuracy - semiMinor
            if(sdsm_core.refPosXYConf.semiMinor && sdsm_core.refPosXYConf.semiMajor != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE){
                long semi_minor = sdsm_core.refPosXYConf.semiMinor;
                if(semi_minor > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM semiMinor above max, setting to max");
                    semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX;
                }
                else if(semi_minor < j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM semiMinor below min, setting to min");
                    semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                }
                output.ref_pos_xy_conf.semi_minor = semi_minor;
            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing semiMinor, set to unavailable");
                output.ref_pos_xy_conf.semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
            }
            // PositionalAccuracy - orientation
            if(sdsm_core.refPosXYConf.orientation && sdsm_core.refPosXYConf.orientation != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE){
                long orientation = sdsm_core.refPosXYConf.orientation;
                if(orientation > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM orientation above max, setting to max");
                    orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX;
                }
                else if(orientation < j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM orientation below min, setting to min");
                    orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN;
                }
                output.ref_pos_xy_conf.orientation = orientation;
            }
            else{
                RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing orientation, set to unavailable");
                output.ref_pos_xy_conf.orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE;  
            }

            // ElevationConfidence
            if(sdsm_core.refPosElConf){
                output.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
                output.ref_pos_el_conf.confidence = *sdsm_core.refPosElConf;
            }

            // objects
            j3224_v2x_msgs::msg::DetectedObjectList detected_objects;

            for(auto obj_itr = 0; obj_itr < sdsm_core.objects.list.count; ++obj_itr){


                if(obj_itr > j3224_v2x_msgs::msg::DetectedObjectList::DETECTED_OBJECT_DATA_MAX_SIZE){
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM DetectedObjectList size greater than max, rejecting points");
                    break;
                }

                j3224_v2x_msgs::msg::DetectedObjectData object_data;

                j3224_v2x_msgs::msg::DetectedObjectCommonData common_data;

                // ObjectType
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.objType){
                    common_data.obj_type.object_type = sdsm_core.objects.list.array[obj_itr]->detObjCommon.objType;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM ObjectType not found, setting to unknown");
                    common_data.obj_type.object_type = j3224_v2x_msgs::msg::ObjectType::UNKNOWN;
                }

                // ClassificationConfidence
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.objTypeCfd){
                    long class_cfd = sdsm_core.objects.list.array[obj_itr]->detObjCommon.objTypeCfd;
                    if(class_cfd > j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM objTypeCfd above max, setting to max");
                        class_cfd = j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE;
                    }
                    else if(class_cfd < j3224_v2x_msgs::msg::ClassificationConfidence::MIN_CLASSIFICATION_CONFIDENCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM objTypeCfd below min, setting to min");
                        class_cfd = j3224_v2x_msgs::msg::ClassificationConfidence::MIN_CLASSIFICATION_CONFIDENCE;
                    }
                    common_data.obj_type_cfd.classification_confidence = class_cfd;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Detected object missing objTypeCfd");
                }

                // ObjectID
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.objectID){
                    long obj_id = sdsm_core.objects.list.array[obj_itr]->detObjCommon.objectID;
                    if(obj_id > j3224_v2x_msgs::msg::ObjectID::MAX_OBJECT_ID){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM objectID above max, setting to max");
                        obj_id = j3224_v2x_msgs::msg::ObjectID::MAX_OBJECT_ID;
                    }
                    if(obj_id < j3224_v2x_msgs::msg::ObjectID::MIN_OBJECT_ID){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM objectID below min, setting to min");
                        obj_id = j3224_v2x_msgs::msg::ObjectID::MIN_OBJECT_ID;
                    }
                    common_data.detected_id.object_id = obj_id;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object missing objectID");
                }
                
                // MeasurementTimeOffset
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.measurementTime){
                    double measurement_time = sdsm_core.objects.list.array[obj_itr]->detObjCommon.measurementTime;
                    if(measurement_time > j3224_v2x_msgs::msg::MeasurementTimeOffset::MAX_MEASUREMENT_TIME_OFFSET){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM measurementTime above max, setting to max");
                        measurement_time = j3224_v2x_msgs::msg::MeasurementTimeOffset::MAX_MEASUREMENT_TIME_OFFSET;
                    }
                    if(measurement_time < j3224_v2x_msgs::msg::MeasurementTimeOffset::MIN_MEASUREMENT_TIME_OFFSET){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM measurementTime below min, setting to min");
                        measurement_time = j3224_v2x_msgs::msg::MeasurementTimeOffset::MIN_MEASUREMENT_TIME_OFFSET;
                    }
                    common_data.measurement_time.measurement_time_offset = measurement_time;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM missing measurementTime");
                }
                
                // TimeConfidence
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.timeConfidence){
                    common_data.time_confidence.confidence = sdsm_core.objects.list.array[obj_itr]->detObjCommon.timeConfidence;
                }
                else{
                    common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::UNAVAILABLE;
                }

                // PositionOffsetXYZ - offsetX
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.pos.offsetX){
                    long offset_x = sdsm_core.objects.list.array[obj_itr]->detObjCommon.pos.offsetX;
                    if(offset_x > j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object offsetX above max, setting to max");
                        offset_x = j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE;
                    }
                    if(offset_x < j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object offsetX below min, setting to min");
                        offset_x = j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE;
                    }
                    common_data.pos.offset_x.object_distance = offset_x;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Detected object missing offsetX");
                }
                // PositionOffsetXYZ - offsetY
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.pos.offsetY){
                    long offset_y = sdsm_core.objects.list.array[obj_itr]->detObjCommon.pos.offsetY;
                    if(offset_y > j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object offsetY above max, setting to max");
                        offset_y = j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE;
                    }
                    if(offset_y < j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object offsetY below min, setting to min");
                        offset_y = j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE;
                    }
                    common_data.pos.offset_y.object_distance = offset_y;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Detected object missing offsetY");
                }
                // PositionOffsetXYZ - *offsetZ
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.pos.offsetZ){
                    common_data.pos.presence_vector |= j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
                    long offset_z = *sdsm_core.objects.list.array[obj_itr]->detObjCommon.pos.offsetZ;
                    if(offset_z > j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object offsetZ above max, setting to max");
                        offset_z = j3224_v2x_msgs::msg::ObjectDistance::MAX_OBJECT_DISTANCE;
                    }
                    if(offset_z < j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object offsetZ below min, setting to min");
                        offset_z = j3224_v2x_msgs::msg::ObjectDistance::MIN_OBJECT_DISTANCE;
                    }
                    common_data.pos.offset_z.object_distance = offset_z;
                }

                // PositionConfidenceSet
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.posConfidence.pos && sdsm_core.objects.list.array[obj_itr]->detObjCommon.posConfidence.pos != j2735_v2x_msgs::msg::PositionConfidence::UNAVAILABLE){
                    common_data.pos_confidence.pos.confidence |= sdsm_core.objects.list.array[obj_itr]->detObjCommon.posConfidence.pos;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacks posConfidence, setting to unavailable");
                    common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::UNAVAILABLE;
                }
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.posConfidence.elevation && sdsm_core.objects.list.array[obj_itr]->detObjCommon.posConfidence.elevation != j2735_v2x_msgs::msg::ElevationConfidence::UNAVAILABLE){
                    common_data.pos_confidence.elevation.confidence |= sdsm_core.objects.list.array[obj_itr]->detObjCommon.posConfidence.elevation;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacks elevation confidence, setting to unavailable");
                    common_data.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::UNAVAILABLE;
                }

                // Speed
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.speed && sdsm_core.objects.list.array[obj_itr]->detObjCommon.speed != j2735_v2x_msgs::msg::Speed::UNAVAILABLE){
                    long speed = sdsm_core.objects.list.array[obj_itr]->detObjCommon.speed;
                    if(speed > j2735_v2x_msgs::msg::Speed::MAX){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object speed above max, setting to max");
                        speed = j2735_v2x_msgs::msg::Speed::MAX;
                    }
                    else if(speed < j2735_v2x_msgs::msg::Speed::MIN){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object speed below min, setting to min");
                        speed = j2735_v2x_msgs::msg::Speed::MIN;
                    }
                    common_data.speed.speed = sdsm_core.objects.list.array[obj_itr]->detObjCommon.speed;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacks speed, setting to unavailable");
                    common_data.speed.speed = j2735_v2x_msgs::msg::Speed::UNAVAILABLE;
                }
                // SpeedConfidence
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedConfidence && sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedConfidence != j2735_v2x_msgs::msg::SpeedConfidence::UNAVAILABLE){
                    common_data.speed_confidence.speed_confidence |= sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedConfidence;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacks speed confidence, setting to unavailable");
                    common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::UNAVAILABLE;
                }

                // *SpeedZ
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedZ){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
                    long speed_z = *sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedZ;
                    if(speed_z > j2735_v2x_msgs::msg::Speed::MAX){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object speedZ above max, setting to max");
                        speed_z = j2735_v2x_msgs::msg::Speed::MAX;
                    }
                    else if(speed_z < j2735_v2x_msgs::msg::Speed::MIN){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object speedZ below min, setting to min");
                        speed_z = j2735_v2x_msgs::msg::Speed::MIN;
                    }
                    common_data.speed_z.speed = speed_z;
                }
                // *SpeedConfidenceZ
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedConfidenceZ){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
                    common_data.speed_confidence_z.speed_confidence |= *sdsm_core.objects.list.array[obj_itr]->detObjCommon.speedConfidenceZ;
                }

                // Heading
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.heading && sdsm_core.objects.list.array[obj_itr]->detObjCommon.heading != j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE){
                    long heading = sdsm_core.objects.list.array[obj_itr]->detObjCommon.heading;
                    if(heading > j2735_v2x_msgs::msg::Heading::HEADING_MAX){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object heading above max, setting to max");
                        heading = j2735_v2x_msgs::msg::Heading::HEADING_MAX;
                    }
                    else if(heading < j2735_v2x_msgs::msg::Heading::HEADING_MIN){
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object heading below min, setting to min");
                        heading = j2735_v2x_msgs::msg::Heading::HEADING_MIN;
                    }
                    common_data.heading.heading = sdsm_core.objects.list.array[obj_itr]->detObjCommon.heading;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacks heading, setting to unavailable");
                    common_data.heading.heading |= j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE;
                }
                // HeadingConf
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.headingConf && sdsm_core.objects.list.array[obj_itr]->detObjCommon.headingConf != j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE){
                    common_data.heading_conf.confidence |= sdsm_core.objects.list.array[obj_itr]->detObjCommon.headingConf;
                }
                else{
                    RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacks heading confidence, setting to unavailable");
                    common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE;
                }

                // *Accel4Way
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
                    // accel4way - long
                    if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->Long){
                        double accel_long = sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->Long;
                        if(accel_long > j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way longitude above max, setting to max");
                            accel_long = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX;
                        }
                        else if(accel_long < j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way longitude below min, setting to min");
                            accel_long = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN;
                        }
                        common_data.accel_4_way.longitudinal = accel_long;
                    }
                    else{
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacking accel4way longitude, setting to unavailable");
                        common_data.accel_4_way.longitudinal = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE;
                    }
                    // accel4way - lat
                    if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->lat){
                        double accel_lat = sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->lat;
                        if(accel_lat > j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way latitude above max, setting to max");
                            accel_lat = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX;
                        }
                        else if(accel_lat < j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way longitude below min, setting to min");
                            accel_lat = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN;
                        }
                        common_data.accel_4_way.lateral = accel_lat;
                    }
                    else{
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacking accel4way latitude, setting to unavailable");
                        common_data.accel_4_way.lateral = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE;
                    }
                    // accel4way - vert
                    if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->vert){
                        double accel_vert = sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->vert;
                        if(accel_vert > j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way vert above max, setting to max");
                            accel_vert = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX;
                        }
                        else if(accel_vert < j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MIN){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way vert below min, setting to min");
                            accel_vert = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MIN;
                        }
                        common_data.accel_4_way.vert = accel_vert;
                    }
                    else{
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacking accel4way vert, setting to unavailable");
                        common_data.accel_4_way.vert = j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE;
                    }
                    // accel4way - yaw
                    if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->yaw){
                        double accel_yaw = sdsm_core.objects.list.array[obj_itr]->detObjCommon.accel4way->yaw;
                        if(accel_yaw > j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way yaw above max, setting to max");
                            accel_yaw = j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX;
                        }
                        else if(accel_yaw < j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN){
                            RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object accel4way yaw below min, setting to min");
                            accel_yaw = j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN;
                        }
                        common_data.accel_4_way.yaw_rate = accel_yaw;
                    }
                    else{
                        RCLCPP_WARN(node_logging_->get_logger(), "Decoded SDSM object lacking accel4way yaw, setting to unavailable");
                        common_data.accel_4_way.yaw_rate = j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_UNAVAILABLE;
                    }
                }

                // *AccelerationConfidence
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdX){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
                    common_data.acc_cfd_x.acceleration_confidence |= *sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdX;
                }

                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdY){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
                    common_data.acc_cfd_y.acceleration_confidence |= *sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdY;
                }

                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdZ){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
                    common_data.acc_cfd_z.acceleration_confidence |= *sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdZ;
                }
                if(sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdYaw){
                    common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
                    common_data.acc_cfd_yaw.yaw_rate_confidence |= *sdsm_core.objects.list.array[obj_itr]->detObjCommon.accCfdYaw;
                }
                // Assign common data to the current object
                object_data.detected_object_common_data = common_data;



                // Detected Object Optional Data

                if(sdsm_core.objects.list.array[obj_itr]->detObjOptData){
                    object_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;

                    // Simplify message access scheme where "opt_data" is the inbound ASN.1 data
                    DetectedObjectOptionalData_t opt_data = *message->value.choice.SensorDataSharingMessage.objects.list.array[obj_itr]->detObjOptData;

                    j3224_v2x_msgs::msg::DetectedObjectOptionalData opt_output;

                    // detVeh
                    if(opt_data.present == DetectedObjectOptionalData_PR_detVeh){
                        opt_output.choice = j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH;

                        // Exteriorlights
                        if(opt_data.choice.detVeh.lights){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS;

                            // NOTE: previous decoders use the same/similar parsing methods for ASN.1 octet strings but
                            //       this may not be the best solution, check for alternatives
                            uint8_t lights=0;
                            for(int i = opt_data.choice.detVeh.lights->size -1 ;i >= 0 ;i--){
                                lights |= (opt_data.choice.detVeh.lights->buf[i] << i);
                                
                            }

                            opt_output.det_veh.lights.exterior_lights = lights;

                        }

                        // Attitude
                        if(opt_data.choice.detVeh.vehAttitude){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE;
                            // Attitude - pitch
                            if(opt_data.choice.detVeh.vehAttitude->pitch){
                                double veh_pitch = opt_data.choice.detVeh.vehAttitude->pitch;
                                if(veh_pitch > j3224_v2x_msgs::msg::PitchDetected::MAX_PITCH_DETECTED){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object attitude pitch above max, setting to max");
                                    opt_output.det_veh.veh_attitude.pitch.pitch_detected = j3224_v2x_msgs::msg::PitchDetected::MAX_PITCH_DETECTED;
                                }
                                else if(veh_pitch < j3224_v2x_msgs::msg::PitchDetected::MIN_PITCH_DETECTED){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object attitude pitch below min, setting to min");
                                    opt_output.det_veh.veh_attitude.pitch.pitch_detected = j3224_v2x_msgs::msg::PitchDetected::MIN_PITCH_DETECTED;
                                }
                                opt_output.det_veh.veh_attitude.pitch.pitch_detected = veh_pitch;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing attitude pitch");
                            }
                            // Attitude - roll
                            if(opt_data.choice.detVeh.vehAttitude->roll){
                                double veh_roll = opt_data.choice.detVeh.vehAttitude->roll;
                                if(veh_roll > j3224_v2x_msgs::msg::RollDetected::MAX_ROLL_DETECTED){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object attitude roll above max, setting to max");
                                    opt_output.det_veh.veh_attitude.roll.roll_detected = j3224_v2x_msgs::msg::RollDetected::MAX_ROLL_DETECTED;
                                }
                                else if(veh_roll < j3224_v2x_msgs::msg::RollDetected::MIN_ROLL_DETECTED){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object attitude roll below min, setting to min");
                                    opt_output.det_veh.veh_attitude.roll.roll_detected = j3224_v2x_msgs::msg::RollDetected::MIN_ROLL_DETECTED;
                                }
                                opt_output.det_veh.veh_attitude.roll.roll_detected = veh_roll;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing attitude roll");
                            }
                            // Attitude - yaw
                            if(opt_data.choice.detVeh.vehAttitude->yaw){
                                double veh_yaw = opt_data.choice.detVeh.vehAttitude->yaw;
                                if(veh_yaw > j3224_v2x_msgs::msg::YawDetected::MAX_YAW_DETECTED){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object attitude yaw above max, setting to max");
                                    opt_output.det_veh.veh_attitude.yaw.yaw_detected = j3224_v2x_msgs::msg::YawDetected::MAX_YAW_DETECTED;
                                }
                                else if(veh_yaw < j3224_v2x_msgs::msg::YawDetected::MIN_YAW_DETECTED){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object attitude yaw below min, setting to min");
                                    opt_output.det_veh.veh_attitude.yaw.yaw_detected = j3224_v2x_msgs::msg::YawDetected::MIN_YAW_DETECTED;
                                }
                                opt_output.det_veh.veh_attitude.yaw.yaw_detected = veh_yaw;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing attitude yaw");
                            }
                        }

                        // AttitudeConfidence
                        if(opt_data.choice.detVeh.vehAttitudeConfidence){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE;

                            if(opt_data.choice.detVeh.vehAttitudeConfidence->pitchConfidence){
                                opt_output.det_veh.veh_attitude_confidence.pitch_confidence.confidence |= opt_data.choice.detVeh.vehAttitudeConfidence->pitchConfidence;
                            }
                            else{
                                opt_output.det_veh.veh_attitude_confidence.pitch_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE;
                            }
                            if(opt_data.choice.detVeh.vehAttitudeConfidence->rollConfidence){
                                opt_output.det_veh.veh_attitude_confidence.roll_confidence.confidence |= opt_data.choice.detVeh.vehAttitudeConfidence->rollConfidence;
                            }
                            else{
                                opt_output.det_veh.veh_attitude_confidence.roll_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE;
                            }
                            if(opt_data.choice.detVeh.vehAttitudeConfidence->yawConfidence){
                                opt_output.det_veh.veh_attitude_confidence.yaw_confidence.confidence |= opt_data.choice.detVeh.vehAttitudeConfidence->yawConfidence;
                            }
                            else{
                                opt_output.det_veh.veh_attitude_confidence.yaw_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::UNAVAILABLE;
                            }
                        }

                        // AngularVelocity
                        if(opt_data.choice.detVeh.vehAngVel){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL;
                            // AngularVelocity - pitch
                            if(opt_data.choice.detVeh.vehAngVel->pitchRate && opt_data.choice.detVeh.vehAngVel->pitchRate != j3224_v2x_msgs::msg::PitchRate::UNAVAILABLE){
                                double ang_vel_pitch = opt_data.choice.detVeh.vehAngVel->pitchRate;

                                if(ang_vel_pitch > j3224_v2x_msgs::msg::PitchRate::MAX_PITCH_RATE){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object ang vel pitch above max, setting to max");
                                    ang_vel_pitch = j3224_v2x_msgs::msg::PitchRate::MAX_PITCH_RATE;
                                }
                                else if(ang_vel_pitch < j3224_v2x_msgs::msg::PitchRate::MIN_PITCH_RATE){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object ang vel pitch below min, setting to min");
                                    ang_vel_pitch = j3224_v2x_msgs::msg::PitchRate::MIN_PITCH_RATE;                                    
                                }
                                opt_output.det_veh.veh_ang_vel.pitch_rate.pitch_rate = ang_vel_pitch;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing ang vel pitch");
                                opt_output.det_veh.veh_ang_vel.pitch_rate.pitch_rate = j3224_v2x_msgs::msg::PitchRate::UNAVAILABLE;
                            }
                            // AngularVelocity - roll
                            if(opt_data.choice.detVeh.vehAngVel->rollRate && opt_data.choice.detVeh.vehAngVel->rollRate != j3224_v2x_msgs::msg::RollRate::UNAVAILABLE){
                                double ang_vel_roll = opt_data.choice.detVeh.vehAngVel->rollRate;

                                if(ang_vel_roll > j3224_v2x_msgs::msg::RollRate::MAX_ROLL_RATE){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object ang vel roll above max, setting to max");
                                    ang_vel_roll = j3224_v2x_msgs::msg::RollRate::MAX_ROLL_RATE;
                                }
                                else if(ang_vel_roll < j3224_v2x_msgs::msg::RollRate::MIN_ROLL_RATE){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object ang vel roll below min, setting to min");
                                    ang_vel_roll = j3224_v2x_msgs::msg::RollRate::MIN_ROLL_RATE;                                    
                                }
                                opt_output.det_veh.veh_ang_vel.roll_rate.roll_rate = ang_vel_roll;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing ang vel roll");
                                opt_output.det_veh.veh_ang_vel.roll_rate.roll_rate = j3224_v2x_msgs::msg::RollRate::UNAVAILABLE;
                            }
                        }

                        // AngularVelocityConfidence
                        if(opt_data.choice.detVeh.vehAngVelConfidence){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE;

                            if(*opt_data.choice.detVeh.vehAngVelConfidence->pitchRateConfidence){
                                opt_output.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_PITCH_RATE_CONFIDENCE;
                                opt_output.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence = *opt_data.choice.detVeh.vehAngVelConfidence->pitchRateConfidence;
                            }
                            if(*opt_data.choice.detVeh.vehAngVelConfidence->rollRateConfidence){
                                opt_output.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_ROLL_RATE_CONFIDENCE;
                                opt_output.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence = *opt_data.choice.detVeh.vehAngVelConfidence->rollRateConfidence;
                            }
                        }

                        // VehicleSize
                        if(opt_data.choice.detVeh.size){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
                            // VehicleSize - width
                            if(opt_data.choice.detVeh.size->width && opt_data.choice.detVeh.size->width != j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_UNAVAILABLE){
                                long veh_width = opt_data.choice.detVeh.size->width;

                                if(veh_width > j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object veh width above max, setting to max");
                                    veh_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX;
                                }
                                else if(veh_width < j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MIN){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object veh width below min, setting to min");
                                    veh_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MIN;
                                }
                                opt_output.det_veh.size.vehicle_width = veh_width;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing veh width");
                                opt_output.det_veh.size.vehicle_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_UNAVAILABLE;
                            }
                            // VehicleSize - length
                            if(opt_data.choice.detVeh.size->length && opt_data.choice.detVeh.size->length != j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_UNAVAILABLE){
                                long veh_length = opt_data.choice.detVeh.size->length;

                                if(veh_length > j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object veh length above max, setting to max");
                                    veh_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX;
                                }
                                else if(veh_length < j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MIN){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object veh length below min, setting to min");
                                    veh_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MIN;
                                }
                                opt_output.det_veh.size.vehicle_length = veh_length;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing veh length");
                                opt_output.det_veh.size.vehicle_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_UNAVAILABLE;
                            }
                        }

                        // VehicleHeight *
                        if(*opt_data.choice.detVeh.height){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;
                            // unnecessary nested ifs?
                            if(opt_data.choice.detVeh.height && *opt_data.choice.detVeh.height != j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE){
                                long veh_height = *opt_data.choice.detVeh.height;

                                if(veh_height > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object veh height above max, setting to max");
                                    veh_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                                }
                                else if(veh_height < j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN){
                                    RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object veh height below min, setting to min");
                                    veh_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN;
                                }
                                opt_output.det_veh.height.vehicle_height = veh_height;
                            }
                            else{
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object missing veh height");
                                opt_output.det_veh.height.vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE;
                            }
                        }

                        // VehicleSizeConfidence
                        if(opt_data.choice.detVeh.vehicleSizeConfidence){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE;

                            if(opt_data.choice.detVeh.vehicleSizeConfidence->vehicleWidthConfidence){
                                opt_output.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence |= opt_data.choice.detVeh.vehicleSizeConfidence->vehicleWidthConfidence;
                            }
                            else{
                                opt_output.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::UNAVAILABLE;
                            }
                            if(opt_data.choice.detVeh.vehicleSizeConfidence->vehicleLengthConfidence){
                                opt_output.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence |= opt_data.choice.detVeh.vehicleSizeConfidence->vehicleLengthConfidence;
                            }
                            else{
                                opt_output.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::UNAVAILABLE;
                            }
                            if(*opt_data.choice.detVeh.vehicleSizeConfidence->vehicleHeightConfidence){
                                opt_output.det_veh.vehicle_size_confidence.presence_vector |= j3224_v2x_msgs::msg::VehicleSizeConfidence::HAS_VEHICLE_HEIGHT_CONFIDENCE;
                                opt_output.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence |= *opt_data.choice.detVeh.vehicleSizeConfidence->vehicleHeightConfidence;
                            }
                        }

                        // BasicVehicleClass
                        if(*opt_data.choice.detVeh.vehicleClass){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS;
                            opt_output.det_veh.vehicle_class.basic_vehicle_class = *opt_data.choice.detVeh.vehicleClass;
                        }

                        // ClassificationConfidence
                        if(*opt_data.choice.detVeh.classConf){
                            opt_output.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF;

                            long veh_class_conf = *opt_data.choice.detVeh.classConf;
                            if(veh_class_conf > j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object class conf above max, setting to max");
                                veh_class_conf = j3224_v2x_msgs::msg::ClassificationConfidence::MAX_CLASSIFICATION_CONFIDENCE;
                            }
                            else if(veh_class_conf < j3224_v2x_msgs::msg::ClassificationConfidence::MIN_CLASSIFICATION_CONFIDENCE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object class conf below min, setting to min");
                                veh_class_conf = j3224_v2x_msgs::msg::ClassificationConfidence::MIN_CLASSIFICATION_CONFIDENCE;
                            }
                            opt_output.det_veh.class_conf.classification_confidence = veh_class_conf;
                        }
                    }



                    // detVRU
                    else if(opt_data.present == DetectedObjectOptionalData_PR_detVRU){
                        opt_output.choice = j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU;

                        // PersonalDeviceUserType
                        if(*opt_data.choice.detVRU.basicType){
                            opt_output.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE;
                            opt_output.det_vru.basic_type.type = *opt_data.choice.detVRU.basicType;
                        }

                        // PropelledInformation
                        if(opt_data.choice.detVRU.propulsion){
                            opt_output.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION;

                            if(opt_data.choice.detVRU.propulsion->present == PropelledInformation_PR_human){
                                opt_output.det_vru.propulsion.human.type = opt_data.choice.detVRU.propulsion->choice.human;
                            }
                            else if(opt_data.choice.detVRU.propulsion->present == PropelledInformation_PR_animal){
                                opt_output.det_vru.propulsion.animal.type = opt_data.choice.detVRU.propulsion->choice.animal;
                            }
                            else if(opt_data.choice.detVRU.propulsion->present == PropelledInformation_PR_motor){
                                opt_output.det_vru.propulsion.motor.type = opt_data.choice.detVRU.propulsion->choice.motor;
                            }
                        }

                        // Attachment
                        if(*opt_data.choice.detVRU.attachment){
                            opt_output.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT;
                            opt_output.det_vru.attachment.type = *opt_data.choice.detVRU.attachment;
                        }

                        // AttachmentRadius
                        if(*opt_data.choice.detVRU.radius){
                            opt_output.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS;

                            long att_radius = *opt_data.choice.detVRU.radius;
                            if(att_radius > j2735_v2x_msgs::msg::AttachmentRadius::ATTACHMENT_RADIUS_MAX){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object radius above max, setting to max");
                                att_radius = j2735_v2x_msgs::msg::AttachmentRadius::ATTACHMENT_RADIUS_MAX;
                            }
                            opt_output.det_vru.radius.attachment_radius = att_radius;
                        }
                    }


                    // detObst
                    else if(opt_data.present == DetectedObjectOptionalData_PR_detObst){
                        opt_output.choice = j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST;
                        
                        // ObstacleSize - width
                        if(opt_data.choice.detObst.obstSize.width){
                            long obst_width = opt_data.choice.detObst.obstSize.width;

                            if(obst_width > j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object obst width above max, setting to max");
                                obst_width = j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE;
                            }
                            else if(obst_width < j3224_v2x_msgs::msg::SizeValue::MIN_SIZE_VALUE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object obst width below min, setting to min");
                                obst_width = j3224_v2x_msgs::msg::SizeValue::MIN_SIZE_VALUE;
                            }
                            opt_output.det_obst.obst_size.width.size_value = obst_width;
                        }
                        // ObstacleSize - length
                        if(opt_data.choice.detObst.obstSize.length){
                            long obst_length = opt_data.choice.detObst.obstSize.length;

                            if(obst_length > j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object obst length above max, setting to max");
                                obst_length = j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE;
                            }
                            else if(obst_length < j3224_v2x_msgs::msg::SizeValue::MIN_SIZE_VALUE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object obst length below min, setting to min");
                                obst_length = j3224_v2x_msgs::msg::SizeValue::MIN_SIZE_VALUE;
                            }
                            opt_output.det_obst.obst_size.length.size_value = obst_length;
                        }
                        // ObstacleSize - height
                        if(*opt_data.choice.detObst.obstSize.height){
                            opt_output.det_obst.obst_size.presence_vector |= j3224_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT;
                            long obst_height = *opt_data.choice.detObst.obstSize.height;

                            if(obst_height > j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object obst height above max, setting to max");
                                obst_height = j3224_v2x_msgs::msg::SizeValue::MAX_SIZE_VALUE;
                            }
                            else if(obst_height < j3224_v2x_msgs::msg::SizeValue::MIN_SIZE_VALUE){
                                RCLCPP_WARN(node_logging_->get_logger(), "OPTIONAL: Decoded SDSM object obst height below min, setting to min");
                                obst_height = j3224_v2x_msgs::msg::SizeValue::MIN_SIZE_VALUE;
                            }
                            opt_output.det_obst.obst_size.height.size_value = obst_height;
                        }

                        // ObstacleSizeConfidence
                        if(opt_data.choice.detObst.obstSizeConfidence.widthConfidence){
                            opt_output.det_obst.obst_size_confidence.width_confidence.size_value_confidence |= opt_data.choice.detObst.obstSizeConfidence.widthConfidence;
                        }
                        else{
                            opt_output.det_obst.obst_size_confidence.width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::UNAVAILABLE;
                        }
                        if(opt_data.choice.detObst.obstSizeConfidence.lengthConfidence){
                            opt_output.det_obst.obst_size_confidence.length_confidence.size_value_confidence |= opt_data.choice.detObst.obstSizeConfidence.lengthConfidence;
                        }
                        else{
                            opt_output.det_obst.obst_size_confidence.length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::UNAVAILABLE;
                        }
                        if(*opt_data.choice.detObst.obstSizeConfidence.heightConfidence){
                            opt_output.det_obst.obst_size_confidence.presence_vector |= j3224_v2x_msgs::msg::ObstacleSizeConfidence::HAS_HEIGHT_CONFIDENCE;
                            opt_output.det_obst.obst_size_confidence.height_confidence.size_value_confidence = *opt_data.choice.detObst.obstSizeConfidence.heightConfidence;
                        }
                    }
                    // Assign optional data if it exists
                    object_data.detected_object_optional_data = opt_output;

                }
                // For each object iterated over, push back the data to DetectedObjectsList
                detected_objects.detected_object_data.push_back(object_data);

            }
            // Set the objects field of the output message
            output.objects = detected_objects;

            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            // Release memory and return decoded message output
            return std::optional<j3224_v2x_msgs::msg::SensorDataSharingMessage>(output);

        }

        RCLCPP_WARN(node_logging_->get_logger(),"SensorDataSharingMessage decoding failed, setting as null");

        ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
        // If decoding fails return an empty SDSM
        return std::optional<j3224_v2x_msgs::msg::SensorDataSharingMessage>{};

    

    }
}