/*
 * Copyright (C) 2022 LEIDOS.
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
 * CPP File containing PSM Message method implementations
 */

#include "cpp_message/PSM_Message.h"

#include <carma_v2x_msgs/msg/psm.hpp>
#include <j2735_v2x_msgs/msg/personal_device_user_type.hpp>
#include <j2735_v2x_msgs/msg/d_second.hpp>
#include <j2735_v2x_msgs/msg/temporary_id.hpp>

namespace cpp_message
{
    boost::optional<carma_v2x_msgs::msg::PSM> PSM_Message::decode_psm_message(std::vector<uint8_t>& binary_array){
        
        carma_v2x_msgs::msg::PSM output;
        asn_dec_rval_t rval;
        MessageFrame_t* message = nullptr;

        //copy from vector to array
        size_t len=binary_array.size();
        uint8_t buf[len];
        std::copy(binary_array.begin(), binary_array.end(), buf);
        //use asn1c lib to decode

        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

        if(rval.code==RC_OK)
        {
            
            //device type has same enum in j2735_v2x_msgs
            output.basic_type.type = message->value.choice.PersonalSafetyMessage.basicType;
            
            output.sec_mark.millisecond = message->value.choice.PersonalSafetyMessage.secMark;

            if(message->value.choice.PersonalSafetyMessage.msgCnt < j2735_v2x_msgs::msg::MsgCount::MSG_COUNT_MAX){
                output.msg_cnt.msg_cnt = message->value.choice.PersonalSafetyMessage.msgCnt; 
            }
            else{
                RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Message count greater than max, defaulting to max value");
                output.msg_cnt.msg_cnt = j2735_v2x_msgs::msg::MsgCount::MSG_COUNT_MAX;
            }
            

            //Temporary ID asn.1 definition: 4 octet random device identifier OCTET STRING (SIZE(4))
            for(int i = 0; i < 4; ++i){

                output.id.id.push_back(message->value.choice.PersonalSafetyMessage.id.buf[i]);   
            }
            
            carma_v2x_msgs::msg::Position3D position;
            
            if(!message->value.choice.PersonalSafetyMessage.position.lat){
                RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "latitude Unavailable");
                position.latitude = carma_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE;
            }
            else{
                float latitude = message->value.choice.PersonalSafetyMessage.position.lat * latitude_conversion_const_;
                if (latitude >= carma_v2x_msgs::msg::Position3D::LATITUDE_MAX){
                    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "Latitude greater than max, defaulting to max latitude");
                    latitude = carma_v2x_msgs::msg::Position3D::LATITUDE_MAX;
                }
                else if(latitude < carma_v2x_msgs::msg::Position3D::LATITUDE_MIN){
                    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "Latitude less than min, defaulting to min latitude");
                    latitude = carma_v2x_msgs::msg::Position3D::LATITUDE_MIN;
                }
                position.latitude = latitude;
            }

            if(!message->value.choice.PersonalSafetyMessage.position.Long){
                RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "longitude Unavailable");
                position.longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE;
            }
            else{
                float longitude = message->value.choice.PersonalSafetyMessage.position.Long * longitude_conversion_const_;
                if(longitude > carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX){
                    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "Longitude greater than max, defaulting to max longitude");
                    longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX;
                }
                else if(longitude <carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN){
                    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "Longitude less than min, defaulting to min longitude");
                    longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN;
                }
                position.longitude = longitude;
            }

            if(message->value.choice.PersonalSafetyMessage.position.elevation = nullptr){
                position.elevation_exists = false;
                RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "elevation Unavailable");
                position.elevation = carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE;
            }
            else{
                position.elevation_exists = true;
                float elevation = *message->value.choice.PersonalSafetyMessage.position.elevation * elevation_conversion_const_;
                if(elevation > carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX){
                    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "Elevation is greater than max, using max elevation");
                    elevation = carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX;
                }
                else if (elevation < carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN){
                    RCLCPP_DEBUG_STREAM(node_logging_->get_logger(), "Elevation in less than min, using min elevation");
                    elevation = carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN;
                }
                position.elevation = elevation;
            }

            output.position = position;


            carma_v2x_msgs::msg::PositionalAccuracy accuracy;
            
            if(message->value.choice.PersonalSafetyMessage.accuracy.semiMajor && message->value.choice.PersonalSafetyMessage.accuracy.semiMinor)
            {
                
                float semi_major = message->value.choice.PersonalSafetyMessage.accuracy.semiMajor * 0.05;
                if( semi_major > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX)
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Accuracy semi major is greater than max, defined as unavailable - Value not assigned");
                }
                else{
                    if(semi_major < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN)
                    {
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Accuracy semi major is less than min, defaulting to min");
                        semi_major = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                    }
                    
                    accuracy.semi_major = semi_major;

                    accuracy.presence_vector |= 1 << (carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE -1);
                }
                
                float semi_minor = message->value.choice.PersonalSafetyMessage.accuracy.semiMinor * 0.05;
                if(semi_minor >= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX)
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Accuracy semi minor is greater than max, defined as unavailable - Value not assigned");
                }
                else{
                    if(semi_minor < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN)
                    {
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Accuracy semi minor is less than min, defaulting to min");
                        semi_minor = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                    }
                    
                    accuracy.semi_minor = semi_minor;

                    accuracy.presence_vector |= 1 << (carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE -1);
                }
                

            }
            else if (message->value.choice.PersonalSafetyMessage.accuracy.orientation){
                
                float orientation = message->value.choice.PersonalSafetyMessage.accuracy.orientation * 0.0054932479;
                if( orientation > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX)
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Accuracy orientation is greater than max, defined as unavailable - Value not assigned");
                }
                else{

                    if(orientation < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN)
                    {
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Accuracy orientation is less than equal to min, defaulting to min");
                        orientation = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN;
                    }
                    
                    accuracy.orientation = message->value.choice.PersonalSafetyMessage.accuracy.orientation;
                    
                    accuracy.presence_vector |= 1 << (carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE -1);
                }
            }

            output.accuracy = accuracy;


            // Velocity ASN.1 Representation:
            // Velocity ::= INTEGER (0..8191) -- Units of 0.02 m/s
            // -- The value 8191 indicates that
            // -- velocity is unavailable
            if(message->value.choice.PersonalSafetyMessage.speed == 8191){
                output.speed.unavailable = true; 
                RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Speed is unavailable, setting to 0.0");
                output.speed.velocity = 0.0;
            }
            else{
                output.speed.unavailable = false;

                float speed = message->value.choice.PersonalSafetyMessage.speed * velocity_conversion_const_;
                if(speed > carma_v2x_msgs::msg::Velocity::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Speed is greater than max, defaulting to max");
                    speed = carma_v2x_msgs::msg::Velocity::MAX;
                }
                else if(speed < carma_v2x_msgs::msg::Velocity::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Speed is less than min, defaulting to min");
                    speed = carma_v2x_msgs::msg::Velocity::MIN;
                }
                
                output.speed.velocity = speed;
            }

            // Heading ASN.1 Representation:
            // Heading ::= INTEGER (0..28800)
            // -- LSB of 0.0125 degrees
            // -- A range of 0 to 359.9875 degrees
            if(message->value.choice.PersonalSafetyMessage.heading == 28800){
                output.heading.unavailable = true;
                RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Heading is unavailable, It is not being set to a default");
            }
            else{
                output.heading.unavailable = false;

                float heading = message->value.choice.PersonalSafetyMessage.heading * heading_conversion_const_;
                if(heading > carma_v2x_msgs::msg::Heading::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Heading is greater than max, defaulting to max");
                    heading = carma_v2x_msgs::msg::Heading::MAX;
                }
                else if(heading < carma_v2x_msgs::msg::Heading::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Heading is less than min, defaulting to min");
                    heading = carma_v2x_msgs::msg::Heading::MIN;
                }
                output.heading.heading = heading;
            }

            //presence_vector
            //A BIT STRING defining the presence of optional fields.
            if(message->value.choice.PersonalSafetyMessage.accelSet){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ACCEL_SET;
                AccelerationSet4Way_t binary_accel_set = *message->value.choice.PersonalSafetyMessage.accelSet;
                output.accel_set = decode_accel_set_message(binary_accel_set);
            }
            
            if(message->value.choice.PersonalSafetyMessage.pathHistory){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY;
                PathHistory_t binary_path_history = *message->value.choice.PersonalSafetyMessage.pathHistory;
                output.path_history = decode_path_history_message(binary_path_history);
            }
            
            if(message->value.choice.PersonalSafetyMessage.pathPrediction){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION;
                PathPrediction_t binary_path_prediction = *message->value.choice.PersonalSafetyMessage.pathPrediction;
                output.path_prediction = decode_path_prediction_message(binary_path_prediction);
            }
            
            if(message->value.choice.PersonalSafetyMessage.propulsion){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PROPULSION;
                PropelledInformation_t binary_propelled_information = *message->value.choice.PersonalSafetyMessage.propulsion;
                output.propulsion = decode_propulsion_message(binary_propelled_information);
            }

            if(message->value.choice.PersonalSafetyMessage.useState){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_USE_STATE;
                PersonalDeviceUsageState_t binary_usage_state = *message->value.choice.PersonalSafetyMessage.useState;
                output.use_state = decode_use_state(binary_usage_state);
            }
            
            if(message->value.choice.PersonalSafetyMessage.crossRequest){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST;
                output.cross_request.cross_request = *message->value.choice.PersonalSafetyMessage.crossRequest;
            }
            
            if(message->value.choice.PersonalSafetyMessage.crossState){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CROSS_STATE;
                output.cross_state.cross_state = *message->value.choice.PersonalSafetyMessage.crossState;
            }
            
            if(message->value.choice.PersonalSafetyMessage.clusterSize){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE;
                output.cluster_size.cluster_size = *message->value.choice.PersonalSafetyMessage.clusterSize;
            }

            if(*message->value.choice.PersonalSafetyMessage.clusterRadius){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS;
                int cluster_radius = *message->value.choice.PersonalSafetyMessage.clusterRadius;
                if(cluster_radius > j2735_v2x_msgs::msg::PersonalClusterRadius::CLUSTER_RADIUS_MAX){

                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Cluster Radius is greater than max, set to max");
                    cluster_radius = j2735_v2x_msgs::msg::PersonalClusterRadius::CLUSTER_RADIUS_MAX;
                }
                output.cluster_radius.cluster_radius = cluster_radius;
            }
            
            if(message->value.choice.PersonalSafetyMessage.eventResponderType){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE;
                output.event_responder_type.type = *message->value.choice.PersonalSafetyMessage.eventResponderType;
            }
            
            // PublicSafetyandRoadWorkerActivity 
            if(!message->value.choice.PersonalSafetyMessage.activityType){
                output.activity_type.activities = j2735_v2x_msgs::msg::PublicSafetyAndRoadWorkerActivity::UNAVAILABLE;
            }
            else{
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE;
                output.activity_type.activities |= message->value.choice.PersonalSafetyMessage.activityType->buf[0];

                for(int i = 1; i< message->value.choice.PersonalSafetyMessage.activityType->size;i++){
                    output.activity_type.activities |= (message->value.choice.PersonalSafetyMessage.activityType->buf[i] << i);
                }
                
            }

            if(!message->value.choice.PersonalSafetyMessage.activitySubType){
                output.activity_sub_type.sub_types = j2735_v2x_msgs::msg::PublicSafetyDirectingTrafficSubType::UNAVAILABLE;
            }
            else{
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE;
                output.activity_sub_type.sub_types |= message->value.choice.PersonalSafetyMessage.activitySubType->buf[0];
                for(int i = 1; i< message->value.choice.PersonalSafetyMessage.activitySubType->size;i++){
                    output.activity_sub_type.sub_types |= (message->value.choice.PersonalSafetyMessage.activitySubType->buf[i] << i);
                }
            }

            if(!message->value.choice.PersonalSafetyMessage.assistType){
                output.assist_type.types = j2735_v2x_msgs::msg::PersonalAssistive::UNAVAILABLE;
            }
            else{
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE;
                output.assist_type.types |= message->value.choice.PersonalSafetyMessage.assistType->buf[0];
                for(int i = 1; i < message->value.choice.PersonalSafetyMessage.assistType->size;i++){
                    output.assist_type.types |= (message->value.choice.PersonalSafetyMessage.assistType->buf[i] << i);
                }
            }

            if(!message->value.choice.PersonalSafetyMessage.sizing){
                output.sizing.sizes_and_behaviors = j2735_v2x_msgs::msg::UserSizeAndBehaviour::UNAVAILABLE;
            }
            else{
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_SIZING;
                output.sizing.sizes_and_behaviors |= message->value.choice.PersonalSafetyMessage.sizing->buf[0];
                for(int i = 1; i < message->value.choice.PersonalSafetyMessage.sizing->size;i++){
                    output.sizing.sizes_and_behaviors |= (message->value.choice.PersonalSafetyMessage.sizing->buf[i] << i);
                }
            }

            if(message->value.choice.PersonalSafetyMessage.attachment){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT;
                output.attachment.type = *message->value.choice.PersonalSafetyMessage.attachment;
            }
            
            if(message->value.choice.PersonalSafetyMessage.attachmentRadius){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS;
                output.attachment_radius.attachment_radius = *message->value.choice.PersonalSafetyMessage.attachmentRadius;
            }
            
            if(message->value.choice.PersonalSafetyMessage.animalType){
                output.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE;
                output.animal_type.type = *message->value.choice.PersonalSafetyMessage.animalType;
            }
            

            return boost::optional<carma_v2x_msgs::msg::PSM>(output);
        }

        return boost::optional<carma_v2x_msgs::msg::PSM>{};
    }

    j2735_v2x_msgs::msg::PersonalDeviceUsageState PSM_Message::decode_use_state(PersonalDeviceUsageState_t& message){
        j2735_v2x_msgs::msg::PersonalDeviceUsageState output;
        
        for(int i = message.size -1;i >= 0; i--){
            output.states |= (message.buf[i] << i);
        }

        return output;
    }

    j2735_v2x_msgs::msg::PropelledInformation PSM_Message::decode_propulsion_message(PropelledInformation_t& message){
        
        j2735_v2x_msgs::msg::PropelledInformation output;

        // output.choice = message.choice;
        if(message.choice.human && !message.choice.human==0){
            output.choice = j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN;
            output.human.type = message.choice.human;
        }
        else if(message.choice.animal && !message.choice.animal==0){
            output.choice = j2735_v2x_msgs::msg::PropelledInformation::CHOICE_ANIMAL;
            output.animal.type  = message.choice.animal;
        }
        else if(message.choice.motor && !message.choice.motor ==0){
            output.choice = j2735_v2x_msgs::msg::PropelledInformation::CHOICE_MOTOR;
            output.motor.type = message.choice.motor;
        }

        return output;
    }

    carma_v2x_msgs::msg::PathPrediction PSM_Message::decode_path_prediction_message(PathPrediction_t& message){
        carma_v2x_msgs::msg::PathPrediction output;

        output.radius_of_curvature = message.radiusOfCurve * 0.01;
        output.confidence = message.confidence * 0.5;

        return output;
    }

    carma_v2x_msgs::msg::PathHistory PSM_Message::decode_path_history_message(PathHistory_t& message){
        
        carma_v2x_msgs::msg::PathHistory output;
        //crumbdata - pathhistorypoints list

        carma_v2x_msgs::msg::PathHistoryPointList crumb_data;

        if(message.crumbData.list.size == 0){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory points list is empty, adding empty point");
            carma_v2x_msgs::msg::PathHistoryPoint point;
            crumb_data.points.push_back(point);
        }

        for(size_t i = 0; i < message.crumbData.list.size; ++i){       //MIN_SIZE = 1, MAX_SIZE = 23
            if(i > carma_v2x_msgs::msg::PathHistoryPointList::MAX_SIZE){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory points size greater than max rejecting points");
                break;
            }

            carma_v2x_msgs::msg::PathHistoryPoint point;

            //latitude
            if( message.crumbData.list.array[i]->latOffset){
                float latOffset = message.crumbData.list.array[i]->latOffset * 1e-7;

                if(latOffset < carma_v2x_msgs::msg::OffsetLLB18::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history latitude offset "<<i<<" less than min, setting to min");
                    latOffset = carma_v2x_msgs::msg::OffsetLLB18::MIN;
                }
                else if(latOffset > carma_v2x_msgs::msg::OffsetLLB18::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history latitude offset "<<i<<" greater than max, setting to max");
                    latOffset = carma_v2x_msgs::msg::OffsetLLB18::MAX;
                }
                point.lat_offset.unavailable = false;
                point.lat_offset.offset = latOffset;
            }
            else{
                point.lat_offset.unavailable = true;
            }
            
            //longitude
            if( message.crumbData.list.array[i]->lonOffset){
                float lonOffset = message.crumbData.list.array[i]->lonOffset * 1e-7;

                if(lonOffset < carma_v2x_msgs::msg::OffsetLLB18::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history longitude offset "<<i<<" less than min, setting to min");
                    lonOffset = carma_v2x_msgs::msg::OffsetLLB18::MIN;
                }
                else if(lonOffset > carma_v2x_msgs::msg::OffsetLLB18::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history longitude offset "<<i<<" greater than max, setting to max");
                    lonOffset = carma_v2x_msgs::msg::OffsetLLB18::MAX;
                }
                point.lon_offset.unavailable = false;
                point.lon_offset.offset = lonOffset;
            }
            else{
                point.lon_offset.unavailable = true;
            }

            //elevation
            if( message.crumbData.list.array[i]->elevationOffset){
                float elevationOffset = message.crumbData.list.array[i]->elevationOffset * 0.1;

                if(elevationOffset < carma_v2x_msgs::msg::VertOffsetB12::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history elevation offset "<<i<<" less than min, setting to min");
                    elevationOffset = carma_v2x_msgs::msg::VertOffsetB12::MIN;
                }
                else if(elevationOffset > carma_v2x_msgs::msg::VertOffsetB12::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history elevation offset "<<i<<" greater than max, setting to max");
                    elevationOffset = carma_v2x_msgs::msg::VertOffsetB12::MAX;
                }

                point.elevation_offset.unavailable = false;
                point.elevation_offset.offset = elevationOffset;
            }
            else{
                point.elevation_offset.unavailable = true;
            }

            //time_offset
            if( message.crumbData.list.array[i]->timeOffset){
                float timeOffset = message.crumbData.list.array[i]->timeOffset * 0.01;

                if(timeOffset < carma_v2x_msgs::msg::TimeOffset::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history time offset "<<i<<" less than min, setting to min");
                    timeOffset = carma_v2x_msgs::msg::TimeOffset::MIN;
                }
                else if(timeOffset > carma_v2x_msgs::msg::TimeOffset::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history time offset "<<i<<" greater than max, setting to max");
                    timeOffset = carma_v2x_msgs::msg::TimeOffset::MAX;
                }

                point.time_offset.unavailable = false;
                point.time_offset.offset = timeOffset;

            }
            else{
                point.time_offset.unavailable = true;
            }

            //speed
            if(message.crumbData.list.array[i]->speed){
                float speed = *message.crumbData.list.array[i]->speed * 0.02;

                if(speed < carma_v2x_msgs::msg::Speed::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history speed "<<i<<" less than min, setting to min");
                    speed = carma_v2x_msgs::msg::Speed::MIN;
                }
                else if(speed > carma_v2x_msgs::msg::Speed::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Path history speed "<<i<<" greater than max, setting to max");
                    speed = carma_v2x_msgs::msg::Speed::MAX;
                }
                point.speed.unavailable = false;
                point.speed.speed = speed;
            }
            else{
                point.speed.unavailable = true;
            }

            // pos accuracy 
            if(message.crumbData.list.array[i]->posAccuracy->semiMajor && message.crumbData.list.array[i]->posAccuracy->semiMinor){
                
                float semi_major = message.crumbData.list.array[i]->posAccuracy->semiMajor * 0.05;
                if( semi_major > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory accuracy semi-major is greater than max, defined as unavailable - Value not assigned");
                }
                else{ 
                    if(semi_major < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory accuracy semi-major less than min, defaulted to min");
                        semi_major = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                    }
                    point.pos_accuracy.semi_major = semi_major;

                    point.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
                }

                float semi_minor = message.crumbData.list.array[i]->posAccuracy->semiMinor * 0.05;
                if(semi_minor > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory accuracy semi-minor is greater than max, defined as unavailable - Value not assigned");
                }
                else{
                    if(semi_minor < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory accuracy semi-minor less than min, defaulted to min");
                        semi_minor = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                    }
                    point.pos_accuracy.semi_minor = semi_minor;

                    point.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
                }

            }

            if(message.crumbData.list.array[i]->posAccuracy->orientation){
                
                float orientation = message.crumbData.list.array[i]->posAccuracy->orientation * 0.0054932479;
                if(orientation > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory accuracy orientation is greater than max, defined as unavailable - Value not assigned");
                }
                else{
                    if(orientation < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory accuracy orientation less than min, defaulted to min");
                        orientation = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN;
                    }
                    point.pos_accuracy.orientation = orientation;

                    point.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
                }
            }
            
            if(!message.crumbData.list.array[i]->heading){
                float heading = *message.crumbData.list.array[i]->heading * 1.5;
                if(heading > carma_v2x_msgs::msg::CoarseHeading::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory heading greater than max, defined as unavailable, value not set");
                    point.heading.unavailable = true;
                }
                else{
                    if(heading < carma_v2x_msgs::msg::CoarseHeading::MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory heading less than min, defaulted to min");
                        heading = carma_v2x_msgs::msg::CoarseHeading::MIN;
                    }
                    point.heading.heading = heading;
                }
            }
            else{
                point.heading.unavailable = true;
            }
            
            crumb_data.points.push_back(point);
        }

        output.crumb_data = crumb_data;

        if(message.initialPosition){
            output.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;

            if(message.initialPosition->utcTime){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;
                //get DDateTime
                
                if(message.initialPosition->utcTime->year){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
                    output.initial_position.utc_time.year.year = *message.initialPosition->utcTime->year;
                }

                if(message.initialPosition->utcTime->month){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
                    output.initial_position.utc_time.month.month =  *message.initialPosition->utcTime->month;
                }

                if(message.initialPosition->utcTime->day){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY; 
                    output.initial_position.utc_time.day.day = *message.initialPosition->utcTime->day;
                }

                if(message.initialPosition->utcTime->hour){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
                    output.initial_position.utc_time.hour.hour = *message.initialPosition->utcTime->hour;
                }

                if(message.initialPosition->utcTime->minute){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
                    output.initial_position.utc_time.minute.minute= *message.initialPosition->utcTime->minute;
                }

                if(message.initialPosition->utcTime->second){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
                    output.initial_position.utc_time.second.millisecond= *message.initialPosition->utcTime->second;
                }

                if(message.initialPosition->utcTime->offset){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET; 
                    output.initial_position.utc_time.offset.offset_minute= *message.initialPosition->utcTime->offset;
                }

            }

            if(message.initialPosition->Long){
                output.initial_position.lon.unavailable = false;
                output.initial_position.lon.longitude = message.initialPosition->Long * longitude_conversion_const_;
            }
            else{
                output.initial_position.lon.unavailable = true;
            }

            if(message.initialPosition->lat){
                output.initial_position.lat.unavailable = false;
                output.initial_position.lat.latitude = message.initialPosition->lat * latitude_conversion_const_;
            }
            else{
                output.initial_position.lat.unavailable = true;
            }

            if(message.initialPosition->elevation){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION; 

                output.initial_position.elevation.unavailable = false;
                output.initial_position.elevation.elevation = *message.initialPosition->elevation * latitude_conversion_const_;
            }
            else{
                output.initial_position.elevation.unavailable = true;
            }

            if(message.initialPosition->heading){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_HEADING;

                output.initial_position.heading.unavailable = false;
                output.initial_position.heading.heading = *message.initialPosition->heading * heading_conversion_const_;
            }

            if(message.initialPosition->speed){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_SPEED;

                output.initial_position.speed.transmission.transmission_state = message.initialPosition->speed->transmisson;
                output.initial_position.speed.speed.unavailable = false;
                output.initial_position.speed.speed.velocity = message.initialPosition->speed->speed;
            }
            else{
                output.initial_position.speed.speed.unavailable = true;
            }

            //positional accuracy
            if(message.initialPosition->posAccuracy){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;

                if(message.initialPosition->posAccuracy->semiMajor && message.initialPosition->posAccuracy->semiMinor){
                    
                    float semi_major = message.initialPosition->posAccuracy->semiMajor * 0.05;
                    if( semi_major > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory initialPosition accuracy semi-major is greater than max, defined as unavailable - Value not assigned");
                    }
                    else{ 
                        if(semi_major < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory initialPosition accuracy semi-major less than min, defaulted to min");
                            semi_major = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                        }

                        output.initial_position.pos_accuracy.semi_major = semi_major;

                        output.initial_position.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
                    }

                    float semi_minor = message.initialPosition->posAccuracy->semiMinor * 0.05;
                    if(semi_minor > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory initialPosition accuracy semi-minor is greater than max, defined as unavailable - Value not assigned");
                    }
                    else{
                        if(semi_minor < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory initialPosition accuracy semi-minor less than min, defaulted to min");
                            semi_minor = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                        }
                        output.initial_position.pos_accuracy.semi_minor = semi_minor;

                        output.initial_position.pos_accuracy.presence_vector |carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
                    }

                }


                if(message.initialPosition->posAccuracy->orientation){
                
                    float orientation = message.initialPosition->posAccuracy->orientation * 0.0054932479;
                    if(orientation > carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "PathHistory initialPosition accuracy orientation is greater than max, defined as unavailable - Value not assigned");
                    }
                    else{
                        if(orientation < carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN){
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"PathHistory initialPosition accuracy orientation less than min, defaulted to min");
                            orientation = carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN;
                        }
                        output.initial_position.pos_accuracy.orientation = orientation;

                        output.initial_position.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
                    }
                }
            }

            //time confidence
            if(message.initialPosition->timeConfidence){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
                output.initial_position.time_confidence.confidence = *message.initialPosition->timeConfidence;
            }

            //pos confidence
            if(message.initialPosition->posConfidence){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
                output.initial_position.pos_confidence.pos.confidence = message.initialPosition->posConfidence->pos;
                output.initial_position.pos_confidence.elevation.confidence = message.initialPosition->posConfidence->elevation;
            }

            // speed confidence
            if(message.initialPosition->speedConfidence){
                output.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
                output.initial_position.speed_confidence.heading.confidence = message.initialPosition->speedConfidence->heading;
                output.initial_position.speed_confidence.speed.speed_confidence = message.initialPosition->speedConfidence->speed;
                output.initial_position.speed_confidence.throttle.confidence = message.initialPosition->speedConfidence->throttle;

            }

        }
        
        if(message.currGNSSstatus){
            output.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;

            uint8_t gnss_status=0;
            for(int i = message.currGNSSstatus->size -1 ;i >= 0 ;i--){
                gnss_status |= (message.currGNSSstatus->buf[i] << i);
                
            }
             
        }

        return output;
    }

    carma_v2x_msgs::msg::AccelerationSet4Way PSM_Message::decode_accel_set_message(AccelerationSet4Way_t& message){
        
        carma_v2x_msgs::msg::AccelerationSet4Way accel_set;
        
        if(message.Long && message.lat){
            
            accel_set.presence_vector  |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE;
            float Long = message.Long * 0.01;

            if(Long > carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX){
                Long = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX;
            }
            else if(Long > carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN){
                Long = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN;
            }
            accel_set.longitudinal = Long;
        

            float lat = message.lat * 0.01;
            if(lat > carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX){
                lat = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX;
            }
            else if(lat > carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN){
                lat = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN;
            }
            accel_set.longitudinal = lat;
        }

        if(message.vert){
            accel_set.presence_vector  |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE;
            float vert = message.vert * 0.196;
            if(vert > carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX){
                vert = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX;
            }
            else if(vert < carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MIN){
                vert = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MIN;
            }
            accel_set.vert = vert;
        }

        if(message.yaw){
            accel_set.presence_vector  |= carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE;
            float yaw = message.yaw * 0.01;
            if(yaw > carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX){
                yaw = carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX;
            }
            else if(yaw < carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN){
                yaw = carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN;
            }
            accel_set.yaw_rate = yaw;
        }

        return accel_set;

    }

    boost::optional<std::vector<uint8_t>> PSM_Message::encode_psm_message(const carma_v2x_msgs::msg::PSM& plainMessage){
        
        //encode result placeholder
        uint8_t buffer[1472] = {0};
        size_t buffer_size = sizeof(buffer);

        asn_enc_rval_t ec;
        std::shared_ptr<MessageFrame_t>message_shared(new MessageFrame_t);
        //if mem allocation fails
        if(!message_shared)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Cannot allocate mem for PSM message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        MessageFrame_t* message = message_shared.get();
        //set message type to PSM
        message->messageId = PSM_TEST_ID_;
        message->value.present = MessageFrame__value_PR_PersonalSafetyMessage;

        // convert basicType
        message->value.choice.PersonalSafetyMessage.basicType = plainMessage.basic_type.type;

        message->value.choice.PersonalSafetyMessage.secMark = plainMessage.sec_mark.millisecond;

        if(plainMessage.msg_cnt.msg_cnt > 127){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoding msg count value is greater than max, setting to max");
            message->value.choice.PersonalSafetyMessage.msgCnt = 127;        
        }
        else if(plainMessage.msg_cnt.msg_cnt < 0){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoding msg count value is less than min, setting to min");
            message->value.choice.PersonalSafetyMessage.msgCnt = 0; 
        }
        else{
            message->value.choice.PersonalSafetyMessage.msgCnt = plainMessage.msg_cnt.msg_cnt;
        }

        // Temporary ID
        message->value.choice.PersonalSafetyMessage.id.size = 4;
        TemporaryID_t* temp_id = new TemporaryID_t;
        uint8_t temp_id_content[4] ={0};
        for(int i = 0; i< 4; ++i){
            temp_id_content[i] = plainMessage.id.id[i];
        }
        temp_id->buf = temp_id_content;
        temp_id->size = 4;
        message->value.choice.PersonalSafetyMessage.id = *temp_id;

        // position
            //Latitude
        if(!plainMessage.position.latitude || plainMessage.position.latitude == carma_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE){

            message->value.choice.PersonalSafetyMessage.position.lat = carma_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE * latitude_conversion_const_;
        
        }
        else{
            long lat = (plainMessage.position.latitude/latitude_conversion_const_);
            if(lat < carma_v2x_msgs::msg::Position3D::LATITUDE_MIN * latitude_conversion_const_){

                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding latitude value less than min, setting to min");
                lat = carma_v2x_msgs::msg::Position3D::LATITUDE_MIN * latitude_conversion_const_;
            }
            else if(lat > carma_v2x_msgs::msg::Position3D::LATITUDE_MAX * latitude_conversion_const_){

                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding latitude value greater than max, setting to max");
                lat = carma_v2x_msgs::msg::Position3D::LATITUDE_MAX * latitude_conversion_const_;
            }
            
            message->value.choice.PersonalSafetyMessage.position.lat = lat;  
        }
        
            //Longitude
        if(!plainMessage.position.longitude || plainMessage.position.longitude == carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE){
            message->value.choice.PersonalSafetyMessage.position.Long = carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE * longitude_conversion_const_;
        }
        else{
            long longitude = (plainMessage.position.latitude/longitude_conversion_const_);
            if(longitude < carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN * longitude_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding longitude value less than min, setting to min");
                longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN * longitude_conversion_const_;
            }
            else if(longitude > carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX * longitude_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding longitude value greater than max, setting to max");
                longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX * longitude_conversion_const_;
            }
            
            message->value.choice.PersonalSafetyMessage.position.Long = longitude;
        }

            //elevation
        std::shared_ptr<DSRC_Elevation_t>elevation_sharedptr (new DSRC_Elevation_t);
        DSRC_Elevation_t* elevation_ptr = elevation_sharedptr.get();
        if(!plainMessage.position.elevation_exists || plainMessage.position.elevation == carma_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE){
            
            *elevation_ptr = carma_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE * elevation_conversion_const_;
        }
        else{
            long elevation = plainMessage.position.elevation / elevation_conversion_const_;
            if(elevation < carma_v2x_msgs::msg::Position3D::ELEVATION_MIN * elevation_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding elevation value less than min, setting to min");
                elevation = carma_v2x_msgs::msg::Position3D::ELEVATION_MIN * elevation_conversion_const_;
            }
            else if (elevation > carma_v2x_msgs::msg::Position3D::ELEVATION_MAX * elevation_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding elevation value greater than max, setting to max");
                elevation = carma_v2x_msgs::msg::Position3D::ELEVATION_MAX * elevation_conversion_const_;
            }
            *elevation_ptr = elevation;
            
        }
        message->value.choice.PersonalSafetyMessage.position.elevation = elevation_ptr;

        //accuracy 
        // ) 
        if(plainMessage.accuracy.presence_vector!=0){
            if((plainMessage.accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) 
            {   //get accuracy 
                message->value.choice.PersonalSafetyMessage.accuracy.semiMajor = plainMessage.accuracy.semi_major;
                message->value.choice.PersonalSafetyMessage.accuracy.semiMinor = plainMessage.accuracy.semi_minor;
            }
            else if((plainMessage.accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE)
            {   //get accuracy orientation
                message->value.choice.PersonalSafetyMessage.accuracy.orientation = plainMessage.accuracy.orientation;
            }
        }
        
        //Speed
        if(!plainMessage.speed.unavailable){
            long speed = plainMessage.speed.velocity / velocity_conversion_const_;
            if(speed > 8191){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding speed value greater than max, setting to max");
                speed = 8191;
            }
            else if (speed < 0){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding speed value less than min, setting to min");
                speed = 0;
            }
            message->value.choice.PersonalSafetyMessage.speed = speed;
        }

        //Heading
        if(!plainMessage.heading.unavailable){
            long heading = plainMessage.heading.heading / heading_conversion_const_;
            if(heading > carma_v2x_msgs::msg::Heading::MAX * heading_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding heading value greater than max, setting to max");
                heading = carma_v2x_msgs::msg::Heading::MAX * heading_conversion_const_;
            }
            else if (heading < carma_v2x_msgs::msg::Heading::MIN * heading_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding heading value less than min, setting to min");
                heading = carma_v2x_msgs::msg::Heading::MIN * heading_conversion_const_;
            }
            message->value.choice.PersonalSafetyMessage.heading = heading;
        }

        //Encoding optional fields
        //accel_set
        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ACCEL_SET){
            carma_v2x_msgs::msg::AccelerationSet4Way msg_accel_set = plainMessage.accel_set;

        }

        boost::optional<std::vector<uint8_t>> b_array;
        return b_array;
    }

    AccelerationSet4Way_t PSM_Message::encode_accel_set(carma_v2x_msgs::msg::AccelerationSet4Way& message){
        AccelerationSet4Way_t output;


        return output;
    }

}