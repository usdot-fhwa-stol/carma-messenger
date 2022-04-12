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
        uint8_t buffer[544];
        size_t buffer_size = sizeof(buffer);

        asn_enc_rval_t ec;
        MessageFrame_t* message;
        message = (MessageFrame_t*) calloc(1, sizeof(MessageFrame_t));
        //if mem allocation fails
        if(!message)
        {
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Cannot allocate mem for PSM message encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }
        
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
        TemporaryID_t* temp_id = new TemporaryID_t;
        uint8_t temp_id_content[4] ={0};
        for(int i = 0; i< 4; ++i){
            temp_id_content[i] = (char) plainMessage.id.id[i];
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
            long lat = (float(plainMessage.position.latitude)/float(latitude_conversion_const_));
            if(lat < carma_v2x_msgs::msg::Position3D::LATITUDE_MIN/ float(latitude_conversion_const_)){

                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding latitude value less than min, setting to min");
                lat = carma_v2x_msgs::msg::Position3D::LATITUDE_MIN /float(latitude_conversion_const_);
            }
            else if(lat > carma_v2x_msgs::msg::Position3D::LATITUDE_MAX / float(latitude_conversion_const_)){

                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding latitude value greater than max, setting to max");
                lat = carma_v2x_msgs::msg::Position3D::LATITUDE_MAX / float(latitude_conversion_const_);
            }
            
            message->value.choice.PersonalSafetyMessage.position.lat = lat;  
        }
        
        
            //Longitude
        if(!plainMessage.position.longitude || plainMessage.position.longitude == carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE){
            message->value.choice.PersonalSafetyMessage.position.Long = carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE * longitude_conversion_const_;
        }
        else{
            long longitude = ((float) plainMessage.position.longitude / (float) longitude_conversion_const_);
            if(longitude < carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN /(float) longitude_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding longitude value less than min, setting to min");
                longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_MIN /(float) longitude_conversion_const_;
            }
            else if(longitude > carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX /(float) longitude_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding longitude value greater than max, setting to max");
                longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_MAX /(float) longitude_conversion_const_;
            }
            
            message->value.choice.PersonalSafetyMessage.position.Long = longitude;
        }
        
        //elevation
        DSRC_Elevation_t* elevation_ptr = new DSRC_Elevation_t;
        if(!plainMessage.position.elevation_exists || plainMessage.position.elevation == carma_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE){
            
            *elevation_ptr = carma_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE /(float) elevation_conversion_const_;
        }
        else{
            long elevation = plainMessage.position.elevation / elevation_conversion_const_;
            if(elevation < carma_v2x_msgs::msg::Position3D::ELEVATION_MIN /(float) elevation_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding elevation value less than min, setting to min");
                elevation = carma_v2x_msgs::msg::Position3D::ELEVATION_MIN /(float) elevation_conversion_const_;
            }
            else if (elevation > carma_v2x_msgs::msg::Position3D::ELEVATION_MAX /(float) elevation_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding elevation value greater than max, setting to max");
                elevation = carma_v2x_msgs::msg::Position3D::ELEVATION_MAX /(float) elevation_conversion_const_;
            }
            *elevation_ptr = elevation;
            
        }
        message->value.choice.PersonalSafetyMessage.position.elevation = elevation_ptr;
        
        //accuracy 
        if(plainMessage.accuracy.presence_vector!=0){
            if(plainMessage.accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE) 
            {   //get accuracy 
                long semi_major = plainMessage.accuracy.semi_major / 0.05;
                message->value.choice.PersonalSafetyMessage.accuracy.semiMajor = semi_major;
                long semi_minor = plainMessage.accuracy.semi_minor / 0.05;
                message->value.choice.PersonalSafetyMessage.accuracy.semiMinor = semi_minor;
            }
            
            if(plainMessage.accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE)
            {   //get accuracy orientation
                long orientation = plainMessage.accuracy.orientation / 0.0054932479;
                message->value.choice.PersonalSafetyMessage.accuracy.orientation = orientation;
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
        Heading_t heading;
        if(!plainMessage.heading.unavailable){
            
            heading = plainMessage.heading.heading / heading_conversion_const_;

            if(heading > carma_v2x_msgs::msg::Heading::MAX / heading_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding heading value greater than max, setting to max");
                heading = carma_v2x_msgs::msg::Heading::MAX / heading_conversion_const_;
            }
            else if (heading < carma_v2x_msgs::msg::Heading::MIN / heading_conversion_const_){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding heading value less than min, setting to min");
                heading = carma_v2x_msgs::msg::Heading::MIN / heading_conversion_const_;
            }
        }
        else{
            RCLCPP_DEBUG_STREAM(node_logging_->get_logger(),"Encoding Heading unavailable");
            heading = 28800;
        }
        message->value.choice.PersonalSafetyMessage.heading = heading;

        //Encoding optional fields
        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ACCEL_SET){
            AccelerationSet4Way_t* accel_set;
            accel_set =(AccelerationSet4Way_t*) calloc(1, sizeof(AccelerationSet4Way_t));

            if(plainMessage.accel_set.presence_vector & carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE){
                
                long Long = plainMessage.accel_set.longitudinal/0.01;
                if(Long > (carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX/0.01))
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding accel_set longitude greater than max, set to max");
                    Long = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX/0.01;
                }
                else if(Long < (carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN/0.01))
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding accel_set longitude less than min, set to min");
                    Long = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN/0.01;
                }
                accel_set->Long = Long;
                
                

                long lat = plainMessage.accel_set.lateral/0.01;
                if(lat > (carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX/0.01))
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding accel_set latitude greater than max, set to max");
                    lat = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MAX/0.01;
                }
                else if(lat < (carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN/0.01))
                {
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding accel_set latitude less than min, set to min");
                    lat = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_MIN/0.01;
                }
                accel_set->lat = lat;
                
            }

            if (plainMessage.accel_set.presence_vector & carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE){
                
                long vert = plainMessage.accel_set.vert / 0.196;

                if(vert > carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX/0.196){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding accel_set vertical greater than max, set to max");
                    vert = carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_MAX/0.196;
                }
                accel_set->vert = vert;
                
            }

            if(plainMessage.accel_set.presence_vector & carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE){
                long yaw = plainMessage.accel_set.yaw_rate / 0.01;

                if(yaw > carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX/0.01){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoding accel_set yaw rate greater than max, set to max");
                    yaw = carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MAX/0.01;
                }
                else if ( yaw < carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN/0.01){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoding accel_set yaw rate less than min, set to min");
                    yaw = carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_MIN/0.01;
                }
                accel_set->yaw = yaw;
            }
            message->value.choice.PersonalSafetyMessage.accelSet = accel_set;
        }

        
        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY){
            PathHistory_t* path_history; 
            path_history = (PathHistory_t*) calloc(1, sizeof(PathHistory_t));
            
            //get crumblist
            PathHistoryPointList_t* path_history_list;
            path_history_list = (PathHistoryPointList_t*) calloc(1, sizeof(PathHistoryPointList_t));
            for(size_t i = 0 ;i < plainMessage.path_history.crumb_data.points.size(); ++i){
                PathHistoryPoint_t* point;
                point = (PathHistoryPoint_t*) calloc(1, sizeof(PathHistoryPoint_t));
                
                if(!plainMessage.path_history.crumb_data.points[i].lat_offset.unavailable){
                    point->latOffset = plainMessage.path_history.crumb_data.points[i].lat_offset.offset / 1e-7;
                }

                if(!plainMessage.path_history.crumb_data.points[i].lon_offset.unavailable){
                    point->lonOffset = plainMessage.path_history.crumb_data.points[i].lon_offset.offset / 1e-7;
                }

                if(!plainMessage.path_history.crumb_data.points[i].elevation_offset.unavailable){
                    point->elevationOffset = plainMessage.path_history.crumb_data.points[i].elevation_offset.offset / 0.1;
                }
                
                if(!plainMessage.path_history.crumb_data.points[i].time_offset.unavailable){
                    point->timeOffset = plainMessage.path_history.crumb_data.points[i].time_offset.offset / 0.01;
                }
                
                if(!plainMessage.path_history.crumb_data.points[i].speed.unavailable){
                    Speed_t* speed;
                    speed = (Speed_t*) calloc(1, sizeof(Speed_t));
                    *speed = plainMessage.path_history.crumb_data.points[i].speed.speed / 0.02;
                    point->speed = speed;
                }

                //positional_accuracy
                if(!plainMessage.path_history.crumb_data.points[i].pos_accuracy.presence_vector == 0){
                    PositionalAccuracy_t* positional_accuracy;
                    positional_accuracy = (PositionalAccuracy_t*) calloc(1, sizeof(PositionalAccuracy_t));
                    if(plainMessage.path_history.crumb_data.points[i].pos_accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE){
                        int semi_major = plainMessage.path_history.crumb_data.points[i].pos_accuracy.semi_major / 0.05;
                        positional_accuracy->semiMajor = semi_major;

                        int semi_minor = plainMessage.path_history.crumb_data.points[i].pos_accuracy.semi_minor / 0.05;
                        positional_accuracy->semiMinor = semi_minor;

                        point->posAccuracy = positional_accuracy;
                    }
                    
                    if(plainMessage.path_history.crumb_data.points[i].pos_accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE){
                        positional_accuracy->orientation = plainMessage.path_history.crumb_data.points[i].pos_accuracy.orientation / 0.0054932479;
                        point->posAccuracy = positional_accuracy;
                    }
                }

                if(!plainMessage.path_history.crumb_data.points[i].heading.unavailable){
                    CoarseHeading_t* heading;
                    heading = (CoarseHeading_t*) calloc(1, sizeof(CoarseHeading_t));
                    long heading_value = plainMessage.path_history.crumb_data.points[i].heading.heading / 1.5;
                    *heading = heading_value;
                    point->heading = heading; 
                }
                
                asn_sequence_add(&path_history->crumbData.list, point);
            
            }
            

            if(plainMessage.path_history.presence_vector & carma_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS){

                GNSSstatus_t* gnss_status;
                gnss_status = (GNSSstatus_t*) calloc(1, sizeof(GNSSstatus_t));

                std::string status = std::to_string(plainMessage.path_history.curr_gnss_status.statuses);
                size_t size = status.size();
                uint8_t status_string[size];
                for(size_t i = 0;i< size;i++){
                    status_string[i] = status[i] - '0';
                }
                gnss_status->size = size;
                gnss_status->buf = status_string;

                path_history->currGNSSstatus = gnss_status;
                
            }
            

            if(plainMessage.path_history.presence_vector & carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION){

                FullPositionVector_t* initial_position;
                initial_position = (FullPositionVector_t*) calloc(1, sizeof(FullPositionVector_t));

                if(plainMessage.path_history.initial_position.presence_vector & carma_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME){
                    DDateTime_t* utc_time;
                    utc_time = (DDateTime_t*) calloc(1, sizeof(DDateTime_t));

                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::YEAR){
                        long* year = new long;
                        *year = plainMessage.path_history.initial_position.utc_time.year.year;
                        utc_time->year = year;
                        
                    }
                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MONTH){
                        long* month = new long;
                        *month = plainMessage.path_history.initial_position.utc_time.month.month;
                        utc_time->month = month;
                    }
                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::DAY){
                        long* day = new long;
                        *day = plainMessage.path_history.initial_position.utc_time.day.day;
                        utc_time->day = day;
                    }
                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::HOUR){
                        long* hour = new long;
                        *hour = plainMessage.path_history.initial_position.utc_time.hour.hour;
                        utc_time->hour = hour;
                    }
                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MINUTE){
                        long* minute = new long;
                        *minute = plainMessage.path_history.initial_position.utc_time.minute.minute;
                        utc_time->minute = minute;
                    }
                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::SECOND){
                        long* second = new long;
                        *second = plainMessage.path_history.initial_position.utc_time.second.millisecond;
                        utc_time->second = second;
                    }
                    if(plainMessage.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::OFFSET){
                        long* offset = new long;
                        *offset = plainMessage.path_history.initial_position.utc_time.offset.offset_minute;
                        utc_time->offset = offset;
                    }
                    initial_position->utcTime = utc_time;
                }
                

                path_history->initialPosition = initial_position;
            }

            message->value.choice.PersonalSafetyMessage.pathHistory = path_history;

        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION){
            PathPrediction_t* path_prediction = new PathPrediction_t;
            path_prediction->radiusOfCurve = plainMessage.path_prediction.radius_of_curvature;
            path_prediction->confidence = plainMessage.path_prediction.confidence;

            message->value.choice.PersonalSafetyMessage.pathPrediction = path_prediction;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_PROPULSION){
            PropelledInformation_t* propulsion;
            propulsion = (PropelledInformation_t*) calloc(1, sizeof(PropelledInformation_t));
            
            if(plainMessage.propulsion.choice == j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN){
                propulsion->present = PropelledInformation_PR_human;
                propulsion->choice.human = plainMessage.propulsion.human.type;
            }
            else if(plainMessage.propulsion.choice == j2735_v2x_msgs::msg::PropelledInformation::CHOICE_ANIMAL){
                propulsion->present = PropelledInformation_PR_animal;
                propulsion->choice.animal = plainMessage.propulsion.animal.type;
            }
            else if(plainMessage.propulsion.choice == j2735_v2x_msgs::msg::PropelledInformation::CHOICE_MOTOR){
                propulsion->present = PropelledInformation_PR_motor;
                propulsion->choice.motor = plainMessage.propulsion.motor.type;
            }

            message->value.choice.PersonalSafetyMessage.propulsion = propulsion;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_USE_STATE){
            PersonalDeviceUsageState_t* use_state;
            use_state = (PersonalDeviceUsageState_t*) calloc(1, sizeof(PersonalDeviceUsageState_t));
            std::string use_state_str = std::to_string(plainMessage.use_state.states);
            size_t size = use_state_str.size();
            uint8_t array[size];
            for(size_t i=0;i<size;i++){
                array[i] = use_state_str[i] - '0';
            }
            use_state->size = size;
            use_state->buf = array;

            message->value.choice.PersonalSafetyMessage.useState = use_state;
            
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST){
            PersonalCrossingRequest_t* cross_request;
            cross_request = (PersonalCrossingRequest_t*) calloc(1, sizeof(PersonalCrossingRequest_t));

            *cross_request = plainMessage.cross_request.cross_request;
            message->value.choice.PersonalSafetyMessage.crossRequest = cross_request;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_CROSS_STATE){
            PersonalCrossingInProgress_t* cross_state;
            cross_state = (PersonalCrossingInProgress_t*) calloc(1, sizeof(PersonalCrossingInProgress_t));

            *cross_state = plainMessage.cross_state.cross_state;
            message->value.choice.PersonalSafetyMessage.crossState = cross_state;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE){
            NumberOfParticipantsInCluster_t* cluster_size;
            cluster_size = (NumberOfParticipantsInCluster_t*) calloc(1, sizeof(NumberOfParticipantsInCluster_t));

            *cluster_size = plainMessage.cluster_size.cluster_size;
            message->value.choice.PersonalSafetyMessage.clusterSize = cluster_size;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS){
            NumberOfParticipantsInCluster_t* cluster_radius;
            cluster_radius = (NumberOfParticipantsInCluster_t*) calloc(1, sizeof(NumberOfParticipantsInCluster_t));

            *cluster_radius = plainMessage.cluster_radius.cluster_radius;
            message->value.choice.PersonalSafetyMessage.clusterRadius = cluster_radius;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE){
            PublicSafetyEventResponderWorkerType_t* event_responder_type;
            event_responder_type = (PublicSafetyEventResponderWorkerType_t*) calloc(1, sizeof(PublicSafetyEventResponderWorkerType_t));

            *event_responder_type = plainMessage.event_responder_type.type;
            message->value.choice.PersonalSafetyMessage.eventResponderType = event_responder_type;
        }
        
        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE){
            PublicSafetyAndRoadWorkerActivity_t* activity_type;
            activity_type = (PublicSafetyAndRoadWorkerActivity_t*) calloc(1, sizeof(PublicSafetyAndRoadWorkerActivity_t));

            std::string activity_type_str = std::to_string(plainMessage.activity_type.activities);
            size_t size = activity_type_str.size();
            uint8_t array[size];
            for(size_t i = 0;i < size;i++){
                array[i] = activity_type_str[i] - '0';
            }
            activity_type->size = size;
            activity_type->buf = array;
            message->value.choice.PersonalSafetyMessage.activityType = activity_type;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE){
            PublicSafetyDirectingTrafficSubType_t* activity_sub_type;
            activity_sub_type = (PublicSafetyDirectingTrafficSubType_t*) calloc(1, sizeof(PublicSafetyDirectingTrafficSubType_t));

            std::string sub_type_str = std::to_string(plainMessage.activity_sub_type.sub_types);
            size_t size = sub_type_str.size();
            uint8_t array[size];
            for(size_t i = 0;i< size; i++){
                array[i] = sub_type_str[i] - '0';
            }
            activity_sub_type->size = size;
            activity_sub_type->buf = array;
            message->value.choice.PersonalSafetyMessage.activitySubType = activity_sub_type;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE){
            PersonalAssistive_t* assist_type;
            assist_type = (PersonalAssistive_t*) calloc(1, sizeof(PersonalAssistive_t));

            std::string assist_type_str = std::to_string(plainMessage.assist_type.types);
            size_t size = assist_type_str.size();
            uint8_t array[size];
            for(size_t i = 0; i < size;i++){
                array[i] = assist_type_str[i] - '0';
            }
            assist_type->size = size;
            assist_type->buf = array;
            message->value.choice.PersonalSafetyMessage.assistType = assist_type;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_SIZING){
            UserSizeAndBehaviour_t* sizing;
            sizing = (UserSizeAndBehaviour_t*) calloc(1, sizeof(UserSizeAndBehaviour_t));
            std::string sizing_string = std::to_string(plainMessage.sizing.sizes_and_behaviors);
            size_t size = sizing_string.size();
            uint8_t array[size];
            for(size_t i = 0; i < size; i++){
                array[i] = sizing_string[i] - '0';
            }
            sizing->size = size;
            sizing->buf = array;
            message->value.choice.PersonalSafetyMessage.sizing = sizing;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT){
            Attachment_t* attachment;
            attachment = (Attachment_t*) calloc(1, sizeof(Attachment_t));

            *attachment = plainMessage.attachment.type;
            message->value.choice.PersonalSafetyMessage.attachment = attachment;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS){
            AttachmentRadius_t* attachment_radius;
            attachment_radius = (AttachmentRadius_t*) calloc(1, sizeof(AttachmentRadius_t));

            *attachment_radius = plainMessage.attachment_radius.attachment_radius * 10;
            message->value.choice.PersonalSafetyMessage.attachmentRadius = attachment_radius;
        }

        if(plainMessage.presence_vector & carma_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE){
            AnimalType_t* animal_type;
            animal_type = (AnimalType_t*) calloc(1, sizeof(AnimalType_t));

            *animal_type = plainMessage.animal_type.type;
            message->value.choice.PersonalSafetyMessage.animalType = animal_type;
        }

        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
    
        //log a warning if that fails
        if(ec.encoded == -1) {
            RCLCPP_WARN_STREAM( node_logging_->get_logger(), "Encoding for PSM Message failed");
            return boost::optional<std::vector<uint8_t>>{};
        }
        //copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
                
        //for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);

    }


}