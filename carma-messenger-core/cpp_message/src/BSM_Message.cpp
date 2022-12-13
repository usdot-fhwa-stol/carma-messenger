/*
 * Copyright (C) 2020-2022 LEIDOS.
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
 * CPP File containing BSM Message method implementations
 */

#include "cpp_message/BSM_Message.h"

namespace cpp_message
{
    template <typename T>
    T *create_store_shared(std::vector<std::shared_ptr<void>> &shared_pointers)
    {
        auto obj_shared = std::make_shared<T>();
        shared_pointers.push_back(obj_shared);
        return obj_shared.get();
    }

    template <typename T>
    T *create_store_shared_array(std::vector<std::shared_ptr<void>> &shared_pointers, int size)
    {
        std::shared_ptr<T[]> array_shared(new T[size]{0});
        shared_pointers.push_back(array_shared);
        return array_shared.get();
    }

    j2735_v2x_msgs::msg::PathHistory BSM_Message::decode_path_history_message(PathHistory_t& message){
        
        j2735_v2x_msgs::msg::PathHistory output;
        
        // crumb_data
        j2735_v2x_msgs::msg::PathHistoryPointList crumb_data;

        if(message.crumbData.list.count == 0){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded PathHistory points list is empty, adding empty point");
            j2735_v2x_msgs::msg::PathHistoryPoint point;
            crumb_data.points.push_back(point);
        }

        for(size_t i = 0; i < message.crumbData.list.count; ++i){       // MIN_SIZE = 1, MAX_SIZE = 23
            
            if(i > j2735_v2x_msgs::msg::PathHistoryPointList::MAX_SIZE){
                RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded PathHistory points size greater than max rejecting points");
                break;
            }

            j2735_v2x_msgs::msg::PathHistoryPoint point;

            // latitude
            if( message.crumbData.list.array[i]->latOffset){
                point.lat_offset.offset =  message.crumbData.list.array[i]->latOffset;
            }
            else{
                point.lat_offset.offset = j2735_v2x_msgs::msg::OffsetLLB18::UNAVAILABLE;
            }

            // longitude
            if( message.crumbData.list.array[i]->lonOffset){
                point.lon_offset.offset = message.crumbData.list.array[i]->lonOffset;
            }
            else{
                point.lon_offset.offset = message.crumbData.list.array[i]->lonOffset;;
            }

            // elevation
            if( message.crumbData.list.array[i]->elevationOffset){
                point.elevation_offset.offset = message.crumbData.list.array[i]->elevationOffset;
            }
            else{
                point.elevation_offset.offset = j2735_v2x_msgs::msg::VertOffsetB12::UNAVAILABLE;
            }

            // time_offset
            if( message.crumbData.list.array[i]->timeOffset){
                point.time_offset.offset = message.crumbData.list.array[i]->timeOffset;
            }
            else{
                point.time_offset.offset = j2735_v2x_msgs::msg::TimeOffset::UNAVAILABLE;
            }

            // speed
            if(message.crumbData.list.array[i]->speed){
                point.speed.speed = *message.crumbData.list.array[i]->speed;
            }
            else{
                point.speed.speed = j2735_v2x_msgs::msg::Speed::UNAVAILABLE;
            }

            // pos_accuracy 
            if(message.crumbData.list.array[i]->posAccuracy){

                if(message.crumbData.list.array[i]->posAccuracy->semiMajor && message.crumbData.list.array[i]->posAccuracy->semiMinor){
                    int semi_major = message.crumbData.list.array[i]->posAccuracy->semiMajor;
                    if( semi_major > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded PathHistory accuracy semi-major is greater than max, defined as unavailable - Value not assigned");
                        semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
                    }
                    else if(semi_major < j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded PathHistory accuracy semi-major less than min, defaulted to min");
                        semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                    }
                    point.pos_accuracy.semi_major = semi_major;

                    int semi_minor = message.crumbData.list.array[i]->posAccuracy->semiMinor;
                    if(semi_minor > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded PathHistory accuracy semi-minor is greater than max, defined as unavailable - Value not assigned");
                        semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
                    }
                    else if(semi_minor < j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded PathHistory accuracy semi-minor less than min, defaulted to min");
                        semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_MIN;
                    }
                    point.pos_accuracy.semi_minor = semi_minor;

                }
                else{
                    point.pos_accuracy.semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
                    point.pos_accuracy.semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
                }


                if(message.crumbData.list.array[i]->posAccuracy->orientation){
                    int orientation = message.crumbData.list.array[i]->posAccuracy->orientation;
                    if(orientation > j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded PathHistory accuracy orientation is greater than max, defined as unavailable - Value not assigned");
                    }
                    else{
                        if(orientation < j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN){
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded PathHistory accuracy orientation less than min, defaulted to min");
                            orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_MIN;
                        }
                        point.pos_accuracy.orientation = orientation;
                    }
                }
            }

            // heading
            if(message.crumbData.list.array[i]->heading){
                int heading = *message.crumbData.list.array[i]->heading;
                
                if(heading > j2735_v2x_msgs::msg::CoarseHeading::MAX){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded PathHistory heading greater than max, defined as unavailable");
                    heading = j2735_v2x_msgs::msg::CoarseHeading::UNAVAILABLE;
                }
                else if(heading < j2735_v2x_msgs::msg::CoarseHeading::MIN){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded PathHistory heading less than min, defaulted to min");
                    heading = j2735_v2x_msgs::msg::CoarseHeading::MIN;
                }
                point.heading.heading = heading;
            }
            else{
                point.heading.heading = j2735_v2x_msgs::msg::CoarseHeading::UNAVAILABLE;
            }

            crumb_data.points.push_back(point);
        }

        output.crumb_data = crumb_data;
        
        // initial_position
        if(message.initialPosition!=nullptr){
            output.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;

            if(message.initialPosition->utcTime){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;

                // initial_position.utc_time
                // utc_time.year
                if(message.initialPosition->utcTime->year){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
                    uint16_t year = *message.initialPosition->utcTime->year;
                    if(year > j2735_v2x_msgs::msg::DYear::MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded year value greater than max, setting to max");
                        year = j2735_v2x_msgs::msg::DYear::MAX;   
                    }
                    output.initial_position.utc_time.year.year = year;
                }
                // utc_time.month
                if(message.initialPosition->utcTime->month){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
                    uint8_t month = *message.initialPosition->utcTime->month;
                    if(month > j2735_v2x_msgs::msg::DMonth::MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded month value greater than max, setting to max");
                        month = j2735_v2x_msgs::msg::DMonth::MAX;   
                    }
                    output.initial_position.utc_time.month.month = month;
                }
                // utc_time.day
                if(message.initialPosition->utcTime->day){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
                    uint8_t day = *message.initialPosition->utcTime->day;
                    if(day > j2735_v2x_msgs::msg::DDay::MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded day value greater than max, setting to max");
                        day = j2735_v2x_msgs::msg::DDay::MAX;   
                    }
                    output.initial_position.utc_time.day.day = day;
                }
                // utc_time.hour
                if(message.initialPosition->utcTime->hour){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
                    uint8_t hour = *message.initialPosition->utcTime->hour;
                    if(hour > j2735_v2x_msgs::msg::DHour::UNAVAILABLE){
                        // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'HOUR_OF_DAY_MAX'
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded hour value greater than max, setting to max");
                        hour = j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX;   
                    }
                    output.initial_position.utc_time.hour.hour = hour;
                }
                // utc_time.minute
                if(message.initialPosition->utcTime->minute){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
                    uint8_t minute = *message.initialPosition->utcTime->minute;
                    if(minute > j2735_v2x_msgs::msg::DMinute::UNAVAILABLE){
                        // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'MINUTE_IN_HOUR_MAX'
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded minute value greater than max, setting to max");
                        minute = j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX;   
                    }
                    output.initial_position.utc_time.minute.minute = minute;
                }
                // utc_time.second
                if(message.initialPosition->utcTime->second){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
                    uint16_t second = *message.initialPosition->utcTime->second;
                    output.initial_position.utc_time.second.millisecond = second;
                }
                // utc_time.offset
                if(message.initialPosition->utcTime->offset){
                    output.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
                    long offset = *message.initialPosition->utcTime->offset;
                    if(offset > j2735_v2x_msgs::msg::DOffset::MAX){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded offset value greater than max, setting to max");
                        offset = j2735_v2x_msgs::msg::DOffset::MAX;   
                    }
                    else if(offset < j2735_v2x_msgs::msg::DOffset::MIN){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded offset value less than min, setting to min");
                        offset = j2735_v2x_msgs::msg::DOffset::MIN;   
                    }
                    output.initial_position.utc_time.offset.offset_minute = offset;
                }
            }

            if(message.initialPosition->Long){
                output.initial_position.lon.longitude = message.initialPosition->Long;
            }
            else{
                output.initial_position.lon.longitude = j2735_v2x_msgs::msg::Longitude::LONGITUDE_UNAVAILABLE;
            }

            if(message.initialPosition->lat){
                output.initial_position.lat.latitude = message.initialPosition->lat;
            }
            else{
                output.initial_position.lat.latitude = j2735_v2x_msgs::msg::Latitude::LATITUDE_UNAVAILABLE;
            }

            if(message.initialPosition->elevation){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION; 
                output.initial_position.elevation.elevation = *message.initialPosition->elevation;
            }

            if(message.initialPosition->heading){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_HEADING;
                output.initial_position.heading.heading = *message.initialPosition->heading;
            }

            if(message.initialPosition->speed){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED;

                output.initial_position.speed.transmission.transmission_state = message.initialPosition->speed->transmisson;
                output.initial_position.speed.speed.velocity = message.initialPosition->speed->speed;
            }

            // positional accuracy
            if(message.initialPosition->posAccuracy){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;

                if(message.initialPosition->posAccuracy->semiMajor && message.initialPosition->posAccuracy->semiMinor){
                    output.initial_position.pos_accuracy.semi_major = message.initialPosition->posAccuracy->semiMajor;
                    output.initial_position.pos_accuracy.semi_minor = message.initialPosition->posAccuracy->semiMinor;
                }
                if(message.initialPosition->posAccuracy->orientation){
                    output.initial_position.pos_accuracy.orientation = message.initialPosition->posAccuracy->orientation;
                }
            }

            // time_confidence
            if(message.initialPosition->timeConfidence){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
                output.initial_position.time_confidence.confidence = *message.initialPosition->timeConfidence;
            }

            // pos_confidence
            if(message.initialPosition->posConfidence){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
                output.initial_position.pos_confidence.pos.confidence = message.initialPosition->posConfidence->pos;
                output.initial_position.pos_confidence.elevation.confidence = message.initialPosition->posConfidence->elevation;
            }

            // speed_confidence
            if(message.initialPosition->speedConfidence){
                output.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
                output.initial_position.speed_confidence.heading.confidence = message.initialPosition->speedConfidence->heading;
                output.initial_position.speed_confidence.speed.speed_confidence = message.initialPosition->speedConfidence->speed;
                output.initial_position.speed_confidence.throttle.confidence = message.initialPosition->speedConfidence->throttle;
            }
        }
        
        // gnss_status
        if(message.currGNSSstatus){
            output.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;

            uint8_t gnss_status=0;
            for(int i = message.currGNSSstatus->size -1 ; i >= 0; i--){
                gnss_status |= (message.currGNSSstatus->buf[i] << i);
            }
            output.curr_gnss_status.statuses = gnss_status;
        }

        return output;
    }

    j2735_v2x_msgs::msg::PivotPointDescription BSM_Message::decode_pivot_point_description(PivotPointDescription_t& pivot_point_description){
        j2735_v2x_msgs::msg::PivotPointDescription j2735_pivot_point_description;

        // offset
        long pivot_offset = pivot_point_description.pivotOffset;
        if(pivot_offset < j2735_v2x_msgs::msg::OffsetB11::OFFSET_MIN){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded pivot offset value less than min, setting to min");
            pivot_offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_MIN;
        }
        else if(pivot_offset > j2735_v2x_msgs::msg::OffsetB11::OFFSET_MAX){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded pivot offset value greater than max, setting to max");
            pivot_offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_MAX;
        }
        j2735_pivot_point_description.pivot_offset.offset = pivot_offset;

        // angle
        long pivot_angle = pivot_point_description.pivotAngle;
        if(pivot_angle < j2735_v2x_msgs::msg::Angle::ANGLE_MIN){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded pivot angle value less than min, setting to min");
            pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MIN;
        }
        else if(pivot_angle > j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
            // Note: Value checked against 'ANGLE_UNAVAILABLE' since this value is larger than 'ANGLE_MAX'
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded pivot angle value greater than max, setting to max");
            pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MAX;
        }
        j2735_pivot_point_description.pivot_angle.angle = pivot_angle;

        // pivotingAllowed
        j2735_pivot_point_description.pivots.pivoting_allowed = pivot_point_description.pivots;

        return j2735_pivot_point_description;
    }

    boost::optional<j2735_v2x_msgs::msg::BSM> BSM_Message::decode_bsm_message(std::vector<uint8_t>& binary_array){
        
        j2735_v2x_msgs::msg::BSM output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message = nullptr;
        
        //copy from vector to array         
        size_t len=binary_array.size();    
        
        uint8_t buf[len];             
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
         
        //if decode success
        if(rval.code==RC_OK)
        {
            BSMcoreData_t core_data_msg = message->value.choice.BasicSafetyMessage.coreData;
            output.core_data.msg_count = core_data_msg.msgCnt; 
            auto id_len = core_data_msg.id.size;
            for(auto i = 0; i < id_len; i++)
            {
                output.core_data.id.push_back(core_data_msg.id.buf[i]);
            }
            output.core_data.sec_mark = core_data_msg.secMark;
            output.core_data.latitude = core_data_msg.lat;
            output.core_data.longitude = core_data_msg.Long; 
            output.core_data.elev = core_data_msg.elev;
            output.core_data.accuracy.orientation = core_data_msg.accuracy.orientation;
            output.core_data.accuracy.semi_major = core_data_msg.accuracy.semiMajor;
            output.core_data.accuracy.semi_minor = core_data_msg.accuracy.semiMinor;
            output.core_data.transmission.transmission_state = core_data_msg.transmission;
            output.core_data.speed = core_data_msg.speed;
            output.core_data.heading = core_data_msg.heading;
            output.core_data.angle = core_data_msg.angle;
            output.core_data.accel_set.lateral = core_data_msg.accelSet.lat;
            output.core_data.accel_set.longitudinal =core_data_msg.accelSet.Long;
            output.core_data.accel_set.vert = core_data_msg.accelSet.vert;
            output.core_data.accel_set.yaw_rate = core_data_msg.accelSet.yaw;
            // brake_applied_status decoding
            // e.g. make 0b0100000 to 0b0000100
            uint8_t binary = core_data_msg.brakes.wheelBrakes.buf[0] >> 3;
            unsigned int brake_applied_status_type = 4;
            // e.g. shift the binary right until it equals to 1 (0b00000001) to determine the location of the non-zero bit
            
            for (int i = 0; i < 4; i ++)
            {
                if ((int)binary == 1) 
                {
                    output.core_data.brakes.wheel_brakes.brake_applied_status = brake_applied_status_type;
                    break;
                }
                else
                {
                    brake_applied_status_type -= 1;
                    binary = binary >> 1;
                }
            }
            output.core_data.brakes.traction.traction_control_status = core_data_msg.brakes.traction;
            output.core_data.brakes.abs.anti_lock_brake_status = core_data_msg.brakes.abs;
            output.core_data.brakes.scs.stability_control_status = core_data_msg.brakes.scs;
            output.core_data.brakes.brake_boost.brake_boost_applied = core_data_msg.brakes.brakeBoost;
            output.core_data.brakes.aux_brakes.auxiliary_brake_status = core_data_msg.brakes.auxBrakes;            
            output.core_data.size.vehicle_length = core_data_msg.size.length;
            output.core_data.size.vehicle_width = core_data_msg.size.width;

            // Decode partII list
            if (message->value.choice.BasicSafetyMessage.partII){
                output.presence_vector |= j2735_v2x_msgs::msg::BSM::HAS_PART_II;

                for(size_t i = 0; i < message->value.choice.BasicSafetyMessage.partII->list.count; ++i){
                    if(i > j2735_v2x_msgs::msg::BSM::PART_II_MAX_SIZE){
                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoded BSM Part II Extensions list size is greater than max. Rejecting list element.");
                        break;
                    }

                    // Obtain the decoded Part II list element
                    BSMpartIIExtension_t part_ii_element = *message->value.choice.BasicSafetyMessage.partII->list.array[i];

                    // Initialize the outputted Part II object
                    j2735_v2x_msgs::msg::BSMPartIIExtension part_ii_output;

                    // Populate part_ii_output fields for extension type VehicleSafetyExtensions
                    if(part_ii_element.partII_Value.present == BSMpartIIExtension__partII_Value_PR_VehicleSafetyExtensions){
                        part_ii_output.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT;

                        // events
                        if(part_ii_element.partII_Value.choice.VehicleSafetyExtensions.events){
                            part_ii_output.vehicle_safety_extensions.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_EVENTS;

                            // Decode BIT STRING
                            part_ii_output.vehicle_safety_extensions.events.vehicle_event_flag |= part_ii_element.partII_Value.choice.VehicleSafetyExtensions.events->buf[0];
                            for(int j = 1; j < part_ii_element.partII_Value.choice.VehicleSafetyExtensions.events->size; j++){
                                part_ii_output.vehicle_safety_extensions.events.vehicle_event_flag |= (part_ii_element.partII_Value.choice.VehicleSafetyExtensions.events->buf[j] << j);
                            }
                        }

                        // pathHistory
                        if(part_ii_element.partII_Value.choice.VehicleSafetyExtensions.pathHistory){
                            part_ii_output.vehicle_safety_extensions.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY;

                            PathHistory_t binary_path_history = *part_ii_element.partII_Value.choice.VehicleSafetyExtensions.pathHistory;
                            part_ii_output.vehicle_safety_extensions.path_history = decode_path_history_message(binary_path_history);
                        }

                        // pathPrediction
                        if(part_ii_element.partII_Value.choice.VehicleSafetyExtensions.pathPrediction){
                            part_ii_output.vehicle_safety_extensions.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION;
                            part_ii_output.vehicle_safety_extensions.path_prediction.radius_of_curvature = part_ii_element.partII_Value.choice.VehicleSafetyExtensions.pathPrediction->radiusOfCurve;
                            part_ii_output.vehicle_safety_extensions.path_prediction.confidence = part_ii_element.partII_Value.choice.VehicleSafetyExtensions.pathPrediction->confidence;
                        }

                        // lights
                        if(part_ii_element.partII_Value.choice.VehicleSafetyExtensions.lights){
                            part_ii_output.vehicle_safety_extensions.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_LIGHTS;

                            // Decode BIT STRING
                            part_ii_output.vehicle_safety_extensions.lights.exterior_lights |= part_ii_element.partII_Value.choice.VehicleSafetyExtensions.lights->buf[0];
                            for(int j = 1; j < part_ii_element.partII_Value.choice.VehicleSafetyExtensions.lights->size; j++){
                                part_ii_output.vehicle_safety_extensions.lights.exterior_lights |= (part_ii_element.partII_Value.choice.VehicleSafetyExtensions.lights->buf[j] << j);
                            }
                        }
                    }

                    // Populate part_ii_output fields for extension type SpecialVehicleExtensions
                    else if(part_ii_element.partII_Value.present == BSMpartIIExtension__partII_Value_PR_SpecialVehicleExtensions){
                        part_ii_output.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;

                        // vehicleAlerts
                        if (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts){
                            part_ii_output.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;

                            // Set the non-optional enums:
                            part_ii_output.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->sirenUse;
                            part_ii_output.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->lightsUse;
                            part_ii_output.special_vehicle_extensions.vehicle_alerts.multi.multi_vehicle_response = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->multi;

                            // vehicleAlerts.events
                            if (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->events){
                                part_ii_output.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_EVENTS;

                                // Decode BIT STRING
                                part_ii_output.special_vehicle_extensions.vehicle_alerts.events.event.privileged_event_flags |= part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->events->event.buf[0];
                                for(int j = 1; j < part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->events->event.size; j++){
                                    part_ii_output.special_vehicle_extensions.vehicle_alerts.events.event.privileged_event_flags |= (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->events->event.buf[j] << j);
                                }
                            }

                            // vehicleAlerts.responseType
                            if (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->responseType){
                                part_ii_output.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_RESPONSE_TYPE;
                                part_ii_output.special_vehicle_extensions.vehicle_alerts.response_type.response_type = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts->responseType;
                            }
                        }

                        // description
                        if (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description){
                            part_ii_output.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION;

                            // Decode SpecialVehicleExtensions.description required fields
                            part_ii_output.special_vehicle_extensions.description.type_event.code = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->typeEvent;
                            
                            // description.description
                            if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->description){
                                part_ii_output.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_DESCRIPTION;
                                for(size_t j = 0; j < part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->description->list.count; j++)
                                {
                                    if(j > j2735_v2x_msgs::msg::EventDescription::DESCRIPTION_SIZE_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded EventDescription list size is greater than max. Rejecting list element.");
                                        break;
                                    }

                                    j2735_v2x_msgs::msg::ITIScodes itis_code;
                                    itis_code.code = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->description->list.array[j];
                                    part_ii_output.special_vehicle_extensions.description.description.push_back(itis_code);
                                }
                            }

                            // description.priority
                            if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->priority){
                                part_ii_output.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_PRIORITY;
                                for(size_t j = 0; j < part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->priority->size; j++)
                                {
                                    part_ii_output.special_vehicle_extensions.description.priority.priority[j] = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->priority->buf[j];
                                }
                            }

                            // description.heading
                            if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->heading){
                                part_ii_output.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_HEADING;

                                // Decode BIT STRING
                                part_ii_output.special_vehicle_extensions.description.heading.heading_slice |= part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->heading->buf[0];
                                for(int j = 1; j < part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->heading->size; j++){
                                    part_ii_output.special_vehicle_extensions.description.heading.heading_slice |= (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->heading->buf[j] << j);
                                }
                            }

                            // description.extent
                            if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->extent){
                                part_ii_output.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_EXTENT;
                                part_ii_output.special_vehicle_extensions.description.extent.extent_value = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.description->extent;
                            }                      
                        }

                        // trailers 
                        if (part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers){
                            part_ii_output.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS;

                            // part_ii_output.special_vehicle_extensions.trailers.connection 
                            part_ii_output.special_vehicle_extensions.trailers.connection = decode_pivot_point_description(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->connection);

                            // trailers.units list
                            for (size_t j = 0; j < part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.count; j++){
                                if(j > j2735_v2x_msgs::msg::TrailerUnitDescriptionList::MAX_SIZE){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded TrailerUnitDescriptionList size is greater than max. Rejecting list element.");
                                    break;
                                }

                                j2735_v2x_msgs::msg::TrailerUnitDescription trailer_unit_description;

                                // isDolly
                                trailer_unit_description.is_dolly.is_dolly = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->isDolly;

                                // width
                                long vehicle_width = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->width;
                                if(vehicle_width < j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle width value less than min, setting to min");
                                    vehicle_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MIN;
                                }
                                else if(vehicle_width > j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle width value greater than max, setting to max");
                                    vehicle_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX;
                                }
                                trailer_unit_description.width.vehicle_width = vehicle_width;

                                // length
                                long vehicle_length = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->length;
                                if(vehicle_length < j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle length value less than min, setting to min");
                                    vehicle_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MIN;
                                }
                                else if(vehicle_length > j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle length value greater than max, setting to max");
                                    vehicle_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX;
                                }
                                trailer_unit_description.length.vehicle_length = vehicle_length;

                                // height
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->height){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT;

                                    long vehicle_height = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->height;
                                    if(vehicle_height < j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle height value less than min, setting to min");
                                        vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN;
                                    }
                                    else if(vehicle_height > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle height value greater than max, setting to max");
                                        vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                                    }
                                    trailer_unit_description.height.vehicle_height = vehicle_height;
                                }

                                // mass
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->mass){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS;

                                    long trailer_mass = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->mass;
                                    if(trailer_mass < j2735_v2x_msgs::msg::TrailerMass::TRAILER_MASS_MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer mass value less than min, setting to min");
                                        trailer_mass = j2735_v2x_msgs::msg::TrailerMass::TRAILER_MASS_MIN;
                                    }
                                    else if(trailer_mass > j2735_v2x_msgs::msg::TrailerMass::TRAILER_MASS_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer mass value greater than max, setting to max");
                                        trailer_mass = j2735_v2x_msgs::msg::TrailerMass::TRAILER_MASS_MAX;
                                    }
                                    trailer_unit_description.mass.trailer_mass = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->mass;
                                }

                                // bumperHeights
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->bumperHeights){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS;

                                    uint8_t front_bumper_height = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->bumperHeights->front;
                                    if(front_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded front bumper height value greater than max, setting to max");
                                        front_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                                    }      
                                    trailer_unit_description.bumper_heights.front.bumper_height = front_bumper_height;

                                    uint8_t rear_bumper_height = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->bumperHeights->rear;
                                    if(rear_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded rear bumper height value greater than max, setting to max");
                                        rear_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                                    }  
                                    trailer_unit_description.bumper_heights.rear.bumper_height = rear_bumper_height;
                                }

                                // centerOfGravity
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->centerOfGravity){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY;

                                    long center_of_gravity = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->centerOfGravity;
                                    if(center_of_gravity < j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded center of gravity value less than min, setting to min");
                                        center_of_gravity = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN;
                                    }
                                    else if(center_of_gravity > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded center of gravity value greater than max, setting to max");
                                        center_of_gravity = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                                    }
                                    trailer_unit_description.center_of_gravity.vehicle_height = center_of_gravity;
                                }

                                // frontPivot
                                trailer_unit_description.front_pivot = decode_pivot_point_description(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->frontPivot);

                                // rearPivot
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->rearPivot){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT;
                                    trailer_unit_description.rear_pivot = decode_pivot_point_description(*part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->rearPivot);
                                }

                                // rearWheelOffset
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->rearWheelOffset){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET;
                                    long rear_wheel_offset = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->rearWheelOffset;
                                    if(rear_wheel_offset < j2735_v2x_msgs::msg::OffsetB12::OFFSET_MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded rear wheel offset value less than min, setting to min");
                                        rear_wheel_offset = j2735_v2x_msgs::msg::OffsetB12::OFFSET_MIN;
                                    }
                                    else if(rear_wheel_offset > j2735_v2x_msgs::msg::OffsetB12::OFFSET_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded rear wheel offset value greater than max, setting to max");
                                        rear_wheel_offset = j2735_v2x_msgs::msg::OffsetB12::OFFSET_MAX;
                                    }
                                    trailer_unit_description.rear_wheel_offset.offset = rear_wheel_offset;
                                }

                                // positionOffset.x
                                long position_offset_x = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->positionOffset.x;
                                if(position_offset_x < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer units position x-offset value less than min, setting to min");
                                    position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                                }
                                else if(position_offset_x > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer units position x-offset value greater than max, setting to max");
                                    position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                                }
                                trailer_unit_description.position_offset.x = position_offset_x;

                                // positionOffset.x
                                long position_offset_y = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->positionOffset.y;
                                if(position_offset_y < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer units position y-offset value less than min, setting to min");
                                    position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                                }
                                else if(position_offset_y > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer units position y-offset value greater than max, setting to max");
                                    position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                                }
                                trailer_unit_description.position_offset.y = position_offset_y;

                                // elevationOffset
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->elevationOffset){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET;
                                    long elevation_offset = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->elevationOffset;
                                    if(elevation_offset < j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded elevation offset value less than min, setting to min");
                                        elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN;
                                    }
                                    else if(elevation_offset > j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded elevation offset value greater than max, setting to max");
                                        elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX;
                                    }
                                    trailer_unit_description.elevation_offset.offset = elevation_offset;
                                }

                                // crumbData
                                if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData){
                                    trailer_unit_description.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA;

                                    for(size_t k = 0; k < part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.count; k++){
                                        if(k > j2735_v2x_msgs::msg::TrailerHistoryPointList::MAX_SIZE){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Decoded TrailerHistoryPointList size is greater than max. Rejecting list element.");
                                            break;
                                        }
                                        
                                        j2735_v2x_msgs::msg::TrailerHistoryPoint trailer_history_point;

                                        // pivotAngle
                                        long pivot_angle = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->pivotAngle;
                                        if(pivot_angle < j2735_v2x_msgs::msg::Angle::ANGLE_MIN){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded pivot angle value less than min, setting to min");
                                            pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MIN;
                                        }
                                        else if(pivot_angle > j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                                            // Note: Value checked against 'ANGLE_UNAVAILABLE' since this value is larger than 'ANGLE_MAX'
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded pivot angle value greater than max, setting to max");
                                            pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MAX;
                                        }
                                        trailer_history_point.pivot_angle.angle = pivot_angle;

                                        // timeOffset
                                        long time_offset = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->timeOffset;
                                        if(time_offset < j2735_v2x_msgs::msg::TimeOffset::MIN){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded time offset value less than min, setting to min");
                                            time_offset = j2735_v2x_msgs::msg::TimeOffset::MIN;
                                        }
                                        else if(time_offset > j2735_v2x_msgs::msg::TimeOffset::MAX){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded time offset value greater than max, setting to max");
                                            time_offset = j2735_v2x_msgs::msg::TimeOffset::MAX;
                                        }
                                        trailer_history_point.time_offset.offset = time_offset;

                                        // positionOffset
                                        long point_position_offset_x = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->positionOffset.x;
                                        if(point_position_offset_x < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer history point x-offset value less than min, setting to min");
                                            point_position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                                        }
                                        else if(point_position_offset_x > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer history point x-offset value greater than max, setting to max");
                                            point_position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                                        }
                                        trailer_history_point.position_offset.x = point_position_offset_x;

                                        long point_position_offset_y = part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->positionOffset.y;
                                        if(point_position_offset_y < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer history point y-offset value less than min, setting to min");
                                            point_position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                                        }
                                        else if(point_position_offset_y > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer history point y-offset value greater than max, setting to max");
                                            point_position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                                        }
                                        trailer_history_point.position_offset.y = point_position_offset_y;

                                        // elevationOffset
                                        if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->elevationOffset){
                                            trailer_history_point.presence_vector |= j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET;

                                            long elevation_offset = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->elevationOffset;
                                            if(elevation_offset < j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN){
                                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded elevation offset value less than min, setting to min");
                                                elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN;
                                            }
                                            else if(elevation_offset > j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX){
                                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded elevation offset value greater than max, setting to max");
                                                elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX;
                                            }
                                            trailer_history_point.elevation_offset.offset = 50;
                                        }

                                        // heading
                                        if(part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->heading){
                                            trailer_history_point.presence_vector |= j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING;

                                            long heading = *part_ii_element.partII_Value.choice.SpecialVehicleExtensions.trailers->units.list.array[j]->crumbData->list.array[k]->heading;
                                            if(heading < j2735_v2x_msgs::msg::CoarseHeading::MIN){
                                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded heading value less than min, setting to min");
                                                heading = j2735_v2x_msgs::msg::CoarseHeading::MIN;
                                            }
                                            else if(heading > j2735_v2x_msgs::msg::CoarseHeading::MAX){
                                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded heading value greater than max, setting to max");
                                                heading = j2735_v2x_msgs::msg::CoarseHeading::MAX;
                                            }
                                            trailer_history_point.heading.heading = 83;                  
                                        }

                                        trailer_unit_description.crumb_data.trailer_history_points.push_back(trailer_history_point);
                                    }
                                }
                                part_ii_output.special_vehicle_extensions.trailers.units.trailer_unit_descriptions.push_back(trailer_unit_description);
                            }
                        }

                    }

                    // Populate part_ii_output fields for extension type SupplementalVehicleExtensions
                    else if(part_ii_element.partII_Value.present == BSMpartIIExtension__partII_Value_PR_SupplementalVehicleExtensions){
                        part_ii_output.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT;
                        
                        // classification
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classification){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION;
                            part_ii_output.supplemental_vehicle_extensions.classification.basic_vehicle_class = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classification;
                        }

                        // classDetails
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS;

                            // classDetails.keyType
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->keyType){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_KEY_TYPE;
                                part_ii_output.supplemental_vehicle_extensions.class_details.key_type.basic_vehicle_class = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->keyType;
                            }
                            // classDetails.role
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->role){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ROLE;
                                part_ii_output.supplemental_vehicle_extensions.class_details.role.basic_vehicle_role = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->role;
                            }
                            // classDetails.iso3883
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->iso3883){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ISO;
                                part_ii_output.supplemental_vehicle_extensions.class_details.iso3833 = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->iso3883;
                            }
                            // classDetails.hpmsType
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->hpmsType){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_HPMS_TYPE;
                                part_ii_output.supplemental_vehicle_extensions.class_details.hpms_type.vehicle_type = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->hpmsType;
                            }
                            // classDetails.vehicleType
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->vehicleType){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_VEHICLE_TYPE;
                                part_ii_output.supplemental_vehicle_extensions.class_details.vehicle_type.vehicle_group_affected = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->vehicleType;
                            }
                            // classDetails.responseEquip
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->responseEquip){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONSE_EQUIP;
                                part_ii_output.supplemental_vehicle_extensions.class_details.response_equip.incident_response_equipment = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->responseEquip;
                            }
                            // classDetails.responderType
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->responderType){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONDER_TYPE;
                                part_ii_output.supplemental_vehicle_extensions.class_details.responder_type.responder_group_affected = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->responderType;
                            }
                            // classDetails.fuelType
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->fuelType){
                                part_ii_output.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_FUEL_TYPE;
                                part_ii_output.supplemental_vehicle_extensions.class_details.fuel_type.fuel_type = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.classDetails->fuelType;
                            }
                        }

                        // vehicleData
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA;

                            // vehicleData.height
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->height){
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_HEIGHT;

                                uint8_t height = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->height;
                                if(height > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded vehicle height value greater than max, setting to max");
                                    height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                                }
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.height.vehicle_height = height;
                            }

                            // vehicleData.bumpers
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->bumpers){
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_BUMPERS;

                                uint8_t front_bumper_height = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->bumpers->front;
                                if(front_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded front bumper height value greater than max, setting to max");
                                    front_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                                }    

                                uint8_t rear_bumper_height = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->bumpers->rear;
                                if(rear_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded rear bumper height value greater than max, setting to max");
                                    rear_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                                }    

                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.bumpers.front.bumper_height = front_bumper_height;
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.bumpers.rear.bumper_height = rear_bumper_height;
                            }

                            // vehicleData.mass
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->mass){
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_MASS;
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.mass.vehicle_mass = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->mass;
                            }

                            // vehicleData.trailerWeight
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->trailerWeight){
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT;

                                uint16_t trailer_weight = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.vehicleData->trailerWeight;
                                if(trailer_weight > j2735_v2x_msgs::msg::TrailerWeight::TRAILER_WEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer weight value greater than max, setting to max");
                                    trailer_weight = j2735_v2x_msgs::msg::TrailerWeight::TRAILER_WEIGHT_MAX;
                                }
                                part_ii_output.supplemental_vehicle_extensions.vehicle_data.trailer_weight.trailer_weight = trailer_weight;
                            }
                        }

                        // weatherReport
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT;

                            part_ii_output.supplemental_vehicle_extensions.weather_report.is_raining.precip_yes_no = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->isRaining;

                            // weatherReport.rainRate
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->rainRate){
                                part_ii_output.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE;
                                part_ii_output.supplemental_vehicle_extensions.weather_report.rain_rate.precip_rate = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->rainRate;
                            }
                            // weatherReport.precipSituation
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->precipSituation){
                                part_ii_output.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION;
                                part_ii_output.supplemental_vehicle_extensions.weather_report.precip_situation.ess_precip_situation = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->precipSituation;
                            }
                            // weatherReport.solarRadiation
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->solarRadiation){
                                part_ii_output.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION;
                                part_ii_output.supplemental_vehicle_extensions.weather_report.solar_radiation.ess_solar_radiation = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->solarRadiation;
                            }
                            // weatherReport.friction
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->friction){
                                part_ii_output.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_FRICTION;
                                part_ii_output.supplemental_vehicle_extensions.weather_report.friction.ess_mobile_friction = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->friction;
                            }
                            // weatherReport.roadFriction
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->roadFriction){
                                part_ii_output.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION;

                                uint8_t road_friction = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherReport->roadFriction;
                                if(road_friction > j2735_v2x_msgs::msg::CoefficientOfFriction::COEFFICIENT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded road friction value greater than max, setting to max");
                                    road_friction = j2735_v2x_msgs::msg::CoefficientOfFriction::COEFFICIENT_MAX;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.weather_report.road_friction.coefficient = road_friction;
                            }
                        }

                        // weatherProbe
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE;

                            // weatherReport.airTemp
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->airTemp){
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP;
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.air_temp.temperature = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->airTemp;
                            }

                            // weatherReport.airPressure
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->airPressure){
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE;
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.air_pressure.pressure = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->airPressure;
                            }

                            // weatherReport.rainRates
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates){
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES;
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.rain_rates.status_front.wiper_status = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates->statusFront;
                                part_ii_output.supplemental_vehicle_extensions.weather_probe.rain_rates.rate_front.wiper_rate = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates->rateFront;

                                if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates->statusRear){
                                    part_ii_output.supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR;
                                    part_ii_output.supplemental_vehicle_extensions.weather_probe.rain_rates.status_rear.wiper_status = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates->statusRear;
                                }

                                if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates->rateRear){
                                    part_ii_output.supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_RATE_REAR;
                                    part_ii_output.supplemental_vehicle_extensions.weather_probe.rain_rates.rate_rear.wiper_rate = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.weatherProbe->rainRates->rateRear;
                                }
                            }
                        }

                        // obstacleDetection
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE;

                            // obDist
                            part_ii_output.supplemental_vehicle_extensions.obstacle.ob_dist.distance = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->obDist;

                            // obDirect
                            long direction_angle = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->obDirect;
                            if(direction_angle < j2735_v2x_msgs::msg::Angle::ANGLE_MIN){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded object direction angle value less than min, setting to min");
                                direction_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MIN;
                            }
                            else if(direction_angle > j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                                // Note: Value checked against 'ANGLE_UNAVAILABLE' since this value is larger than 'ANGLE_MAX'
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded object direction angle value greater than max, setting to max");
                                direction_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MAX;
                            }
                            part_ii_output.supplemental_vehicle_extensions.obstacle.ob_direct.direction.angle = direction_angle;

                            // dateTime.year
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.year){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
                                uint16_t year = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.year;
                                if(year > j2735_v2x_msgs::msg::DYear::MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded year value greater than max, setting to max");
                                    year = j2735_v2x_msgs::msg::DYear::MAX;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.year.year = year;
                            }
                            // dateTime.month
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.month){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
                                uint8_t month = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.month;
                                if(month > j2735_v2x_msgs::msg::DMonth::MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded month value greater than max, setting to max");
                                    month = j2735_v2x_msgs::msg::DMonth::MAX;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.month.month = month;
                            }
                            // dateTime.day
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.day){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
                                uint8_t day = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.day;
                                if(day > j2735_v2x_msgs::msg::DDay::MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded day value greater than max, setting to max");
                                    day = j2735_v2x_msgs::msg::DDay::MAX;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.day.day = day;
                            }
                            // dateTime.hour
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.hour){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
                                uint8_t hour = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.hour;
                                if(hour > j2735_v2x_msgs::msg::DHour::UNAVAILABLE){
                                    // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'HOUR_OF_DAY_MAX'
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded hour value greater than max, setting to max");
                                    hour = j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.hour.hour = hour;
                            }
                            // dateTime.minute
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.minute){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
                                uint8_t minute = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.minute;
                                if(minute > j2735_v2x_msgs::msg::DMinute::UNAVAILABLE){
                                    // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'MINUTE_IN_HOUR_MAX'
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded minute value greater than max, setting to max");
                                    minute = j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.minute.minute = minute;
                            }
                            // dateTime.second
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.second){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
                                uint16_t second = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.second;
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.second.millisecond = second;
                            }
                            // dateTime.offset
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.offset){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
                                long offset = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->dateTime.offset;
                                if(offset > j2735_v2x_msgs::msg::DOffset::MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded offset value greater than max, setting to max");
                                    offset = j2735_v2x_msgs::msg::DOffset::MAX;   
                                }
                                else if(offset < j2735_v2x_msgs::msg::DOffset::MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded offset value less than min, setting to min");
                                    offset = j2735_v2x_msgs::msg::DOffset::MIN;   
                                }
                                part_ii_output.supplemental_vehicle_extensions.obstacle.date_time.offset.offset_minute = offset;
                            }

                            // description
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->description){
                                int description_code = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->description;
                                if(description_code > j2735_v2x_msgs::msg::ObstacleDetection::DESCRIPTION_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded obstacle description code greater than max, this field will not be added to output");
                                }
                                else if(description_code < j2735_v2x_msgs::msg::ObstacleDetection::DESCRIPTION_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded obstacle description code less than min, this field will not be added to output");
                                }
                                else{
                                    part_ii_output.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION;
                                    part_ii_output.supplemental_vehicle_extensions.obstacle.description.code = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->description;
                                }                                
                            }

                            // locationDetails
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->locationDetails){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS;
                                part_ii_output.supplemental_vehicle_extensions.obstacle.location_details.generic_locations = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->locationDetails;
                            }

                            // vertEvent
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->vertEvent){
                                part_ii_output.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT;

                                part_ii_output.supplemental_vehicle_extensions.obstacle.vert_event.exceeded_wheels |= part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->vertEvent->buf[0];
                                for(int j = 1; j < part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->vertEvent->size; j++){
                                    part_ii_output.supplemental_vehicle_extensions.obstacle.vert_event.exceeded_wheels |= (part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.obstacle->vertEvent->buf[j]);
                                }
                            }
                        }

                        // status
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.status){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS;

                            // statusDetails
                            part_ii_output.supplemental_vehicle_extensions.status.status_details.code = part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.status->statusDetails;

                            // locationDetails
                            if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.status->locationDetails){
                                part_ii_output.supplemental_vehicle_extensions.status.presence_vector |= j2735_v2x_msgs::msg::DisabledVehicle::HAS_LOCATION_DETAILS;
                                part_ii_output.supplemental_vehicle_extensions.status.location_details.generic_locations = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.status->locationDetails;
                            }
                        }

                        // speedProfile
                        if(part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.speedProfile){
                            part_ii_output.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE;

                            for(size_t j = 0; j < part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.speedProfile->speedReports.list.count; j++){
                                long speed = *part_ii_element.partII_Value.choice.SupplementalVehicleExtensions.speedProfile->speedReports.list.array[j];
                                if(speed > j2735_v2x_msgs::msg::GrossSpeed::SPEED_UNAVAILABLE){
                                    // Note: Value checked against 'SPEED_UNAVAILABLE' since this value is larger than 'SPEED_MAX'
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded speed value greater than max, setting to max");
                                    speed = j2735_v2x_msgs::msg::GrossSpeed::SPEED_MAX;
                                }
                                else if(speed < j2735_v2x_msgs::msg::GrossSpeed::SPEED_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded speed value less than min, setting to min");
                                    speed = j2735_v2x_msgs::msg::GrossSpeed::SPEED_MIN;
                                }
                                j2735_v2x_msgs::msg::GrossSpeed speed_point;
                                speed_point.speed = speed;

                                part_ii_output.supplemental_vehicle_extensions.speed_profile.push_back(speed_point);
                            }
                        }
                    }
                    
                    // Add part_ii_output to output's part_ii list
                    output.part_ii.push_back(part_ii_output);
                }
            }

            // Decode regional list
            if (message->value.choice.BasicSafetyMessage.regional){
                // TODO
            }
            
            ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
            return boost::optional<j2735_v2x_msgs::msg::BSM>(output);
        }
        RCLCPP_WARN_STREAM( node_logging_->get_logger(), "BasicSafetyMessage decoding failed");
        ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
        return boost::optional<j2735_v2x_msgs::msg::BSM>{};

    }
    
    boost::optional<std::vector<uint8_t>> BSM_Message::encode_bsm_message(const j2735_v2x_msgs::msg::BSM& plain_msg)
    {
        //Uncomment below (and one line at the end of the function) to print the message in human readable form
        //FILE *fp;
        //fp = fopen("encoded-bsm-output.txt", "w");
        //fprintf(fp, "test encodeBSM function is called\n");

        std::vector<std::shared_ptr<void>> shared_ptrs; // keep references to the objects until the encoding is complete
        
        //encode result placeholder
        uint8_t buffer[544];
        size_t buffer_size=sizeof(buffer);
        asn_enc_rval_t ec;
        MessageFrame_t *message = create_store_shared<MessageFrame_t>(shared_ptrs);

        //set message type to BasicSafetyMessage
        message->messageId = 20;  
        message->value.present = MessageFrame__value_PR_BasicSafetyMessage;

        // BSMcoreData
        BSMcoreData_t core_data;
        core_data.msgCnt = plain_msg.core_data.msg_count;
        //Set the fields
        uint8_t id_content[4] = {0};
        for(auto i = 0; i < 4; i++)
        {
            id_content[i] = (char) plain_msg.core_data.id[i];
        }
        TemporaryID_t temp_id;
        temp_id.buf = id_content; 
        temp_id.size = 4;
        core_data.id = temp_id;
        core_data.secMark = plain_msg.core_data.sec_mark;

        core_data.lat = plain_msg.core_data.latitude;
        core_data.Long = plain_msg.core_data.longitude;
        core_data.elev = plain_msg.core_data.elev;
        PositionalAccuracy_t pos_acc;
        pos_acc.orientation = plain_msg.core_data.accuracy.orientation;
        pos_acc.semiMajor = plain_msg.core_data.accuracy.semi_major;
        pos_acc.semiMinor = plain_msg.core_data.accuracy.semi_minor;
        core_data.accuracy = pos_acc;
        core_data.transmission = plain_msg.core_data.transmission.transmission_state;
        core_data.speed = plain_msg.core_data.speed;
        core_data.heading = plain_msg.core_data.heading;
        core_data.angle = plain_msg.core_data.angle;
        AccelerationSet4Way_t accel;
        accel.lat = plain_msg.core_data.accel_set.lateral;
        accel.Long = plain_msg.core_data.accel_set.longitudinal;
        accel.vert= plain_msg.core_data.accel_set.vert;
        accel.yaw = plain_msg.core_data.accel_set.yaw_rate;
        core_data.accelSet = accel;
        VehicleSize_t vehicle_size;
        vehicle_size.length = plain_msg.core_data.size.vehicle_length;
        vehicle_size.width = plain_msg.core_data.size.vehicle_width;
        core_data.size = vehicle_size;
        BrakeSystemStatus_t brakes;
    
        brakes.traction = plain_msg.core_data.brakes.traction.traction_control_status;
        brakes.abs = plain_msg.core_data.brakes.abs.anti_lock_brake_status;
        brakes.scs = plain_msg.core_data.brakes.scs.stability_control_status;
        brakes.brakeBoost = plain_msg.core_data.brakes.brake_boost.brake_boost_applied;
        brakes.auxBrakes = plain_msg.core_data.brakes.aux_brakes.auxiliary_brake_status;
        
        BrakeAppliedStatus_t brake_applied_status;
        
        uint8_t wheel_brake[1] = {8}; // dummy 8 value

        // there are 3 unused bits in the end: 0b000
        // which makes every possible encoded value to be multiples of 8: 0b00001000 (8), 0b00010000 (16), 0b00011000 (24) etc
        // so num in brackets indicate the position in the bit string:
        // unavailable: 0b10000000, leftFront: 0b01000000 etc
        wheel_brake[0] = (char) (8 << (4 - plain_msg.core_data.brakes.wheel_brakes.brake_applied_status)); 
        brake_applied_status.buf = wheel_brake;
        brake_applied_status.size = 1;
        brake_applied_status.bits_unused = 3;
        brakes.wheelBrakes = brake_applied_status;
        
        core_data.brakes = brakes;

        message->value.choice.BasicSafetyMessage.coreData = core_data;

        // Encode Part II Content
        if(plain_msg.presence_vector & j2735_v2x_msgs::msg::BSM::HAS_PART_II){

            // Initialize part_ii_list that elements will be added to
            auto part_ii_list = create_store_shared<BasicSafetyMessage_t::BasicSafetyMessage__partII>(shared_ptrs);

            // Encode each Part II list element
            for(size_t i = 0 ; i < plain_msg.part_ii.size(); ++i){
            
                if(i > j2735_v2x_msgs::msg::BSM::PART_II_MAX_SIZE){
                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoded Part II list size is greater than max. Rejecting list element.");
                    break;
                }

                auto part_ii_element = create_store_shared<BSMpartIIExtension_t>(shared_ptrs);
                
                // Encode Part II element of type VehicleSafetyExtensions
                if(plain_msg.part_ii[i].part_ii_id == j2735_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT){
                    j2735_v2x_msgs::msg::VehicleSafetyExtensions vehicle_safety_ext_msg = plain_msg.part_ii[i].vehicle_safety_extensions;

                    part_ii_element->partII_Id = PartII_Id_vehicleSafetyExt;
                    part_ii_element->partII_Value.present = BSMpartIIExtension__partII_Value_PR_VehicleSafetyExtensions;

                    // events
                    if(vehicle_safety_ext_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_EVENTS){
                        auto output_event = create_store_shared<VehicleEventFlags_t>(shared_ptrs);

                        std::string event_str = std::to_string(vehicle_safety_ext_msg.events.vehicle_event_flag);
                        size_t size = event_str.size();

                        auto array = create_store_shared_array<uint8_t>(shared_ptrs, size);
                        for(size_t j = 0; j < size; j++){
                            array[j] = event_str[j] - '0';
                        }
                        output_event->size = size;
                        output_event->buf = array;

                        part_ii_element->partII_Value.choice.VehicleSafetyExtensions.events = output_event;
                    }

                    // path_history
                    if(vehicle_safety_ext_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY){
                        auto path_history = create_store_shared<PathHistory_t>(shared_ptrs);
            
                        // path_history.crumbData
                        for(size_t j = 0 ; j < vehicle_safety_ext_msg.path_history.crumb_data.points.size(); ++j){
                            auto point = create_store_shared<PathHistoryPoint_t>(shared_ptrs);
                            
                            // offsets
                            point->latOffset = vehicle_safety_ext_msg.path_history.crumb_data.points[j].lat_offset.offset;
                            point->lonOffset = vehicle_safety_ext_msg.path_history.crumb_data.points[j].lon_offset.offset;
                            point->elevationOffset = vehicle_safety_ext_msg.path_history.crumb_data.points[j].elevation_offset.offset;
                            point->timeOffset = vehicle_safety_ext_msg.path_history.crumb_data.points[j].time_offset.offset;
                            
                            // speed
                            auto speed = create_store_shared<Speed_t>(shared_ptrs);
                            *speed = vehicle_safety_ext_msg.path_history.crumb_data.points[j].speed.speed;
                            point->speed = speed;

                            // posAccuracy
                            auto positional_accuracy = create_store_shared<PositionalAccuracy_t>(shared_ptrs);
                            
                            positional_accuracy->semiMajor = vehicle_safety_ext_msg.path_history.crumb_data.points[j].pos_accuracy.semi_major;
                            positional_accuracy->semiMinor = vehicle_safety_ext_msg.path_history.crumb_data.points[j].pos_accuracy.semi_minor;
                            positional_accuracy->orientation = vehicle_safety_ext_msg.path_history.crumb_data.points[j].pos_accuracy.orientation;

                            point->posAccuracy = positional_accuracy;
                                
                            // heading
                            auto heading = create_store_shared<CoarseHeading_t>(shared_ptrs);
                            *heading = vehicle_safety_ext_msg.path_history.crumb_data.points[j].heading.heading;
                            point->heading = heading; 
                            
                            asn_sequence_add(&path_history->crumbData.list, point);
                        }
                        
                        // path_history.currGNSSstatus
                        if(vehicle_safety_ext_msg.path_history.presence_vector & j2735_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS){
                            auto output_gnss_status = create_store_shared<GNSSstatus_t>(shared_ptrs);

                            std::string gnss_status_str = std::to_string(vehicle_safety_ext_msg.path_history.curr_gnss_status.statuses);
                            size_t size = gnss_status_str.size();

                            auto array = create_store_shared_array<uint8_t>(shared_ptrs, size);
                            for(size_t j = 0; j< size; j++){
                                array[j] = gnss_status_str[j] - '0';
                            }
                            output_gnss_status->size = size;
                            output_gnss_status->buf = array;

                            path_history->currGNSSstatus = output_gnss_status;
                        }
                        
                        // path_history.initial_position
                        if(vehicle_safety_ext_msg.path_history.presence_vector & j2735_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION){
                            auto initial_position = create_store_shared<FullPositionVector_t>(shared_ptrs);

                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME){                               

                                // utcTime
                                auto utc_time = create_store_shared<DDateTime_t>(shared_ptrs);

                                // utcTime.year
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::YEAR){
                                    auto year_ptr = create_store_shared<DYear_t>(shared_ptrs);

                                    uint16_t year = vehicle_safety_ext_msg.path_history.initial_position.utc_time.year.year;
                                    if(year > j2735_v2x_msgs::msg::DYear::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded year value greater than max, setting to max");
                                        year = j2735_v2x_msgs::msg::DYear::MAX;   
                                    }
                                    *year_ptr = year;

                                    utc_time->year = year_ptr;                           
                                }

                                // utcTime.month
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MONTH){
                                    auto month_ptr = create_store_shared<DMonth_t>(shared_ptrs);

                                    uint16_t month = vehicle_safety_ext_msg.path_history.initial_position.utc_time.month.month;
                                    if(month > j2735_v2x_msgs::msg::DMonth::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded month value greater than max, setting to max");
                                        month = j2735_v2x_msgs::msg::DMonth::MAX;   
                                    }
                                    *month_ptr = month;

                                    utc_time->month = month_ptr;                           
                                }

                                // utcTime.day
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::DAY){
                                    auto day_ptr = create_store_shared<DDay_t>(shared_ptrs);

                                    uint16_t day = vehicle_safety_ext_msg.path_history.initial_position.utc_time.day.day;
                                    if(day > j2735_v2x_msgs::msg::DDay::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded day value greater than max, setting to max");
                                        day = j2735_v2x_msgs::msg::DDay::MAX;   
                                    }
                                    *day_ptr = day;

                                    utc_time->day = day_ptr;                           
                                }
                                
                                // utcTime.hour
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::HOUR){
                                    auto hour_ptr = create_store_shared<DHour_t>(shared_ptrs);

                                    uint8_t hour = vehicle_safety_ext_msg.path_history.initial_position.utc_time.hour.hour;
                                    if(hour > j2735_v2x_msgs::msg::DHour::UNAVAILABLE){
                                        // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'HOUR_OF_DAY_MAX'
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded hour value greater than max, setting to max");
                                        hour = j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX;   
                                    }
                                    *hour_ptr = hour;

                                    utc_time->hour = hour_ptr;                           
                                }

                                // utcTime.minute
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MINUTE){
                                    auto minute_ptr = create_store_shared<DMinute_t>(shared_ptrs);

                                    uint8_t minute = vehicle_safety_ext_msg.path_history.initial_position.utc_time.minute.minute;
                                    if(minute > j2735_v2x_msgs::msg::DMinute::UNAVAILABLE){
                                        // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'MINUTE_IN_HOUR_MAX'
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded minute value greater than max, setting to max");
                                        minute = j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX;   
                                    }
                                    *minute_ptr = minute;

                                    utc_time->minute = minute_ptr;                           
                                }

                                // utcTime.second
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::SECOND){
                                    auto second_ptr = create_store_shared<DSecond_t>(shared_ptrs);
                                    *second_ptr = vehicle_safety_ext_msg.path_history.initial_position.utc_time.second.millisecond;
                                    utc_time->second = second_ptr;                           
                                }

                                // utcTime.offset
                                if(vehicle_safety_ext_msg.path_history.initial_position.utc_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::OFFSET){
                                    auto offset_ptr = create_store_shared<DSecond_t>(shared_ptrs);

                                    long offset = vehicle_safety_ext_msg.path_history.initial_position.utc_time.offset.offset_minute;
                                    if(offset > j2735_v2x_msgs::msg::DOffset::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded millisecond value greater than max, setting to max");
                                        offset = j2735_v2x_msgs::msg::DOffset::MAX;   
                                    }
                                    else if(offset < j2735_v2x_msgs::msg::DOffset::MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded millisecond value less than min, setting to min");
                                        offset = j2735_v2x_msgs::msg::DOffset::MIN;   
                                    }
                                    *offset_ptr = offset;

                                    utc_time->offset = offset_ptr;                           
                                }
                                initial_position->utcTime = utc_time;
                            }
                            
                            // Long
                            auto longitude = create_store_shared<Longitude_t>(shared_ptrs);
                            *longitude = vehicle_safety_ext_msg.path_history.initial_position.lon.longitude;
                            initial_position->Long = *longitude;
                            
                            // lat
                            auto latitude = create_store_shared<Latitude_t>(shared_ptrs);
                            *latitude = vehicle_safety_ext_msg.path_history.initial_position.lat.latitude;
                            initial_position->lat = *latitude;
                            
                            // elevation
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION){
                                auto elevation = create_store_shared<DSRC_Elevation_t>(shared_ptrs);
                                *elevation = vehicle_safety_ext_msg.path_history.initial_position.elevation.elevation;
                                initial_position->elevation = elevation;
                            }
                            
                            // heading
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_HEADING){
                                auto heading = create_store_shared<Heading_t>(shared_ptrs);
                                *heading = vehicle_safety_ext_msg.path_history.initial_position.heading.heading;
                                initial_position->heading = heading;
                            }

                            // speed
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED){
                                TransmissionAndSpeed_t* transmission_and_speed;
                                transmission_and_speed = (TransmissionAndSpeed_t*) calloc(1, sizeof(TransmissionAndSpeed_t));
                                transmission_and_speed->transmisson = vehicle_safety_ext_msg.path_history.initial_position.speed.transmission.transmission_state;
                                transmission_and_speed->speed =  vehicle_safety_ext_msg.path_history.initial_position.speed.speed.velocity;
                                initial_position->speed = transmission_and_speed;
                            }

                            // pos_accuracy
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY){
                                auto pos_accuracy = create_store_shared<PositionalAccuracy_t>(shared_ptrs);
                                pos_accuracy->semiMajor = vehicle_safety_ext_msg.path_history.initial_position.pos_accuracy.semi_major;
                                pos_accuracy->semiMinor = vehicle_safety_ext_msg.path_history.initial_position.pos_accuracy.semi_minor;
                            
                                pos_accuracy->orientation = vehicle_safety_ext_msg.path_history.initial_position.pos_accuracy.orientation;
                                
                                initial_position->posAccuracy = pos_accuracy;
                            }

                            // time_confidence
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE){
                                auto time_confidence = create_store_shared<TimeConfidence_t>(shared_ptrs);
                                time_confidence = (TimeConfidence_t*) calloc(1, sizeof(TimeConfidence_t));
                                *time_confidence = vehicle_safety_ext_msg.path_history.initial_position.time_confidence.confidence;

                                initial_position->timeConfidence = time_confidence;
                            }

                            // pos_confidence
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE){
                                auto position_confidence = create_store_shared<PositionConfidenceSet_t>(shared_ptrs);
                                position_confidence->pos = vehicle_safety_ext_msg.path_history.initial_position.pos_confidence.pos.confidence;
                                position_confidence->elevation = vehicle_safety_ext_msg.path_history.initial_position.pos_confidence.elevation.confidence;

                                initial_position->posConfidence = position_confidence;
                            }

                            // speed_confidence
                            if(vehicle_safety_ext_msg.path_history.initial_position.presence_vector & j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE){
                                auto speed_confidence = create_store_shared<SpeedandHeadingandThrottleConfidence_t>(shared_ptrs);
                                speed_confidence->heading = vehicle_safety_ext_msg.path_history.initial_position.speed_confidence.heading.confidence;
                                speed_confidence->speed = vehicle_safety_ext_msg.path_history.initial_position.speed_confidence.speed.speed_confidence;
                                speed_confidence->throttle = vehicle_safety_ext_msg.path_history.initial_position.speed_confidence.throttle.confidence;

                                initial_position->speedConfidence = speed_confidence;
                            }

                            path_history->initialPosition = initial_position;
                        }

                        part_ii_element->partII_Value.choice.VehicleSafetyExtensions.pathHistory = path_history;
                    }

                    // path_prediction
                    if(vehicle_safety_ext_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION){
                        auto path_prediction = create_store_shared<PathPrediction_t>(shared_ptrs);

                        path_prediction->radiusOfCurve = vehicle_safety_ext_msg.path_prediction.radius_of_curvature;
                        path_prediction->confidence = vehicle_safety_ext_msg.path_prediction.confidence;

                        part_ii_element->partII_Value.choice.VehicleSafetyExtensions.pathPrediction = path_prediction;
                    }

                    // lights
                    if(vehicle_safety_ext_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_LIGHTS){
                        auto output_lights = create_store_shared<VehicleEventFlags_t>(shared_ptrs);

                        std::string lights_str = std::to_string(vehicle_safety_ext_msg.lights.exterior_lights);
                        size_t size = lights_str.size();

                        auto array = create_store_shared_array<uint8_t>(shared_ptrs, size);
                        for(size_t j = 0; j < size; j++){
                            array[j] = lights_str[j] - '0';
                        }
                        output_lights->size = size;
                        output_lights->buf = array;

                        part_ii_element->partII_Value.choice.VehicleSafetyExtensions.lights = output_lights;
                    }
                }

                // Encode Part II element of type SpecialVehicleExtensions
                else if(plain_msg.part_ii[i].part_ii_id == j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT){  
                    j2735_v2x_msgs::msg::SpecialVehicleExtensions special_vehicle_ext_msg = plain_msg.part_ii[i].special_vehicle_extensions;

                    part_ii_element->partII_Id = PartII_Id_specialVehicleExt;
                    part_ii_element->partII_Value.present = BSMpartIIExtension__partII_Value_PR_SpecialVehicleExtensions;

                    // vehicle_alerts
                    if(special_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS){
                        auto vehicle_alerts = create_store_shared<EmergencyDetails_t>(shared_ptrs);

                        vehicle_alerts->sirenUse = special_vehicle_ext_msg.vehicle_alerts.siren_use.siren_in_use;
                        vehicle_alerts->lightsUse = special_vehicle_ext_msg.vehicle_alerts.lights_use.lightbar_in_use;
                        vehicle_alerts->multi = special_vehicle_ext_msg.vehicle_alerts.multi.multi_vehicle_response;
                        
                        // events
                        if(special_vehicle_ext_msg.vehicle_alerts.presence_vector & j2735_v2x_msgs::msg::EmergencyDetails::HAS_EVENTS){
                            // Populate PrivilegedEventFlags bit string
                            auto output_event = create_store_shared<PrivilegedEvents_t>(shared_ptrs);
                            auto output_flag = create_store_shared<PrivilegedEventFlags_t>(shared_ptrs);
                            
                            std::string flag_str = std::to_string(special_vehicle_ext_msg.vehicle_alerts.events.event.privileged_event_flags);
                            size_t size = flag_str.size();

                            auto array = create_store_shared_array<uint8_t>(shared_ptrs, size);
                            for(size_t j = 0; j < size; j++){
                                array[j] = flag_str[j] - '0';
                            }
                            output_flag->size = size;
                            output_flag->buf = array;

                            output_event->event = *output_flag;
                            vehicle_alerts->events = output_event;
                        }

                        // response_type
                        if(special_vehicle_ext_msg.vehicle_alerts.presence_vector & j2735_v2x_msgs::msg::EmergencyDetails::HAS_RESPONSE_TYPE){
                            auto response_type = create_store_shared<ResponseType_t>(shared_ptrs);

                            *response_type = special_vehicle_ext_msg.vehicle_alerts.response_type.response_type;
                            vehicle_alerts->responseType = response_type;
                        }

                        part_ii_element->partII_Value.choice.SpecialVehicleExtensions.vehicleAlerts = vehicle_alerts;
                    }

                    // description
                    if(special_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION){
                        auto description = create_store_shared<EventDescription_t>(shared_ptrs);

                        description->typeEvent = special_vehicle_ext_msg.description.type_event.code; 

                        // description
                        if(special_vehicle_ext_msg.description.presence_vector & j2735_v2x_msgs::msg::EventDescription::HAS_DESCRIPTION){
                            auto description_list = create_store_shared<EventDescription::EventDescription__description>(shared_ptrs);

                            for(size_t j = 0; j < special_vehicle_ext_msg.description.description.size(); j++){
                                if(j > j2735_v2x_msgs::msg::EventDescription::DESCRIPTION_SIZE_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoded EventDescription list size is greater than max. Rejecting list element.");
                                    break;
                                }
                                auto itis_code = create_store_shared<ITIScodes_t>(shared_ptrs);
                                *itis_code = special_vehicle_ext_msg.description.description[j].code;
                                asn_sequence_add(&description_list->list, itis_code);
                            }

                            description->description = description_list;
                        }
                        
                        // priority
                        if(special_vehicle_ext_msg.description.presence_vector & j2735_v2x_msgs::msg::EventDescription::HAS_PRIORITY){
                            size_t size = special_vehicle_ext_msg.description.priority.priority.size();
                            auto output_priority = create_store_shared<Priority_t>(shared_ptrs);

                            auto output_priority_buf = create_store_shared_array<uint8_t>(shared_ptrs, size);
                            for(size_t j = 0; j < size; j++){
                                output_priority_buf[j] = special_vehicle_ext_msg.description.priority.priority[j];
                            }
                            
                            output_priority->size = size;
                            output_priority->buf = output_priority_buf;
                            description->priority = output_priority;
                        }

                        // heading
                        if(special_vehicle_ext_msg.description.presence_vector & j2735_v2x_msgs::msg::EventDescription::HAS_HEADING){
                            auto output_heading_slice = create_store_shared<HeadingSlice_t>(shared_ptrs);

                            std::string heading_slice_str = std::to_string(special_vehicle_ext_msg.description.heading.heading_slice);
                            size_t size = heading_slice_str.size();

                            auto heading_slice_buf = create_store_shared_array<uint8_t>(shared_ptrs, size);
                            for(size_t j = 0; j < size; j++){
                                heading_slice_buf[j] = heading_slice_str[j] - '0';
                            }
                            output_heading_slice->size = size;
                            output_heading_slice->buf = heading_slice_buf;

                            description->heading = output_heading_slice;
                        }

                        // extent
                        if(special_vehicle_ext_msg.description.presence_vector & j2735_v2x_msgs::msg::EventDescription::HAS_EXTENT){
                            auto extent = create_store_shared<Extent_t>(shared_ptrs);

                            *extent = special_vehicle_ext_msg.description.extent.extent_value;
                            description->extent = extent;
                        }

                        part_ii_element->partII_Value.choice.SpecialVehicleExtensions.description = description;
                    }

                    // trailers
                    if(special_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS){
                        auto trailer_data = create_store_shared<TrailerData_t>(shared_ptrs);

                        // connection
                        long pivot_offset = special_vehicle_ext_msg.trailers.connection.pivot_offset.offset; 
                        long pivot_angle = special_vehicle_ext_msg.trailers.connection.pivot_angle.angle;
                        bool pivoting_allowed = special_vehicle_ext_msg.trailers.connection.pivots.pivoting_allowed;
                        PivotPointDescription_t connection = encode_pivot_point_description(pivot_offset, pivot_angle, pivoting_allowed);
                        trailer_data->connection = connection;

                        // TrailerUnitDescriptionList
                        auto trailer_unit_description_list = create_store_shared<TrailerUnitDescriptionList_t>(shared_ptrs);
                        for(size_t j = 0; j < special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions.size(); j++){
                            if(j > j2735_v2x_msgs::msg::TrailerUnitDescriptionList::MAX_SIZE){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(), "Encoded TrailerUnitDescriptionList size is greater than max. Rejecting list element.");
                                break;
                            }

                            // TrailerUnitDescription element
                            auto trailer_unit_description = create_store_shared<TrailerUnitDescription_t>(shared_ptrs);

                            // TrailerUnitDescription.isDolly
                            trailer_unit_description->isDolly = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].is_dolly.is_dolly;

                            // TrailerUnitDescription.width
                            long vehicle_width = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].width.vehicle_width;
                            if(vehicle_width < j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MIN){

                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle width value less than min, setting to min");
                                vehicle_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MIN;
                            }
                            else if(vehicle_width > j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX){

                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle width value greater than max, setting to max");
                                vehicle_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_MAX;
                            }
                            trailer_unit_description->width = vehicle_width;

                            // TrailerUnitDescription.length
                            long vehicle_length = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].length.vehicle_length;
                            if(vehicle_length < j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MIN){

                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle length value less than min, setting to min");
                                vehicle_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MIN;
                            }
                            else if(vehicle_length > j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX){

                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle length value greater than max, setting to max");
                                vehicle_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_MAX;
                            }
                            trailer_unit_description->length = vehicle_length;

                            // TrailerUnitDescription.height
                            auto vehicle_height_ptr = create_store_shared<VehicleHeight_t>(shared_ptrs);
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT){
                                long vehicle_height = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].height.vehicle_height;
                                if(vehicle_height < j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle height value less than min, setting to min");
                                    vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN;
                                }
                                else if(vehicle_height > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle height value greater than max, setting to max");
                                    vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                                }

                                *vehicle_height_ptr = vehicle_height;
                                trailer_unit_description->height = vehicle_height_ptr;
                            }
                            else{
                                *vehicle_height_ptr = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE;
                                trailer_unit_description->height = vehicle_height_ptr;
                            }
                            
                            // TrailerUnitDescription.mass
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS){
                                auto trailer_mass_ptr = create_store_shared<TrailerMass_t>(shared_ptrs);
                                *trailer_mass_ptr = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].mass.trailer_mass;
                                trailer_unit_description->mass = trailer_mass_ptr;
                            }

                            // TrailerUnitDescription.bumperHeights
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS){
                                auto bumper_heights = create_store_shared<BumperHeights_t>(shared_ptrs);

                                // bumpers.front
                                uint8_t front_bumper_height = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].bumper_heights.front.bumper_height;
                                if(front_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded front bumper height value greater than max, setting to max");
                                    front_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                                }         
                                bumper_heights->front = front_bumper_height;

                                // bumpers.rear
                                uint8_t rear_bumper_height = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].bumper_heights.rear.bumper_height;
                                if(rear_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded rear bumper height value greater than max, setting to max");
                                    rear_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                                }         
                                bumper_heights->rear = rear_bumper_height;

                                trailer_unit_description->bumperHeights = bumper_heights;
                            }

                            // TrailerUnitDescription.centerOfGravity
                            auto center_of_gravity_ptr = create_store_shared<VehicleHeight_t>(shared_ptrs);
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY){
                                long center_of_gravity = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].center_of_gravity.vehicle_height;
                                if(center_of_gravity < j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded center of gravity value less than min, setting to min");
                                    center_of_gravity = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MIN;
                                }
                                else if(center_of_gravity > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded center of gravity value greater than max, setting to max");
                                    center_of_gravity = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                                }
                                *center_of_gravity_ptr = center_of_gravity;
                                trailer_unit_description->centerOfGravity = center_of_gravity_ptr;
                            }

                            // TrailerUnitDescription.frontPivot
                            long front_pivot_offset = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].front_pivot.pivot_offset.offset;
                            long front_pivot_angle = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].front_pivot.pivot_angle.angle;
                            bool front_pivoting_allowed = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].front_pivot.pivots.pivoting_allowed;
                            PivotPointDescription_t front_pivot_point_description = encode_pivot_point_description(front_pivot_offset, front_pivot_angle, front_pivoting_allowed);
                            trailer_unit_description->frontPivot = front_pivot_point_description;

                            // TrailerUnitDescription.rearPivot
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT){
                                auto rear_pivot_point_description_ptr = create_store_shared<PivotPointDescription_t>(shared_ptrs);

                                long rear_pivot_offset = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].rear_pivot.pivot_offset.offset;
                                long rear_pivot_angle = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].rear_pivot.pivot_angle.angle;
                                bool rear_pivoting_allowed = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].rear_pivot.pivots.pivoting_allowed;
                                PivotPointDescription_t rear_pivot_point_description = encode_pivot_point_description(rear_pivot_offset, rear_pivot_angle, rear_pivoting_allowed);

                                *rear_pivot_point_description_ptr = rear_pivot_point_description;
                                trailer_unit_description->rearPivot = rear_pivot_point_description_ptr;
                            }
                            
                            // TrailerUnitDescription.rearWheelOffset
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET){
                                auto rear_wheel_offset_ptr = create_store_shared<Offset_B12_t>(shared_ptrs);

                                long rear_wheel_offset = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].rear_wheel_offset.offset;
                                if(rear_wheel_offset < j2735_v2x_msgs::msg::OffsetB12::OFFSET_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded rear wheel offset value less than min, setting to min");
                                    rear_wheel_offset = j2735_v2x_msgs::msg::OffsetB12::OFFSET_MIN;
                                }
                                else if(rear_wheel_offset > j2735_v2x_msgs::msg::OffsetB12::OFFSET_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded rear wheel offset value greater than max, setting to max");
                                    rear_wheel_offset = j2735_v2x_msgs::msg::OffsetB12::OFFSET_MAX;
                                }
                                *rear_wheel_offset_ptr = rear_wheel_offset;
                                trailer_unit_description->rearWheelOffset = rear_wheel_offset_ptr;
                            }

                            // TrailerUnitDescription.positionOffset
                            long position_offset_x = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].position_offset.x;
                            if(position_offset_x < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer units position x-offset value less than min, setting to min");
                                position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                            }
                            else if(position_offset_x > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer units position x-offset value greater than max, setting to max");
                                position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                            }
                            trailer_unit_description->positionOffset.x = position_offset_x;

                            long position_offset_y = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].position_offset.y;
                            if(position_offset_y < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer units position y-offset value less than min, setting to min");
                                position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                            }
                            else if(position_offset_y > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Decoded trailer units position y-offset value greater than max, setting to max");
                                position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                            }
                            trailer_unit_description->positionOffset.y = position_offset_y;



                            // TrailerUnitDescription.elevationOffset
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET){
                                auto elevation_offset_ptr = create_store_shared<VertOffset_B07_t>(shared_ptrs);

                                long elevation_offset = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].elevation_offset.offset;
                                if(elevation_offset < j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded elevation offset value less than min, setting to min");
                                    elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN;
                                }
                                else if(elevation_offset > j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX){
                                    RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded elevation offset value greater than max, setting to max");
                                    elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX;
                                }
                                *elevation_offset_ptr = elevation_offset;
                                trailer_unit_description->elevationOffset = elevation_offset_ptr;
                            }

                            // TrailerUnitDescription.crumbData
                            if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA){
                                auto trailer_history_point_list = create_store_shared<TrailerHistoryPointList_t>(shared_ptrs);

                                for(size_t k = 0; k < special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points.size(); k++){
                                    auto trailer_history_point_ptr = create_store_shared<TrailerHistoryPoint_t>(shared_ptrs);
                                    
                                    // pivotAngle
                                    long pivot_angle = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].pivot_angle.angle;
                                    if(pivot_angle < j2735_v2x_msgs::msg::Angle::ANGLE_MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded pivot angle value less than min, setting to min");
                                        pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MIN;
                                    }
                                    else if(pivot_angle > j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                                        // Note: Value checked against 'ANGLE_UNAVAILABLE' since this value is larger than 'ANGLE_MAX'
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded pivot angle value greater than max, setting to max");
                                        pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MAX;
                                    }
                                    trailer_history_point_ptr->pivotAngle = pivot_angle;

                                    // timeOffset
                                    long time_offset = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].time_offset.offset;
                                    if(time_offset < j2735_v2x_msgs::msg::TimeOffset::MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded time offset value less than min, setting to min");
                                        time_offset = j2735_v2x_msgs::msg::TimeOffset::MIN;
                                    }
                                    else if(time_offset > j2735_v2x_msgs::msg::TimeOffset::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded time offset value greater than max, setting to max");
                                        time_offset = j2735_v2x_msgs::msg::TimeOffset::MAX;
                                    }
                                    trailer_history_point_ptr->timeOffset = time_offset;

                                    // positionOffset
                                    long point_position_offset_x = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].position_offset.x;
                                    if(point_position_offset_x < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer history point x-offset value less than min, setting to min");
                                        point_position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                                    }
                                    else if(point_position_offset_x > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer history point x-offset value greater than max, setting to max");
                                        point_position_offset_x = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                                    }
                                    trailer_history_point_ptr->positionOffset.x = point_position_offset_x;

                                    long point_position_offset_y = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].position_offset.y;
                                    if(point_position_offset_y < j2735_v2x_msgs::msg::NodeXY24b::MIN){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer history point y-offset value less than min, setting to min");
                                        point_position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MIN;
                                    }
                                    else if(point_position_offset_y > j2735_v2x_msgs::msg::NodeXY24b::MAX){
                                        RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer history point y-offset value greater than max, setting to max");
                                        point_position_offset_y = j2735_v2x_msgs::msg::NodeXY24b::MAX;
                                    }
                                    trailer_history_point_ptr->positionOffset.y = point_position_offset_y;

                                    // elevationOffset
                                    if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].presence_vector & j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET){
                                        auto elevation_offset_ptr = create_store_shared<VertOffset_B07_t>(shared_ptrs);

                                        long elevation_offset = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].elevation_offset.offset;
                                        if(elevation_offset < j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded elevation offset value less than min, setting to min");
                                            elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MIN;
                                        }
                                        else if(elevation_offset > j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded elevation offset value greater than max, setting to max");
                                            elevation_offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_MAX;
                                        }
                                        *elevation_offset_ptr = elevation_offset;
                                        trailer_history_point_ptr->elevationOffset = elevation_offset_ptr;
                                    }

                                    // heading
                                    if(special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].presence_vector & j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING){
                                        auto heading_ptr = create_store_shared<CoarseHeading_t>(shared_ptrs);

                                        long heading = special_vehicle_ext_msg.trailers.units.trailer_unit_descriptions[j].crumb_data.trailer_history_points[k].heading.heading;
                                        if(heading < j2735_v2x_msgs::msg::CoarseHeading::MIN){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded heading value less than min, setting to min");
                                            heading = j2735_v2x_msgs::msg::CoarseHeading::MIN;
                                        }
                                        else if(heading > j2735_v2x_msgs::msg::CoarseHeading::MAX){
                                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded heading value greater than max, setting to max");
                                            heading = j2735_v2x_msgs::msg::CoarseHeading::MAX;
                                        }
                                        *heading_ptr = heading;
                                        trailer_history_point_ptr->heading = heading_ptr;
                                    }

                                    asn_sequence_add(&trailer_history_point_list->list, trailer_history_point_ptr);
                                }

                                trailer_unit_description->crumbData = trailer_history_point_list;
                            }

                            asn_sequence_add(&trailer_unit_description_list->list, trailer_unit_description);
                        }

                        trailer_data->units = *trailer_unit_description_list;

                        part_ii_element->partII_Value.choice.SpecialVehicleExtensions.trailers = trailer_data;
                    }
                }

                // Encode Part II element of type SupplementalVehicleExtensions
                else if (plain_msg.part_ii[i].part_ii_id == j2735_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT) {
                    j2735_v2x_msgs::msg::SupplementalVehicleExtensions supplemental_vehicle_ext_msg = plain_msg.part_ii[i].supplemental_vehicle_extensions;

                    part_ii_element->partII_Id = PartII_Id_supplementalVehicleExt;
                    part_ii_element->partII_Value.present = BSMpartIIExtension__partII_Value_PR_SupplementalVehicleExtensions;

                    // classification
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION){
                        auto classification = create_store_shared<BasicVehicleClass_t>(shared_ptrs);
                        *classification = supplemental_vehicle_ext_msg.classification.basic_vehicle_class;
                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.classification = classification;
                    }

                    // classDetails
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS){
                        auto class_details = create_store_shared<VehicleClassification_t>(shared_ptrs);

                        // keyType
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_KEY_TYPE){
                            auto key_type = create_store_shared<BasicVehicleClass_t>(shared_ptrs);
                            *key_type = supplemental_vehicle_ext_msg.class_details.key_type.basic_vehicle_class;
                            class_details->keyType = key_type;
                        }

                        // role
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_ROLE){
                            auto role = create_store_shared<BasicVehicleRole_t>(shared_ptrs);
                            *role = supplemental_vehicle_ext_msg.class_details.role.basic_vehicle_role;
                            class_details->role = role;
                        }

                        // iso3883
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_ISO){
                            auto iso_3833 = create_store_shared<Iso3833VehicleType_t>(shared_ptrs);
                            *iso_3833 = supplemental_vehicle_ext_msg.class_details.iso3833;
                            class_details->iso3883 = iso_3833;
                        }

                        // hpmsType
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_HPMS_TYPE){
                            auto hpms_type = create_store_shared<VehicleType_t>(shared_ptrs);
                            *hpms_type = supplemental_vehicle_ext_msg.class_details.hpms_type.vehicle_type;
                            class_details->hpmsType = hpms_type;
                        } 

                        // vehicleType
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_VEHICLE_TYPE){
                            auto vehicle_type = create_store_shared<VehicleGroupAffected_t>(shared_ptrs);
                            *vehicle_type = supplemental_vehicle_ext_msg.class_details.vehicle_type.vehicle_group_affected;
                            class_details->vehicleType = vehicle_type;
                        }

                        // responseEquip
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONSE_EQUIP){
                            auto response_equip = create_store_shared<IncidentResponseEquipment_t>(shared_ptrs);
                            *response_equip = supplemental_vehicle_ext_msg.class_details.response_equip.incident_response_equipment;
                            class_details->responseEquip = response_equip;
                        }

                        // responderType
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONDER_TYPE){
                            auto responder_type = create_store_shared<ResponderGroupAffected_t>(shared_ptrs);
                            *responder_type = supplemental_vehicle_ext_msg.class_details.responder_type.responder_group_affected;
                            class_details->responderType = responder_type;
                        }

                        // fuelType
                        if(supplemental_vehicle_ext_msg.class_details.presence_vector & j2735_v2x_msgs::msg::VehicleClassification::HAS_FUEL_TYPE){
                            auto fuel_type = create_store_shared<FuelType_t>(shared_ptrs);
                            *fuel_type = supplemental_vehicle_ext_msg.class_details.fuel_type.fuel_type;
                            class_details->fuelType = fuel_type;
                        }

                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.classDetails = class_details;
                    }

                    // vehicleData
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA){
                        auto vehicle_data = create_store_shared<VehicleData_t>(shared_ptrs);

                        // height
                        if(supplemental_vehicle_ext_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_HEIGHT){
                            auto height_ptr = create_store_shared<VehicleHeight_t>(shared_ptrs);

                            uint8_t height = supplemental_vehicle_ext_msg.vehicle_data.height.vehicle_height;
                            if(height > j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded vehicle height value greater than max, setting to max");
                                height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_MAX;
                            }

                            *height_ptr = height;
                            vehicle_data->height = height_ptr;
                        }

                        // bumpers
                        if(supplemental_vehicle_ext_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_BUMPERS){
                            auto bumper_heights = create_store_shared<BumperHeights_t>(shared_ptrs);

                            // bumpers.front
                            uint8_t front_bumper_height = supplemental_vehicle_ext_msg.vehicle_data.bumpers.front.bumper_height;
                            if(front_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded front bumper height value greater than max, setting to max");
                                front_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                            }         
                            bumper_heights->front = front_bumper_height;

                            // bumpers.rear
                            uint8_t rear_bumper_height = supplemental_vehicle_ext_msg.vehicle_data.bumpers.rear.bumper_height;
                            if(rear_bumper_height > j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded rear bumper height value greater than max, setting to max");
                                rear_bumper_height = j2735_v2x_msgs::msg::BumperHeight::BUMPER_HEIGHT_MAX;
                            }         
                            bumper_heights->rear = rear_bumper_height;

                            vehicle_data->bumpers = bumper_heights;
                        }

                        // mass
                        if(supplemental_vehicle_ext_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_MASS){
                            auto mass = create_store_shared<VehicleMass_t>(shared_ptrs);
                            *mass = supplemental_vehicle_ext_msg.vehicle_data.mass.vehicle_mass;
                            vehicle_data->mass = mass;
                        }

                        // trailerWeight
                        if(supplemental_vehicle_ext_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT){
                            auto trailer_weight_ptr = create_store_shared<TrailerWeight_t>(shared_ptrs);

                            uint16_t trailer_weight = supplemental_vehicle_ext_msg.vehicle_data.trailer_weight.trailer_weight;
                            if(trailer_weight > j2735_v2x_msgs::msg::TrailerWeight::TRAILER_WEIGHT_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded trailer weight value greater than max, setting to max");
                                trailer_weight = j2735_v2x_msgs::msg::TrailerWeight::TRAILER_WEIGHT_MAX;
                            }
                            *trailer_weight_ptr = trailer_weight;
                            
                            vehicle_data->trailerWeight = trailer_weight_ptr;
                        }
 
                         part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.vehicleData = vehicle_data;
                    }

                    // weatherReport
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT){
                        auto weather_report = create_store_shared<WeatherReport_t>(shared_ptrs);

                        // isRaining
                        weather_report->isRaining = supplemental_vehicle_ext_msg.weather_report.is_raining.precip_yes_no;

                        // rainRate
                        if(supplemental_vehicle_ext_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE){
                            auto rain_rate = create_store_shared<EssPrecipRate_t>(shared_ptrs);
                            *rain_rate = supplemental_vehicle_ext_msg.weather_report.rain_rate.precip_rate;
                            weather_report->rainRate = rain_rate;
                        }

                        // precipSituation
                        if(supplemental_vehicle_ext_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION){
                            auto precip_situation = create_store_shared<EssPrecipSituation_t>(shared_ptrs);
                            *precip_situation = supplemental_vehicle_ext_msg.weather_report.precip_situation.ess_precip_situation;
                            weather_report->precipSituation = precip_situation;
                        }

                        // SolarRadiation
                        if(supplemental_vehicle_ext_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION){
                            auto solar_radiation = create_store_shared<EssSolarRadiation_t>(shared_ptrs);
                            *solar_radiation = supplemental_vehicle_ext_msg.weather_report.solar_radiation.ess_solar_radiation;
                            weather_report->solarRadiation = solar_radiation;
                        }

                        // friction
                        if(supplemental_vehicle_ext_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_FRICTION){
                            auto friction = create_store_shared<EssMobileFriction_t>(shared_ptrs);
                            *friction = supplemental_vehicle_ext_msg.weather_report.friction.ess_mobile_friction;
                            weather_report->friction = friction;
                        }

                        // roadFriction
                        if(supplemental_vehicle_ext_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION){
                            auto road_friction_ptr = create_store_shared<CoefficientOfFriction_t>(shared_ptrs);

                            uint8_t road_friction = supplemental_vehicle_ext_msg.weather_report.road_friction.coefficient;
                            if(road_friction > j2735_v2x_msgs::msg::CoefficientOfFriction::COEFFICIENT_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded road friction value greater than max, setting to max");
                                road_friction = j2735_v2x_msgs::msg::CoefficientOfFriction::COEFFICIENT_MAX;   
                            }
                            *road_friction_ptr = road_friction;

                            weather_report->roadFriction = road_friction_ptr;
                        }

                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.weatherReport = weather_report;
                    }

                    // weatherProbe
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE){
                        auto weather_probe = create_store_shared<WeatherProbe_t>(shared_ptrs);

                        // airTemp
                        if(supplemental_vehicle_ext_msg.weather_probe.presence_vector & j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP){
                            auto air_temp_ptr = create_store_shared<AmbientAirTemperature_t>(shared_ptrs);

                            uint8_t air_temp = supplemental_vehicle_ext_msg.weather_probe.air_temp.temperature;
                            if(air_temp > j2735_v2x_msgs::msg::AmbientAirTemperature::TEMPERATURE_UNKNOWN){
                                // Note: Value checked against 'TEMPERATURE_UNKONWN' since this value is larger than 'TEMPERATURE_MAX'
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded ambient air temperature value greater than max, setting to max");
                                air_temp = j2735_v2x_msgs::msg::AmbientAirTemperature::TEMPERATURE_MAX;   
                            }
                            *air_temp_ptr = air_temp;

                            weather_probe->airTemp = air_temp_ptr;
                        }

                        // airPressure
                        if(supplemental_vehicle_ext_msg.weather_probe.presence_vector & j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE){
                            auto air_pressure = create_store_shared<AmbientAirPressure_t>(shared_ptrs);
                            *air_pressure = supplemental_vehicle_ext_msg.weather_probe.air_pressure.pressure;
                            weather_probe->airPressure = air_pressure;
                        }

                        // rainRates
                        if(supplemental_vehicle_ext_msg.weather_probe.presence_vector & j2735_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES){
                            auto rain_rates = create_store_shared<WiperSet_t>(shared_ptrs);

                            // rainRates.statusFront
                            rain_rates->statusFront = supplemental_vehicle_ext_msg.weather_probe.rain_rates.status_front.wiper_status;

                            // rainRates.rateFront
                            uint8_t front_wiper_rate = supplemental_vehicle_ext_msg.weather_probe.rain_rates.rate_front.wiper_rate;
                            if(front_wiper_rate > j2735_v2x_msgs::msg::WiperRate::WIPER_RATE_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded ambient air temperature value greater than max, setting to max");
                                front_wiper_rate = j2735_v2x_msgs::msg::WiperRate::WIPER_RATE_MAX;   
                            }
                            rain_rates->rateFront = front_wiper_rate;

                            // rainRates.statusRear
                            if(supplemental_vehicle_ext_msg.weather_probe.rain_rates.presence_vector & j2735_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR){
                                auto status_rear = create_store_shared<WiperStatus_t>(shared_ptrs);
                                *status_rear = supplemental_vehicle_ext_msg.weather_probe.rain_rates.status_rear.wiper_status;
                                rain_rates->statusRear = status_rear;
                            }

                            // rainRates.rateRear
                            if(supplemental_vehicle_ext_msg.weather_probe.rain_rates.presence_vector & j2735_v2x_msgs::msg::WiperSet::HAS_RATE_REAR){
                                auto rate_rear = create_store_shared<WiperRate_t>(shared_ptrs);
                                *rate_rear = supplemental_vehicle_ext_msg.weather_probe.rain_rates.rate_rear.wiper_rate;
                                rain_rates->rateRear = rate_rear;
                            }

                            weather_probe->rainRates = rain_rates;
                        }

                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.weatherProbe = weather_probe;
                    }

                    // obstacleDetection
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE){
                        auto obstacle_detection = create_store_shared<ObstacleDetection_t>(shared_ptrs);

                        // obDist
                        obstacle_detection->obDist = supplemental_vehicle_ext_msg.obstacle.ob_dist.distance;

                        // obDirect
                        uint16_t ob_direct_angle = supplemental_vehicle_ext_msg.obstacle.ob_direct.direction.angle;
                        if(ob_direct_angle > j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                            // Note: Value checked against 'ANGLE_UNAVAILABLE' since this value is larger than 'ANGLE_MAX'
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded object direction angle value greater than max, setting to max");
                            ob_direct_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MAX;
                        }
                        obstacle_detection->obDirect = ob_direct_angle;

                        // dateTime
                        auto date_time = create_store_shared<DDateTime_t>(shared_ptrs);

                        // dateTime.year
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::YEAR){
                            auto year_ptr = create_store_shared<DYear_t>(shared_ptrs);

                            uint16_t year = supplemental_vehicle_ext_msg.obstacle.date_time.year.year;
                            if(year > j2735_v2x_msgs::msg::DYear::MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded year value greater than max, setting to max");
                                year = j2735_v2x_msgs::msg::DYear::MAX;   
                            }
                            *year_ptr = year;

                            date_time->year = year_ptr;                           
                        }

                        // dateTime.month
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MONTH){
                            auto month_ptr = create_store_shared<DMonth_t>(shared_ptrs);

                            uint16_t month = supplemental_vehicle_ext_msg.obstacle.date_time.month.month;
                            if(month > j2735_v2x_msgs::msg::DMonth::MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded month value greater than max, setting to max");
                                month = j2735_v2x_msgs::msg::DMonth::MAX;   
                            }
                            *month_ptr = month;

                            date_time->month = month_ptr;                           
                        }

                        // dateTime.day
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::DAY){
                            auto day_ptr = create_store_shared<DDay_t>(shared_ptrs);

                            uint16_t day = supplemental_vehicle_ext_msg.obstacle.date_time.day.day;
                            if(day > j2735_v2x_msgs::msg::DDay::MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded day value greater than max, setting to max");
                                day = j2735_v2x_msgs::msg::DDay::MAX;   
                            }
                            *day_ptr = day;

                            date_time->day = day_ptr;                           
                        }
                        
                        // dateTime.hour
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::HOUR){
                            auto hour_ptr = create_store_shared<DHour_t>(shared_ptrs);

                            uint8_t hour = supplemental_vehicle_ext_msg.obstacle.date_time.hour.hour;
                            if(hour > j2735_v2x_msgs::msg::DHour::UNAVAILABLE){
                                // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'HOUR_OF_DAY_MAX'
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded hour value greater than max, setting to max");
                                hour = j2735_v2x_msgs::msg::DHour::HOUR_OF_DAY_MAX;   
                            }
                            *hour_ptr = hour;

                            date_time->hour = hour_ptr;                           
                        }

                        // dateTime.minute
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::MINUTE){
                            auto minute_ptr = create_store_shared<DMinute_t>(shared_ptrs);

                            uint8_t minute = supplemental_vehicle_ext_msg.obstacle.date_time.minute.minute;
                            if(minute > j2735_v2x_msgs::msg::DMinute::UNAVAILABLE){
                                // Note: Value checked against 'UNAVAILABLE' since this value is larger than 'MINUTE_IN_HOUR_MAX'
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded minute value greater than max, setting to max");
                                minute = j2735_v2x_msgs::msg::DMinute::MINUTE_IN_HOUR_MAX;   
                            }
                            *minute_ptr = minute;

                            date_time->minute = minute_ptr;                           
                        }

                        // dateTime.second
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::SECOND){
                            auto second_ptr = create_store_shared<DSecond_t>(shared_ptrs);
                            *second_ptr = supplemental_vehicle_ext_msg.obstacle.date_time.second.millisecond;
                            date_time->second = second_ptr;                           
                        }

                        // dateTime.offset
                        if(supplemental_vehicle_ext_msg.obstacle.date_time.presence_vector & j2735_v2x_msgs::msg::DDateTime::OFFSET){
                            auto offset_ptr = create_store_shared<DSecond_t>(shared_ptrs);

                            long offset = supplemental_vehicle_ext_msg.obstacle.date_time.offset.offset_minute;
                            if(offset > j2735_v2x_msgs::msg::DOffset::MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded millisecond value greater than max, setting to max");
                                offset = j2735_v2x_msgs::msg::DOffset::MAX;   
                            }
                            else if(offset < j2735_v2x_msgs::msg::DOffset::MIN){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded millisecond value less than min, setting to min");
                                offset = j2735_v2x_msgs::msg::DOffset::MIN;   
                            }
                            *offset_ptr = offset;

                            date_time->offset = offset_ptr;                           
                        }

                        obstacle_detection->dateTime = *date_time;

                        // description
                        if(supplemental_vehicle_ext_msg.obstacle.presence_vector & j2735_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION){
                            if(supplemental_vehicle_ext_msg.obstacle.description.code > j2735_v2x_msgs::msg::ObstacleDetection::DESCRIPTION_MAX){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded obstacle description value greater than max, this optional field will not be encoded");
                            }
                            else if(supplemental_vehicle_ext_msg.obstacle.description.code < j2735_v2x_msgs::msg::ObstacleDetection::DESCRIPTION_MIN){
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded obstacle description value less than min, this optional field will not be encoded");
                            }
                            else{
                                auto itis_code2 = create_store_shared<ITIScodes_t>(shared_ptrs);
                                *itis_code2 = supplemental_vehicle_ext_msg.obstacle.description.code;
                                obstacle_detection->description = itis_code2;
                            }
                        }

                        // locationDetails
                        if(supplemental_vehicle_ext_msg.obstacle.presence_vector & j2735_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS){
                            auto location_details = create_store_shared<GenericLocations_t>(shared_ptrs);
                            *location_details = supplemental_vehicle_ext_msg.obstacle.location_details.generic_locations;
                            obstacle_detection->locationDetails = location_details;
                        }

                        // vertEvent
                        if(supplemental_vehicle_ext_msg.obstacle.presence_vector & j2735_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT){
                            auto vert_event = create_store_shared<VerticalAccelerationThreshold_t>(shared_ptrs);

                            std::string vert_event_str = std::to_string(supplemental_vehicle_ext_msg.obstacle.vert_event.exceeded_wheels);
                            size_t size = vert_event_str.size();

                            auto vert_event_buf = create_store_shared_array<uint8_t>(shared_ptrs, size);
                            for(size_t j = 0; j < size; j++){
                                vert_event_buf[j] = vert_event_str[j] - '0';
                            }
                            vert_event->size = size;
                            vert_event->buf = vert_event_buf;

                            obstacle_detection->vertEvent = vert_event;
                        }
                        
                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.obstacle = obstacle_detection;
                    }

                    // status
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS){
                        auto status = create_store_shared<DisabledVehicle_t>(shared_ptrs);

                        // statusDetails
                        if(supplemental_vehicle_ext_msg.status.status_details.code > j2735_v2x_msgs::msg::DisabledVehicle::STATUS_DETAILS_MAX){
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded disabled vehicle status value greater than max, this optional field will not be encoded");
                        }
                        else if(supplemental_vehicle_ext_msg.status.status_details.code < j2735_v2x_msgs::msg::DisabledVehicle::STATUS_DETAILS_MIN){
                            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded disabled vehicle status value less than min, this optional field will not be encoded");
                        }
                        else{
                            status->statusDetails = supplemental_vehicle_ext_msg.status.status_details.code;
                        }

                        // locationDetails
                        if(supplemental_vehicle_ext_msg.status.presence_vector & j2735_v2x_msgs::msg::DisabledVehicle::HAS_LOCATION_DETAILS){
                            auto location_details = create_store_shared<GenericLocations_t>(shared_ptrs);
                            *location_details = supplemental_vehicle_ext_msg.status.location_details.generic_locations;
                            status->locationDetails = location_details;
                        }

                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.status = status;
                    }

                    // speedProfile
                    if(supplemental_vehicle_ext_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE){
                        auto speed_profile = create_store_shared<SpeedProfile_t>(shared_ptrs);
                        auto speed_reports = create_store_shared<SpeedProfileMeasurementList_t>(shared_ptrs);

                        for(size_t j = 0; j < supplemental_vehicle_ext_msg.speed_profile.size(); j++){
                            auto speed_ptr = create_store_shared<GrossSpeed_t>(shared_ptrs);

                            uint8_t speed = supplemental_vehicle_ext_msg.speed_profile[j].speed;
                            if(speed > j2735_v2x_msgs::msg::GrossSpeed::SPEED_UNAVAILABLE){
                                // Note: Value checked against 'SPEED_UNAVAILABLE' since this value is larger than 'SPEED_MAX'
                                RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded speed value greater than max, setting to max");
                                speed = j2735_v2x_msgs::msg::GrossSpeed::SPEED_MAX;
                            }
                            *speed_ptr = speed;

                            asn_sequence_add(&speed_reports->list, speed_ptr);
                        }

                        speed_profile->speedReports = *speed_reports;


                        part_ii_element->partII_Value.choice.SupplementalVehicleExtensions.speedProfile = speed_profile;
                    }
                }
                asn_sequence_add(&part_ii_list->list, part_ii_element);
            }
            message->value.choice.BasicSafetyMessage.partII = part_ii_list;
        }
        else {
            message->value.choice.BasicSafetyMessage.partII = nullptr;
        }

        // Encode Regional Extensions
        if(plain_msg.presence_vector & j2735_v2x_msgs::msg::BSM::HAS_REGIONAL){
            // TODO
        }
        else {
            message->value.choice.BasicSafetyMessage.regional = nullptr;
        }

        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
        // Uncomment below to enable logging in human readable form
        // asn_fprint(fp, &asn_DEF_MessageFrame, message);
        
        //log a warning if that fails
        if(ec.encoded == -1) {
            RCLCPP_WARN_STREAM( node_logging_->get_logger(), "Encoding for BasicSafetyMessage has failed");
            std::cout << "Failed: " << ec.failed_type->name << std::endl;
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
        
        //Debugging/Unit Testing
        // for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }

    PivotPointDescription_t BSM_Message::encode_pivot_point_description(long& pivot_offset, long& pivot_angle, bool& pivoting_allowed){
        PivotPointDescription_t pivot_point_description;

        // pivot_offset
        if(pivot_offset < j2735_v2x_msgs::msg::OffsetB11::OFFSET_MIN){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded pivot offset value less than min, setting to min");
            pivot_offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_MIN;
        }
        else if(pivot_offset > j2735_v2x_msgs::msg::OffsetB11::OFFSET_MAX){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded pivot offset value greater than max, setting to max");
            pivot_offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_MAX;
        }
        pivot_point_description.pivotOffset = pivot_offset; 

        // pivot_angle
        if(pivot_angle < j2735_v2x_msgs::msg::Angle::ANGLE_MIN){
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded pivot angle value less than min, setting to min");
            pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MIN;
        }
        else if(pivot_angle > j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
            // Note: Value checked against 'ANGLE_UNAVAILABLE' since this value is larger than 'ANGLE_MAX'
            RCLCPP_WARN_STREAM(node_logging_->get_logger(),"Encoded pivot angle value greater than max, setting to max");
            pivot_angle = j2735_v2x_msgs::msg::Angle::ANGLE_MAX;
        }
        pivot_point_description.pivotAngle = pivot_angle;

        // pivoting_allowed
        pivot_point_description.pivots = pivoting_allowed;

        return pivot_point_description;
    }
} 
