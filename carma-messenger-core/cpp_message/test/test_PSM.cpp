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

#include "cpp_message/PSM_Message.h"
#include "carma_v2x_msgs/msg/psm.hpp"
#include<gtest/gtest.h>

namespace cpp_message
{

    TEST(PSMTest, testdDecodePSM){
        //Convert hex string to byte message
        // std::vector<uint8_t> binary_input={ 0, 32, 95, 127, 255, 195, 220, 140, 4, 4, 8, 12, 17, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 255, 254, 125, 20, 170, 40, 156, 65, 154, 41, 192, 200, 28, 242, 208, 30, 27, 125, 60, 228, 78, 83, 161, 233, 233, 215, 34, 61, 81, 187, 192, 15, 195, 80, 112, 212, 31, 163, 56, 192, 27, 206, 158, 157, 90, 252, 132, 9, 91, 34, 55, 131, 34, 149, 228, 59, 194, 151, 143, 11, 74};
        std::vector<uint8_t> binary_input={0, 32, 96, 127, 255, 195, 220, 140, 4, 4, 8, 12, 17, 77, 226, 92, 192, 63, 71, 249, 127, 16, 40, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 255, 254, 125, 20, 170, 40, 156, 65, 154, 41, 192, 200, 28, 242, 208, 30, 27, 125, 60, 228, 78, 83, 161, 233, 233, 215, 34, 61, 81, 186, 64, 15, 195, 80, 112, 212, 31, 163, 56, 192, 27, 206, 158, 157, 90, 252, 132, 9, 91, 34, 52, 131, 34, 149, 36, 66, 65, 205, 34, 66, 210, 128};
        rclcpp::NodeOptions options;
        auto node = std::make_shared<rclcpp::Node>("base_node");
        
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ = node->get_node_logging_interface();;
        cpp_message::PSM_Message worker(node_logging_);

        boost::optional<carma_v2x_msgs::msg::PSM> res = worker.decode_psm_message(binary_input);
        if(res) {
            auto psm_msg = res.get();
            
            EXPECT_EQ(int(psm_msg.basic_type.type), 1);
            EXPECT_EQ(psm_msg.sec_mark.millisecond, 60998);
            EXPECT_NEAR(psm_msg.position.latitude, 406680509 *0.0000001, 0.00001);
            EXPECT_NEAR(psm_msg.position.longitude, -738318466 *0.0000001, 0.00001);
            EXPECT_EQ(psm_msg.position.elevation, 4);
        }

        
    }

    TEST(PSMTest, testEncodePSM){
        
        carma_v2x_msgs::msg::PSM message;

        //2. basic_type
        j2735_v2x_msgs::msg::PersonalDeviceUserType basic_type;
        basic_type.type |= j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;
        message.basic_type = basic_type;

        //3.sec_mark
        j2735_v2x_msgs::msg::DSecond sec_mark;
        sec_mark.millisecond = 60998;
        message.sec_mark = sec_mark;

        //4.msg_cnt
        j2735_v2x_msgs::msg::MsgCount msg_cnt;
        msg_cnt.msg_cnt = 1;

        message.msg_cnt = msg_cnt;

        //5. id
        j2735_v2x_msgs::msg::TemporaryID id;
        id.id = {1,2,3,4};

        message.id = id;

        //6. position
        carma_v2x_msgs::msg::Position3D position;
        position.latitude = 406680509 * (float)0.0000001;
        position.longitude = -738318466 * (float)0.0000001;
        position.elevation_exists = true;
        position.elevation = 40 * 0.1;

        message.position = position;

        //7. positional accuracy
        carma_v2x_msgs::msg::PositionalAccuracy accuracy;
        accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
        accuracy.semi_major = 11.7;
        accuracy.semi_minor = 11.7;
        accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
        accuracy.orientation = 300;

        message.accuracy = accuracy;

        //8. velocity
        carma_v2x_msgs::msg::Velocity speed;
        speed.unavailable = false;
        speed.velocity = 75 * 0.02;

        message.speed = speed;
        //9. heading
        carma_v2x_msgs::msg::Heading heading;
        heading.unavailable = false;
        heading.heading = 3672 * 0.0125;

        message.heading = heading;

        //Optional fields
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ACCEL_SET;
        carma_v2x_msgs::msg::AccelerationSet4Way accel_set;
        accel_set.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE;
        accel_set.longitudinal = 5.0;
        accel_set.lateral = 5.0;

        accel_set.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE;
        accel_set.vert = 20.0;

        accel_set.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE;
        accel_set.yaw_rate = 300.0;

        message.accel_set = accel_set;

        //PathHistory
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY;
        carma_v2x_msgs::msg::PathHistory path_history;

        carma_v2x_msgs::msg::PathHistoryPoint point1;
        point1.lat_offset.unavailable = false;
        point1.lat_offset.offset = 0.01;
        point1.lon_offset.unavailable = false;
        point1.lon_offset.offset = 0.01;
        point1.elevation_offset.unavailable = false;
        point1.elevation_offset.offset = 200.1;
        point1.time_offset.unavailable = false;
        point1.time_offset.offset = 400.34;
        point1.speed.unavailable = false;
        point1.speed.speed = 8.9;
        point1.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
        point1.pos_accuracy.semi_major = 11.7;
        point1.pos_accuracy.semi_minor = 11.7;
        point1.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
        point1.pos_accuracy.orientation = 300.5;
        point1.heading.unavailable = false;
        point1.heading.heading = 300.5;

        carma_v2x_msgs::msg::PathHistoryPointList points_list;
        points_list.points.push_back(point1);
        path_history.crumb_data = points_list;

        path_history.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;
        j2735_v2x_msgs::msg::GNSSStatus gns_status;
        gns_status.statuses = j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY;
        path_history.curr_gnss_status = gns_status;
        
        

        carma_v2x_msgs::msg::FullPositionVector initial_position;
        path_history.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;

        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;

        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
        initial_position.utc_time.year.year = 1000;
        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
        initial_position.utc_time.month.month = 10;
        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
        initial_position.utc_time.day.day = 10;
        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
        initial_position.utc_time.hour.hour = 20;
        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
        initial_position.utc_time.minute.minute = 20;
        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
        initial_position.utc_time.second.millisecond = 20000;
        initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
        initial_position.utc_time.offset.offset_minute = 800;

        initial_position.lon.unavailable = false;
        initial_position.lon.longitude = 100.20;
        initial_position.lat.unavailable = false;
        initial_position.lat.latitude = 80.5;
        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION;
        initial_position.elevation.unavailable = false;
        initial_position.elevation.elevation = 6002.5;

        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_HEADING;
        initial_position.heading.unavailable = false;
        initial_position.heading.heading = 320.98;

        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_SPEED;
        initial_position.speed.transmission.transmission_state |= j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS;
        initial_position.speed.speed.unavailable = false;
        initial_position.speed.speed.velocity = 100.5;

        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;
        initial_position.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
        initial_position.pos_accuracy.semi_major = 11.7;
        initial_position.pos_accuracy.semi_minor = 11.7;
        initial_position.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
        initial_position.pos_accuracy.orientation = 302.54;

        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
        initial_position.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001;

        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
        initial_position.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A20M;
        initial_position.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00;


        initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
        initial_position.speed_confidence.heading.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
        initial_position.speed_confidence.speed.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        initial_position.speed_confidence.throttle.confidence |= j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT;


        path_history.initial_position = initial_position;
        message.path_history = path_history;

        carma_v2x_msgs::msg::PathPrediction path_prediction;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION;
        path_prediction.radius_of_curvature = 300.5;
        path_prediction.confidence = 100.5;

        message.path_prediction = path_prediction;


        j2735_v2x_msgs::msg::PropelledInformation propelled_information;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PROPULSION;
        propelled_information.choice = j2735_v2x_msgs::msg::PropelledInformation::CHOICE_MOTOR;   //Motor propelled

        propelled_information.motor.type = 3; //Bicycle

        message.propulsion = propelled_information;


        j2735_v2x_msgs::msg::PersonalDeviceUsageState personal_device_state;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_USE_STATE;
        personal_device_state.states |= j2735_v2x_msgs::msg::PersonalDeviceUsageState::LISTENING_TO_AUDIO;

        message.use_state = personal_device_state;

        j2735_v2x_msgs::msg::PersonalCrossingRequest cross_request;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST;
        cross_request.cross_request = true;

        message.cross_request = cross_request;

        j2735_v2x_msgs::msg::PersonalCrossingInProgress cross_state;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CROSS_STATE;
        cross_state.cross_state = true;

        message.cross_state = cross_state;
        
        j2735_v2x_msgs::msg::NumberOfParticipantsInCluster cluster;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE;
        cluster.cluster_size |= j2735_v2x_msgs::msg::NumberOfParticipantsInCluster::SMALL;

        message.cluster_size = cluster;

        j2735_v2x_msgs::msg::PersonalClusterRadius cluster_radius;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS;
        cluster_radius.cluster_radius = 10;

        message.cluster_radius = cluster_radius;

        j2735_v2x_msgs::msg::PublicSafetyEventResponderWorkerType event_responder_type;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE;
        event_responder_type.type = j2735_v2x_msgs::msg::PublicSafetyEventResponderWorkerType::HAZMAT_RESPONDER;

        message.event_responder_type = event_responder_type;

        j2735_v2x_msgs::msg::PublicSafetyAndRoadWorkerActivity activity_type;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE;
        activity_type.activities |= j2735_v2x_msgs::msg::PublicSafetyAndRoadWorkerActivity::DIRECTING_TRAFFIC;

        message.activity_type = activity_type;

        j2735_v2x_msgs::msg::PublicSafetyDirectingTrafficSubType activity_sub_type;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE;
        activity_sub_type.sub_types |= j2735_v2x_msgs::msg::PublicSafetyDirectingTrafficSubType::EMERGENCY_ORGANIZATION_PERSONNEL;

        message.activity_sub_type = activity_sub_type;

        j2735_v2x_msgs::msg::PersonalAssistive assist_type;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE;
        assist_type.types |= j2735_v2x_msgs::msg::PersonalAssistive::MOVEMENT;

        message.assist_type = assist_type;

        j2735_v2x_msgs::msg::UserSizeAndBehaviour sizing;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_SIZING;
        sizing.sizes_and_behaviors |=  j2735_v2x_msgs::msg::UserSizeAndBehaviour::ERRATIC_MOVING;

        message.sizing = sizing;

        j2735_v2x_msgs::msg::Attachment attachment;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT;
        attachment.type = j2735_v2x_msgs::msg::Attachment::STROLLER;

        message.attachment = attachment;

        carma_v2x_msgs::msg::AttachmentRadius attachment_radius;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS;
        attachment_radius.attachment_radius = 10.5;

        message.attachment_radius = attachment_radius;

        j2735_v2x_msgs::msg::AnimalType animal_type;
        message.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE;
        animal_type.type |= j2735_v2x_msgs::msg::AnimalType::PET;

        message.animal_type = animal_type;


        rclcpp::NodeOptions options;
        auto node = std::make_shared<rclcpp::Node>("base_node");
        
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ = node->get_node_logging_interface();;
        cpp_message::PSM_Message worker(node_logging_);

       boost::optional<std::vector<uint8_t>> b_array = worker.encode_psm_message(message);
       if(b_array){
        std::vector<uint8_t> array = b_array.get();
        // for(size_t i = 0; i < array.size(); i++) std::cout<< int(array[i])<< ", ";
        EXPECT_FALSE(array.empty());
       }

        
    }
}
