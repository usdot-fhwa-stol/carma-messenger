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

    // TEST(PSMTest, testdDecodePSM){
    //     //Convert hex string to byte message
    //     std::vector<uint8_t> binary_input={ 0, 32, 95, 127, 255, 195, 220, 140, 4, 4, 8, 12, 17, 77, 226, 92, 189, 63, 71, 249, 125, 16, 40, 217, 217, 234, 146, 78, 32, 229, 141, 177, 219, 29, 159, 87, 38, 255, 254, 125, 20, 170, 40, 156, 65, 154, 52, 39, 112, 216, 178, 169, 248, 177, 253, 67, 206, 122, 95, 154, 207, 207, 234, 131, 61, 81, 184, 64, 15, 255, 255, 127, 255, 223, 221, 213, 3, 249, 188, 252, 254, 168, 61, 23, 253, 219, 34, 48, 131, 34, 148, 36, 56, 65, 112, 129, 11, 74};
    //     rclcpp::NodeOptions options;
    //     auto node = std::make_shared<rclcpp::Node>("base_node");
        
    //     rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_ = node->get_node_logging_interface();;
    //     cpp_message::PSM_Message worker(node_logging_);

    //     boost::optional<j2735_v2x_msgs::msg::PSM> res = worker.decode_psm_message(binary_input);
    //     if(res) {
    //         auto psm_msg = res.get();
            
    //         EXPECT_EQ(int(psm_msg.basic_type.type), 1);
    //         EXPECT_EQ(psm_msg.sec_mark.millisecond, 60998);
    //         EXPECT_NEAR(psm_msg.position.latitude, 406680509, 0.00001);
    //         EXPECT_NEAR(psm_msg.position.longitude, -738318466, 0.00001);
    //         EXPECT_EQ(psm_msg.position.elevation, 40);
    //     }

        
    // }

    TEST(PSMTest, testEncodePSM){
        
        j2735_v2x_msgs::msg::PSM message;

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
        j2735_v2x_msgs::msg::Position3D position;
        position.latitude = 406680509;
        position.longitude = -738318466;
        position.elevation_exists = true;
        position.elevation = 40;

        message.position = position;

        //7. positional accuracy
        j2735_v2x_msgs::msg::PositionalAccuracy accuracy;
        accuracy.semi_major = 217;
        accuracy.semi_minor = 217;
        accuracy.orientation = 60050;

        message.accuracy = accuracy;

        //8. velocity
        j2735_v2x_msgs::msg::Velocity speed;
        speed.velocity = 2500;

        message.speed = speed;
        //9. heading
        j2735_v2x_msgs::msg::Heading heading;
        heading.heading = 3672;

        message.heading = heading;

        //Optional fields
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ACCEL_SET;
        j2735_v2x_msgs::msg::AccelerationSet4Way accel_set;
        accel_set.longitudinal = 1505;
        accel_set.lateral = 1505;
        accel_set.vert = 90;
        accel_set.yaw_rate = 30067;

        message.accel_set = accel_set;

        //PathHistory
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_PATH_HISTORY;
        j2735_v2x_msgs::msg::PathHistory path_history;

        j2735_v2x_msgs::msg::PathHistoryPoint point1;
        point1.lat_offset.offset = 131070;
        point1.lon_offset.offset = 131070;
        point1.elevation_offset.offset = 2030;
        point1.time_offset.offset = 60034;
        point1.speed.speed = 8091;
        
        point1.pos_accuracy.semi_major = 207;
        point1.pos_accuracy.semi_minor = 207;
        point1.pos_accuracy.orientation = 60035;
        point1.heading.heading = 209;

        j2735_v2x_msgs::msg::PathHistoryPointList points_list;
        points_list.points.push_back(point1);
        path_history.crumb_data = points_list;

        path_history.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;
        j2735_v2x_msgs::msg::GNSSStatus gns_status;
        gns_status.statuses = j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY;
        path_history.curr_gnss_status = gns_status;


        j2735_v2x_msgs::msg::FullPositionVector initial_position;
        path_history.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;

        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;

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

        initial_position.lon.longitude = 1700000099;
        initial_position.lat.latitude = 800000099;
        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION;
        initial_position.elevation.elevation = 60039;

        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_HEADING;
        initial_position.heading.heading = 20090;

        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED;
        initial_position.speed.transmission.transmission_state |= j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS;
        initial_position.speed.speed.velocity = 8090;

        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;
        initial_position.pos_accuracy.semi_major = 207;
        initial_position.pos_accuracy.semi_minor = 207;
        initial_position.pos_accuracy.orientation = 60035;

        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
        initial_position.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001;

        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
        initial_position.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A20M;
        initial_position.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00;


        initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
        initial_position.speed_confidence.heading.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
        initial_position.speed_confidence.speed.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        initial_position.speed_confidence.throttle.confidence |= j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT;


        path_history.initial_position = initial_position;
        message.path_history = path_history;

        j2735_v2x_msgs::msg::PathPrediction path_prediction;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION;
        path_prediction.radius_of_curvature = 32700;
        path_prediction.confidence = 100;

        message.path_prediction = path_prediction;


        j2735_v2x_msgs::msg::PropelledInformation propelled_information;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_PROPULSION;
        propelled_information.choice = j2735_v2x_msgs::msg::PropelledInformation::CHOICE_MOTOR;   //Motor propelled

        propelled_information.motor.type = 3; //Bicycle

        message.propulsion = propelled_information;


        j2735_v2x_msgs::msg::PersonalDeviceUsageState personal_device_state;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_USE_STATE;
        personal_device_state.states |= j2735_v2x_msgs::msg::PersonalDeviceUsageState::LISTENING_TO_AUDIO;

        message.use_state = personal_device_state;

        j2735_v2x_msgs::msg::PersonalCrossingRequest cross_request;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST;
        cross_request.cross_request = true;

        message.cross_request = cross_request;

        j2735_v2x_msgs::msg::PersonalCrossingInProgress cross_state;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_CROSS_STATE;
        cross_state.cross_state = true;

        message.cross_state = cross_state;
        
        j2735_v2x_msgs::msg::NumberOfParticipantsInCluster cluster;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE;
        cluster.cluster_size |= j2735_v2x_msgs::msg::NumberOfParticipantsInCluster::SMALL;

        message.cluster_size = cluster;

        j2735_v2x_msgs::msg::PersonalClusterRadius cluster_radius;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS;
        cluster_radius.cluster_radius = 10;

        message.cluster_radius = cluster_radius;

        j2735_v2x_msgs::msg::PublicSafetyEventResponderWorkerType event_responder_type;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE;
        event_responder_type.type = j2735_v2x_msgs::msg::PublicSafetyEventResponderWorkerType::HAZMAT_RESPONDER;

        message.event_responder_type = event_responder_type;

        j2735_v2x_msgs::msg::PublicSafetyAndRoadWorkerActivity activity_type;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE;
        activity_type.activities |= j2735_v2x_msgs::msg::PublicSafetyAndRoadWorkerActivity::DIRECTING_TRAFFIC;

        message.activity_type = activity_type;

        j2735_v2x_msgs::msg::PublicSafetyDirectingTrafficSubType activity_sub_type;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE;
        activity_sub_type.sub_types |= j2735_v2x_msgs::msg::PublicSafetyDirectingTrafficSubType::EMERGENCY_ORGANIZATION_PERSONNEL;

        message.activity_sub_type = activity_sub_type;

        j2735_v2x_msgs::msg::PersonalAssistive assist_type;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE;
        assist_type.types |= j2735_v2x_msgs::msg::PersonalAssistive::MOVEMENT;

        message.assist_type = assist_type;

        j2735_v2x_msgs::msg::UserSizeAndBehaviour sizing;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_SIZING;
        sizing.sizes_and_behaviors |=  j2735_v2x_msgs::msg::UserSizeAndBehaviour::ERRATIC_MOVING;

        message.sizing = sizing;

        j2735_v2x_msgs::msg::Attachment attachment;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ATTACHMENT;
        attachment.type = j2735_v2x_msgs::msg::Attachment::STROLLER;

        message.attachment = attachment;

        j2735_v2x_msgs::msg::AttachmentRadius attachment_radius;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS;
        attachment_radius.attachment_radius = 105;

        message.attachment_radius = attachment_radius;

        j2735_v2x_msgs::msg::AnimalType animal_type;
        message.presence_vector |= j2735_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE;
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
