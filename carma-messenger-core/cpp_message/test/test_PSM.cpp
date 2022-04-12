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
    int char2int(char input)
    {
    if(input >= '0' && input <= '9')
        return input - '0';
    if(input >= 'A' && input <= 'F')
        return input - 'A' + 10;
    if(input >= 'a' && input <= 'f')
        return input - 'a' + 10;
    throw std::invalid_argument("Invalid input string");
    }

    void hex2bin(const char* src, char* target)
    {
        while(*src && src[1])
        {
            *(target++) = char2int(*src)*16 + char2int(src[1]);
            src += 2;
        }
    }

    std::vector<char> HexToBytes(const std::string& hex) {
        std::vector<char> bytes;

        for (unsigned int i = 0; i < hex.length(); i += 2) {
            std::string byteString = hex.substr(i, 2);
            char byte = (char) strtol(byteString.c_str(), NULL, 16);
            bytes.push_back(byte);
        }

        return bytes;
    }

    // TEST(PSMTest, testdDecodePSM){
    //     std::string hex_message = "3C4D6573736167654672616D653E0D0A20203C6D65737361676549643E33323C2F6D65737361676549643E0D0A20203C76616C75653E0D0A202020203C506572736F6E616C5361666574794D6573736167653E0D0A2020202020203C6261736963547970653E3C61";
        
    //     //Convert hex string to byte message
    //     std::vector<uint8_t> binary_input;

    //     std::vector<char> new_binary_input = HexToBytes(hex_message);
    //     for(int i =0;i<new_binary_input.size();++i){
    //         std::cout<<new_binary_input[i]<<" "<<",";
    //     }
    //     std::cout<<"\n";
    // }

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
        id.id = {0,0,0,0};

        message.id = id;

        //6. position
        carma_v2x_msgs::msg::Position3D position;
        position.latitude = 406680509 * (float)0.0000001;
        position.longitude = -738318466 * (float)0.0000001;
        position.elevation_exists = false;
        // position.elevation = 40 * 0.1;

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
        for(size_t i = 0; i < array.size(); i++) std::cout<< int(array[i])<< ", ";
       }

    //    Binary examples -
    //1. Till longitude - 0, 32, 26, 0, 0, 3, 220, 140, 4, 0, 0, 0, 0, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 0, 0, 0, 0, 0, 0
    //2. Till Header -  0, 32, 28, 0, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 128
    //3. till accel_set - 0, 32, 34, 64, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 250, 14, 95, 82, 240
    //4. till path_history, speed -  0, 32, 46, 96, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 240, 5, 225, 168, 56, 106, 15, 209, 156, 96, 13, 224, 0
    //5. till path history-complete -  0, 32, 50, 96, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 240, 7, 225, 168, 56, 106, 15, 209, 156, 96, 13, 231, 79, 78, 173, 126, 64
    //6. till gnss_status -  0, 32, 51, 96, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 242, 32, 7, 225, 168, 56, 106, 15, 209, 156, 96, 13, 231, 79, 78, 173, 126, 64,
    //7. initial_position - till utc time: 0, 32, 68, 96, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 128, 254, 125, 20, 170, 40, 156, 65, 154, 26, 210, 116, 127, 218, 210, 116, 128, 96, 1, 248, 106, 14, 26, 131, 244, 103, 24, 3, 121, 211, 211, 171, 95, 144
    //8. use_state(without propulsion): 0, 32, 73, 116, 0, 3, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 128, 254, 125, 20, 170, 40, 156, 65, 154, 26, 210, 116, 127, 218, 210, 116, 128, 48, 1, 248, 106, 14, 26, 131, 244, 103, 24, 3, 121, 211, 211, 171, 95, 144, 129, 43, 100, 48, 0
    //9. All (without propulsion) : 0, 32, 82, 119, 255, 195, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 128, 254, 125, 20, 170, 40, 156, 65, 154, 26, 210, 116, 127, 218, 210, 116, 128, 120, 1, 248, 106, 14, 26, 131, 244, 103, 24, 3, 121, 211, 211, 171, 95, 144, 129, 43, 100, 120, 50, 41, 94, 67, 252, 37, 188, 120, 90, 80
    //10. All : 0, 32, 83, 127, 255, 195, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 128, 254, 125, 20, 170, 40, 156, 65, 154, 26, 210, 116, 127, 218, 210, 116, 128, 24, 1, 248, 106, 14, 26, 131, 244, 103, 24, 3, 121, 211, 211, 171, 95, 144, 129, 43, 100, 70, 48, 100, 82, 140, 134, 152, 12, 96, 194, 210, 128
    // All(2): 0, 32, 83, 127, 255, 195, 220, 140, 4, 0, 0, 0, 1, 77, 226, 92, 192, 63, 71, 249, 127, 0, 0, 233, 233, 213, 84, 2, 88, 229, 137, 196, 156, 78, 95, 82, 246, 128, 254, 125, 20, 170, 40, 156, 65, 154, 26, 210, 116, 127, 218, 210, 116, 128, 8, 1, 248, 106, 14, 26, 131, 244, 103, 24, 3, 121, 211, 211, 171, 95, 144, 129, 43, 100, 70, 16, 100, 82, 132, 136, 8, 42, 132, 8, 90, 80
    }
}
