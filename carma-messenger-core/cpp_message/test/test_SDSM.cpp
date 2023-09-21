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

#include "cpp_message/SDSM_Message.h"
#include<gtest/gtest.h>
#include <boost/optional/optional_io.hpp> 

namespace cpp_message
{
    TEST(SDSMTest, testEncodeSDSM)
    {
        auto node = std::make_shared<rclcpp::Node>("test_node");
        cpp_message::SDSM_Message worker(node->get_node_logging_interface());

        j3224_v2x_msgs::msg::SensorDataSharingMessage message;

        message.msg_cnt.msg_cnt = 1;
        message.source_id.id = {1,2,3,4};
        message.equipment_type.equipment_type |= j3224_v2x_msgs::msg::EquipmentType::RSU;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
        message.sdsm_time_stamp.year.year = 2007;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
        message.sdsm_time_stamp.month.month = 7;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
        message.sdsm_time_stamp.day.day = 4;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
        message.sdsm_time_stamp.hour.hour = 19;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
        message.sdsm_time_stamp.minute.minute = 15;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
        message.sdsm_time_stamp.second.millisecond = 5000;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
        message.sdsm_time_stamp.offset.offset_minute = 400;
        message.ref_pos.latitude = 400000000;
        message.ref_pos.longitude = 600000000;
        message.ref_pos.elevation_exists = true;
        message.ref_pos.elevation = 30;
        message.ref_pos_xy_conf.semi_major = 235;
        message.ref_pos_xy_conf.semi_minor = 200;
        message.ref_pos_xy_conf.orientation = 25000;
        j3224_v2x_msgs::msg::DetectedObjectList detected_obj_test;
        message.objects = detected_obj_test;
        message.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
        message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_000_50;

        // create detected object data and push it to the detected object list
        j3224_v2x_msgs::msg::DetectedObjectData object1;

        // common data
        object1.detected_object_common_data.obj_type.object_type |= j3224_v2x_msgs::msg::ObjectType::VEHICLE;
        object1.detected_object_common_data.obj_type_cfd.classification_confidence = 65;
        object1.detected_object_common_data.detected_id.object_id = 12200;
        object1.detected_object_common_data.measurement_time.measurement_time_offset = -1100;
        object1.detected_object_common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000;
        object1.detected_object_common_data.pos.offset_x.object_distance = 4000;
        object1.detected_object_common_data.pos.offset_y.object_distance = -720;
        object1.detected_object_common_data.pos.presence_vector |= j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
        object1.detected_object_common_data.pos.offset_z.object_distance = 20;
        object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A200M;
        object1.detected_object_common_data.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_200_00;
        object1.detected_object_common_data.speed.speed = 2500;
        object1.detected_object_common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        object1.detected_object_common_data.heading.heading = 16000;
        object1.detected_object_common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
        object1.detected_object_common_data.speed_z.speed = 1000;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
        object1.detected_object_common_data.speed_confidence_z.speed_confidence = j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
        object1.detected_object_common_data.accel_4_way.longitudinal = 200;
        object1.detected_object_common_data.accel_4_way.lateral = -500;
        object1.detected_object_common_data.accel_4_way.vert = 1;
        object1.detected_object_common_data.accel_4_way.yaw_rate = 400;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
        object1.detected_object_common_data.acc_cfd_x.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
        object1.detected_object_common_data.acc_cfd_y.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
        object1.detected_object_common_data.acc_cfd_z.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05;
        
        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
        object1.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence |= j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00;


        object1.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
        object1.detected_object_optional_data.choice |= j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH;
        // Detected Vehicle Data
        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS;
        object1.detected_object_optional_data.det_veh.lights.exterior_lights |= j2735_v2x_msgs::msg::ExteriorLights::HAZARD_SIGNAL_ON;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE;
        object1.detected_object_optional_data.det_veh.veh_attitude.pitch.pitch_detected = 2400;
        object1.detected_object_optional_data.det_veh.veh_attitude.roll.roll_detected = -12000;
        object1.detected_object_optional_data.det_veh.veh_attitude.yaw.yaw_detected = 400;
        
        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_attitude_confidence.pitch_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_05_DEG;
        object1.detected_object_optional_data.det_veh.veh_attitude_confidence.roll_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
        object1.detected_object_optional_data.det_veh.veh_attitude_confidence.yaw_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL;
        object1.detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate = 600;
        object1.detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate = -800;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_PITCH_RATE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence |= j3224_v2x_msgs::msg::PitchRateConfidence::DEG_SEC_005_00;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_ROLL_RATE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence |= j3224_v2x_msgs::msg::RollRateConfidence::DEG_SEC_001_00;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
        object1.detected_object_optional_data.det_veh.size.vehicle_width = 300;
        object1.detected_object_optional_data.det_veh.size.vehicle_length = 700;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;
        object1.detected_object_optional_data.det_veh.height.vehicle_height = 70;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.presence_vector |= j3224_v2x_msgs::msg::VehicleSizeConfidence::HAS_VEHICLE_HEIGHT_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_005_00;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS;
        object1.detected_object_optional_data.det_veh.vehicle_class.basic_vehicle_class |= j2735_v2x_msgs::msg::BasicVehicleClass::PASSENGER_VEHICLE_TYPE_OTHER;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF;
        object1.detected_object_optional_data.det_veh.class_conf.classification_confidence = 75;

        message.objects.detected_object_data.push_back(object1);


        j3224_v2x_msgs::msg::DetectedObjectData object2;
        object2.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
        object2.detected_object_optional_data.choice |= j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU;

        // Detected VRU Data
        object2.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE;
        object2.detected_object_optional_data.det_vru.basic_type.type |= j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;

        object2.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION;
        object2.detected_object_optional_data.det_vru.propulsion.choice |= j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN;
        object2.detected_object_optional_data.det_vru.propulsion.human.type |= j2735_v2x_msgs::msg::HumanPropelledType::ON_FOOT;

        object2.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT;
        object2.detected_object_optional_data.det_vru.attachment.type |= j2735_v2x_msgs::msg::Attachment::CART;

        object2.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS;
        object2.detected_object_optional_data.det_vru.radius.attachment_radius = 30;

        message.objects.detected_object_data.push_back(object2);


        j3224_v2x_msgs::msg::DetectedObjectData object3;
        object3.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
        object3.detected_object_optional_data.choice |= j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST;

        // Detected Obstacle Data
        object3.detected_object_optional_data.det_obst.obst_size.width.size_value = 400;
        object3.detected_object_optional_data.det_obst.obst_size.length.size_value = 300;
        object3.detected_object_optional_data.det_obst.obst_size.presence_vector |= j3224_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT;
        object3.detected_object_optional_data.det_obst.obst_size.height.size_value = 100;

        object3.detected_object_optional_data.det_obst.obst_size_confidence.width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00;
        object3.detected_object_optional_data.det_obst.obst_size_confidence.length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00;
        object3.detected_object_optional_data.det_obst.obst_size_confidence.presence_vector |= j3224_v2x_msgs::msg::ObstacleSizeConfidence::HAS_HEIGHT_CONFIDENCE;
        object3.detected_object_optional_data.det_obst.obst_size_confidence.height_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_000_50;

        message.objects.detected_object_data.push_back(object3);

        // Encode the compiled test message and test if it succeeds
        auto res = worker.encode_sdsm_message(message);

        if(res){
            // // Output of encoded SDSM for debugging
            // std::vector<uint8_t> to_read=res.get();
            // size_t len=to_read.size();
            // for(size_t i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
            // std::cout<<"\n";
            EXPECT_TRUE(true);
        }
        else
        {
            std::cout << "Encoding failed while unit testing SDSM encoder!\n";
            EXPECT_TRUE(false);
        }

    }

    // Test for encoding/decoding twice in succession on SDSM Common data
    TEST(SDSMTest, testEncodeDecodeSDSMCom){

        auto node = std::make_shared<rclcpp::Node>("test_node");
        cpp_message::SDSM_Message worker(node->get_node_logging_interface());

        j3224_v2x_msgs::msg::SensorDataSharingMessage message;

        message.msg_cnt.msg_cnt = 1;
        message.source_id.id = {1,2,3,4};
        message.equipment_type.equipment_type |= j3224_v2x_msgs::msg::EquipmentType::RSU;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
        message.sdsm_time_stamp.year.year = 2007;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
        message.sdsm_time_stamp.month.month = 7;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
        message.sdsm_time_stamp.day.day = 4;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
        message.sdsm_time_stamp.hour.hour = 19;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
        message.sdsm_time_stamp.minute.minute = 15;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
        message.sdsm_time_stamp.second.millisecond = 5000;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
        message.sdsm_time_stamp.offset.offset_minute = 400;
        message.ref_pos.latitude = 400000000;
        message.ref_pos.longitude = 600000000;
        message.ref_pos.elevation_exists = true;
        message.ref_pos.elevation = 30;
        message.ref_pos_xy_conf.semi_major = 235;
        message.ref_pos_xy_conf.semi_minor = 200;
        message.ref_pos_xy_conf.orientation = 25000;
        j3224_v2x_msgs::msg::DetectedObjectList detected_obj_test;
        message.objects = detected_obj_test;
        message.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
        message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_000_50;

        // create detected object data and push it to the detected object list
        j3224_v2x_msgs::msg::DetectedObjectData object1;

        // common data
        object1.detected_object_common_data.obj_type.object_type |= j3224_v2x_msgs::msg::ObjectType::VEHICLE;
        object1.detected_object_common_data.obj_type_cfd.classification_confidence = 65;
        object1.detected_object_common_data.detected_id.object_id = 12200;
        object1.detected_object_common_data.measurement_time.measurement_time_offset = -1100;
        object1.detected_object_common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000;
        object1.detected_object_common_data.pos.offset_x.object_distance = 4000;
        object1.detected_object_common_data.pos.offset_y.object_distance = -720;
        object1.detected_object_common_data.pos.presence_vector |= j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
        object1.detected_object_common_data.pos.offset_z.object_distance = 20;
        object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A200M;
        object1.detected_object_common_data.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_200_00;
        object1.detected_object_common_data.speed.speed = 2500;
        object1.detected_object_common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        object1.detected_object_common_data.heading.heading = 16000;
        object1.detected_object_common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
        object1.detected_object_common_data.speed_z.speed = 1000;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
        object1.detected_object_common_data.speed_confidence_z.speed_confidence = j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
        object1.detected_object_common_data.accel_4_way.longitudinal = 200;
        object1.detected_object_common_data.accel_4_way.lateral = -500;
        object1.detected_object_common_data.accel_4_way.vert = 1;
        object1.detected_object_common_data.accel_4_way.yaw_rate = 400;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
        object1.detected_object_common_data.acc_cfd_x.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
        object1.detected_object_common_data.acc_cfd_y.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
        object1.detected_object_common_data.acc_cfd_z.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05;
        
        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
        object1.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence |= j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00;

        message.objects.detected_object_data.push_back(object1);

        // Initial SDSM encode and test
        auto res = worker.encode_sdsm_message(message);
        if(res){
            EXPECT_TRUE(true);
        }
        else{
            std::cout << "Encoding failed while unit testing SDSM encoder!\n";
            EXPECT_TRUE(false);
        }

        // Decode encoded SDSM
        auto res_decoded = worker.decode_sdsm_message(res.get());
        if(res_decoded) EXPECT_TRUE(true);
        else{
            std::cout << "Decoding of encoded SDSM failed!\n";
            EXPECT_TRUE(false);
        }

        // Compare decoded SDSM to expected result
        j3224_v2x_msgs::msg::SensorDataSharingMessage result = res_decoded.get();
        EXPECT_EQ(message, result);

        // Repeat encode/decode process on prior result data
        auto res2 = worker.encode_sdsm_message(result);
        if(res2){
            EXPECT_TRUE(true);
        }
        else{
            std::cout << "Encoding for decoded SDSM failed in unit testing!\n";
            EXPECT_TRUE(false);
        }

        auto res2_decoded = worker.decode_sdsm_message(res2.get());
        if(res2_decoded){
            EXPECT_TRUE(true);
        }
        else{
            std::cout << "Secondary decode for SDSM failed!\n";
            EXPECT_TRUE(false);
        }

    }

    // Test for encoding/decoding optional data - vehicle data
    TEST(SDSMTest, testEncodeDecodeSDSMOpt1){

        auto node = std::make_shared<rclcpp::Node>("test_node");
        cpp_message::SDSM_Message worker(node->get_node_logging_interface());

        j3224_v2x_msgs::msg::SensorDataSharingMessage message;

        message.msg_cnt.msg_cnt = 1;
        message.source_id.id = {1,2,3,4};
        message.equipment_type.equipment_type |= j3224_v2x_msgs::msg::EquipmentType::RSU;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
        message.sdsm_time_stamp.year.year = 2007;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
        message.sdsm_time_stamp.month.month = 7;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
        message.sdsm_time_stamp.day.day = 4;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
        message.sdsm_time_stamp.hour.hour = 19;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
        message.sdsm_time_stamp.minute.minute = 15;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
        message.sdsm_time_stamp.second.millisecond = 5000;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
        message.sdsm_time_stamp.offset.offset_minute = 400;
        message.ref_pos.latitude = 400000000;
        message.ref_pos.longitude = 600000000;
        message.ref_pos.elevation_exists = true;
        message.ref_pos.elevation = 30;
        message.ref_pos_xy_conf.semi_major = 235;
        message.ref_pos_xy_conf.semi_minor = 200;
        message.ref_pos_xy_conf.orientation = 25000;
        j3224_v2x_msgs::msg::DetectedObjectList detected_obj_test;
        message.objects = detected_obj_test;
        // std::cout << "Test list: " << detected_obj_test.size() << "\n";
        message.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
        message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_000_50;

        // create detected object data and push it to the detected object list
        j3224_v2x_msgs::msg::DetectedObjectData object1;

        // common data
        object1.detected_object_common_data.obj_type.object_type |= j3224_v2x_msgs::msg::ObjectType::VEHICLE;
        object1.detected_object_common_data.obj_type_cfd.classification_confidence = 65;
        object1.detected_object_common_data.detected_id.object_id = 12200;
        object1.detected_object_common_data.measurement_time.measurement_time_offset = -1100;
        object1.detected_object_common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000;
        object1.detected_object_common_data.pos.offset_x.object_distance = 4000;
        object1.detected_object_common_data.pos.offset_y.object_distance = -720;
        object1.detected_object_common_data.pos.presence_vector |= j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
        object1.detected_object_common_data.pos.offset_z.object_distance = 20;
        object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A200M;
        object1.detected_object_common_data.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_200_00;
        object1.detected_object_common_data.speed.speed = 2500;
        object1.detected_object_common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        object1.detected_object_common_data.heading.heading = 16000;
        object1.detected_object_common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
        object1.detected_object_common_data.speed_z.speed = 1000;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
        object1.detected_object_common_data.speed_confidence_z.speed_confidence = j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
        object1.detected_object_common_data.accel_4_way.longitudinal = 200;
        object1.detected_object_common_data.accel_4_way.lateral = -500;
        object1.detected_object_common_data.accel_4_way.vert = 1;
        object1.detected_object_common_data.accel_4_way.yaw_rate = 400;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
        object1.detected_object_common_data.acc_cfd_x.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
        object1.detected_object_common_data.acc_cfd_y.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
        object1.detected_object_common_data.acc_cfd_z.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05;
        
        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
        object1.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence |= j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00;


        object1.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
        object1.detected_object_optional_data.choice |= j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH;
        // Detected Vehicle Data
        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS;
        object1.detected_object_optional_data.det_veh.lights.exterior_lights |= j2735_v2x_msgs::msg::ExteriorLights::LOW_BEAM_HEADLIGHTS_ON;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE;
        object1.detected_object_optional_data.det_veh.veh_attitude.pitch.pitch_detected = 2400;
        object1.detected_object_optional_data.det_veh.veh_attitude.roll.roll_detected = -12000;
        object1.detected_object_optional_data.det_veh.veh_attitude.yaw.yaw_detected = 400;
        
        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_attitude_confidence.pitch_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_05_DEG;
        object1.detected_object_optional_data.det_veh.veh_attitude_confidence.roll_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
        object1.detected_object_optional_data.det_veh.veh_attitude_confidence.yaw_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL;
        object1.detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate = 600;
        object1.detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate = -800;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_PITCH_RATE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence |= j3224_v2x_msgs::msg::PitchRateConfidence::DEG_SEC_005_00;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_ROLL_RATE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence |= j3224_v2x_msgs::msg::RollRateConfidence::DEG_SEC_001_00;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
        object1.detected_object_optional_data.det_veh.size.vehicle_width = 300;
        object1.detected_object_optional_data.det_veh.size.vehicle_length = 700;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;
        object1.detected_object_optional_data.det_veh.height.vehicle_height = 70;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.presence_vector |= j3224_v2x_msgs::msg::VehicleSizeConfidence::HAS_VEHICLE_HEIGHT_CONFIDENCE;
        object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_005_00;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS;
        object1.detected_object_optional_data.det_veh.vehicle_class.basic_vehicle_class |= j2735_v2x_msgs::msg::BasicVehicleClass::PASSENGER_VEHICLE_TYPE_OTHER;

        object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF;
        object1.detected_object_optional_data.det_veh.class_conf.classification_confidence = 75;

        message.objects.detected_object_data.push_back(object1);


        auto res = worker.encode_sdsm_message(message);
        if(res){
            EXPECT_TRUE(true);
        }
        else{
            std::cout << "Encoding failed while unit testing SDSM encoder!\n";
            EXPECT_TRUE(false);
        }

        auto res_decoded = worker.decode_sdsm_message(res.get());
        if(res_decoded) EXPECT_TRUE(true);
        else{
            std::cout << "Decoding of encoded SDSM failed!\n";
            EXPECT_TRUE(false);
        }

        j3224_v2x_msgs::msg::SensorDataSharingMessage result = res_decoded.get();


        EXPECT_EQ(message, result);

    }

    // Test for encoding/decoding optional data - VRU data
    TEST(SDSMTest, testEncodeDecodeSDSMOpt2){

        auto node = std::make_shared<rclcpp::Node>("test_node");
        cpp_message::SDSM_Message worker(node->get_node_logging_interface());

        j3224_v2x_msgs::msg::SensorDataSharingMessage message;

        message.msg_cnt.msg_cnt = 1;
        message.source_id.id = {1,2,3,4};
        message.equipment_type.equipment_type |= j3224_v2x_msgs::msg::EquipmentType::RSU;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
        message.sdsm_time_stamp.year.year = 2007;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
        message.sdsm_time_stamp.month.month = 7;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
        message.sdsm_time_stamp.day.day = 4;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
        message.sdsm_time_stamp.hour.hour = 19;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
        message.sdsm_time_stamp.minute.minute = 15;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
        message.sdsm_time_stamp.second.millisecond = 5000;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
        message.sdsm_time_stamp.offset.offset_minute = 400;
        message.ref_pos.latitude = 400000000;
        message.ref_pos.longitude = 600000000;
        message.ref_pos.elevation_exists = true;
        message.ref_pos.elevation = 30;
        message.ref_pos_xy_conf.semi_major = 235;
        message.ref_pos_xy_conf.semi_minor = 200;
        message.ref_pos_xy_conf.orientation = 25000;
        j3224_v2x_msgs::msg::DetectedObjectList detected_obj_test;
        message.objects = detected_obj_test;
        message.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
        message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_000_50;

        // create detected object data and push it to the detected object list
        j3224_v2x_msgs::msg::DetectedObjectData object1;

        // common data
        object1.detected_object_common_data.obj_type.object_type |= j3224_v2x_msgs::msg::ObjectType::VEHICLE;
        object1.detected_object_common_data.obj_type_cfd.classification_confidence = 65;
        object1.detected_object_common_data.detected_id.object_id = 12200;
        object1.detected_object_common_data.measurement_time.measurement_time_offset = -1100;
        object1.detected_object_common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000;
        object1.detected_object_common_data.pos.offset_x.object_distance = 4000;
        object1.detected_object_common_data.pos.offset_y.object_distance = -720;
        object1.detected_object_common_data.pos.presence_vector |= j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
        object1.detected_object_common_data.pos.offset_z.object_distance = 20;
        object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A200M;
        object1.detected_object_common_data.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_200_00;
        object1.detected_object_common_data.speed.speed = 2500;
        object1.detected_object_common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        object1.detected_object_common_data.heading.heading = 16000;
        object1.detected_object_common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
        object1.detected_object_common_data.speed_z.speed = 1000;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
        object1.detected_object_common_data.speed_confidence_z.speed_confidence = j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
        object1.detected_object_common_data.accel_4_way.longitudinal = 200;
        object1.detected_object_common_data.accel_4_way.lateral = -500;
        object1.detected_object_common_data.accel_4_way.vert = 1;
        object1.detected_object_common_data.accel_4_way.yaw_rate = 400;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
        object1.detected_object_common_data.acc_cfd_x.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
        object1.detected_object_common_data.acc_cfd_y.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
        object1.detected_object_common_data.acc_cfd_z.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05;
        
        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
        object1.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence |= j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00;


        object1.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;

        // Detected VRU Data
        object1.detected_object_optional_data.choice |= j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU;
        object1.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE;
        object1.detected_object_optional_data.det_vru.basic_type.type |= j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;

        object1.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION;
        object1.detected_object_optional_data.det_vru.propulsion.choice |= j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN;
        object1.detected_object_optional_data.det_vru.propulsion.human.type |= j2735_v2x_msgs::msg::HumanPropelledType::ON_FOOT;

        object1.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT;
        object1.detected_object_optional_data.det_vru.attachment.type |= j2735_v2x_msgs::msg::Attachment::CART;

        object1.detected_object_optional_data.det_vru.presence_vector |= j3224_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS;
        object1.detected_object_optional_data.det_vru.radius.attachment_radius = 30;

        message.objects.detected_object_data.push_back(object1);

        auto res = worker.encode_sdsm_message(message);
        if(res){
            EXPECT_TRUE(true);
        }
        else{
            std::cout << "Encoding failed while unit testing SDSM encoder!\n";
            EXPECT_TRUE(false);
        }

        auto res_decoded = worker.decode_sdsm_message(res.get());
        if(res_decoded) EXPECT_TRUE(true);
        else{
            std::cout << "Decoding of encoded SDSM failed!\n";
            EXPECT_TRUE(false);
        }

        
        j3224_v2x_msgs::msg::SensorDataSharingMessage result = res_decoded.get();

        auto res2 = worker.encode_sdsm_message(result);

        EXPECT_EQ(message, result);

    }

    // Test for encoding/decoding optional data - obstacle data
    TEST(SDSMTest, testEncodeDecodeSDSMOpt3){

        auto node = std::make_shared<rclcpp::Node>("test_node");
        cpp_message::SDSM_Message worker(node->get_node_logging_interface());

        j3224_v2x_msgs::msg::SensorDataSharingMessage message;

        message.msg_cnt.msg_cnt = 1;
        message.source_id.id = {1,2,3,4};
        message.equipment_type.equipment_type |= j3224_v2x_msgs::msg::EquipmentType::RSU;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
        message.sdsm_time_stamp.year.year = 2007;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
        message.sdsm_time_stamp.month.month = 7;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
        message.sdsm_time_stamp.day.day = 4;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
        message.sdsm_time_stamp.hour.hour = 19;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
        message.sdsm_time_stamp.minute.minute = 15;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
        message.sdsm_time_stamp.second.millisecond = 5000;
        message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
        message.sdsm_time_stamp.offset.offset_minute = 400;
        message.ref_pos.latitude = 400000000;
        message.ref_pos.longitude = 600000000;
        message.ref_pos.elevation_exists = true;
        message.ref_pos.elevation = 30;
        message.ref_pos_xy_conf.semi_major = 235;
        message.ref_pos_xy_conf.semi_minor = 200;
        message.ref_pos_xy_conf.orientation = 25000;
        j3224_v2x_msgs::msg::DetectedObjectList detected_obj_test;
        message.objects = detected_obj_test;
        message.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
        message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_000_50;

        // create detected object data and push it to the detected object list
        j3224_v2x_msgs::msg::DetectedObjectData object1;

        // common data
        object1.detected_object_common_data.obj_type.object_type |= j3224_v2x_msgs::msg::ObjectType::VEHICLE;
        object1.detected_object_common_data.obj_type_cfd.classification_confidence = 65;
        object1.detected_object_common_data.detected_id.object_id = 12200;
        object1.detected_object_common_data.measurement_time.measurement_time_offset = -1100;
        object1.detected_object_common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000;
        object1.detected_object_common_data.pos.offset_x.object_distance = 4000;
        object1.detected_object_common_data.pos.offset_y.object_distance = -720;
        object1.detected_object_common_data.pos.presence_vector |= j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
        object1.detected_object_common_data.pos.offset_z.object_distance = 20;
        object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A200M;
        object1.detected_object_common_data.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_200_00;
        object1.detected_object_common_data.speed.speed = 2500;
        object1.detected_object_common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
        object1.detected_object_common_data.heading.heading = 16000;
        object1.detected_object_common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
        object1.detected_object_common_data.speed_z.speed = 1000;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
        object1.detected_object_common_data.speed_confidence_z.speed_confidence = j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
        object1.detected_object_common_data.accel_4_way.longitudinal = 200;
        object1.detected_object_common_data.accel_4_way.lateral = -500;
        object1.detected_object_common_data.accel_4_way.vert = 1;
        object1.detected_object_common_data.accel_4_way.yaw_rate = 400;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
        object1.detected_object_common_data.acc_cfd_x.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
        object1.detected_object_common_data.acc_cfd_y.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10;

        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
        object1.detected_object_common_data.acc_cfd_z.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05;
        
        object1.detected_object_common_data.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
        object1.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence |= j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00;


        object1.presence_vector |= j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;

        // Detected Obstacle Data
        object1.detected_object_optional_data.choice |= j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST;

        object1.detected_object_optional_data.det_obst.obst_size.width.size_value = 400;
        object1.detected_object_optional_data.det_obst.obst_size.length.size_value = 300;
        object1.detected_object_optional_data.det_obst.obst_size.presence_vector |= j3224_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT;
        object1.detected_object_optional_data.det_obst.obst_size.height.size_value = 100;

        object1.detected_object_optional_data.det_obst.obst_size_confidence.width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00;
        object1.detected_object_optional_data.det_obst.obst_size_confidence.length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00;
        object1.detected_object_optional_data.det_obst.obst_size_confidence.presence_vector |= j3224_v2x_msgs::msg::ObstacleSizeConfidence::HAS_HEIGHT_CONFIDENCE;
        object1.detected_object_optional_data.det_obst.obst_size_confidence.height_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_000_50;

        message.objects.detected_object_data.push_back(object1);


        auto res = worker.encode_sdsm_message(message);
        if(res){
            EXPECT_TRUE(true);
        }
        else{
            std::cout << "Encoding failed while unit testing SDSM encoder!\n";
            EXPECT_TRUE(false);
        }

        auto res_decoded = worker.decode_sdsm_message(res.get());
        if(res_decoded) EXPECT_TRUE(true);
        else{
            std::cout << "Decoding of encoded SDSM failed!\n";
            EXPECT_TRUE(false);
        }

        
        j3224_v2x_msgs::msg::SensorDataSharingMessage result = res_decoded.get();

        auto res2 = worker.encode_sdsm_message(result);

        EXPECT_EQ(message, result);

    }

}