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

#include <gtest/gtest.h>
#include <j2735_convertor/sdsm_convertor.hpp>

   // detected_object_list
     // detected_object_data
       // presence_vector
       // detected_object_common_data
        // obj_type                     - no conversion
        // obj_type_cfd                 - no conversion
        // detected_id                  - no conversion
        // measurement_time
        // time_confidence              - no conversion
        // pos
          // offset_x
            // object_distance
          // offset_y
            // object_distance
          // presence_vector
          // offset_z
            // object_distance
        // pos_confidence               - no conversion
        // speed
        // speed_confidence             - no conversion
        // heading
        // heading_conf                 - no conversion
        // presence_vector
        // speed_z
        // speed_confidence_z           - no conversion
        // accel_4_way
        // acc_cfd_x                    - no conversion
        // acc_cfd_y                    - no conversion
        // acc_cfd_z                    - no conversion
        // acc_cfd_yaw                  - no conversion
     // detected_object_optional_data
        // det_veh
          // presence_vector
          // lights                     - no conversion
          // veh_attitude
            // pitch
            // roll
            // yaw
          // veh_attitude_confidence    - no conversion
          // veh_ang_vel
            // pitch_rate
            // roll_rate
          // veh_ang_vel_confidence     - no conversion
          // size
            // vehicle_length
            // vehicle_width
          // height
          // vehicle_size_confidence    - no conversion
          // vehicle_class              - no conversion
          // class_conf                 - no conversion
        // det_vru
          // presence_vector
          // basic_type                 - no conversion
          // propulsion                 - no conversion
          // attachment                 - no conversion
          // radius
        // det_obst
          // obst_size
            // width
            // length
            // presence_vector
            // height
          // obst_size_confidence       - no conversion

namespace j2735_convertor
{

TEST(ControlRequest, convertSDSMj3224ToCAV)
{
    j3224_v2x_msgs::msg::SensorDataSharingMessage message;

    // Main SDSM message contents
    message.msg_cnt.msg_cnt = 1;
    message.source_id.id = {1,2,3,4};
    message.equipment_type.equipment_type = j3224_v2x_msgs::msg::EquipmentType::RSU;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    message.sdsm_time_stamp.year.year = 2000;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    message.sdsm_time_stamp.month.month = 7;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    message.sdsm_time_stamp.day.day = 4;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    message.sdsm_time_stamp.hour.hour = 21;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    message.sdsm_time_stamp.minute.minute = 15;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    message.sdsm_time_stamp.second.millisecond = 10000;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    message.sdsm_time_stamp.offset.offset_minute = 600;
    
    message.ref_pos.latitude = 40000000;
    message.ref_pos.longitude = 50000000;
    message.ref_pos.elevation_exists = true;
    message.ref_pos.elevation = 70;

    message.ref_pos_xy_conf.semi_major = 160;
    message.ref_pos_xy_conf.semi_minor = 120;
    message.ref_pos_xy_conf.orientation = 364;

    message.presence_vector |= j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
    message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_001_00;


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
    object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A50M;
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
    object1.detected_object_optional_data.det_veh.lights.exterior_lights |= j2735_v2x_msgs::msg::ExteriorLights::HAZARD_SIGNAL_ON; // bit string for each light, may need to change

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

    object1.detected_object_optional_data.det_veh.presence_vector |= j3224_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE; // width & length have their own msgs?
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


    // Create a carma message and convert the j3224 message into it
    carma_v2x_msgs::msg::SensorDataSharingMessage out_message;
    j2735_convertor::SDSMConvertor::convert(message, out_message);

    // Verification of conversion
    ASSERT_EQ(out_message.msg_cnt.msg_cnt, 1);
    std::vector<uint8_t> id = {1,2,3,4};
    ASSERT_EQ(out_message.source_id.id, id);
    ASSERT_EQ(out_message.equipment_type.equipment_type, j3224_v2x_msgs::msg::EquipmentType::RSU);
    ASSERT_EQ(out_message.sdsm_time_stamp.year.year, 2000);
    ASSERT_EQ(out_message.sdsm_time_stamp.month.month, 7);
    ASSERT_EQ(out_message.sdsm_time_stamp.day.day, 4);
    ASSERT_EQ(out_message.sdsm_time_stamp.hour.hour, 21);
    ASSERT_EQ(out_message.sdsm_time_stamp.minute.minute, 15);
    ASSERT_EQ(out_message.sdsm_time_stamp.second.millisecond, 10000);
    ASSERT_EQ(out_message.sdsm_time_stamp.offset.offset_minute, 600);
    ASSERT_EQ(out_message.ref_pos.latitude, 4);
    ASSERT_EQ(out_message.ref_pos.longitude, 5);
    ASSERT_EQ(out_message.ref_pos.elevation, 7);
    ASSERT_EQ(out_message.ref_pos_xy_conf.semi_major, 8);
    ASSERT_EQ(out_message.ref_pos_xy_conf.semi_minor, 6);
    ASSERT_NEAR(out_message.ref_pos_xy_conf.orientation, 2, 0.01);
    ASSERT_EQ(out_message.ref_pos_el_conf.confidence, j2735_v2x_msgs::msg::ElevationConfidence::ELEV_001_00);

    // Checking object common data and optional detected vehicle data
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.obj_type.object_type, j3224_v2x_msgs::msg::ObjectType::VEHICLE);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.obj_type_cfd.classification_confidence, 65);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.detected_id.object_id, 12200);
    ASSERT_NEAR(out_message.objects.detected_object_data[0].detected_object_common_data.measurement_time.measurement_time_offset, -1.1, 0.01);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.time_confidence.confidence, j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos.offset_x.object_distance, 400);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos.offset_y.object_distance, -72);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos.offset_z.object_distance, 2);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos_confidence.pos.confidence, j2735_v2x_msgs::msg::PositionConfidence::A50M);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed.speed, 50);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed_confidence.speed_confidence, j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.heading.heading, 200);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.heading_conf.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG);

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed_z.speed, 20);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed_confidence_z.speed_confidence, j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS);

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.longitudinal, 2);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.lateral, -5);
    ASSERT_NEAR(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.vert, 0.196, 0.01);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.yaw_rate, 4);

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_x.acceleration_confidence, j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_y.acceleration_confidence, j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_z.acceleration_confidence, j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence, j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00);
    

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.choice, j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.lights.exterior_lights, j2735_v2x_msgs::msg::ExteriorLights::HAZARD_SIGNAL_ON);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude.pitch.pitch_detected, 30);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude.roll.roll_detected, -150);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude.yaw.yaw_detected, 5);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude_confidence.pitch_confidence.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_05_DEG);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude_confidence.roll_confidence.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude_confidence.yaw_confidence.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate, 6);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate, -8);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence, j3224_v2x_msgs::msg::PitchRateConfidence::DEG_SEC_005_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence, j3224_v2x_msgs::msg::RollRateConfidence::DEG_SEC_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.size.vehicle_width, 3);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.size.vehicle_length, 7);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.height.vehicle_height, 3.5);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_005_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_class.basic_vehicle_class, j2735_v2x_msgs::msg::BasicVehicleClass::PASSENGER_VEHICLE_TYPE_OTHER);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.class_conf.classification_confidence, 75);

    // Checking optional VRU data
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.choice, j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.basic_type.type, j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.propulsion.choice, j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.propulsion.human.type, j2735_v2x_msgs::msg::HumanPropelledType::ON_FOOT);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.attachment.type, j2735_v2x_msgs::msg::Attachment::CART);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.radius.attachment_radius, 3);

    // Checking optional detected obstacle data
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.choice, j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size.width.size_value, 40);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size.length.size_value, 30);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size.height.size_value, 10);

    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size_confidence.width_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size_confidence.length_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size_confidence.height_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_000_50);

}



TEST(ControlRequest, convertSDSMcavToJ3224)
{
    carma_v2x_msgs::msg::SensorDataSharingMessage message;

    // Main SDSM message contents
    message.msg_cnt.msg_cnt = 1;
    message.source_id.id = {1,2,3,4};
    message.equipment_type.equipment_type = j3224_v2x_msgs::msg::EquipmentType::RSU;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    message.sdsm_time_stamp.year.year = 2000;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    message.sdsm_time_stamp.month.month = 7;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    message.sdsm_time_stamp.day.day = 4;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    message.sdsm_time_stamp.hour.hour = 21;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    message.sdsm_time_stamp.minute.minute = 15;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    message.sdsm_time_stamp.second.millisecond = 10000;
    message.sdsm_time_stamp.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    message.sdsm_time_stamp.offset.offset_minute = 600;
    
    message.ref_pos.latitude = 4;
    message.ref_pos.longitude = 5;
    message.ref_pos.elevation_exists = true;
    message.ref_pos.elevation = 7;

    message.ref_pos_xy_conf.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
    message.ref_pos_xy_conf.semi_major = 8;
    message.ref_pos_xy_conf.semi_minor = 6;
    message.ref_pos_xy_conf.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
    message.ref_pos_xy_conf.orientation = 2;

    message.presence_vector |= carma_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF;
    message.ref_pos_el_conf.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_001_00;


    // create detected object data and push it to the detected object list
    carma_v2x_msgs::msg::DetectedObjectData object1;

    // common data
    object1.detected_object_common_data.obj_type.object_type |= j3224_v2x_msgs::msg::ObjectType::VEHICLE;
    object1.detected_object_common_data.obj_type_cfd.classification_confidence = 65;
    object1.detected_object_common_data.detected_id.object_id = 12200;
    object1.detected_object_common_data.measurement_time.measurement_time_offset = -1.1;
    object1.detected_object_common_data.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000;
    object1.detected_object_common_data.pos.offset_x.object_distance = 400;
    object1.detected_object_common_data.pos.offset_y.object_distance = -72;
    object1.detected_object_common_data.pos.presence_vector |= carma_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;
    object1.detected_object_common_data.pos.offset_z.object_distance = 2;
    object1.detected_object_common_data.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A50M;
    object1.detected_object_common_data.speed.speed = 50;
    object1.detected_object_common_data.speed_confidence.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
    object1.detected_object_common_data.heading.heading = 200;
    object1.detected_object_common_data.heading_conf.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
    object1.detected_object_common_data.speed_z.speed = 20;

    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z;
    object1.detected_object_common_data.speed_confidence_z.speed_confidence = j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS;

    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY;
    object1.detected_object_common_data.accel_4_way.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE; 
    object1.detected_object_common_data.accel_4_way.longitudinal = 2;
    object1.detected_object_common_data.accel_4_way.lateral = -5;
    object1.detected_object_common_data.accel_4_way.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE;
    object1.detected_object_common_data.accel_4_way.vert = 1;
    object1.detected_object_common_data.accel_4_way.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE;
    object1.detected_object_common_data.accel_4_way.yaw_rate = 4;

    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X;
    object1.detected_object_common_data.acc_cfd_x.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00;

    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y;
    object1.detected_object_common_data.acc_cfd_y.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10;

    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z;
    object1.detected_object_common_data.acc_cfd_z.acceleration_confidence |= j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05;
    
    object1.detected_object_common_data.presence_vector |= carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW;
    object1.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence |= j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00;


    object1.presence_vector |= carma_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
    object1.detected_object_optional_data.choice |= carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH;
    // Detected Vehicle Data
    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS;
    object1.detected_object_optional_data.det_veh.lights.exterior_lights |= j2735_v2x_msgs::msg::ExteriorLights::HAZARD_SIGNAL_ON;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE;
    object1.detected_object_optional_data.det_veh.veh_attitude.pitch.pitch_detected = 30;
    object1.detected_object_optional_data.det_veh.veh_attitude.roll.roll_detected = -150;
    object1.detected_object_optional_data.det_veh.veh_attitude.yaw.yaw_detected = 5;
    
    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE;
    object1.detected_object_optional_data.det_veh.veh_attitude_confidence.pitch_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_05_DEG;
    object1.detected_object_optional_data.det_veh.veh_attitude_confidence.roll_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
    object1.detected_object_optional_data.det_veh.veh_attitude_confidence.yaw_confidence.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL;
    object1.detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate = 6;
    object1.detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate = -8;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE;
    object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_PITCH_RATE_CONFIDENCE;
    object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence |= j3224_v2x_msgs::msg::PitchRateConfidence::DEG_SEC_005_00;
    object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.presence_vector |= j3224_v2x_msgs::msg::AngularVelocityConfidence::HAS_ROLL_RATE_CONFIDENCE;
    object1.detected_object_optional_data.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence |= j3224_v2x_msgs::msg::RollRateConfidence::DEG_SEC_001_00;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
    object1.detected_object_optional_data.det_veh.size.presence_vector |= carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE;
    object1.detected_object_optional_data.det_veh.size.vehicle_width = 3;
    object1.detected_object_optional_data.det_veh.size.presence_vector |= carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE;
    object1.detected_object_optional_data.det_veh.size.vehicle_length = 7;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;
    object1.detected_object_optional_data.det_veh.height.vehicle_height = 3.5;      

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE;
    object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00;
    object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00;
    object1.detected_object_optional_data.det_veh.vehicle_size_confidence.presence_vector |= j3224_v2x_msgs::msg::VehicleSizeConfidence::HAS_VEHICLE_HEIGHT_CONFIDENCE;
    object1.detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_005_00;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS;
    object1.detected_object_optional_data.det_veh.vehicle_class.basic_vehicle_class |= j2735_v2x_msgs::msg::BasicVehicleClass::PASSENGER_VEHICLE_TYPE_OTHER;

    object1.detected_object_optional_data.det_veh.presence_vector |= carma_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF;
    object1.detected_object_optional_data.det_veh.class_conf.classification_confidence = 75;

    message.objects.detected_object_data.push_back(object1);



    carma_v2x_msgs::msg::DetectedObjectData object2;
    object2.presence_vector |= carma_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
    object2.detected_object_optional_data.choice |= carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU;

    // Detected VRU Data
    object2.detected_object_optional_data.det_vru.presence_vector |= carma_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE;
    object2.detected_object_optional_data.det_vru.basic_type.type |= j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;

    object2.detected_object_optional_data.det_vru.presence_vector |= carma_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION;
    object2.detected_object_optional_data.det_vru.propulsion.choice |= j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN;
    object2.detected_object_optional_data.det_vru.propulsion.human.type |= j2735_v2x_msgs::msg::HumanPropelledType::ON_FOOT;

    object2.detected_object_optional_data.det_vru.presence_vector |= carma_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT;
    object2.detected_object_optional_data.det_vru.attachment.type |= j2735_v2x_msgs::msg::Attachment::CART;

    object2.detected_object_optional_data.det_vru.presence_vector |= carma_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS;
    object2.detected_object_optional_data.det_vru.radius.attachment_radius = 3;

    message.objects.detected_object_data.push_back(object2);



    carma_v2x_msgs::msg::DetectedObjectData object3;
    object3.presence_vector |= carma_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA;
    object3.detected_object_optional_data.choice |= carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST;

    // Detected Obstacle Data
    object3.detected_object_optional_data.det_obst.obst_size.width.size_value = 40;
    object3.detected_object_optional_data.det_obst.obst_size.length.size_value = 30;
    object3.detected_object_optional_data.det_obst.obst_size.presence_vector |= carma_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT;
    object3.detected_object_optional_data.det_obst.obst_size.height.size_value = 10;

    object3.detected_object_optional_data.det_obst.obst_size_confidence.width_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00;
    object3.detected_object_optional_data.det_obst.obst_size_confidence.length_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00;
    object3.detected_object_optional_data.det_obst.obst_size_confidence.presence_vector |= j3224_v2x_msgs::msg::ObstacleSizeConfidence::HAS_HEIGHT_CONFIDENCE;
    object3.detected_object_optional_data.det_obst.obst_size_confidence.height_confidence.size_value_confidence |= j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_000_50;

    message.objects.detected_object_data.push_back(object3);


    // Create a j3224 message and convert the carma message into it
    j3224_v2x_msgs::msg::SensorDataSharingMessage out_message;
    j2735_convertor::SDSMConvertor::convert(message, out_message);

    // Verification of conversion
    ASSERT_EQ(out_message.msg_cnt.msg_cnt, 1);
    std::vector<uint8_t> id = {1,2,3,4};
    ASSERT_EQ(out_message.source_id.id, id);
    ASSERT_EQ(out_message.equipment_type.equipment_type, j3224_v2x_msgs::msg::EquipmentType::RSU);
    ASSERT_EQ(out_message.sdsm_time_stamp.year.year, 2000);
    ASSERT_EQ(out_message.sdsm_time_stamp.month.month, 7);
    ASSERT_EQ(out_message.sdsm_time_stamp.day.day, 4);
    ASSERT_EQ(out_message.sdsm_time_stamp.hour.hour, 21);
    ASSERT_EQ(out_message.sdsm_time_stamp.minute.minute, 15);
    ASSERT_EQ(out_message.sdsm_time_stamp.second.millisecond, 10000);
    ASSERT_EQ(out_message.sdsm_time_stamp.offset.offset_minute, 600);
    ASSERT_EQ(out_message.ref_pos.latitude, 40000000);
    ASSERT_EQ(out_message.ref_pos.longitude, 50000000);
    ASSERT_EQ(out_message.ref_pos.elevation, 70);
    ASSERT_EQ(out_message.ref_pos_xy_conf.semi_major, 160);
    ASSERT_EQ(out_message.ref_pos_xy_conf.semi_minor, 120);
    ASSERT_NEAR(out_message.ref_pos_xy_conf.orientation, 364, 0.1);
    ASSERT_EQ(out_message.ref_pos_el_conf.confidence, j2735_v2x_msgs::msg::ElevationConfidence::ELEV_001_00);

    // Checking object common data and optional detected vehicle data
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.obj_type.object_type, j3224_v2x_msgs::msg::ObjectType::VEHICLE);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.obj_type_cfd.classification_confidence, 65);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.detected_id.object_id, 12200);
    ASSERT_NEAR(out_message.objects.detected_object_data[0].detected_object_common_data.measurement_time.measurement_time_offset, -1100, 0.1);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.time_confidence.confidence, j2735_v2x_msgs::msg::TimeConfidence::TIME_050_000);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos.offset_x.object_distance, 4000);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos.offset_y.object_distance, -720);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos.offset_z.object_distance, 20);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.pos_confidence.pos.confidence, j2735_v2x_msgs::msg::PositionConfidence::A50M);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed.speed, 2500);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed_confidence.speed_confidence, j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.heading.heading, 16000);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.heading_conf.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG);

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed_z.speed, 1000);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.speed_confidence_z.speed_confidence, j2735_v2x_msgs::msg::SpeedConfidence::PREC1MS);

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.longitudinal, 200);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.lateral, -500);
    ASSERT_NEAR(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.vert, 5, 0.01);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.accel_4_way.yaw_rate, 400);

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_x.acceleration_confidence, j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_y.acceleration_confidence, j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_10);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_z.acceleration_confidence, j2735_v2x_msgs::msg::AccelerationConfidence::ACCL_000_05);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence, j2735_v2x_msgs::msg::YawRateConfidence::DEG_SEC_005_00);
    

    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.choice, carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.lights.exterior_lights, j2735_v2x_msgs::msg::ExteriorLights::HAZARD_SIGNAL_ON);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude.pitch.pitch_detected, 2400);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude.roll.roll_detected, -12000);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude.yaw.yaw_detected, 400);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude_confidence.pitch_confidence.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_05_DEG);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude_confidence.roll_confidence.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_attitude_confidence.yaw_confidence.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_001_DEG);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel.pitch_rate.pitch_rate, 600);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel.roll_rate.roll_rate, -800);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel_confidence.pitch_rate_confidence.pitch_rate_confidence, j3224_v2x_msgs::msg::PitchRateConfidence::DEG_SEC_005_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.veh_ang_vel_confidence.roll_rate_confidence.roll_rate_confidence, j3224_v2x_msgs::msg::RollRateConfidence::DEG_SEC_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.size.vehicle_width, 300);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.size.vehicle_length, 700);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.height.vehicle_height, 70);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_width_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_length_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_size_confidence.vehicle_height_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_005_00);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.vehicle_class.basic_vehicle_class, j2735_v2x_msgs::msg::BasicVehicleClass::PASSENGER_VEHICLE_TYPE_OTHER);
    ASSERT_EQ(out_message.objects.detected_object_data[0].detected_object_optional_data.det_veh.class_conf.classification_confidence, 75);

    // Checking optional VRU data
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.choice, carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.basic_type.type, j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.propulsion.choice, j2735_v2x_msgs::msg::PropelledInformation::CHOICE_HUMAN);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.propulsion.human.type, j2735_v2x_msgs::msg::HumanPropelledType::ON_FOOT);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.attachment.type, j2735_v2x_msgs::msg::Attachment::CART);
    ASSERT_EQ(out_message.objects.detected_object_data[1].detected_object_optional_data.det_vru.radius.attachment_radius, 30);

    // Checking optional detected obstacle data
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.choice, carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size.width.size_value, 400);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size.length.size_value, 300);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size.height.size_value, 100);

    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size_confidence.width_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_002_00);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size_confidence.length_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_001_00);
    ASSERT_EQ(out_message.objects.detected_object_data[2].detected_object_optional_data.det_obst.obst_size_confidence.height_confidence.size_value_confidence, j3224_v2x_msgs::msg::SizeValueConfidence::SIZE_000_50);

}

} // namespace j2735_convertor