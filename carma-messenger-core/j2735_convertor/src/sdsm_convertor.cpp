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

#include <j2735_convertor/sdsm_convertor.hpp>

/**
 * CPP File containing SDSMConvertor method definitions
 */

namespace j2735_convertor
{

void SDSMConvertor::convert(const j3224_v2x_msgs::msg::SensorDataSharingMessage& in_msg, carma_v2x_msgs::msg::SensorDataSharingMessage& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.ref_pos, out_msg.ref_pos);
    convert(in_msg.ref_pos_xy_conf, out_msg.ref_pos_xy_conf);

    if(in_msg.presence_vector & j3224_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF){
        out_msg.ref_pos_el_conf = in_msg.ref_pos_el_conf;
    }

    out_msg.msg_cnt = in_msg.msg_cnt;
    out_msg.source_id = in_msg.source_id;
    out_msg.equipment_type = in_msg.equipment_type;
    out_msg.sdsm_time_stamp = in_msg.sdsm_time_stamp;

    // Create a carma_v2x_msgs list to copy each detected object into
    carma_v2x_msgs::msg::DetectedObjectList detected_object_list;

    // Go through the array of j3224 detected objects and convert their DetectedObjectData
    for(auto i : in_msg.objects.detected_object_data){
        carma_v2x_msgs::msg::DetectedObjectData j;
        
        convert(i, j);

        // Add the converted object to the carma_v2x_msgs array
        detected_object_list.detected_object_data.push_back(j);
    }
    // Set the placeholder list as the carma_v2x_msgs output
    out_msg.objects = detected_object_list;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::SensorDataSharingMessage& in_msg, j3224_v2x_msgs::msg::SensorDataSharingMessage& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.ref_pos, out_msg.ref_pos);
    convert(in_msg.ref_pos_xy_conf, out_msg.ref_pos_xy_conf);


    if(in_msg.presence_vector & carma_v2x_msgs::msg::SensorDataSharingMessage::HAS_REF_POS_EL_CONF){
        out_msg.ref_pos_el_conf = in_msg.ref_pos_el_conf;
    }

    out_msg.msg_cnt = in_msg.msg_cnt;
    out_msg.source_id = in_msg.source_id;
    out_msg.equipment_type = in_msg.equipment_type;
    out_msg.sdsm_time_stamp = in_msg.sdsm_time_stamp;

    // Create a j3224_v2x_msgs list to copy each detected object into
    j3224_v2x_msgs::msg::DetectedObjectList detected_object_list;

    // Go through the array of carma detected objects and convert their DetectedObjectData
    for(auto i : in_msg.objects.detected_object_data){
        j3224_v2x_msgs::msg::DetectedObjectData j;
        
        convert(i, j);

        // Add the converted object to the j3224_v2x_msgs array
        detected_object_list.detected_object_data.push_back(j);
    }
    // Set the placeholder list as the j3224_v2x_msgs output
    out_msg.objects = detected_object_list;
}


////
// Convert j2735_msgs
////


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::YawRate& in_msg, carma_v2x_msgs::msg::YawRate& out_msg)
{
    out_msg.yaw_rate = in_msg.yaw_rate * units::HUNDREDTH_DEG_PER_S;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::YawRate& in_msg, j2735_v2x_msgs::msg::YawRate& out_msg)
{
    out_msg.yaw_rate = in_msg.yaw_rate / units::HUNDREDTH_DEG_PER_S;
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::Position3D& in_msg, carma_v2x_msgs::msg::Position3D& out_msg)
{
    out_msg.latitude = in_msg.latitude / units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.longitude = in_msg.longitude / units::TENTH_MICRO_DEG_PER_DEG;
    if(in_msg.elevation_exists){
        out_msg.elevation_exists = true;
        out_msg.elevation = in_msg.elevation / units::DECI_M_PER_M;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::Position3D& in_msg, j2735_v2x_msgs::msg::Position3D& out_msg)
{
    out_msg.latitude = in_msg.latitude * units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.longitude = in_msg.longitude * units::TENTH_MICRO_DEG_PER_DEG;
    if(in_msg.elevation_exists){
        out_msg.elevation_exists = true;
        out_msg.elevation = in_msg.elevation * units::DECI_M_PER_M;
    } 
}


// From psm_convertor
void SDSMConvertor::convert(const j2735_v2x_msgs::msg::PositionalAccuracy& in_msg, carma_v2x_msgs::msg::PositionalAccuracy& out_msg)
{
  // Convert semi_major Axis
  out_msg.semi_major = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.semi_major, units::TWENTIETH_M_PER_M, out_msg.presence_vector,
      carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE);

  // Convert semi_minor Axis
  out_msg.semi_minor = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.semi_minor, units::TWENTIETH_M_PER_M, out_msg.presence_vector,
      carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE);

  // Convert Orientation
  out_msg.orientation = ValueConvertor::valueJ2735ToCav<double>(
      in_msg.orientation, units::DEG_360_OVER_65535_PER_DEG, out_msg.presence_vector,
      carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE,
      j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE);
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::PositionalAccuracy& in_msg, j2735_v2x_msgs::msg::PositionalAccuracy& out_msg)
{
  // Convert semi_major Axis
  out_msg.semi_major = ValueConvertor::valueCavToJ2735<uint8_t>(
      in_msg.semi_major, units::TWENTIETH_M_PER_M, in_msg.presence_vector,
      carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE);

  // Convert semi_minor Axis
  out_msg.semi_minor = ValueConvertor::valueCavToJ2735<uint8_t>(
      in_msg.semi_minor, units::TWENTIETH_M_PER_M, in_msg.presence_vector,
      carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE, j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE);

  // Convert Orientation
  out_msg.orientation = ValueConvertor::valueCavToJ2735<uint16_t>(
      in_msg.orientation, units::DEG_360_OVER_65535_PER_DEG, in_msg.presence_vector,
      carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE,
      j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE);
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::Speed& in_msg, carma_v2x_msgs::msg::Speed& out_msg)
{
    if(in_msg.speed != j2735_v2x_msgs::msg::Speed::UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.speed = in_msg.speed / units::FIFTIETH_M_PER_M;
    }
    else{
        out_msg.unavailable = true;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::Speed& in_msg, j2735_v2x_msgs::msg::Speed& out_msg)
{
    if(!in_msg.unavailable){
    out_msg.speed = in_msg.speed * units::FIFTIETH_M_PER_M;
    }
    else{
    out_msg.speed = j2735_v2x_msgs::msg::Speed::UNAVAILABLE;
    }
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::Heading& in_msg, carma_v2x_msgs::msg::Heading& out_msg)
{
    if(in_msg.heading != j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.heading = in_msg.heading / units::EIGHTIETH_DEG_PER_DEG;
    }
    else{
        out_msg.unavailable = true;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::Heading& in_msg, j2735_v2x_msgs::msg::Heading& out_msg)
{
    if(!in_msg.unavailable){
        out_msg.heading = in_msg.heading * units::EIGHTIETH_DEG_PER_DEG;
    }
    else{
        out_msg.heading = j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE;
    }
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::AccelerationSet4Way& in_msg, carma_v2x_msgs::msg::AccelerationSet4Way& out_msg)
{
  // Convert Longitudinal and Lateral
  out_msg.longitudinal = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.longitudinal, units::CM_PER_M, out_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);

  out_msg.lateral = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.lateral, units::CM_PER_M, out_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);

  // Convert Vertical
  out_msg.vert = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.vert, units::FIFTIETH_G_PER_M_PER_SEC_SQR, out_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE,
      j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE);

  // Convert Yaw Rate
  out_msg.yaw_rate = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.yaw_rate, units::CENTI_DEG_PER_DEG, out_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE, j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_UNAVAILABLE);
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::AccelerationSet4Way& in_msg, j2735_v2x_msgs::msg::AccelerationSet4Way& out_msg)
{
  // Convert Longitudinal and Lateral
  out_msg.longitudinal = ValueConvertor::valueCavToJ2735<int16_t>(
      in_msg.longitudinal, units::CM_PER_M, in_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);

  out_msg.lateral = ValueConvertor::valueCavToJ2735<int16_t>(in_msg.lateral, units::CM_PER_M, in_msg.presence_vector,
                                                             carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE,
                                                             j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);

  // Convert Vertical
  out_msg.vert =
      ValueConvertor::valueCavToJ2735<int8_t>(in_msg.vert, units::FIFTIETH_G_PER_M_PER_SEC_SQR, in_msg.presence_vector,
                                              carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE,
                                              j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_UNAVAILABLE);

  // Convert Yaw Rate
  out_msg.yaw_rate = ValueConvertor::valueCavToJ2735<int16_t>(
      in_msg.yaw_rate, units::CENTI_DEG_PER_DEG, in_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE, j2735_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_UNAVAILABLE);
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::VehicleSize& in_msg, carma_v2x_msgs::msg::VehicleSize& out_msg)
{
  // Convert Vehicle Width
  out_msg.vehicle_width = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.vehicle_width, units::CM_PER_M, out_msg.presence_vector, carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE,
      j2735_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_UNAVAILABLE);

  // Convert Vehicle length
  out_msg.vehicle_length = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.vehicle_length, units::CM_PER_M, out_msg.presence_vector, carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE,
      j2735_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_UNAVAILABLE);
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::VehicleSize& in_msg, j2735_v2x_msgs::msg::VehicleSize& out_msg)
{
  // Convert Vehicle Width
  out_msg.vehicle_width = ValueConvertor::valueCavToJ2735<float>(
      in_msg.vehicle_width, units::CM_PER_M, in_msg.presence_vector, carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE,
      j2735_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_UNAVAILABLE);

  // Convert Vehicle length
  out_msg.vehicle_length = ValueConvertor::valueCavToJ2735<float>(
      in_msg.vehicle_length, units::CM_PER_M, in_msg.presence_vector, carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE,
      j2735_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_UNAVAILABLE);
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::VehicleHeight& in_msg, carma_v2x_msgs::msg::VehicleHeight& out_msg)
{
    if(in_msg.vehicle_height != j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.vehicle_height = in_msg.vehicle_height / units::FIVE_CM_PER_M;
    }
    else{
        out_msg.unavailable = true;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::VehicleHeight& in_msg, j2735_v2x_msgs::msg::VehicleHeight& out_msg)
{
    if(!in_msg.unavailable){
        out_msg.vehicle_height = in_msg.vehicle_height * units::FIVE_CM_PER_M;
    }
    else{
        out_msg.vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE;
    }
}


void SDSMConvertor::convert(const j2735_v2x_msgs::msg::AttachmentRadius& in_msg, carma_v2x_msgs::msg::AttachmentRadius& out_msg) 
{
    out_msg.attachment_radius = in_msg.attachment_radius / units::DECI_M_PER_M;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::AttachmentRadius& in_msg, j2735_v2x_msgs::msg::AttachmentRadius& out_msg) 
{
    out_msg.attachment_radius = in_msg.attachment_radius * units::DECI_M_PER_M;
}


////
// Convert j3224_msgs
////


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::DetectedObjectData& in_msg, carma_v2x_msgs::msg::DetectedObjectData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.detected_object_common_data, out_msg.detected_object_common_data);

    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA){
        convert(in_msg.detected_object_optional_data, out_msg.detected_object_optional_data);
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::DetectedObjectData& in_msg, j3224_v2x_msgs::msg::DetectedObjectData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.detected_object_common_data, out_msg.detected_object_common_data);

    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectData::HAS_DETECTED_OBJECT_OPTIONAL_DATA){
        convert(in_msg.detected_object_optional_data, out_msg.detected_object_optional_data);
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::DetectedObjectCommonData& in_msg, carma_v2x_msgs::msg::DetectedObjectCommonData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.measurement_time, out_msg.measurement_time);
    convert(in_msg.pos, out_msg.pos);
    convert(in_msg.speed, out_msg.speed);
    convert(in_msg.heading, out_msg.heading);

    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z){
        convert(in_msg.speed_z, out_msg.speed_z);
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z){
        out_msg.speed_confidence_z = in_msg.speed_confidence_z;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY){
        convert(in_msg.accel_4_way, out_msg.accel_4_way);
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X){
        out_msg.acc_cfd_x = in_msg.acc_cfd_x;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y){
        out_msg.acc_cfd_y = in_msg.acc_cfd_y;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z){
        out_msg.acc_cfd_z = in_msg.acc_cfd_z;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW){
        out_msg.acc_cfd_yaw = in_msg.acc_cfd_yaw;
    }

    out_msg.obj_type = in_msg.obj_type;
    out_msg.obj_type_cfd = in_msg.obj_type_cfd;
    out_msg.detected_id = in_msg.detected_id;
    out_msg.time_confidence = in_msg.time_confidence;
    out_msg.pos_confidence = in_msg.pos_confidence;
    out_msg.speed_confidence = in_msg.speed_confidence;
    out_msg.heading_conf = in_msg.heading_conf;

}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::DetectedObjectCommonData& in_msg, j3224_v2x_msgs::msg::DetectedObjectCommonData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.measurement_time, out_msg.measurement_time);
    convert(in_msg.pos, out_msg.pos);
    convert(in_msg.speed, out_msg.speed);
    convert(in_msg.heading, out_msg.heading);

    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z){
        convert(in_msg.speed_z, out_msg.speed_z);
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_CONFIDENCE_Z){
        out_msg.speed_confidence_z = in_msg.speed_confidence_z;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACCEL_4_WAY){
        convert(in_msg.accel_4_way, out_msg.accel_4_way);
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_X){
        out_msg.acc_cfd_x = in_msg.acc_cfd_x;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Y){
        out_msg.acc_cfd_y = in_msg.acc_cfd_y;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_Z){
        out_msg.acc_cfd_z = in_msg.acc_cfd_z;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_ACC_CFD_YAW){
        out_msg.acc_cfd_yaw = in_msg.acc_cfd_yaw;
    }

    out_msg.obj_type = in_msg.obj_type;
    out_msg.obj_type_cfd = in_msg.obj_type_cfd;
    out_msg.detected_id = in_msg.detected_id;
    out_msg.time_confidence = in_msg.time_confidence;
    out_msg.pos_confidence = in_msg.pos_confidence;
    out_msg.speed_confidence = in_msg.speed_confidence;
    out_msg.heading_conf = in_msg.heading_conf;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::DetectedObjectOptionalData& in_msg, carma_v2x_msgs::msg::DetectedObjectOptionalData& out_msg)
{
    out_msg.choice = in_msg.choice;

    if(in_msg.choice == j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH){
        convert(in_msg.det_veh, out_msg.det_veh);
    }
    if(in_msg.choice == j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU){
        convert(in_msg.det_vru, out_msg.det_vru);
    }
    if(in_msg.choice == j3224_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST){
        convert(in_msg.det_obst, out_msg.det_obst);
    }

}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::DetectedObjectOptionalData& in_msg, j3224_v2x_msgs::msg::DetectedObjectOptionalData& out_msg)
{
    out_msg.choice = in_msg.choice;

    if(in_msg.choice == carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_VEH){
        convert(in_msg.det_veh, out_msg.det_veh);
    }
    if(in_msg.choice == carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_VRU){
        convert(in_msg.det_vru, out_msg.det_vru);
    }
    if(in_msg.choice == carma_v2x_msgs::msg::DetectedObjectOptionalData::DET_OBST){
        convert(in_msg.det_obst, out_msg.det_obst);
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::MeasurementTimeOffset& in_msg, carma_v2x_msgs::msg::MeasurementTimeOffset& out_msg)
{
    out_msg.measurement_time_offset = in_msg.measurement_time_offset / units::MS_PER_S;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::MeasurementTimeOffset& in_msg, j3224_v2x_msgs::msg::MeasurementTimeOffset& out_msg)
{
    out_msg.measurement_time_offset = in_msg.measurement_time_offset * units::MS_PER_S;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::PositionOffsetXYZ& in_msg, carma_v2x_msgs::msg::PositionOffsetXYZ& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    if(in_msg.presence_vector){
        convert(in_msg.offset_x, out_msg.offset_x);
        convert(in_msg.offset_y, out_msg.offset_y);
    }

    if(j3224_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z & in_msg.presence_vector){
        convert(in_msg.offset_z, out_msg.offset_z);
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::PositionOffsetXYZ& in_msg, j3224_v2x_msgs::msg::PositionOffsetXYZ& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    if(in_msg.presence_vector){
        convert(in_msg.offset_x, out_msg.offset_x);
        convert(in_msg.offset_y, out_msg.offset_y);
    }

    if(carma_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z & in_msg.presence_vector){
        convert(in_msg.offset_z, out_msg.offset_z);
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::ObjectDistance& in_msg, carma_v2x_msgs::msg::ObjectDistance& out_msg)
{
    out_msg.object_distance = in_msg.object_distance / units::DECI_M_PER_M;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::ObjectDistance& in_msg, j3224_v2x_msgs::msg::ObjectDistance& out_msg)
{
    out_msg.object_distance = in_msg.object_distance * units::DECI_M_PER_M;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::DetectedVehicleData& in_msg, carma_v2x_msgs::msg::DetectedVehicleData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS){
        out_msg.lights.exterior_lights = in_msg.lights.exterior_lights;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE){
        convert(in_msg.veh_attitude, out_msg.veh_attitude);
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE){
        out_msg.veh_attitude_confidence = in_msg.veh_attitude_confidence;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL){
        convert(in_msg.veh_ang_vel, out_msg.veh_ang_vel);
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE){
        out_msg.veh_ang_vel_confidence = in_msg.veh_ang_vel_confidence;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE){
        convert(in_msg.size, out_msg.size);
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT){
        convert(in_msg.height, out_msg.height);
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE){
        out_msg.vehicle_size_confidence = in_msg.vehicle_size_confidence;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS){
        out_msg.vehicle_class = in_msg.vehicle_class;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF){
        out_msg.class_conf = in_msg.class_conf;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::DetectedVehicleData& in_msg, j3224_v2x_msgs::msg::DetectedVehicleData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_LIGHTS){
        out_msg.lights.exterior_lights = in_msg.lights.exterior_lights;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE){
        convert(in_msg.veh_attitude, out_msg.veh_attitude);
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEH_ATTITUDE_CONFIDENCE){
        out_msg.veh_attitude_confidence = in_msg.veh_attitude_confidence;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL){
        convert(in_msg.veh_ang_vel, out_msg.veh_ang_vel);
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_ANG_VEL_CONFIDENCE){
        out_msg.veh_ang_vel_confidence = in_msg.veh_ang_vel_confidence;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE){
        convert(in_msg.size, out_msg.size);
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT){
        convert(in_msg.height, out_msg.height);
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_SIZE_CONFIDENCE){
        out_msg.vehicle_size_confidence = in_msg.vehicle_size_confidence;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_VEHICLE_CLASS){
        out_msg.vehicle_class = in_msg.vehicle_class;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVehicleData::HAS_CLASS_CONF){
        out_msg.class_conf = in_msg.class_conf;
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::Attitude& in_msg, carma_v2x_msgs::msg::Attitude& out_msg)
{
    convert(in_msg.pitch, out_msg.pitch);
    convert(in_msg.roll, out_msg.roll);
    convert(in_msg.yaw, out_msg.yaw);
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::Attitude& in_msg, j3224_v2x_msgs::msg::Attitude& out_msg)
{
    convert(in_msg.pitch, out_msg.pitch);
    convert(in_msg.roll, out_msg.roll);
    convert(in_msg.yaw, out_msg.yaw);
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::PitchDetected& in_msg, carma_v2x_msgs::msg::PitchDetected& out_msg)
{
    out_msg.pitch_detected = in_msg.pitch_detected / units::EIGHTIETH_DEG_PER_DEG;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::PitchDetected& in_msg, j3224_v2x_msgs::msg::PitchDetected& out_msg)
{
    out_msg.pitch_detected = in_msg.pitch_detected * units::EIGHTIETH_DEG_PER_DEG;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::RollDetected& in_msg, carma_v2x_msgs::msg::RollDetected& out_msg)
{
    out_msg.roll_detected = in_msg.roll_detected / units::EIGHTIETH_DEG_PER_DEG;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::RollDetected& in_msg, j3224_v2x_msgs::msg::RollDetected& out_msg)
{
    out_msg.roll_detected = in_msg.roll_detected * units::EIGHTIETH_DEG_PER_DEG;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::YawDetected& in_msg, carma_v2x_msgs::msg::YawDetected& out_msg)
{
    out_msg.yaw_detected = in_msg.yaw_detected / units::EIGHTIETH_DEG_PER_DEG;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::YawDetected& in_msg, j3224_v2x_msgs::msg::YawDetected& out_msg)
{
    out_msg.yaw_detected = in_msg.yaw_detected * units::EIGHTIETH_DEG_PER_DEG;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::AngularVelocity& in_msg, carma_v2x_msgs::msg::AngularVelocity& out_msg)
{
    convert(in_msg.pitch_rate, out_msg.pitch_rate);
    convert(in_msg.roll_rate, out_msg.roll_rate);
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::AngularVelocity& in_msg, j3224_v2x_msgs::msg::AngularVelocity& out_msg)
{
    convert(in_msg.pitch_rate, out_msg.pitch_rate);
    convert(in_msg.roll_rate, out_msg.roll_rate);
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::PitchRate& in_msg, carma_v2x_msgs::msg::PitchRate& out_msg)
{
    if(in_msg.pitch_rate != j3224_v2x_msgs::msg::PitchRate::UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.pitch_rate = in_msg.pitch_rate * units::HUNDREDTH_DEG_PER_S;
    }
    else{
        out_msg.unavailable = true;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::PitchRate& in_msg, j3224_v2x_msgs::msg::PitchRate& out_msg)
{
    if(!in_msg.unavailable){
        out_msg.pitch_rate = in_msg.pitch_rate / units::HUNDREDTH_DEG_PER_S;
    }
    else{
        out_msg.pitch_rate = j3224_v2x_msgs::msg::PitchRate::UNAVAILABLE;
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::RollRate& in_msg, carma_v2x_msgs::msg::RollRate& out_msg)
{
    if(in_msg.roll_rate != j3224_v2x_msgs::msg::RollRate::UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.roll_rate = in_msg.roll_rate * units::HUNDREDTH_DEG_PER_S;
    }
    else{
        out_msg.unavailable = true;
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::RollRate& in_msg, j3224_v2x_msgs::msg::RollRate& out_msg)
{
    if(!in_msg.unavailable){
        out_msg.roll_rate = in_msg.roll_rate / units::HUNDREDTH_DEG_PER_S;
    }
    else{
        out_msg.roll_rate = j3224_v2x_msgs::msg::RollRate::UNAVAILABLE;
    }    
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::DetectedVRUData& in_msg, carma_v2x_msgs::msg::DetectedVRUData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE){
        out_msg.basic_type = in_msg.basic_type;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION){
        out_msg.propulsion = in_msg.propulsion;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT){
        out_msg.attachment = in_msg.attachment;
    }
    if(in_msg.presence_vector & j3224_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS){
        convert(in_msg.radius, out_msg.radius);
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::DetectedVRUData& in_msg, j3224_v2x_msgs::msg::DetectedVRUData& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE){
        out_msg.basic_type = in_msg.basic_type;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVRUData::HAS_PROPULSION){
        out_msg.propulsion = in_msg.propulsion;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVRUData::HAS_ATTACHMENT){
        out_msg.attachment = in_msg.attachment;
    }
    if(in_msg.presence_vector & carma_v2x_msgs::msg::DetectedVRUData::HAS_RADIUS){
        convert(in_msg.radius, out_msg.radius);
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::DetectedObstacleData& in_msg, carma_v2x_msgs::msg::DetectedObstacleData& out_msg)
{
    convert(in_msg.obst_size, out_msg.obst_size);
    out_msg.obst_size_confidence = in_msg.obst_size_confidence;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::DetectedObstacleData& in_msg, j3224_v2x_msgs::msg::DetectedObstacleData& out_msg)
{
    convert(in_msg.obst_size, out_msg.obst_size);
    out_msg.obst_size_confidence = in_msg.obst_size_confidence;
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::ObstacleSize& in_msg, carma_v2x_msgs::msg::ObstacleSize& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.width, out_msg.width);
    convert(in_msg.length, out_msg.length);

    if(in_msg.presence_vector & j3224_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT){
        convert(in_msg.height, out_msg.height);
    }
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::ObstacleSize& in_msg, j3224_v2x_msgs::msg::ObstacleSize& out_msg)
{
    out_msg.presence_vector |= in_msg.presence_vector;

    convert(in_msg.width, out_msg.width);
    convert(in_msg.length, out_msg.length);

    if(in_msg.presence_vector & carma_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT){
        convert(in_msg.height, out_msg.height);
    }
}


void SDSMConvertor::convert(const j3224_v2x_msgs::msg::SizeValue& in_msg, carma_v2x_msgs::msg::SizeValue& out_msg)
{
    out_msg.size_value = in_msg.size_value / units::TEN_CM_PER_M;
}
void SDSMConvertor::convert(const carma_v2x_msgs::msg::SizeValue& in_msg, j3224_v2x_msgs::msg::SizeValue& out_msg)
{
    out_msg.size_value = in_msg.size_value * units::TEN_CM_PER_M;
}


} // namespace j2735_convertor