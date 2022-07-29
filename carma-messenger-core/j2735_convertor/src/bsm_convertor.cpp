/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <j2735_convertor/bsm_convertor.hpp>

/**
 * CPP File containing BSMConvertor method definitions
 */

namespace j2735_convertor
{
void BSMConvertor::convert(const j2735_v2x_msgs::msg::VehicleSize& in_msg, carma_v2x_msgs::msg::VehicleSize& out_msg)
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

void BSMConvertor::convert(const j2735_v2x_msgs::msg::AccelerationSet4Way& in_msg, carma_v2x_msgs::msg::AccelerationSet4Way& out_msg)
{
  // Convert Longitudinal and Lateral
  out_msg.longitudinal = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.longitudinal, units::CM_PER_M, out_msg.presence_vector,
      carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE, j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);

  out_msg.lateral = ValueConvertor::valueJ2735ToCav<float>(in_msg.lateral, units::CM_PER_M, out_msg.presence_vector,
                                                             carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE,
                                                             j2735_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_UNAVAILABLE);

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

void BSMConvertor::convert(const j2735_v2x_msgs::msg::PositionalAccuracy& in_msg, carma_v2x_msgs::msg::PositionalAccuracy& out_msg)
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

void BSMConvertor::convert(const j2735_v2x_msgs::msg::BSMCoreData& in_msg, carma_v2x_msgs::msg::BSMCoreData& out_msg)
{
  // Convert basic data
  // Convert basic data
  out_msg.msg_count = in_msg.msg_count;
  out_msg.id = in_msg.id;

  // Convert TimeMark
  out_msg.sec_mark = ValueConvertor::valueJ2735ToCav<uint16_t>(
      in_msg.sec_mark, units::UNCHANGED, out_msg.presence_vector, carma_v2x_msgs::msg::BSMCoreData::SEC_MARK_AVAILABLE,
      j2735_v2x_msgs::msg::BSMCoreData::SEC_MARK_UNAVAILABLE);

  // Convert Lat/Lon
  out_msg.latitude = ValueConvertor::valueJ2735ToCav<double>(
      in_msg.latitude, units::TENTH_MICRO_DEG_PER_DEG, out_msg.presence_vector,
      carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE, j2735_v2x_msgs::msg::BSMCoreData::LATITUDE_UNAVAILABLE);

  out_msg.longitude = ValueConvertor::valueJ2735ToCav<double>(
      in_msg.longitude, units::TENTH_MICRO_DEG_PER_DEG, out_msg.presence_vector,
      carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE, j2735_v2x_msgs::msg::BSMCoreData::LONGITUDE_UNAVAILABLE);

  out_msg.elev = ValueConvertor::valueJ2735ToCav<float>(in_msg.elev, units::DECI_M_PER_M, out_msg.presence_vector,
                                                          carma_v2x_msgs::msg::BSMCoreData::ELEVATION_AVAILABLE,
                                                          j2735_v2x_msgs::msg::BSMCoreData::ELEVATION_UNAVAILABLE);

  // Convert Speed
  out_msg.speed = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.speed, units::FIFTIETH_M_PER_M, out_msg.presence_vector, carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE,
      j2735_v2x_msgs::msg::BSMCoreData::SPEED_UNAVAILABLE);

  // Convert Heading
  out_msg.heading = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.heading, units::EIGHTIETH_DEG_PER_DEG, out_msg.presence_vector, carma_v2x_msgs::msg::BSMCoreData::HEADING_AVAILABLE,
      j2735_v2x_msgs::msg::BSMCoreData::HEADING_UNAVAILABLE);

  // Convert Steering Angle
  out_msg.angle = ValueConvertor::valueJ2735ToCav<float>(
      in_msg.angle, units::ONE_AND_A_HALF_DEG_PER_DEG, out_msg.presence_vector,
      carma_v2x_msgs::msg::BSMCoreData::STEER_WHEEL_ANGLE_AVAILABLE, j2735_v2x_msgs::msg::BSMCoreData::STEER_WHEEL_ANGLE_UNAVAILABLE);

  // Convert nested messages
  out_msg.transmission = in_msg.transmission;
  out_msg.brakes = in_msg.brakes;
  convert(in_msg.accuracy, out_msg.accuracy);
  convert(in_msg.accel_set, out_msg.accel_set);
  convert(in_msg.size, out_msg.size);
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::BSM& in_msg, carma_v2x_msgs::msg::BSM& out_msg)
{
  out_msg.header = in_msg.header;
  convert(in_msg.core_data, out_msg.core_data);
}

////
// Convert cav_msgs to j2735_msgs
////

void BSMConvertor::convert(const carma_v2x_msgs::msg::VehicleSize& in_msg, j2735_v2x_msgs::msg::VehicleSize& out_msg)
{
  // Convert Vehicle Width
  out_msg.vehicle_width = ValueConvertor::valueCavToJ2735<uint16_t>(
      in_msg.vehicle_width, units::CM_PER_M, in_msg.presence_vector, carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE,
      j2735_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_UNAVAILABLE);

  // Convert Vehicle length
  out_msg.vehicle_length = ValueConvertor::valueCavToJ2735<uint16_t>(
      in_msg.vehicle_length, units::CM_PER_M, in_msg.presence_vector, carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE,
      j2735_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_UNAVAILABLE);
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::AccelerationSet4Way& in_msg, j2735_v2x_msgs::msg::AccelerationSet4Way& out_msg)
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

void BSMConvertor::convert(const carma_v2x_msgs::msg::PositionalAccuracy& in_msg, j2735_v2x_msgs::msg::PositionalAccuracy& out_msg)
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

void BSMConvertor::convert(const carma_v2x_msgs::msg::BSMCoreData& in_msg, j2735_v2x_msgs::msg::BSMCoreData& out_msg)
{
  // Convert basic data
  out_msg.msg_count = in_msg.msg_count;
  out_msg.id = in_msg.id;

  // Convert TimeMark
  out_msg.sec_mark = ValueConvertor::valueCavToJ2735<uint16_t>(
      in_msg.sec_mark, units::UNCHANGED, in_msg.presence_vector, carma_v2x_msgs::msg::BSMCoreData::SEC_MARK_AVAILABLE,
      j2735_v2x_msgs::msg::BSMCoreData::SEC_MARK_UNAVAILABLE);

  // Convert Lat/Lon
  out_msg.latitude = ValueConvertor::valueCavToJ2735<int32_t>(
      in_msg.latitude, units::TENTH_MICRO_DEG_PER_DEG, in_msg.presence_vector,
      carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE, j2735_v2x_msgs::msg::BSMCoreData::LATITUDE_UNAVAILABLE);

  out_msg.longitude = ValueConvertor::valueCavToJ2735<int32_t>(
      in_msg.longitude, units::TENTH_MICRO_DEG_PER_DEG, in_msg.presence_vector,
      carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE, j2735_v2x_msgs::msg::BSMCoreData::LONGITUDE_UNAVAILABLE);

  out_msg.elev = ValueConvertor::valueCavToJ2735<int32_t>(in_msg.elev, units::DECI_M_PER_M, in_msg.presence_vector,
                                                          carma_v2x_msgs::msg::BSMCoreData::ELEVATION_AVAILABLE,
                                                          j2735_v2x_msgs::msg::BSMCoreData::ELEVATION_UNAVAILABLE);

  // Convert Speed
  out_msg.speed = ValueConvertor::valueCavToJ2735<uint16_t>(
      in_msg.speed, units::FIFTIETH_M_PER_M, in_msg.presence_vector, carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE,
      j2735_v2x_msgs::msg::BSMCoreData::SPEED_UNAVAILABLE);

  // Convert Heading
  out_msg.heading = ValueConvertor::valueCavToJ2735<uint16_t>(
      in_msg.heading, units::EIGHTIETH_DEG_PER_DEG, in_msg.presence_vector, carma_v2x_msgs::msg::BSMCoreData::HEADING_AVAILABLE,
      j2735_v2x_msgs::msg::BSMCoreData::HEADING_UNAVAILABLE);

  // Convert Steering Angle
  out_msg.angle = ValueConvertor::valueCavToJ2735<int8_t>(
      in_msg.angle, units::ONE_AND_A_HALF_DEG_PER_DEG, in_msg.presence_vector,
      carma_v2x_msgs::msg::BSMCoreData::STEER_WHEEL_ANGLE_AVAILABLE, j2735_v2x_msgs::msg::BSMCoreData::STEER_WHEEL_ANGLE_UNAVAILABLE);

  // Convert nested messages
  out_msg.transmission = in_msg.transmission;
  out_msg.brakes = in_msg.brakes;
  convert(in_msg.accuracy, out_msg.accuracy);
  convert(in_msg.accel_set, out_msg.accel_set);
  convert(in_msg.size, out_msg.size);
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::BSM& in_msg, j2735_v2x_msgs::msg::BSM& out_msg)
{
  out_msg.header = in_msg.header;
  convert(in_msg.core_data, out_msg.core_data);
}
}  // namespace j2735_convertor
