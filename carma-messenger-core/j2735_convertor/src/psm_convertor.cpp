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

#include <j2735_convertor/psm_convertor.hpp>

/**
 * CPP File containing PSMConvertor method definitions
 */

namespace j2735_convertor
{

void PSMConvertor::convert(const j2735_v2x_msgs::msg::PathPrediction& in_msg, carma_v2x_msgs::msg::PathPrediction& out_msg)
{
  out_msg.radius_of_curvature = in_msg.radius_of_curvature / radius_of_curvature_const_;
  out_msg.confidence = in_msg.confidence / confidence_conversion_const_;
}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::AccelerationSet4Way& in_msg, carma_v2x_msgs::msg::AccelerationSet4Way& out_msg)
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

void PSMConvertor::convert(const j2735_v2x_msgs::msg::PositionalAccuracy& in_msg, carma_v2x_msgs::msg::PositionalAccuracy& out_msg)
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

void PSMConvertor::convert(const j2735_v2x_msgs::msg::PathHistory& in_msg, carma_v2x_msgs::msg::PathHistory& out_msg)
{
    out_msg.curr_gnss_status = in_msg.curr_gnss_status;
    out_msg.initial_position.lat.latitude = in_msg.initial_position.lat.latitude * latitude_conversion_const_;
    out_msg.initial_position.lon.longitude = in_msg.initial_position.lon.longitude * longitude_conversion_const_;
    out_msg.initial_position.elevation.elevation = in_msg.initial_position.elevation.elevation * elevation_conversion_const_;
    out_msg.initial_position.utc_time = in_msg.initial_position.utc_time;
    out_msg.initial_position.time_confidence = in_msg.initial_position.time_confidence;
    out_msg.initial_position.pos_confidence = in_msg.initial_position.pos_confidence;
    out_msg.initial_position.speed_confidence = in_msg.initial_position.speed_confidence;

    out_msg.initial_position.lat.unavailable = 0;
    out_msg.initial_position.lon.unavailable = 0;
    out_msg.initial_position.elevation.unavailable = 0;

    out_msg.initial_position.speed.transmission = in_msg.initial_position.speed.transmission;

    convert(in_msg.initial_position.heading, out_msg.initial_position.heading);
    convert(in_msg.initial_position.speed.speed, out_msg.initial_position.speed.speed);
    convert(in_msg.initial_position.pos_accuracy, out_msg.initial_position.pos_accuracy);

    for (auto i : out_msg.crumb_data.points) {

        carma_v2x_msgs::msg::PathHistoryPoint j;

        j.lat_offset.offset = i.lat_offset.offset * latitude_conversion_const_;
        j.lon_offset.offset = i.lon_offset.offset * longitude_conversion_const_;
        j.elevation_offset.offset = i.elevation_offset.offset * elevation_conversion_const_;
        j.time_offset.offset = i.time_offset.offset * time_offset_const_;
        j.heading.heading = i.heading.heading * heading_conversion_const_;
        j.speed.speed = i.speed.speed * velocity_conversion_const_;
        j.pos_accuracy.semi_major = i.pos_accuracy.semi_major * semi_major_const_;
        j.pos_accuracy.orientation = i.pos_accuracy.orientation * pos_accuracy_orientation_const_;
        out_msg.crumb_data.points.push_back(j);
    }

}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::AttachmentRadius& in_msg, carma_v2x_msgs::msg::AttachmentRadius& out_msg) {
    out_msg.attachment_radius = in_msg.attachment_radius;
}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::Heading& in_msg, carma_v2x_msgs::msg::Heading& out_msg) {
        out_msg.heading = in_msg.heading / heading_conversion_const_;
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::Heading& in_msg, j2735_v2x_msgs::msg::Heading& out_msg) {
        out_msg.heading = in_msg.heading * heading_conversion_const_;
}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::Velocity& in_msg, carma_v2x_msgs::msg::Velocity& out_msg) {
        out_msg.velocity = in_msg.velocity / velocity_conversion_const_;
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::Velocity& in_msg, j2735_v2x_msgs::msg::Velocity& out_msg) {
        out_msg.velocity = in_msg.velocity * velocity_conversion_const_;
}


void PSMConvertor::convert(const j2735_v2x_msgs::msg::PSM& in_msg, carma_v2x_msgs::msg::PSM& out_msg)
{
    if(in_msg.HAS_ACCEL_SET & in_msg.presence_vector) {
      convert(in_msg.accel_set, out_msg.accel_set);
    }

    if(in_msg.HAS_PATH_PREDICTION == j2735_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION & in_msg.presence_vector) {
      convert(in_msg.path_prediction, out_msg.path_prediction);
    }

    convert(in_msg.accuracy, out_msg.accuracy);
    convert(in_msg.position, out_msg.position);

    convert(in_msg.heading, out_msg.heading);
    convert(in_msg.speed, out_msg.speed);

    if((in_msg.HAS_PATH_HISTORY == j2735_v2x_msgs::msg::PSM::HAS_PATH_HISTORY) & in_msg.presence_vector) {
        convert(in_msg.path_history, out_msg.path_history);
    }

    if(in_msg.HAS_ATTACHMENT_RADIUS == j2735_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS & in_msg.presence_vector) {
        convert(in_msg.attachment_radius, out_msg.attachment_radius);
    }

    if(in_msg.HAS_ATTACHMENT == j2735_v2x_msgs::msg::PSM::HAS_ATTACHMENT & in_msg.presence_vector) {
      out_msg.attachment = in_msg.attachment;
    }

    if(in_msg.HAS_ANIMAL_TYPE == j2735_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE & in_msg.presence_vector) {
      out_msg.animal_type = in_msg.animal_type;
    }

    if(in_msg.HAS_CLUSTER_SIZE == j2735_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE & in_msg.presence_vector) {
      out_msg.cluster_size = in_msg.cluster_size;
    }

    if(in_msg.HAS_CLUSTER_RADIUS == j2735_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS & in_msg.presence_vector) {
      out_msg.cluster_radius = in_msg.cluster_radius;
    }

    if(in_msg.HAS_CROSS_REQUEST == j2735_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST & in_msg.presence_vector) {
      out_msg.cross_request = in_msg.cross_request;
    }

    if(in_msg.HAS_CROSS_STATE == j2735_v2x_msgs::msg::PSM::HAS_CROSS_STATE & in_msg.presence_vector) {
      out_msg.cross_state = in_msg.cross_state;
    }

    if(in_msg.HAS_USE_STATE == j2735_v2x_msgs::msg::PSM::HAS_USE_STATE & in_msg.presence_vector) {
      out_msg.use_state = in_msg.use_state;
    }

    if(in_msg.HAS_PROPULSION == j2735_v2x_msgs::msg::PSM::HAS_PROPULSION & in_msg.presence_vector) {
      out_msg.propulsion = in_msg.propulsion;
    }

    if(in_msg.HAS_EVENT_RESPONDER_TYPE == j2735_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE & in_msg.presence_vector) {
        out_msg.event_responder_type = in_msg.event_responder_type;
    }
    
    if(in_msg.HAS_ACTIVITY_TYPE == j2735_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE & in_msg.presence_vector){
      out_msg.activity_type = in_msg.activity_type;
    }

    if(in_msg.HAS_ACTIVITY_SUB_TYPE == j2735_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE & in_msg.presence_vector){
      out_msg.activity_sub_type = in_msg.activity_sub_type;
    }

    if(in_msg.HAS_ASSIST_TYPE == j2735_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE & in_msg.presence_vector) {
      out_msg.assist_type = in_msg.assist_type;
    }

    if(in_msg.HAS_SIZING == j2735_v2x_msgs::msg::PSM::HAS_SIZING & in_msg.presence_vector) {
      out_msg.sizing = in_msg.sizing;
    }

    out_msg.basic_type = in_msg.basic_type;
    out_msg.sec_mark = in_msg.sec_mark;
    out_msg.msg_cnt = in_msg.msg_cnt;
    out_msg.id = in_msg.id;

}

////
// Convert cav_msgs to j2735_msgs
////

void PSMConvertor::convert(const carma_v2x_msgs::msg::PathPrediction& in_msg, j2735_v2x_msgs::msg::PathPrediction& out_msg)
{
  out_msg.radius_of_curvature = in_msg.radius_of_curvature * radius_of_curvature_const_;
  out_msg.confidence = in_msg.confidence * confidence_conversion_const_;
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::AccelerationSet4Way& in_msg, j2735_v2x_msgs::msg::AccelerationSet4Way& out_msg)
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

void PSMConvertor::convert(const carma_v2x_msgs::msg::PositionalAccuracy& in_msg, j2735_v2x_msgs::msg::PositionalAccuracy& out_msg)
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

void PSMConvertor::convert(const carma_v2x_msgs::msg::PathHistory& in_msg, j2735_v2x_msgs::msg::PathHistory& out_msg)
{
    if(in_msg.HAS_CURR_GNSS_STATUS & in_msg.HAS_INITIAL_POSITION){
        out_msg.curr_gnss_status = in_msg.curr_gnss_status;
        out_msg.initial_position.presence_vector = in_msg.initial_position.presence_vector;
        out_msg.initial_position.lat.latitude = in_msg.initial_position.lat.latitude / latitude_conversion_const_;
        out_msg.initial_position.lon.longitude = in_msg.initial_position.lon.longitude / longitude_conversion_const_;
        out_msg.initial_position.elevation.elevation = in_msg.initial_position.elevation.elevation / elevation_conversion_const_;
        out_msg.initial_position.utc_time = in_msg.initial_position.utc_time;
        out_msg.initial_position.time_confidence = in_msg.initial_position.time_confidence;
        out_msg.initial_position.pos_confidence = in_msg.initial_position.pos_confidence;
        out_msg.initial_position.speed_confidence = in_msg.initial_position.speed_confidence;

        out_msg.initial_position.speed.transmission = in_msg.initial_position.speed.transmission;
        convert(in_msg.initial_position.heading, out_msg.initial_position.heading);
        convert(in_msg.initial_position.speed.speed, out_msg.initial_position.speed.speed);
        convert(in_msg.initial_position.pos_accuracy, out_msg.initial_position.pos_accuracy);
    }

    for (auto i : out_msg.crumb_data.points) {

        j2735_v2x_msgs::msg::PathHistoryPoint j;

        j.lat_offset.offset = i.lat_offset.offset / latitude_conversion_const_;
        j.lon_offset.offset = i.lon_offset.offset / longitude_conversion_const_;
        j.elevation_offset.offset = i.elevation_offset.offset / elevation_conversion_const_;
        j.time_offset.offset = i.time_offset.offset / time_offset_const_;
        j.heading.heading = i.heading.heading / heading_conversion_const_;
        j.speed.speed = i.speed.speed / velocity_conversion_const_;
        j.pos_accuracy.semi_major = i.pos_accuracy.semi_major / semi_major_const_;
        j.pos_accuracy.orientation = i.pos_accuracy.orientation / pos_accuracy_orientation_const_;
        out_msg.crumb_data.points.push_back(j);
    }
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::PSM& in_msg, j2735_v2x_msgs::msg::PSM& out_msg)
{
    out_msg.presence_vector = in_msg.presence_vector;

    if(in_msg.HAS_ACCEL_SET) {
      convert(in_msg.accel_set, out_msg.accel_set);
    }

    if(in_msg.HAS_PATH_PREDICTION) {
      convert(in_msg.path_prediction, out_msg.path_prediction);
    }

    convert(in_msg.accuracy, out_msg.accuracy);
    convert(in_msg.position, out_msg.position);

    if(in_msg.heading.unavailable) {
        convert(in_msg.heading, out_msg.heading);
    }

    if(in_msg.speed.unavailable) {
        convert(in_msg.speed, out_msg.speed);
    }

    if(in_msg.HAS_PATH_HISTORY) {
    convert(in_msg.path_history, out_msg.path_history);
    }

    if(in_msg.HAS_ATTACHMENT_RADIUS) {
      convert(in_msg.attachment_radius, out_msg.attachment_radius);
    }

    if(in_msg.HAS_ATTACHMENT) {
      out_msg.attachment = in_msg.attachment;
    }

    if(in_msg.HAS_ANIMAL_TYPE) {
      out_msg.animal_type = in_msg.animal_type;
    }

    if(in_msg.HAS_CLUSTER_SIZE) {
      out_msg.cluster_size = in_msg.cluster_size;
    }

    if(in_msg.HAS_CLUSTER_RADIUS) {
      out_msg.cluster_radius = in_msg.cluster_radius;
    }

    if(in_msg.HAS_CROSS_REQUEST) {
      out_msg.cross_request = in_msg.cross_request;
    }

    if(in_msg.HAS_CROSS_STATE) {
      out_msg.cross_state = in_msg.cross_state;
    }

    if(in_msg.HAS_USE_STATE) {
      out_msg.use_state = in_msg.use_state;
    }

    if(in_msg.HAS_PROPULSION) {
      out_msg.propulsion = in_msg.propulsion;
    }

    if(in_msg.HAS_EVENT_RESPONDER_TYPE) {
        out_msg.event_responder_type = in_msg.event_responder_type;
    }
    
    if(in_msg.HAS_ACTIVITY_TYPE){
      out_msg.activity_type = in_msg.activity_type;
    }

    if(in_msg.HAS_ACTIVITY_SUB_TYPE){
      out_msg.activity_sub_type = in_msg.activity_sub_type;
    }

    if(in_msg.HAS_ASSIST_TYPE) {
      out_msg.assist_type = in_msg.assist_type;
    }

    if(in_msg.HAS_SIZING) {
      out_msg.sizing = in_msg.sizing;
    }

    out_msg.basic_type = in_msg.basic_type;
    out_msg.sec_mark = in_msg.sec_mark;
    out_msg.msg_cnt = in_msg.msg_cnt;
    out_msg.id = in_msg.id;
    }
}  // namespace j2735_convertor
