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
  out_msg.radius_of_curvature = in_msg.radius_of_curvature / units::CENTI_DEG_PER_DEG;
  out_msg.confidence = (double) in_msg.confidence / 200;
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
  out_msg.presence_vector |= in_msg.presence_vector;

  if(in_msg.presence_vector & j2735_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS){
    out_msg.curr_gnss_status = in_msg.curr_gnss_status;
  }
  if(in_msg.presence_vector & j2735_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION){
    out_msg.initial_position.presence_vector |= in_msg.initial_position.presence_vector;
    out_msg.initial_position.lat.latitude = in_msg.initial_position.lat.latitude / units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.initial_position.lon.longitude = in_msg.initial_position.lon.longitude / units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.initial_position.elevation.elevation = in_msg.initial_position.elevation.elevation / units::DECI_M_PER_M;
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
  }
  carma_v2x_msgs::msg::PathHistoryPointList path_history_point_list;
  for (auto i : in_msg.crumb_data.points) {

    carma_v2x_msgs::msg::PathHistoryPoint j;
    if(i.lat_offset.offset != j2735_v2x_msgs::msg::OffsetLLB18::UNAVAILABLE){
      j.lat_offset.unavailable = false;
      j.lat_offset.offset = i.lat_offset.offset / units::TENTH_MICRO_DEG_PER_DEG;
    }
    else{
      j.lat_offset.unavailable = true;
    }
    
    if(i.lat_offset.offset != j2735_v2x_msgs::msg::OffsetLLB18::UNAVAILABLE){
      j.lon_offset.unavailable = false;
      j.lon_offset.offset = i.lon_offset.offset / units::TENTH_MICRO_DEG_PER_DEG;
    }
    else{
      j.lon_offset.unavailable = true;
    }
    
    if(i.elevation_offset.offset != j2735_v2x_msgs::msg::VertOffsetB12::UNAVAILABLE){
      j.elevation_offset.unavailable = false;
      j.elevation_offset.offset = i.elevation_offset.offset / units::DECI_M_PER_M;
    }
    else{
      j.elevation_offset.unavailable = true;
    }
    
    if(i.time_offset.offset != j2735_v2x_msgs::msg::TimeOffset::UNAVAILABLE){
      j.time_offset.unavailable = false;
      j.time_offset.offset = i.time_offset.offset / units::CM_PER_M;
    }
    else{
      j.time_offset.unavailable = true;
    }
    
    if(i.heading.heading != j2735_v2x_msgs::msg::CoarseHeading::UNAVAILABLE){
      j.heading.unavailable = false;
      j.heading.heading = i.heading.heading * units::ONE_AND_A_HALF_DEG;
    }
    else{
      j.heading.unavailable = true;
    }
    j.speed.speed = i.speed.speed / units::FIFTIETH_M_PER_M;
    
    if(i.pos_accuracy.semi_major != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE){
      j.pos_accuracy.semi_major = i.pos_accuracy.semi_major / units::TWENTIETH_M_PER_M;
    }
    if(i.pos_accuracy.semi_minor != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE){
      j.pos_accuracy.semi_minor = i.pos_accuracy.semi_minor / units::TWENTIETH_M_PER_M;
    }
    
    //set presence_vector if both semi_major and semi_minor available
    if((i.pos_accuracy.semi_major != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE) && 
      (i.pos_accuracy.semi_minor != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE))
    {
      j.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;  
    }
    
    if(i.pos_accuracy.orientation != j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE){
      
      j.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE ;
      j.pos_accuracy.orientation = i.pos_accuracy.orientation / units::DEG_360_OVER_65535_PER_DEG;
    }
    
    path_history_point_list.points.push_back(j);
  }
  out_msg.crumb_data = path_history_point_list;
}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::AttachmentRadius& in_msg, carma_v2x_msgs::msg::AttachmentRadius& out_msg) {
    out_msg.attachment_radius = in_msg.attachment_radius / units::DECI_M_PER_M;
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::AttachmentRadius& in_msg, j2735_v2x_msgs::msg::AttachmentRadius& out_msg) {
    out_msg.attachment_radius = in_msg.attachment_radius * units::DECI_M_PER_M;
}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::Heading& in_msg, carma_v2x_msgs::msg::Heading& out_msg) {
      if(in_msg.heading != j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.heading = in_msg.heading / units::EIGHTIETH_DEG_PER_DEG;
      }
      else{
        out_msg.unavailable = true;
      }
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::Heading& in_msg, j2735_v2x_msgs::msg::Heading& out_msg) {
      if(!in_msg.unavailable){
        out_msg.heading = in_msg.heading * units::EIGHTIETH_DEG_PER_DEG;
      }
      else{
        out_msg.heading = j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE;
      }
}

void PSMConvertor::convert(const j2735_v2x_msgs::msg::Velocity& in_msg, carma_v2x_msgs::msg::Velocity& out_msg) {
        if(in_msg.velocity != j2735_v2x_msgs::msg::Velocity::UNAVAILABLE){
          out_msg.unavailable = false;
          out_msg.velocity = in_msg.velocity / units::FIFTIETH_M_PER_M;
        }
        else{
          out_msg.unavailable = true;
        }
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::Velocity& in_msg, j2735_v2x_msgs::msg::Velocity& out_msg) {
      if(!in_msg.unavailable){
        out_msg.velocity = in_msg.velocity * units::FIFTIETH_M_PER_M;
      }
      else{
        out_msg.velocity = j2735_v2x_msgs::msg::Velocity::UNAVAILABLE;
      }
}


void PSMConvertor::convert(const j2735_v2x_msgs::msg::PSM& in_msg, carma_v2x_msgs::msg::PSM& out_msg)
{
  out_msg.presence_vector = in_msg.presence_vector;

  if(j2735_v2x_msgs::msg::PSM::HAS_ACCEL_SET & in_msg.presence_vector) {
    convert(in_msg.accel_set, out_msg.accel_set);
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION & in_msg.presence_vector) {
    convert(in_msg.path_prediction, out_msg.path_prediction);
  }

  convert(in_msg.accuracy, out_msg.accuracy);

  out_msg.position.latitude = in_msg.position.latitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.position.longitude = in_msg.position.longitude / units::TENTH_MICRO_DEG_PER_DEG;
  if(in_msg.position.elevation_exists){
    out_msg.position.elevation_exists = true;
    out_msg.position.elevation = in_msg.position.elevation / units::DECI_M_PER_M;
  }
  

  convert(in_msg.heading, out_msg.heading);
  convert(in_msg.speed, out_msg.speed);

  if(j2735_v2x_msgs::msg::PSM::HAS_PATH_HISTORY & in_msg.presence_vector) {
    convert(in_msg.path_history, out_msg.path_history);
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS & in_msg.presence_vector) {
    convert(in_msg.attachment_radius, out_msg.attachment_radius);
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_ATTACHMENT & in_msg.presence_vector) {
    out_msg.attachment = in_msg.attachment;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE & in_msg.presence_vector) {
    out_msg.animal_type = in_msg.animal_type;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE & in_msg.presence_vector) {
    out_msg.cluster_size = in_msg.cluster_size;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS & in_msg.presence_vector) {
    out_msg.cluster_radius = in_msg.cluster_radius;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST & in_msg.presence_vector) {
    out_msg.cross_request = in_msg.cross_request;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_CROSS_STATE & in_msg.presence_vector) {
    out_msg.cross_state = in_msg.cross_state;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_USE_STATE & in_msg.presence_vector) {
    out_msg.use_state = in_msg.use_state;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_PROPULSION & in_msg.presence_vector) {
    out_msg.propulsion = in_msg.propulsion;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE & in_msg.presence_vector) {
    out_msg.event_responder_type = in_msg.event_responder_type;
  }
  
  if(j2735_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE & in_msg.presence_vector){
    out_msg.activity_type = in_msg.activity_type;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE & in_msg.presence_vector){
    out_msg.activity_sub_type = in_msg.activity_sub_type;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE & in_msg.presence_vector) {
    out_msg.assist_type = in_msg.assist_type;
  }

  if(j2735_v2x_msgs::msg::PSM::HAS_SIZING & in_msg.presence_vector) {
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
  out_msg.radius_of_curvature = in_msg.radius_of_curvature * units::CENTI_DEG_PER_DEG;
  out_msg.confidence = in_msg.confidence * 200;
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
  if(in_msg.presence_vector & carma_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS){
    out_msg.curr_gnss_status = in_msg.curr_gnss_status;
  }
  if(in_msg.presence_vector & carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION){
    out_msg.initial_position.presence_vector |= in_msg.initial_position.presence_vector;
    out_msg.initial_position.lat.latitude = in_msg.initial_position.lat.latitude * units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.initial_position.lon.longitude = in_msg.initial_position.lon.longitude * units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.initial_position.elevation.elevation = in_msg.initial_position.elevation.elevation * units::DECI_M_PER_M;
    out_msg.initial_position.utc_time = in_msg.initial_position.utc_time;
    out_msg.initial_position.time_confidence = in_msg.initial_position.time_confidence;
    out_msg.initial_position.pos_confidence = in_msg.initial_position.pos_confidence;
    out_msg.initial_position.speed_confidence = in_msg.initial_position.speed_confidence;

    out_msg.initial_position.speed.transmission = in_msg.initial_position.speed.transmission;
    out_msg.initial_position.speed.speed.velocity = in_msg.initial_position.speed.speed.velocity * units::FIFTIETH_M_PER_M;
    convert(in_msg.initial_position.heading, out_msg.initial_position.heading);
    convert(in_msg.initial_position.speed.speed, out_msg.initial_position.speed.speed);
    convert(in_msg.initial_position.pos_accuracy, out_msg.initial_position.pos_accuracy);
  }

  for (auto i : in_msg.crumb_data.points) {

    j2735_v2x_msgs::msg::PathHistoryPoint j;

    if(!i.lat_offset.unavailable){
      j.lat_offset.offset = i.lat_offset.offset * units::TENTH_MICRO_DEG_PER_DEG;
    }
    else{
      j.lat_offset.offset = j2735_v2x_msgs::msg::OffsetLLB18::UNAVAILABLE;
    }
    
    if(!i.lon_offset.unavailable){
      j.lon_offset.offset = i.lon_offset.offset * units::TENTH_MICRO_DEG_PER_DEG;
    }
    else{
      j.lon_offset.offset = j2735_v2x_msgs::msg::OffsetLLB18::UNAVAILABLE;
    }
    
    if(!i.elevation_offset.unavailable){
      j.elevation_offset.offset = i.elevation_offset.offset * units::DECI_M_PER_M;
    }
    else{
      j.elevation_offset.offset = j2735_v2x_msgs::msg::VertOffsetB12::UNAVAILABLE;
    }
    
    if(!i.time_offset.unavailable){
      j.time_offset.offset = i.time_offset.offset * units::CM_PER_M;
    }
    else{
      j.time_offset.offset = j2735_v2x_msgs::msg::TimeOffset::UNAVAILABLE;
    }
    
    if(!i.heading.unavailable){
      j.heading.heading = i.heading.heading / units::ONE_AND_A_HALF_DEG;  
    }
    else{
      j.heading.heading = j2735_v2x_msgs::msg::CoarseHeading::UNAVAILABLE;
    }
    
    if(!i.speed.unavailable){
      j.speed.speed = i.speed.speed * units::FIFTIETH_M_PER_M;
    }
    else{
      j.speed.speed = j2735_v2x_msgs::msg::Speed::UNAVAILABLE;
    }
    
    if(i.pos_accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE){
      j.pos_accuracy.semi_major = i.pos_accuracy.semi_major * units::TWENTIETH_M_PER_M;
      j.pos_accuracy.semi_minor = i.pos_accuracy.semi_minor * units::TWENTIETH_M_PER_M;
    }
    else{
      j.pos_accuracy.semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
      j.pos_accuracy.semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
    }

    if(i.pos_accuracy.presence_vector & carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE){
      j.pos_accuracy.orientation = i.pos_accuracy.orientation * units::DEG_360_OVER_65535_PER_DEG;
    }
    else{
      j.pos_accuracy.orientation = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE;
    }

    out_msg.crumb_data.points.push_back(j);
  }
}

void PSMConvertor::convert(const carma_v2x_msgs::msg::PSM& in_msg, j2735_v2x_msgs::msg::PSM& out_msg)
{
    out_msg.presence_vector = in_msg.presence_vector;

    if(carma_v2x_msgs::msg::PSM::HAS_ACCEL_SET & in_msg.presence_vector) {
      convert(in_msg.accel_set, out_msg.accel_set);
    }

    if(carma_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION & in_msg.presence_vector) {
      convert(in_msg.path_prediction, out_msg.path_prediction);
    }

    convert(in_msg.accuracy, out_msg.accuracy);

    out_msg.position.latitude = in_msg.position.latitude * units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.position.longitude = in_msg.position.longitude * units::TENTH_MICRO_DEG_PER_DEG;
    out_msg.position.elevation = in_msg.position.elevation * units::DECI_M_PER_M;

    convert(in_msg.heading, out_msg.heading);
    convert(in_msg.speed, out_msg.speed);

    if(carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY & in_msg.presence_vector) {
        convert(in_msg.path_history, out_msg.path_history);
    }

    if(carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT_RADIUS & in_msg.presence_vector) {
        convert(in_msg.attachment_radius, out_msg.attachment_radius);
    }

    if(carma_v2x_msgs::msg::PSM::HAS_ATTACHMENT & in_msg.presence_vector) {
      out_msg.attachment = in_msg.attachment;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_ANIMAL_TYPE & in_msg.presence_vector) {
      out_msg.animal_type = in_msg.animal_type;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_CLUSTER_SIZE & in_msg.presence_vector) {
      out_msg.cluster_size = in_msg.cluster_size;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_CLUSTER_RADIUS & in_msg.presence_vector) {
      out_msg.cluster_radius = in_msg.cluster_radius;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_CROSS_REQUEST & in_msg.presence_vector) {
      out_msg.cross_request = in_msg.cross_request;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_CROSS_STATE & in_msg.presence_vector) {
      out_msg.cross_state = in_msg.cross_state;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_USE_STATE & in_msg.presence_vector) {
      out_msg.use_state = in_msg.use_state;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_PROPULSION & in_msg.presence_vector) {
      out_msg.propulsion = in_msg.propulsion;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_EVENT_RESPONDER_TYPE & in_msg.presence_vector) {
        out_msg.event_responder_type = in_msg.event_responder_type;
    }
    
    if(carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_TYPE & in_msg.presence_vector){
      out_msg.activity_type = in_msg.activity_type;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_ACTIVITY_SUB_TYPE & in_msg.presence_vector){
      out_msg.activity_sub_type = in_msg.activity_sub_type;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_ASSIST_TYPE & in_msg.presence_vector) {
      out_msg.assist_type = in_msg.assist_type;
    }

    if(carma_v2x_msgs::msg::PSM::HAS_SIZING & in_msg.presence_vector) {
      out_msg.sizing = in_msg.sizing;
    }

    out_msg.basic_type = in_msg.basic_type;
    out_msg.sec_mark = in_msg.sec_mark;
    out_msg.msg_cnt = in_msg.msg_cnt;
    out_msg.id = in_msg.id;
}



}  // namespace j2735_convertor
