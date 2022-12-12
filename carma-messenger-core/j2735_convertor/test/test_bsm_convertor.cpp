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

#include <gtest/gtest.h>
#include <j2735_convertor/bsm_convertor.hpp>


namespace j2735_convertor
{

TEST(ControlRequest, convertBSMj2735ToCAV)
{ 
    // Create j2735_v2x_msgs::msg::BSM message, which will be converted to carma_v2x_msgs::msg::BSM out_message
    j2735_v2x_msgs::msg::BSM message;

    // BSM.core_data (carma_v2x_msgs)
    message.core_data.msg_count = 1;
    message.core_data.id = {1,2,3,4};
    message.core_data.sec_mark = 2;
    message.core_data.longitude = 30000000;
    message.core_data.latitude = 40000000;
    message.core_data.elev = 50;
    message.core_data.accuracy.semi_major = 200;
    message.core_data.accuracy.semi_minor = 40;
    message.core_data.accuracy.orientation = 364;
    message.core_data.transmission.transmission_state = j2735_v2x_msgs::msg::TransmissionState::PARK;
    message.core_data.speed = 250;
    message.core_data.heading = 8000;
    message.core_data.angle = -26.0;
    message.core_data.accel_set.longitudinal = 500;
    message.core_data.accel_set.lateral = -600;
    message.core_data.accel_set.vert = 1;
    message.core_data.accel_set.yaw_rate = -5000;
    message.core_data.brakes.wheel_brakes.brake_applied_status = j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR;
    message.core_data.brakes.traction.traction_control_status = 1;
    message.core_data.brakes.abs.anti_lock_brake_status = 1;
    message.core_data.brakes.scs.stability_control_status = 1;
    message.core_data.brakes.brake_boost.brake_boost_applied = 1;
    message.core_data.brakes.aux_brakes.auxiliary_brake_status = 1;
    message.core_data.size.vehicle_width = 1000;
    message.core_data.size.vehicle_length = 2000;

    // Part II
    message.presence_vector |= j2735_v2x_msgs::msg::BSM::HAS_PART_II;

    // BSMPartIIExtension.special_vehicle_extensions
    j2735_v2x_msgs::msg::BSMPartIIExtension part_ii_special;
    part_ii_special.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;

    // BSMPartIIExtension.special_vehicle_extensions.vehicle_alerts
    part_ii_special.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = j2735_v2x_msgs::msg::SirenInUse::IN_USE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = j2735_v2x_msgs::msg::LightbarInUse::IN_USE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.multi.multi_vehicle_response = j2735_v2x_msgs::msg::MultiVehicleResponse::SINGLE_VEHICLE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_EVENTS;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.events.event.privileged_event_flags = j2735_v2x_msgs::msg::PrivilegedEventFlags::PE_EMERGENCY_RESPONSE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_RESPONSE_TYPE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.response_type.response_type = j2735_v2x_msgs::msg::ResponseType::EMERGENCY;

    // BSMPartIIExtension.special_vehicle_extensions.description
    part_ii_special.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION;
    part_ii_special.special_vehicle_extensions.description.type_event.code = 6;
    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_DESCRIPTION;
    j2735_v2x_msgs::msg::ITIScodes code;
    code.code = 40;
    part_ii_special.special_vehicle_extensions.description.description.push_back(code);

    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_PRIORITY;
    j2735_v2x_msgs::msg::Priority priority;
    for(int i = 0; i < priority.priority.size(); i++){
        priority.priority[i] = 5;
    }
    part_ii_special.special_vehicle_extensions.description.priority = priority;

    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_HEADING;
    part_ii_special.special_vehicle_extensions.description.heading.heading_slice = j2735_v2x_msgs::msg::HeadingSlice::FROM_045_0_TO_067_5_DEGREES;
    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_EXTENT;
    part_ii_special.special_vehicle_extensions.description.extent.extent_value = j2735_v2x_msgs::msg::Extent::USE_FOR_50000_METERS; 

    // BSMPartIIExtension.special_vehicle_extensions.trailers
    part_ii_special.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS;
    part_ii_special.special_vehicle_extensions.trailers.ssp_index = 0;
    part_ii_special.special_vehicle_extensions.trailers.connection.pivot_offset.offset = 500;
    part_ii_special.special_vehicle_extensions.trailers.connection.pivot_angle.angle = 8320;
    part_ii_special.special_vehicle_extensions.trailers.connection.pivots.pivoting_allowed = true;

    j2735_v2x_msgs::msg::TrailerUnitDescription trailer_unit_description_msg;
    trailer_unit_description_msg.is_dolly.is_dolly = true;
    trailer_unit_description_msg.width.vehicle_width = 600;
    trailer_unit_description_msg.length.vehicle_length = 700;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT;
    trailer_unit_description_msg.height.vehicle_height = 40;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS;
    trailer_unit_description_msg.mass.trailer_mass = 12;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS;
    trailer_unit_description_msg.bumper_heights.front.bumper_height = 110;
    trailer_unit_description_msg.bumper_heights.rear.bumper_height = 120;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY;
    trailer_unit_description_msg.center_of_gravity.vehicle_height = 50;
    trailer_unit_description_msg.front_pivot.pivot_offset.offset = 939;
    trailer_unit_description_msg.front_pivot.pivot_angle.angle = 1600;
    trailer_unit_description_msg.front_pivot.pivots.pivoting_allowed = true;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT;
    trailer_unit_description_msg.rear_pivot.pivot_offset.offset = 839;
    trailer_unit_description_msg.rear_pivot.pivot_angle.angle = 1760;
    trailer_unit_description_msg.rear_pivot.pivots.pivoting_allowed = false;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET;
    trailer_unit_description_msg.rear_wheel_offset.offset = 189;
    trailer_unit_description_msg.position_offset.x = 1800;
    trailer_unit_description_msg.position_offset.y = 1900;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET;
    trailer_unit_description_msg.elevation_offset.offset = 12;

    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA;
    j2735_v2x_msgs::msg::TrailerHistoryPoint trailer_history_point_msg;
    trailer_history_point_msg.pivot_angle.angle = 2400;
    trailer_history_point_msg.time_offset.offset = 10000;
    trailer_history_point_msg.position_offset.x = 1000;
    trailer_history_point_msg.position_offset.y = 2000;
    trailer_history_point_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET;
    trailer_history_point_msg.elevation_offset.offset = 11;
    trailer_history_point_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING;
    trailer_history_point_msg.heading.heading = 56;
    trailer_unit_description_msg.crumb_data.trailer_history_points.push_back(trailer_history_point_msg);

    part_ii_special.special_vehicle_extensions.trailers.units.trailer_unit_descriptions.push_back(trailer_unit_description_msg);

    message.part_ii.push_back(part_ii_special);

    // BSMPartIIExtension.supplemental_vehicle_extensions
    j2735_v2x_msgs::msg::BSMPartIIExtension part_ii_supp;
    part_ii_supp.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT;

    // BSMPartIIExtension.supplemental_vehicle_extensions.classification
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION;
    part_ii_supp.supplemental_vehicle_extensions.classification.basic_vehicle_class = j2735_v2x_msgs::msg::BasicVehicleClass::SPECIAL_VEHICLE_CLASS;

    // BSMPartIIExtension.supplemental_vehicle_extensions.class_details
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_KEY_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.key_type.basic_vehicle_class = j2735_v2x_msgs::msg::BasicVehicleClass::SPECIAL_VEHICLE_CLASS;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ROLE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.role.basic_vehicle_role = j2735_v2x_msgs::msg::BasicVehicleRole::POLICE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ISO;
    part_ii_supp.supplemental_vehicle_extensions.class_details.iso3833 = 23;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_HPMS_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.hpms_type.vehicle_type = j2735_v2x_msgs::msg::VehicleType::SPECIAL;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_VEHICLE_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.vehicle_type.vehicle_group_affected = j2735_v2x_msgs::msg::ITISVehicleGroupAffected::HEAVY_VEHICLES;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONSE_EQUIP;
    part_ii_supp.supplemental_vehicle_extensions.class_details.response_equip.incident_response_equipment = j2735_v2x_msgs::msg::ITISIncidentResponseEquipment::HAZMAT_UNIT;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONDER_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.responder_type.responder_group_affected = j2735_v2x_msgs::msg::ITISResponderGroupAffected::EMERGENCY_VEHICLE_UNITS;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_FUEL_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.fuel_type.fuel_type = j2735_v2x_msgs::msg::FuelType::HYBRID;

    // BSMPartIIExtension.supplemental_vehicle_extensions.vehicle_data
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_HEIGHT;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.height.vehicle_height = 60;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_BUMPERS;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.bumpers.front.bumper_height = 110;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.bumpers.rear.bumper_height = 80;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_MASS;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.mass.vehicle_mass = 89;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.trailer_weight.trailer_weight = 2400;

    // BSMPartIIExtension.supplemental_vehicle_extensions.weather_report
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.is_raining.precip_yes_no = j2735_v2x_msgs::msg::NTCIPEssPrecipYesNo::PRECIP;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.rain_rate.precip_rate = 2050;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.precip_situation.ess_precip_situation = j2735_v2x_msgs::msg::NTCIPEssPrecipSituation::RAIN_SLIGHT;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.solar_radiation.ess_solar_radiation = 206;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_FRICTION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.friction.ess_mobile_friction = 90;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.road_friction.coefficient = 15;

    // BSMPartIIExtension.supplemental_vehicle_extensions.weather_probe
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.air_temp.temperature = 140;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.air_pressure.pressure = 120;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.status_front.wiper_status = j2735_v2x_msgs::msg::WiperStatus::INTERMITTENT;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.rate_front.wiper_rate = 60;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.status_rear.wiper_status = j2735_v2x_msgs::msg::WiperStatus::LOW;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_RATE_REAR;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.rate_rear.wiper_rate = 90;

    // BSMPartIIExtension.supplemental_vehicle_extensions.obstacle
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.ob_dist.distance = 500;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.year.year = 1000;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.month.month = 10;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.day.day = 20;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.hour.hour = 21;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.minute.minute = 22;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.second.millisecond = 20000;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.offset.offset_minute = 800;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.description.code = 540;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.location_details.generic_locations = j2735_v2x_msgs::msg::ITISGenericLocations::IN_STREET;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.vert_event.exceeded_wheels = j2735_v2x_msgs::msg::VerticalAccelerationThreshold::RIGHT_REAR;

    // BSMPartIIExtension.supplemental_vehicle_extensions.status
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS;
    part_ii_supp.supplemental_vehicle_extensions.status.status_details.code = 539;
    part_ii_supp.supplemental_vehicle_extensions.status.presence_vector |= j2735_v2x_msgs::msg::DisabledVehicle::HAS_LOCATION_DETAILS;
    part_ii_supp.supplemental_vehicle_extensions.status.location_details.generic_locations = j2735_v2x_msgs::msg::ITISGenericLocations::CROSS_ROAD;

    // BSMPartIIExtension.supplemental_vehicle_extensions.speed_profile
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE;
    j2735_v2x_msgs::msg::GrossSpeed speed_point1;
    speed_point1.speed = 20;
    j2735_v2x_msgs::msg::GrossSpeed speed_point2;
    speed_point2.speed = 25;
    j2735_v2x_msgs::msg::GrossSpeed speed_point3;
    speed_point3.speed = j2735_v2x_msgs::msg::GrossSpeed::SPEED_UNAVAILABLE;
    part_ii_supp.supplemental_vehicle_extensions.speed_profile.push_back(speed_point1);
    part_ii_supp.supplemental_vehicle_extensions.speed_profile.push_back(speed_point2);
    part_ii_supp.supplemental_vehicle_extensions.speed_profile.push_back(speed_point3);

    message.part_ii.push_back(part_ii_supp);

    // BSMPartIIExtension.vehicle_safety_extensions
    j2735_v2x_msgs::msg::BSMPartIIExtension part_ii_safety;
    part_ii_safety.part_ii_id = carma_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT;

    // BSMPartIIExtension.vehicle_safety_extensions.path_history
    part_ii_safety.vehicle_safety_extensions.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY;
    j2735_v2x_msgs::msg::PathHistoryPoint point1;
    point1.lat_offset.offset = 131067;
    point1.lon_offset.offset = 131068;
    point1.elevation_offset.offset = -2010;
    point1.time_offset.offset = 60034;
    point1.speed.speed = 5000;
    point1.pos_accuracy.semi_major = 200;
    point1.pos_accuracy.semi_minor = 100;
    point1.pos_accuracy.orientation = 18204;
    point1.heading.heading = 100;
    part_ii_safety.vehicle_safety_extensions.path_history.crumb_data.points.push_back(point1);

    part_ii_safety.vehicle_safety_extensions.path_history.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;
    part_ii_safety.vehicle_safety_extensions.path_history.curr_gnss_status.statuses = j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY;

    carma_v2x_msgs::msg::FullPositionVector initial_position;
    part_ii_safety.vehicle_safety_extensions.path_history.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.year.year = 1000;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.month.month = 10;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.day.day = 20;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.hour.hour = 21;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.minute.minute = 22;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.second.millisecond = 20000;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.offset.offset_minute = 800;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.lon.longitude = 1701000000;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.lat.latitude = 900000000;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.elevation.elevation = 5100;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_HEADING;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.heading.heading = 80;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed.transmission.transmission_state |= j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed.speed.velocity = 2500;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_major = 200;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_minor = 100;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.orientation = 18204;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A20M;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed_confidence.heading.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed_confidence.speed.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed_confidence.throttle.confidence |= j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT;

    // BSMPartIIExtension.vehicle_safety_extensions.path_prediction
    part_ii_safety.vehicle_safety_extensions.presence_vector |= carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION;
    part_ii_safety.vehicle_safety_extensions.path_prediction.radius_of_curvature = 10000;
    part_ii_safety.vehicle_safety_extensions.path_prediction.confidence = 100;

    message.part_ii.push_back(part_ii_safety);

    // Regional Extension
    message.presence_vector |= j2735_v2x_msgs::msg::BSM::HAS_REGIONAL;

    // BSMRegionalExtension.route_destination_points
    j2735_v2x_msgs::msg::BSMRegionalExtension regional_ext;
    regional_ext.regional_extension_id = j2735_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS;

    j2735_v2x_msgs::msg::Position3D position1;
    position1.latitude = 405000000;
    position1.longitude = -1602000000;
    position1.elevation_exists = true;
    position1.elevation = 1300;

    j2735_v2x_msgs::msg::Position3D position2;
    position2.latitude = 435000000;
    position2.longitude = -1422000000;
    position2.elevation_exists = true;
    position2.elevation = 1100;

    regional_ext.route_destination_points.push_back(position1);
    regional_ext.route_destination_points.push_back(position2);
    message.regional.push_back(regional_ext);

    // Convert 'message' (j2735_v2x_msgs::msg::BSM) to 'out_message' (carma_v2x_msgs::msg::BSM)
    carma_v2x_msgs::msg::BSM out_message;
    j2735_convertor::BSMConvertor::convert(message, out_message);

    // Verify BSM.core_data
    ASSERT_EQ(out_message.core_data.msg_count, 1);
    std::vector<uint8_t> id = {1,2,3,4};
    ASSERT_EQ(out_message.core_data.id, id);
    ASSERT_EQ(out_message.core_data.sec_mark, 2);
    ASSERT_EQ(out_message.core_data.longitude, 3);
    ASSERT_EQ(out_message.core_data.latitude, 4);
    ASSERT_EQ(out_message.core_data.elev, 5);
    ASSERT_EQ(out_message.core_data.accuracy.semi_major, 10);
    ASSERT_EQ(out_message.core_data.accuracy.semi_minor, 2);
    ASSERT_NEAR(out_message.core_data.accuracy.orientation, 2, 0.01);
    ASSERT_EQ(out_message.core_data.transmission.transmission_state, j2735_v2x_msgs::msg::TransmissionState::PARK);
    ASSERT_EQ(out_message.core_data.speed, 5);
    ASSERT_EQ(out_message.core_data.heading, 100);
    ASSERT_EQ(out_message.core_data.angle, -39);
    ASSERT_EQ(out_message.core_data.accel_set.longitudinal, 5);
    ASSERT_EQ(out_message.core_data.accel_set.lateral, -6);
    ASSERT_EQ(out_message.core_data.accel_set.yaw_rate, -50);
    ASSERT_EQ(out_message.core_data.brakes.wheel_brakes.brake_applied_status, j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR);
    ASSERT_EQ(out_message.core_data.brakes.traction.traction_control_status, 1);
    ASSERT_EQ(out_message.core_data.brakes.abs.anti_lock_brake_status, 1);
    ASSERT_EQ(out_message.core_data.brakes.scs.stability_control_status, 1);
    ASSERT_EQ(out_message.core_data.size.vehicle_width, 10);
    ASSERT_EQ(out_message.core_data.size.vehicle_length, 20);

    // Verify BSM Part II Content
    ASSERT_EQ(out_message.presence_vector, message.presence_vector);

    // Verify BSM.part_ii[0] (SpecialVehicleExtensions)
    ASSERT_EQ(out_message.part_ii[0].part_ii_id, carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT);
    ASSERT_EQ(out_message.part_ii[0].special_vehicle_extensions.presence_vector, message.part_ii[0].special_vehicle_extensions.presence_vector);
    ASSERT_EQ(out_message.part_ii[0].special_vehicle_extensions.vehicle_alerts, message.part_ii[0].special_vehicle_extensions.vehicle_alerts);
    ASSERT_EQ(out_message.part_ii[0].special_vehicle_extensions.description, message.part_ii[0].special_vehicle_extensions.description);

    carma_v2x_msgs::msg::TrailerData out_trailers = out_message.part_ii[0].special_vehicle_extensions.trailers;
    ASSERT_EQ(out_trailers.ssp_index, 0);
    ASSERT_EQ(out_trailers.connection.pivot_offset.offset, 5);
    ASSERT_EQ(out_trailers.connection.pivot_angle.angle, 104);
    ASSERT_EQ(out_trailers.connection.pivots.pivoting_allowed, true);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].presence_vector, message.part_ii[0].special_vehicle_extensions.trailers.units.trailer_unit_descriptions[0].presence_vector);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].is_dolly.is_dolly, true);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].width.vehicle_width, 6);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].length.vehicle_length, 7);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].height.vehicle_height, 2);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].mass.trailer_mass, 6000);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].bumper_heights.front.bumper_height, 1.1, 0.01);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].bumper_heights.rear.bumper_height, 1.2, 0.01);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].center_of_gravity.vehicle_height, 2.5);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].front_pivot.pivot_offset.offset, 9.39, 0.01);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].front_pivot.pivot_angle.angle, 20);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].front_pivot.pivots.pivoting_allowed, true);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].rear_pivot.pivot_offset.offset, 8.39, 0.01);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].rear_pivot.pivot_angle.angle, 22);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].rear_pivot.pivots.pivoting_allowed, false);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].rear_wheel_offset.offset, 1.89, 0.01);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].position_offset.x.offset, 18);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].position_offset.y.offset, 19);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].elevation_offset.offset, 1.2, 0.01);

    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].presence_vector, 
        message.part_ii[0].special_vehicle_extensions.trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].presence_vector);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].pivot_angle.angle, 30);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].time_offset.offset, 100);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].position_offset.x.offset, 10);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].position_offset.y.offset, 20);
    ASSERT_NEAR(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].elevation_offset.offset, 1.1, 0.01);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].heading.heading, 84);

    // Verify BSM.part_ii[1] (SupplementalVehicleExtensions)
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.classification, message.part_ii[1].supplemental_vehicle_extensions.classification);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.class_details, message.part_ii[1].supplemental_vehicle_extensions.class_details);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.height.vehicle_height, 3);
    ASSERT_NEAR(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.bumpers.front.bumper_height, 1.1, 0.01);
    ASSERT_NEAR(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.bumpers.rear.bumper_height, 0.8, 0.01);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.mass.vehicle_mass, 8500);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.trailer_weight.trailer_weight, 4800);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.weather_report.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.is_raining.precip_yes_no, j2735_v2x_msgs::msg::NTCIPEssPrecipYesNo::PRECIP);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.rain_rate.precip_rate, 205);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.precip_situation.ess_precip_situation, j2735_v2x_msgs::msg::NTCIPEssPrecipSituation::RAIN_SLIGHT);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.solar_radiation.ess_solar_radiation, 206);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.friction.ess_mobile_friction, 90);
    ASSERT_NEAR(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.road_friction.coefficient, 0.30, 0.01);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.weather_probe.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.air_temp.temperature, 100);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.air_pressure.pressure, 60000);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.status_front.wiper_status, j2735_v2x_msgs::msg::WiperStatus::INTERMITTENT);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.rate_front.wiper_rate, 1);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.status_rear.wiper_status, j2735_v2x_msgs::msg::WiperStatus::LOW);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.rate_rear.wiper_rate, 1.5);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.obstacle.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.ob_dist.distance, 500);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.date_time, message.part_ii[1].supplemental_vehicle_extensions.obstacle.date_time);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.description.code, 540);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.location_details.generic_locations, j2735_v2x_msgs::msg::ITISGenericLocations::IN_STREET);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.vert_event.exceeded_wheels, j2735_v2x_msgs::msg::VerticalAccelerationThreshold::RIGHT_REAR);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.status.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.status.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.status.status_details.code, 539);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.status.location_details.generic_locations, j2735_v2x_msgs::msg::ITISGenericLocations::CROSS_ROAD);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.speed_profile[0].speed, 20);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.speed_profile[1].speed, 25);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.speed_profile[2].unavailable, true);

    // Verify BSM.part_ii[2] (VehicleSafetyExtensions)
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.presence_vector, message.part_ii[2].vehicle_safety_extensions.presence_vector);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].lat_offset.offset, 0.0131067);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].lon_offset.offset, 0.0131068);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].elevation_offset.offset, -201);
    ASSERT_NEAR(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].time_offset.offset, 600.34, 0.01);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].speed.speed, 100);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].pos_accuracy.semi_major, 10);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].pos_accuracy.semi_minor, 5);
    ASSERT_NEAR(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].pos_accuracy.orientation, 100, 0.01);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].heading.heading, 150);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.curr_gnss_status.statuses, j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.presence_vector, 
        message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.presence_vector);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.utc_time, 
        message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.utc_time);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.lon.longitude, 170.1);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.lat.latitude, 90.0);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.elevation.elevation, 510);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.heading.heading, 1);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed.speed.velocity, 50);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed.transmission.transmission_state, j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_major, 10);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_minor, 5);
    ASSERT_NEAR(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_accuracy.orientation, 100, 0.01);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.time_confidence.confidence, j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_confidence.pos.confidence, j2735_v2x_msgs::msg::PositionConfidence::A20M);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_confidence.elevation.confidence, j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed_confidence.heading.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed_confidence.speed.speed_confidence, j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed_confidence.throttle.confidence, j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_prediction.radius_of_curvature, 100);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_prediction.confidence, 0.5);

    // Verify BSM.regional[0] (ROUTE_DESTINATIONS)
    ASSERT_EQ(out_message.regional[0].regional_extension_id, carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS);
    ASSERT_EQ(out_message.regional[0].route_destination_points[0].latitude, 40.5);
    ASSERT_EQ(out_message.regional[0].route_destination_points[0].longitude, -160.2);
    ASSERT_EQ(out_message.regional[0].route_destination_points[0].elevation, 130);
    ASSERT_EQ(out_message.regional[0].route_destination_points[1].latitude, 43.5);
    ASSERT_EQ(out_message.regional[0].route_destination_points[1].longitude, -142.2);
    ASSERT_EQ(out_message.regional[0].route_destination_points[1].elevation, 110);
}

TEST(ControlRequest, convertBSMcavToJ2735)
{ 
    // Create carma_v2x_msgs::msg::BSM message, which will be converted to j2735_v2x_msgs::msg::BSM out_message
    carma_v2x_msgs::msg::BSM message;

    // BSM.core_data (carma_v2x_msgs)
    message.core_data.msg_count = 1;
    message.core_data.id = {1,2,3,4};
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::SEC_MARK_AVAILABLE;
    message.core_data.sec_mark = 2;
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE;
    message.core_data.longitude = 3;
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE;
    message.core_data.latitude = 4;
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::ELEVATION_AVAILABLE;
    message.core_data.elev = 5;
    message.core_data.accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
    message.core_data.accuracy.semi_major = 10;
    message.core_data.accuracy.semi_minor = 2;
    message.core_data.accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
    message.core_data.accuracy.orientation = 2;
    message.core_data.transmission.transmission_state = j2735_v2x_msgs::msg::TransmissionState::PARK;
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE;
    message.core_data.speed = 5;
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::HEADING_AVAILABLE;
    message.core_data.heading = 100;
    message.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::STEER_WHEEL_ANGLE_AVAILABLE;
    message.core_data.angle = -40.0;
    message.core_data.accel_set.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_AVAILABLE;
    message.core_data.accel_set.longitudinal = 5;
    message.core_data.accel_set.lateral = -6;
    message.core_data.accel_set.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::ACCELERATION_VERTICAL_AVAILABLE;
    message.core_data.accel_set.vert = 1;
    message.core_data.accel_set.presence_vector |= carma_v2x_msgs::msg::AccelerationSet4Way::YAWRATE_AVAILABLE;
    message.core_data.accel_set.yaw_rate = -50;
    message.core_data.brakes.wheel_brakes.brake_applied_status = j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR;
    message.core_data.brakes.traction.traction_control_status = 1;
    message.core_data.brakes.abs.anti_lock_brake_status = 1;
    message.core_data.brakes.scs.stability_control_status = 1;
    message.core_data.brakes.brake_boost.brake_boost_applied = 1;
    message.core_data.brakes.aux_brakes.auxiliary_brake_status = 1;
    message.core_data.size.presence_vector |= carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE;
    message.core_data.size.vehicle_width = 10;
    message.core_data.size.presence_vector |= carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE;
    message.core_data.size.vehicle_length = 20;

    // Part II
    message.presence_vector |= carma_v2x_msgs::msg::BSM::HAS_PART_II;

    // BSMPartIIExtension.special_vehicle_extensions
    carma_v2x_msgs::msg::BSMPartIIExtension part_ii_special;
    part_ii_special.part_ii_id = carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;

    // BSMPartIIExtension.special_vehicle_extensions.vehicle_alerts
    part_ii_special.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = j2735_v2x_msgs::msg::SirenInUse::IN_USE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = j2735_v2x_msgs::msg::LightbarInUse::IN_USE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.multi.multi_vehicle_response = j2735_v2x_msgs::msg::MultiVehicleResponse::SINGLE_VEHICLE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_EVENTS;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.events.event.privileged_event_flags = j2735_v2x_msgs::msg::PrivilegedEventFlags::PE_EMERGENCY_RESPONSE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_RESPONSE_TYPE;
    part_ii_special.special_vehicle_extensions.vehicle_alerts.response_type.response_type = j2735_v2x_msgs::msg::ResponseType::EMERGENCY;

    // BSMPartIIExtension.special_vehicle_extensions.description
    part_ii_special.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION;
    part_ii_special.special_vehicle_extensions.description.type_event.code = 6;
    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_DESCRIPTION;
    j2735_v2x_msgs::msg::ITIScodes code;
    code.code = 40;
    part_ii_special.special_vehicle_extensions.description.description.push_back(code);

    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_PRIORITY;
    j2735_v2x_msgs::msg::Priority priority;
    for(int i = 0; i < priority.priority.size(); i++){
        priority.priority[i] = 5;
    }
    part_ii_special.special_vehicle_extensions.description.priority = priority;

    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_HEADING;
    part_ii_special.special_vehicle_extensions.description.heading.heading_slice = j2735_v2x_msgs::msg::HeadingSlice::FROM_045_0_TO_067_5_DEGREES;
    part_ii_special.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_EXTENT;
    part_ii_special.special_vehicle_extensions.description.extent.extent_value = j2735_v2x_msgs::msg::Extent::USE_FOR_50000_METERS; 

    // BSMPartIIExtension.special_vehicle_extensions.trailers
    part_ii_special.special_vehicle_extensions.presence_vector |= carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS;
    part_ii_special.special_vehicle_extensions.trailers.ssp_index = 0;
    part_ii_special.special_vehicle_extensions.trailers.connection.pivot_offset.offset = 5;
    part_ii_special.special_vehicle_extensions.trailers.connection.pivot_angle.angle = 104;
    part_ii_special.special_vehicle_extensions.trailers.connection.pivots.pivoting_allowed = true;
    
    carma_v2x_msgs::msg::TrailerUnitDescription trailer_unit_description_msg;
    trailer_unit_description_msg.is_dolly.is_dolly = true;
    trailer_unit_description_msg.width.vehicle_width = 6;
    trailer_unit_description_msg.length.vehicle_length = 7;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT;
    trailer_unit_description_msg.height.vehicle_height = 2;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS;
    trailer_unit_description_msg.mass.trailer_mass = 6000;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS;
    trailer_unit_description_msg.bumper_heights.front.bumper_height = 1.1;
    trailer_unit_description_msg.bumper_heights.rear.bumper_height = 1.2;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY;
    trailer_unit_description_msg.center_of_gravity.vehicle_height = 2.5;
    trailer_unit_description_msg.front_pivot.pivot_offset.offset = 9.4;
    trailer_unit_description_msg.front_pivot.pivot_angle.angle = 20;
    trailer_unit_description_msg.front_pivot.pivots.pivoting_allowed = true;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT;
    trailer_unit_description_msg.rear_pivot.pivot_offset.offset = 8.4;
    trailer_unit_description_msg.rear_pivot.pivot_angle.angle = 22;
    trailer_unit_description_msg.rear_pivot.pivots.pivoting_allowed = false;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET;
    trailer_unit_description_msg.rear_wheel_offset.offset = 1.9;
    trailer_unit_description_msg.position_offset.x.offset = 18;
    trailer_unit_description_msg.position_offset.y.offset = 19;
    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET;
    trailer_unit_description_msg.elevation_offset.offset = 1.2;

    trailer_unit_description_msg.presence_vector |= carma_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA;
    carma_v2x_msgs::msg::TrailerHistoryPoint trailer_history_point_msg;
    trailer_history_point_msg.pivot_angle.angle = 30;
    trailer_history_point_msg.time_offset.offset = 100;
    trailer_history_point_msg.position_offset.x.offset = 10;
    trailer_history_point_msg.position_offset.y.offset = 20;
    trailer_history_point_msg.presence_vector |= carma_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET;
    trailer_history_point_msg.elevation_offset.offset = 1.1;
    trailer_history_point_msg.presence_vector |= carma_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING;
    trailer_history_point_msg.heading.heading = 84;
    trailer_unit_description_msg.crumb_data.trailer_history_points.push_back(trailer_history_point_msg);

    part_ii_special.special_vehicle_extensions.trailers.units.trailer_unit_descriptions.push_back(trailer_unit_description_msg);

    message.part_ii.push_back(part_ii_special);

    // BSMPartIIExtension.supplemental_vehicle_extensions
    carma_v2x_msgs::msg::BSMPartIIExtension part_ii_supp;
    part_ii_supp.part_ii_id = carma_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT;

    // BSMPartIIExtension.supplemental_vehicle_extensions.classification
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION;
    part_ii_supp.supplemental_vehicle_extensions.classification.basic_vehicle_class = j2735_v2x_msgs::msg::BasicVehicleClass::SPECIAL_VEHICLE_CLASS;

    // BSMPartIIExtension.supplemental_vehicle_extensions.class_details
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_KEY_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.key_type.basic_vehicle_class = j2735_v2x_msgs::msg::BasicVehicleClass::SPECIAL_VEHICLE_CLASS;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ROLE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.role.basic_vehicle_role = j2735_v2x_msgs::msg::BasicVehicleRole::POLICE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ISO;
    part_ii_supp.supplemental_vehicle_extensions.class_details.iso3833 = 23;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_HPMS_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.hpms_type.vehicle_type = j2735_v2x_msgs::msg::VehicleType::SPECIAL;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_VEHICLE_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.vehicle_type.vehicle_group_affected = j2735_v2x_msgs::msg::ITISVehicleGroupAffected::HEAVY_VEHICLES;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONSE_EQUIP;
    part_ii_supp.supplemental_vehicle_extensions.class_details.response_equip.incident_response_equipment = j2735_v2x_msgs::msg::ITISIncidentResponseEquipment::HAZMAT_UNIT;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONDER_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.responder_type.responder_group_affected = j2735_v2x_msgs::msg::ITISResponderGroupAffected::EMERGENCY_VEHICLE_UNITS;
    part_ii_supp.supplemental_vehicle_extensions.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_FUEL_TYPE;
    part_ii_supp.supplemental_vehicle_extensions.class_details.fuel_type.fuel_type = j2735_v2x_msgs::msg::FuelType::HYBRID;

    // BSMPartIIExtension.supplemental_vehicle_extensions.vehicle_data
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_HEIGHT;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.height.vehicle_height = 3;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_BUMPERS;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.bumpers.front.bumper_height = 1.1;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.bumpers.rear.bumper_height = 0.8;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_MASS;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.mass.vehicle_mass = 8500;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT;
    part_ii_supp.supplemental_vehicle_extensions.vehicle_data.trailer_weight.trailer_weight = 4800;

    // BSMPartIIExtension.supplemental_vehicle_extensions.weather_report
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.is_raining.precip_yes_no = j2735_v2x_msgs::msg::NTCIPEssPrecipYesNo::PRECIP;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.rain_rate.precip_rate = 205;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.precip_situation.ess_precip_situation = j2735_v2x_msgs::msg::NTCIPEssPrecipSituation::RAIN_SLIGHT;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.solar_radiation.ess_solar_radiation = 206;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_FRICTION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.friction.ess_mobile_friction = 90;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION;
    part_ii_supp.supplemental_vehicle_extensions.weather_report.road_friction.coefficient = 0.30;

    // BSMPartIIExtension.supplemental_vehicle_extensions.weather_probe
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.air_temp.temperature = 100;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.air_pressure.pressure = 60000;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.status_front.wiper_status = j2735_v2x_msgs::msg::WiperStatus::INTERMITTENT;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.rate_front.wiper_rate = 1;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.status_rear.wiper_status = j2735_v2x_msgs::msg::WiperStatus::LOW;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_RATE_REAR;
    part_ii_supp.supplemental_vehicle_extensions.weather_probe.rain_rates.rate_rear.wiper_rate = 1.5;

    // BSMPartIIExtension.supplemental_vehicle_extensions.obstacle
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.ob_dist.distance = 500;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.ob_direct.direction.angle = 1600;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.year.year = 1000;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.month.month = 10;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.day.day = 20;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.hour.hour = 21;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.minute.minute = 22;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.second.millisecond = 20000;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.date_time.offset.offset_minute = 800;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.description.code = 540;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.location_details.generic_locations = j2735_v2x_msgs::msg::ITISGenericLocations::IN_STREET;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT;
    part_ii_supp.supplemental_vehicle_extensions.obstacle.vert_event.exceeded_wheels = j2735_v2x_msgs::msg::VerticalAccelerationThreshold::RIGHT_REAR;

    // BSMPartIIExtension.supplemental_vehicle_extensions.status
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS;
    part_ii_supp.supplemental_vehicle_extensions.status.status_details.code = 539;
    part_ii_supp.supplemental_vehicle_extensions.status.presence_vector |= j2735_v2x_msgs::msg::DisabledVehicle::HAS_LOCATION_DETAILS;
    part_ii_supp.supplemental_vehicle_extensions.status.location_details.generic_locations = j2735_v2x_msgs::msg::ITISGenericLocations::CROSS_ROAD;

    // BSMPartIIExtension.supplemental_vehicle_extensions.speed_profile
    part_ii_supp.supplemental_vehicle_extensions.presence_vector |= carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE;
    carma_v2x_msgs::msg::GrossSpeed speed_point1;
    speed_point1.speed = 20;
    carma_v2x_msgs::msg::GrossSpeed speed_point2;
    speed_point2.speed = 25;
    carma_v2x_msgs::msg::GrossSpeed speed_point3;
    speed_point3.unavailable = true;
    part_ii_supp.supplemental_vehicle_extensions.speed_profile.push_back(speed_point1);
    part_ii_supp.supplemental_vehicle_extensions.speed_profile.push_back(speed_point2);
    part_ii_supp.supplemental_vehicle_extensions.speed_profile.push_back(speed_point3);

    message.part_ii.push_back(part_ii_supp);

    // BSMPartIIExtension.vehicle_safety_extensions
    carma_v2x_msgs::msg::BSMPartIIExtension part_ii_safety;
    part_ii_safety.part_ii_id = carma_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT;

    // BSMPartIIExtension.vehicle_safety_extensions.path_history
    part_ii_safety.vehicle_safety_extensions.presence_vector |= carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY;
    carma_v2x_msgs::msg::PathHistoryPoint point1;
    point1.lat_offset.offset = 0.0131067;
    point1.lon_offset.offset = 0.0131068;
    point1.elevation_offset.offset = -201;
    point1.time_offset.offset = 600.34;
    point1.speed.speed = 100;
    point1.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
    point1.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
    point1.pos_accuracy.semi_major = 10;
    point1.pos_accuracy.semi_minor = 5;
    point1.pos_accuracy.orientation = 100;
    point1.heading.heading = 150;
    part_ii_safety.vehicle_safety_extensions.path_history.crumb_data.points.push_back(point1);

    part_ii_safety.vehicle_safety_extensions.path_history.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;
    part_ii_safety.vehicle_safety_extensions.path_history.curr_gnss_status.statuses = j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY;

    carma_v2x_msgs::msg::FullPositionVector initial_position;
    part_ii_safety.vehicle_safety_extensions.path_history.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.year.year = 1000;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.month.month = 10;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.day.day = 20;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.hour.hour = 21;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.minute.minute = 22;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.second.millisecond = 20000;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.utc_time.offset.offset_minute = 800;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.lon.longitude = 170.1;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.lat.latitude = 90.0;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= carma_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.elevation.elevation = 510;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_HEADING;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.heading.heading = 1;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed.transmission.transmission_state |= j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed.speed.velocity = 50;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_major = 10;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_minor = 5;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_accuracy.orientation = 100;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A20M;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00;

    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed_confidence.heading.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed_confidence.speed.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
    part_ii_safety.vehicle_safety_extensions.path_history.initial_position.speed_confidence.throttle.confidence |= j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT;

    // BSMPartIIExtension.vehicle_safety_extensions.path_prediction
    part_ii_safety.vehicle_safety_extensions.presence_vector |= carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION;
    part_ii_safety.vehicle_safety_extensions.path_prediction.radius_of_curvature = 100;
    part_ii_safety.vehicle_safety_extensions.path_prediction.confidence = 0.5;

    message.part_ii.push_back(part_ii_safety);

    // Regional Extension
    message.presence_vector |= carma_v2x_msgs::msg::BSM::HAS_REGIONAL;

    // BSMRegionalExtension.route_destination_points
    carma_v2x_msgs::msg::BSMRegionalExtension regional_ext;
    regional_ext.regional_extension_id = carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS;

    carma_v2x_msgs::msg::Position3D position1;
    position1.latitude = 40.5;
    position1.longitude = -160.2;
    position1.elevation_exists = true;
    position1.elevation = 130;

    carma_v2x_msgs::msg::Position3D position2;
    position2.latitude = 43.5;
    position2.longitude = -142.2;
    position2.elevation_exists = true;
    position2.elevation = 110;

    regional_ext.route_destination_points.push_back(position1);
    regional_ext.route_destination_points.push_back(position2);
    message.regional.push_back(regional_ext);

    // Convert 'message' (carma_v2x_msgs::msg::BSM) to 'out_message' (j2735_v2x_msgs::msg::BSM)
    j2735_v2x_msgs::msg::BSM out_message;
    j2735_convertor::BSMConvertor::convert(message, out_message);

    // Verify BSM.core_data
    ASSERT_EQ(out_message.core_data.msg_count, 1);
    std::vector<uint8_t> id = {1,2,3,4};
    ASSERT_EQ(out_message.core_data.id, id);
    ASSERT_EQ(out_message.core_data.sec_mark, 2);
    ASSERT_EQ(out_message.core_data.longitude, 30000000);
    ASSERT_EQ(out_message.core_data.latitude, 40000000);
    ASSERT_EQ(out_message.core_data.elev, 50);
    ASSERT_EQ(out_message.core_data.accuracy.semi_major, 200);
    ASSERT_EQ(out_message.core_data.accuracy.semi_minor, 40);
    ASSERT_EQ(out_message.core_data.accuracy.orientation, 364);
    ASSERT_EQ(out_message.core_data.transmission.transmission_state, j2735_v2x_msgs::msg::TransmissionState::PARK);
    ASSERT_EQ(out_message.core_data.speed, 250);
    ASSERT_EQ(out_message.core_data.heading, 8000);
    ASSERT_EQ(out_message.core_data.angle, -26);
    ASSERT_EQ(out_message.core_data.accel_set.longitudinal, 500);
    ASSERT_EQ(out_message.core_data.accel_set.lateral, -600);
    ASSERT_EQ(out_message.core_data.accel_set.yaw_rate, -5000);
    ASSERT_EQ(out_message.core_data.brakes.wheel_brakes.brake_applied_status, j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR);
    ASSERT_EQ(out_message.core_data.brakes.traction.traction_control_status, 1);
    ASSERT_EQ(out_message.core_data.brakes.abs.anti_lock_brake_status, 1);
    ASSERT_EQ(out_message.core_data.brakes.scs.stability_control_status, 1);
    ASSERT_EQ(out_message.core_data.size.vehicle_width, 1000);
    ASSERT_EQ(out_message.core_data.size.vehicle_length, 2000);

    // Verify BSM Part II Content
    ASSERT_EQ(out_message.presence_vector, message.presence_vector);

    // Verify BSM.part_ii[0] (SpecialVehicleExtensions)
    ASSERT_EQ(out_message.part_ii[0].part_ii_id, j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT);
    ASSERT_EQ(out_message.part_ii[0].special_vehicle_extensions.presence_vector, message.part_ii[0].special_vehicle_extensions.presence_vector);
    ASSERT_EQ(out_message.part_ii[0].special_vehicle_extensions.vehicle_alerts, message.part_ii[0].special_vehicle_extensions.vehicle_alerts);
    ASSERT_EQ(out_message.part_ii[0].special_vehicle_extensions.description, message.part_ii[0].special_vehicle_extensions.description);

    j2735_v2x_msgs::msg::TrailerData out_trailers = out_message.part_ii[0].special_vehicle_extensions.trailers;
    ASSERT_EQ(out_trailers.ssp_index, 0);
    ASSERT_EQ(out_trailers.connection.pivot_offset.offset, 500);
    ASSERT_EQ(out_trailers.connection.pivot_angle.angle, 8320);
    ASSERT_EQ(out_trailers.connection.pivots.pivoting_allowed, true);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].presence_vector, message.part_ii[0].special_vehicle_extensions.trailers.units.trailer_unit_descriptions[0].presence_vector);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].is_dolly.is_dolly, true);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].width.vehicle_width, 600);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].length.vehicle_length, 700);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].height.vehicle_height, 40);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].mass.trailer_mass, 12);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].bumper_heights.front.bumper_height, 110);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].bumper_heights.rear.bumper_height, 120);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].center_of_gravity.vehicle_height, 50);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].front_pivot.pivot_offset.offset, 939);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].front_pivot.pivot_angle.angle, 1600);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].front_pivot.pivots.pivoting_allowed, true);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].rear_pivot.pivot_offset.offset, 839);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].rear_pivot.pivot_angle.angle, 1760);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].rear_pivot.pivots.pivoting_allowed, false);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].rear_wheel_offset.offset, 189);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].position_offset.x, 1800);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].position_offset.y, 1900);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].elevation_offset.offset, 12);

    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].presence_vector, 
        message.part_ii[0].special_vehicle_extensions.trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].presence_vector);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].pivot_angle.angle, 2400);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].time_offset.offset, 10000);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].position_offset.x, 1000);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].position_offset.y, 2000);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].elevation_offset.offset, 11);
    ASSERT_EQ(out_trailers.units.trailer_unit_descriptions[0].crumb_data.trailer_history_points[0].heading.heading, 56);

    // Verify BSM.part_ii[1] (SupplementalVehicleExtensions)
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.classification, message.part_ii[1].supplemental_vehicle_extensions.classification);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.class_details, message.part_ii[1].supplemental_vehicle_extensions.class_details);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.height.vehicle_height, 60);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.bumpers.front.bumper_height, 110);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.bumpers.rear.bumper_height, 80);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.mass.vehicle_mass, 89);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.vehicle_data.trailer_weight.trailer_weight, 2400);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.weather_report.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.is_raining.precip_yes_no, j2735_v2x_msgs::msg::NTCIPEssPrecipYesNo::PRECIP);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.rain_rate.precip_rate, 2050);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.precip_situation.ess_precip_situation, j2735_v2x_msgs::msg::NTCIPEssPrecipSituation::RAIN_SLIGHT);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.solar_radiation.ess_solar_radiation, 206);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.friction.ess_mobile_friction, 90);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_report.road_friction.coefficient, 15);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.weather_probe.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.air_temp.temperature, 140);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.air_pressure.pressure, 120);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.status_front.wiper_status, j2735_v2x_msgs::msg::WiperStatus::INTERMITTENT);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.rate_front.wiper_rate, 60);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.status_rear.wiper_status, j2735_v2x_msgs::msg::WiperStatus::LOW);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.weather_probe.rain_rates.rate_rear.wiper_rate, 90);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.obstacle.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.ob_dist.distance, 500);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.date_time, message.part_ii[1].supplemental_vehicle_extensions.obstacle.date_time);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.description.code, 540);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.location_details.generic_locations, j2735_v2x_msgs::msg::ITISGenericLocations::IN_STREET);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.obstacle.vert_event.exceeded_wheels, j2735_v2x_msgs::msg::VerticalAccelerationThreshold::RIGHT_REAR);

    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.status.presence_vector, message.part_ii[1].supplemental_vehicle_extensions.status.presence_vector);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.status.status_details.code, 539);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.status.location_details.generic_locations, j2735_v2x_msgs::msg::ITISGenericLocations::CROSS_ROAD);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.speed_profile[0].speed, 20);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.speed_profile[1].speed, 25);
    ASSERT_EQ(out_message.part_ii[1].supplemental_vehicle_extensions.speed_profile[2].speed, 31);

    // Verify BSM.part_ii[2] (VehicleSafetyExtensions)
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.presence_vector, message.part_ii[2].vehicle_safety_extensions.presence_vector);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].lat_offset.offset, 131067);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].lon_offset.offset, 131068);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].elevation_offset.offset, -2010);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].time_offset.offset, 60034);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].speed.speed, 5000);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].pos_accuracy.semi_major, 200);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].pos_accuracy.semi_minor, 100);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].pos_accuracy.orientation, 18204);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.crumb_data.points[0].heading.heading, 100);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.curr_gnss_status.statuses, j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.presence_vector, 
        message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.presence_vector);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.utc_time, 
        message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.utc_time);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.lon.longitude, 1701000000);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.lat.latitude, 900000000);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.elevation.elevation, 5100);

    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.heading.heading, 80);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed.speed.velocity, 2500);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed.transmission.transmission_state, j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_major, 200);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_accuracy.semi_minor, 100);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_accuracy.orientation, 18204);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.time_confidence.confidence, j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_confidence.pos.confidence, j2735_v2x_msgs::msg::PositionConfidence::A20M);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.pos_confidence.elevation.confidence, j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed_confidence.heading.confidence, j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed_confidence.speed.speed_confidence, j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_history.initial_position.speed_confidence.throttle.confidence, j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_prediction.radius_of_curvature, 10000);
    ASSERT_EQ(out_message.part_ii[2].vehicle_safety_extensions.path_prediction.confidence, 100);

    // Verify BSM.regional[0] (ROUTE_DESTINATIONS)
    ASSERT_EQ(out_message.regional[0].regional_extension_id, j2735_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS);
    ASSERT_EQ(out_message.regional[0].route_destination_points[0].latitude, 405000000);
    ASSERT_EQ(out_message.regional[0].route_destination_points[0].longitude, -1602000000);
    ASSERT_EQ(out_message.regional[0].route_destination_points[0].elevation, 1300);
    ASSERT_EQ(out_message.regional[0].route_destination_points[1].latitude, 435000000);
    ASSERT_EQ(out_message.regional[0].route_destination_points[1].longitude, -1422000000);
    ASSERT_EQ(out_message.regional[0].route_destination_points[1].elevation, 1100);
}

}// namespace j2735_convertor*/