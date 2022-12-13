/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "cpp_message/BSM_Message.h"
#include <gtest/gtest.h>
#include <boost/optional/optional_io.hpp>   //to print boost::optional

TEST(BSMTest, testDecodeBSM)
{

    std::vector<uint8_t> binary_input = {0,20,37,0,64,64,128,193,0,0,90,210,116,128,53,164,233,0,8,0,0,0,0,0,128,0,0,0,126,125,7,208,127,128,0,10,170,0,128,8};
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::BSM_Message worker(node->get_node_logging_interface());
    auto res = worker.decode_bsm_message(binary_input);
    j2735_v2x_msgs::msg::BSM to_read;
    if(res)
    {
        to_read=res.get();
        EXPECT_EQ(to_read.core_data.msg_count, 1);
        
        EXPECT_EQ(to_read.core_data.id[0], (uint8_t)1);
        EXPECT_EQ(to_read.core_data.id[1], (uint8_t)2);
        EXPECT_EQ(to_read.core_data.id[2], (uint8_t)3);
        EXPECT_EQ(to_read.core_data.id[3], (uint8_t)4);

        EXPECT_EQ(to_read.core_data.sec_mark, 1);
        EXPECT_EQ(to_read.core_data.latitude, 0); 
        EXPECT_EQ(to_read.core_data.longitude, 1); 
        EXPECT_EQ(to_read.core_data.elev, 0);
        EXPECT_EQ(to_read.core_data.accuracy.orientation, 1); 
        EXPECT_EQ(to_read.core_data.accuracy.semi_major, 0); 
        EXPECT_EQ(to_read.core_data.accuracy.semi_minor, 0); 
        EXPECT_EQ(to_read.core_data.transmission.transmission_state, 0); 
        EXPECT_EQ(to_read.core_data.speed, 0); 
        EXPECT_EQ(to_read.core_data.heading, 0); 
        EXPECT_EQ(to_read.core_data.angle, 0); 
        EXPECT_EQ(to_read.core_data.accel_set.lateral, 0); 
        EXPECT_EQ(to_read.core_data.accel_set.longitudinal, 0);
        EXPECT_EQ(to_read.core_data.accel_set.vert, 0);
        EXPECT_EQ(to_read.core_data.accel_set.yaw_rate, 1); 
        EXPECT_EQ(to_read.core_data.brakes.wheel_brakes.brake_applied_status, j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR); 
        EXPECT_EQ(to_read.core_data.brakes.traction.traction_control_status, 1); 
        EXPECT_EQ(to_read.core_data.brakes.abs.anti_lock_brake_status, 1); 
        EXPECT_EQ(to_read.core_data.brakes.scs.stability_control_status, 1); 
        EXPECT_EQ(to_read.core_data.brakes.brake_boost.brake_boost_applied, 1);
        EXPECT_EQ(to_read.core_data.brakes.aux_brakes.auxiliary_brake_status, 1);    
        EXPECT_EQ(to_read.core_data.size.vehicle_length, 1); 
        EXPECT_EQ(to_read.core_data.size.vehicle_width, 1); 
    }
    else EXPECT_TRUE(false);
}

TEST(BSMTest, testEncodeBSM)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::BSM_Message worker(node->get_node_logging_interface());    
    j2735_v2x_msgs::msg::BSM message;
    message.core_data.msg_count = 1;
    message.core_data.id = {1,2,3,4};
    message.core_data.sec_mark = 1;
    message.core_data.longitude = 1;
    message.core_data.accuracy.orientation = 1;
    message.core_data.brakes.wheel_brakes.brake_applied_status = j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR;
    message.core_data.brakes.traction.traction_control_status = 1;
    message.core_data.brakes.abs.anti_lock_brake_status = 1;
    message.core_data.brakes.scs.stability_control_status = 1;
    message.core_data.brakes.brake_boost.brake_boost_applied = 1;
    message.core_data.brakes.aux_brakes.auxiliary_brake_status = 1;
    message.core_data.accel_set.yaw_rate = 1;
    message.core_data.size.vehicle_width = 1;
    message.core_data.size.vehicle_length = 1;

    message.presence_vector |= j2735_v2x_msgs::msg::BSM::HAS_PART_II;

    j2735_v2x_msgs::msg::BSMPartIIExtension part_ii_element;
    part_ii_element.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;
    part_ii_element.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = j2735_v2x_msgs::msg::SirenInUse::IN_USE;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = j2735_v2x_msgs::msg::LightbarInUse::IN_USE;

    part_ii_element.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION;
    part_ii_element.special_vehicle_extensions.description.type_event.code = 6;
    part_ii_element.special_vehicle_extensions.description.presence_vector = j2735_v2x_msgs::msg::EventDescription::HAS_PRIORITY;
    j2735_v2x_msgs::msg::Priority priority;
    for(int i = 0; i < priority.priority.size(); i++){
        priority.priority[i] = 5;
    }
    part_ii_element.special_vehicle_extensions.description.priority = priority;

    message.part_ii.push_back(part_ii_element);

    auto res = worker.encode_bsm_message(message);

    if(res) {
        std::vector<uint8_t> to_read=res.get();
        size_t len=to_read.size();
        for(size_t i=0;i<len;i++)std::cout<<int(to_read[i])<<",";
        std::cout<<"\n";
        EXPECT_TRUE(true);
    }
    else
    {
        std::cout << "Encoding failed while unit testing BSM encoder!\n";
        EXPECT_TRUE(false);
    }
}

TEST(BSMTest, testEncodeDecodeBSM)
{
    auto node = std::make_shared<rclcpp::Node>("test_node");
    cpp_message::BSM_Message worker(node->get_node_logging_interface());    
    j2735_v2x_msgs::msg::BSM message;
    message.core_data.msg_count = 1;
    message.core_data.id = {1,2,3,4};
    message.core_data.sec_mark = 2;
    message.core_data.longitude = 3;
    message.core_data.accuracy.orientation = 4;
    message.core_data.brakes.wheel_brakes.brake_applied_status = j2735_v2x_msgs::msg::BrakeAppliedStatus::RIGHT_REAR;
    message.core_data.brakes.traction.traction_control_status = 1;
    message.core_data.brakes.abs.anti_lock_brake_status = 1;
    message.core_data.brakes.scs.stability_control_status = 1;
    message.core_data.brakes.brake_boost.brake_boost_applied = 1;
    message.core_data.brakes.aux_brakes.auxiliary_brake_status = 1;
    message.core_data.accel_set.yaw_rate = 18;
    message.core_data.size.vehicle_width = 19;
    message.core_data.size.vehicle_length = 20;

    message.presence_vector |= j2735_v2x_msgs::msg::BSM::HAS_PART_II;

    // Add special_vehicle_extensions
    j2735_v2x_msgs::msg::BSMPartIIExtension part_ii_element;
    part_ii_element.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;

    // Add special_vehicle_extensions.vehicle_alerts
    part_ii_element.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = j2735_v2x_msgs::msg::SirenInUse::IN_USE;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = j2735_v2x_msgs::msg::LightbarInUse::IN_USE;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.multi.multi_vehicle_response = j2735_v2x_msgs::msg::MultiVehicleResponse::SINGLE_VEHICLE;

    part_ii_element.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_EVENTS;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.events.event.privileged_event_flags = j2735_v2x_msgs::msg::PrivilegedEventFlags::PE_EMERGENCY_RESPONSE;

    part_ii_element.special_vehicle_extensions.vehicle_alerts.presence_vector |= j2735_v2x_msgs::msg::EmergencyDetails::HAS_RESPONSE_TYPE;
    part_ii_element.special_vehicle_extensions.vehicle_alerts.response_type.response_type = j2735_v2x_msgs::msg::ResponseType::EMERGENCY;

    // Add special_vehicle_extensions.description
    part_ii_element.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION;
    part_ii_element.special_vehicle_extensions.description.type_event.code = 6;

    part_ii_element.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_DESCRIPTION;
    j2735_v2x_msgs::msg::ITIScodes code;
    code.code = 40;
    part_ii_element.special_vehicle_extensions.description.description.push_back(code);

    part_ii_element.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_PRIORITY;
    j2735_v2x_msgs::msg::Priority priority;
    for(int i = 0; i < priority.priority.size(); i++){
        priority.priority[i] = 5;
    }
    part_ii_element.special_vehicle_extensions.description.priority = priority;

    part_ii_element.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_HEADING;
    part_ii_element.special_vehicle_extensions.description.heading.heading_slice = j2735_v2x_msgs::msg::HeadingSlice::FROM_045_0_TO_067_5_DEGREES;

    part_ii_element.special_vehicle_extensions.description.presence_vector |= j2735_v2x_msgs::msg::EventDescription::HAS_EXTENT;
    part_ii_element.special_vehicle_extensions.description.extent.extent_value = j2735_v2x_msgs::msg::Extent::USE_FOR_50000_METERS; 

    // Add special_vehicle_extensions.trailers
    part_ii_element.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS;
    part_ii_element.special_vehicle_extensions.trailers.connection.pivot_offset.offset = 50;
    part_ii_element.special_vehicle_extensions.trailers.connection.pivot_angle.angle = 104;
    part_ii_element.special_vehicle_extensions.trailers.connection.pivots.pivoting_allowed = true;

    j2735_v2x_msgs::msg::TrailerUnitDescription trailer_unit_description_msg;
    trailer_unit_description_msg.is_dolly.is_dolly = true;
    trailer_unit_description_msg.width.vehicle_width = 220;
    trailer_unit_description_msg.length.vehicle_length = 955;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT;
    trailer_unit_description_msg.height.vehicle_height = 53;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS;
    trailer_unit_description_msg.mass.trailer_mass = 91;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS;
    trailer_unit_description_msg.bumper_heights.front.bumper_height = 48;
    trailer_unit_description_msg.bumper_heights.rear.bumper_height = 49;
    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY;
    trailer_unit_description_msg.center_of_gravity.vehicle_height = 50;

    trailer_unit_description_msg.front_pivot.pivot_offset.offset = 19;
    trailer_unit_description_msg.front_pivot.pivot_angle.angle = 20;
    trailer_unit_description_msg.front_pivot.pivots.pivoting_allowed = true;

    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT;
    trailer_unit_description_msg.rear_pivot.pivot_offset.offset = 21;
    trailer_unit_description_msg.rear_pivot.pivot_angle.angle = 22;
    trailer_unit_description_msg.rear_pivot.pivots.pivoting_allowed = false;

    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET;
    trailer_unit_description_msg.rear_wheel_offset.offset = 199;

    trailer_unit_description_msg.position_offset.x = 200;
    trailer_unit_description_msg.position_offset.y = 201;

    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET;
    trailer_unit_description_msg.elevation_offset.offset = 50;

    trailer_unit_description_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA;
    j2735_v2x_msgs::msg::TrailerHistoryPoint trailer_history_point_msg;
    trailer_history_point_msg.pivot_angle.angle = 30;
    trailer_history_point_msg.time_offset.offset = 3400;
    trailer_history_point_msg.position_offset.x = 99;
    trailer_history_point_msg.position_offset.y = 100;
    trailer_history_point_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET;
    trailer_history_point_msg.elevation_offset.offset = 50;
    trailer_history_point_msg.presence_vector |= j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING;
    trailer_history_point_msg.heading.heading = 83;
    trailer_unit_description_msg.crumb_data.trailer_history_points.push_back(trailer_history_point_msg);

    part_ii_element.special_vehicle_extensions.trailers.units.trailer_unit_descriptions.push_back(trailer_unit_description_msg);

    message.part_ii.push_back(part_ii_element);

    // Add supplemental_vehicle_extensions
    j2735_v2x_msgs::msg::BSMPartIIExtension extension2;
    extension2.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT;

    j2735_v2x_msgs::msg::SupplementalVehicleExtensions supp_vehicle_ext;

    // Add supplemental_vehicle_extensions.classification
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION;
    supp_vehicle_ext.classification.basic_vehicle_class = j2735_v2x_msgs::msg::BasicVehicleClass::SPECIAL_VEHICLE_CLASS;

    // Add supplemental_vehicle_extensions.class_details
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_KEY_TYPE;
    supp_vehicle_ext.class_details.key_type.basic_vehicle_class = j2735_v2x_msgs::msg::BasicVehicleClass::SPECIAL_VEHICLE_CLASS;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ROLE;
    supp_vehicle_ext.class_details.role.basic_vehicle_role = j2735_v2x_msgs::msg::BasicVehicleRole::POLICE;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_ISO;
    supp_vehicle_ext.class_details.iso3833 = 23;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_HPMS_TYPE;
    supp_vehicle_ext.class_details.hpms_type.vehicle_type = j2735_v2x_msgs::msg::VehicleType::SPECIAL;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_VEHICLE_TYPE;
    supp_vehicle_ext.class_details.vehicle_type.vehicle_group_affected = j2735_v2x_msgs::msg::ITISVehicleGroupAffected::HEAVY_VEHICLES;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONSE_EQUIP;
    supp_vehicle_ext.class_details.response_equip.incident_response_equipment = j2735_v2x_msgs::msg::ITISIncidentResponseEquipment::HAZMAT_UNIT;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_RESPONDER_TYPE;
    supp_vehicle_ext.class_details.responder_type.responder_group_affected = j2735_v2x_msgs::msg::ITISResponderGroupAffected::EMERGENCY_VEHICLE_UNITS;
    supp_vehicle_ext.class_details.presence_vector |= j2735_v2x_msgs::msg::VehicleClassification::HAS_FUEL_TYPE;
    supp_vehicle_ext.class_details.fuel_type.fuel_type = j2735_v2x_msgs::msg::FuelType::HYBRID;

    // Add supplemental_vehicle_extensions.vehicle_data
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA;
    supp_vehicle_ext.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_HEIGHT;
    supp_vehicle_ext.vehicle_data.height.vehicle_height = 120;
    supp_vehicle_ext.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_BUMPERS;
    supp_vehicle_ext.vehicle_data.bumpers.front.bumper_height = 88;
    supp_vehicle_ext.vehicle_data.bumpers.rear.bumper_height = 89;
    supp_vehicle_ext.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_MASS;
    supp_vehicle_ext.vehicle_data.mass.vehicle_mass = 90;
    supp_vehicle_ext.vehicle_data.presence_vector |= j2735_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT;
    supp_vehicle_ext.vehicle_data.trailer_weight.trailer_weight = 450;

    // Add supplemental_vehicle_extensions.weather_report
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT;
    supp_vehicle_ext.weather_report.is_raining.precip_yes_no = j2735_v2x_msgs::msg::NTCIPEssPrecipYesNo::PRECIP;
    supp_vehicle_ext.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE;
    supp_vehicle_ext.weather_report.rain_rate.precip_rate = 205;
    supp_vehicle_ext.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION;
    supp_vehicle_ext.weather_report.precip_situation.ess_precip_situation = j2735_v2x_msgs::msg::NTCIPEssPrecipSituation::RAIN_SLIGHT;
    supp_vehicle_ext.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION;
    supp_vehicle_ext.weather_report.solar_radiation.ess_solar_radiation = 206;
    supp_vehicle_ext.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_FRICTION;
    supp_vehicle_ext.weather_report.friction.ess_mobile_friction = 90;
    supp_vehicle_ext.weather_report.presence_vector |= j2735_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION;
    supp_vehicle_ext.weather_report.road_friction.coefficient = 30;

    // Add supplemental_vehicle_extensions.weather_probe
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE;
    supp_vehicle_ext.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP;
    supp_vehicle_ext.weather_probe.air_temp.temperature = 180;
    supp_vehicle_ext.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE;
    supp_vehicle_ext.weather_probe.air_pressure.pressure = 200;
    supp_vehicle_ext.weather_probe.presence_vector |= j2735_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES;
    supp_vehicle_ext.weather_probe.rain_rates.status_front.wiper_status = j2735_v2x_msgs::msg::WiperStatus::INTERMITTENT;
    supp_vehicle_ext.weather_probe.rain_rates.rate_front.wiper_rate = 40;
    supp_vehicle_ext.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR;
    supp_vehicle_ext.weather_probe.rain_rates.status_rear.wiper_status = j2735_v2x_msgs::msg::WiperStatus::LOW;
    supp_vehicle_ext.weather_probe.rain_rates.presence_vector |= j2735_v2x_msgs::msg::WiperSet::HAS_RATE_REAR;
    supp_vehicle_ext.weather_probe.rain_rates.rate_rear.wiper_rate = 41;

    // Add supplemental_vehicle_extensions.obstacle_detection
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE;
    supp_vehicle_ext.obstacle.ob_dist.distance = 500;
    supp_vehicle_ext.obstacle.ob_direct.direction.angle = 1600;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    supp_vehicle_ext.obstacle.date_time.year.year = 1000;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    supp_vehicle_ext.obstacle.date_time.month.month = 10;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    supp_vehicle_ext.obstacle.date_time.day.day = 20;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    supp_vehicle_ext.obstacle.date_time.hour.hour = 21;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    supp_vehicle_ext.obstacle.date_time.minute.minute = 22;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    supp_vehicle_ext.obstacle.date_time.second.millisecond = 20000;
    supp_vehicle_ext.obstacle.date_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    supp_vehicle_ext.obstacle.date_time.offset.offset_minute = 800;
    supp_vehicle_ext.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION;
    supp_vehicle_ext.obstacle.description.code = 540;
    supp_vehicle_ext.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS;
    supp_vehicle_ext.obstacle.location_details.generic_locations = j2735_v2x_msgs::msg::ITISGenericLocations::IN_STREET;
    supp_vehicle_ext.obstacle.presence_vector |= j2735_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT;
    supp_vehicle_ext.obstacle.vert_event.exceeded_wheels = j2735_v2x_msgs::msg::VerticalAccelerationThreshold::RIGHT_REAR;

    // Add supplemental_vehicle_extensions.status
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS;
    supp_vehicle_ext.status.status_details.code = 539;
    supp_vehicle_ext.status.presence_vector |= j2735_v2x_msgs::msg::DisabledVehicle::HAS_LOCATION_DETAILS;
    supp_vehicle_ext.status.location_details.generic_locations = j2735_v2x_msgs::msg::ITISGenericLocations::CROSS_ROAD;

    // Add supplemental_vehicle_extensions.speed_profile
    supp_vehicle_ext.presence_vector |= j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE;
    j2735_v2x_msgs::msg::GrossSpeed speed_point1;
    speed_point1.speed = 20;
    j2735_v2x_msgs::msg::GrossSpeed speed_point2;
    speed_point2.speed = 25;
    supp_vehicle_ext.speed_profile.push_back(speed_point1);
    supp_vehicle_ext.speed_profile.push_back(speed_point2);

    extension2.supplemental_vehicle_extensions = supp_vehicle_ext;

    message.part_ii.push_back(extension2);

    // Add vehicle_safety_extensions
    j2735_v2x_msgs::msg::BSMPartIIExtension extension3;
    extension3.part_ii_id = j2735_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT;

    j2735_v2x_msgs::msg::VehicleSafetyExtensions vehicle_safety_ext;

    // Add vehicle_safety_extensions.events
    vehicle_safety_ext.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_EVENTS;
    vehicle_safety_ext.events.vehicle_event_flag = j2735_v2x_msgs::msg::VehicleEventFlags::EVENT_HAZARD_LIGHTS;

    // Add vehicle_safety_extensions.path_history
    vehicle_safety_ext.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY;
    j2735_v2x_msgs::msg::PathHistoryPoint point1;
    point1.lat_offset.offset = 131067;
    point1.lon_offset.offset = 131068;
    point1.elevation_offset.offset = -2010;
    point1.time_offset.offset = 60034;
    point1.speed.speed = 5000;
    point1.pos_accuracy.semi_major = 202;
    point1.pos_accuracy.semi_minor = 99;
    point1.pos_accuracy.orientation = 18204;
    point1.heading.heading = 100;
    vehicle_safety_ext.path_history.crumb_data.points.push_back(point1);

    vehicle_safety_ext.path_history.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_CURR_GNSS_STATUS;
    vehicle_safety_ext.path_history.curr_gnss_status.statuses = j2735_v2x_msgs::msg::GNSSStatus::IS_HEALTHY;

    vehicle_safety_ext.path_history.presence_vector |= j2735_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;
    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
    vehicle_safety_ext.path_history.initial_position.utc_time.year.year = 1000;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
    vehicle_safety_ext.path_history.initial_position.utc_time.month.month = 10;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
    vehicle_safety_ext.path_history.initial_position.utc_time.day.day = 20;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
    vehicle_safety_ext.path_history.initial_position.utc_time.hour.hour = 21;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
    vehicle_safety_ext.path_history.initial_position.utc_time.minute.minute = 22;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
    vehicle_safety_ext.path_history.initial_position.utc_time.second.millisecond = 20000;
    vehicle_safety_ext.path_history.initial_position.utc_time.presence_vector |= j2735_v2x_msgs::msg::DDateTime::OFFSET;
    vehicle_safety_ext.path_history.initial_position.utc_time.offset.offset_minute = 800;

    vehicle_safety_ext.path_history.initial_position.lon.longitude = 1701000000;
    vehicle_safety_ext.path_history.initial_position.lat.latitude = 900000000;
    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_ELEVATION;
    vehicle_safety_ext.path_history.initial_position.elevation.elevation = 5100;

    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_HEADING;
    vehicle_safety_ext.path_history.initial_position.heading.heading = 80;

    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED;
    vehicle_safety_ext.path_history.initial_position.speed.transmission.transmission_state |= j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS;
    vehicle_safety_ext.path_history.initial_position.speed.speed.velocity = 2500;

    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_ACCURACY;
    vehicle_safety_ext.path_history.initial_position.pos_accuracy.semi_major = 200;
    vehicle_safety_ext.path_history.initial_position.pos_accuracy.semi_minor = 100;
    vehicle_safety_ext.path_history.initial_position.pos_accuracy.orientation = 18204;

    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_TIME_CONFIDENCE;
    vehicle_safety_ext.path_history.initial_position.time_confidence.confidence |= j2735_v2x_msgs::msg::TimeConfidence::TIME_000_001;

    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_POS_CONFIDENCE;
    vehicle_safety_ext.path_history.initial_position.pos_confidence.pos.confidence |= j2735_v2x_msgs::msg::PositionConfidence::A20M;
    vehicle_safety_ext.path_history.initial_position.pos_confidence.elevation.confidence |= j2735_v2x_msgs::msg::ElevationConfidence::ELEV_050_00;

    vehicle_safety_ext.path_history.initial_position.presence_vector |= j2735_v2x_msgs::msg::FullPositionVector::HAS_SPEED_CONFIDENCE;
    vehicle_safety_ext.path_history.initial_position.speed_confidence.heading.confidence |= j2735_v2x_msgs::msg::HeadingConfidence::PREC_01_DEG;
    vehicle_safety_ext.path_history.initial_position.speed_confidence.speed.speed_confidence |= j2735_v2x_msgs::msg::SpeedConfidence::PREC5MS;
    vehicle_safety_ext.path_history.initial_position.speed_confidence.throttle.confidence |= j2735_v2x_msgs::msg::ThrottleConfidence::PREC_1_PERCENT;

    // Add vehicle_safety_extensions.path_prediction
    vehicle_safety_ext.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION;
    vehicle_safety_ext.path_prediction.radius_of_curvature = 32700;
    vehicle_safety_ext.path_prediction.confidence = 100;

    // Add vehicle_safety_extensions.lights
    vehicle_safety_ext.presence_vector |= j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_LIGHTS;
    vehicle_safety_ext.lights.exterior_lights |= j2735_v2x_msgs::msg::ExteriorLights::RIGHT_TURN_SIGNAL_ON;

    extension3.vehicle_safety_extensions = vehicle_safety_ext;

    message.part_ii.push_back(extension3);

    // Add BSM Regional Extension with type 'ROUTE_DESTINATIONS'
    message.presence_vector |= j2735_v2x_msgs::msg::BSM::HAS_REGIONAL;

    j2735_v2x_msgs::msg::BSMRegionalExtension regional_ext;
    regional_ext.regional_extension_id = j2735_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS;

    j2735_v2x_msgs::msg::Position3D position1;
    position1.latitude = 405011000;
    position1.longitude = -1602000000;

    j2735_v2x_msgs::msg::Position3D position2;
    position2.latitude = 435000000;
    position2.longitude = -1422000000;

    regional_ext.route_destination_points.push_back(position1);
    regional_ext.route_destination_points.push_back(position2);
    message.regional.push_back(regional_ext);

    // Test that j2735_v2x_msgs::msg::BSM can be encoded
    auto res = worker.encode_bsm_message(message);
    if(res) EXPECT_TRUE(true);
    else 
    {
        std::cout << "Encoding failed while unit testing BSM encoder!\n";
        EXPECT_TRUE(false);
    }

    // Test that j2735_v2x_msgs::msg::BSM can be decoded
    auto res_decoded = worker.decode_bsm_message(res.get());
    if(res_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
    j2735_v2x_msgs::msg::BSM result = res_decoded.get();

    // if(result.regional.size() == message.regional.size()){
    //     std::cout<<"Size equal: " << result.regional.size() << "\n";
    // }
    // if(result.regional[0] == message.regional[0]){
    //     std::cout<<"Element 0 equal\n";
    // }

    // Test that BSM is unchanged after being encoded and decoded
    EXPECT_EQ(message, result);

    // Test that j2735_v2x_msgs::msg::BSM can be encoded again
    auto res2 = worker.encode_bsm_message(result);
    if(res2) EXPECT_TRUE(true);
    else 
    {
        std::cout << "Encoding failed while unit testing BSM encoder!\n";
        EXPECT_TRUE(false);
    }

    // Test that j2735_v2x_msgs::msg::BSM can be decoded again
    auto res2_decoded = worker.decode_bsm_message(res2.get());
    if(res2_decoded) EXPECT_TRUE(true);
    else
    {
        std::cout << "decoding of encoded file failed! \n";
        EXPECT_TRUE(false);
    }
}