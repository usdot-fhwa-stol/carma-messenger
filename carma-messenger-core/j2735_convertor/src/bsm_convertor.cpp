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
void BSMConvertor::convert(const std::vector<j2735_v2x_msgs::msg::Position3D>& in_msg, std::vector<carma_v2x_msgs::msg::Position3D>& out_msg)
{
    for(size_t i = 0; i < in_msg.size(); i++){
        j2735_v2x_msgs::msg::Position3D in_position_3d = in_msg[i];
        carma_v2x_msgs::msg::Position3D out_position_3d;
                
        // latitude
        if(in_position_3d.latitude == j2735_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE){
            out_position_3d.latitude = carma_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE;
        }
        else{
            out_position_3d.latitude = in_position_3d.latitude / units::TENTH_MICRO_DEG_PER_DEG;
        }

        // longitude
        if(in_position_3d.longitude == j2735_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE){
            out_position_3d.longitude = carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE;
        }
        else{
            out_position_3d.longitude = in_position_3d.longitude / units::TENTH_MICRO_DEG_PER_DEG;
        }                

        // elevation
        if(in_position_3d.elevation_exists){
            out_position_3d.elevation = in_position_3d.elevation / units::DECI_M_PER_M;
        }
        else{
            out_position_3d.elevation = carma_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE;
        }

        out_msg.push_back(out_position_3d);
    }
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::PathPrediction& in_msg, carma_v2x_msgs::msg::PathPrediction& out_msg)
{
    out_msg.radius_of_curvature = in_msg.radius_of_curvature / units::CENTI_DEG_PER_DEG;
    out_msg.confidence = (double) in_msg.confidence / 200;
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::Heading& in_msg, carma_v2x_msgs::msg::Heading& out_msg) {
    if(in_msg.heading != j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.heading = in_msg.heading / units::EIGHTIETH_DEG_PER_DEG;
    }
    else{
        out_msg.unavailable = true;
    }
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::Velocity& in_msg, carma_v2x_msgs::msg::Velocity& out_msg) {
    if(in_msg.velocity != j2735_v2x_msgs::msg::Velocity::UNAVAILABLE){
        out_msg.unavailable = false;
        out_msg.velocity = in_msg.velocity / units::FIFTIETH_M_PER_M;
    }
    else{
        out_msg.unavailable = true;
    }
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::PathHistory& in_msg, carma_v2x_msgs::msg::PathHistory& out_msg)
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
            j.pos_accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
            j.pos_accuracy.orientation = i.pos_accuracy.orientation / units::DEG_360_OVER_65535_PER_DEG;
        }
        
        path_history_point_list.points.push_back(j);
    }
    out_msg.crumb_data = path_history_point_list;
}

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

void BSMConvertor::convert(const j2735_v2x_msgs::msg::VehicleSafetyExtensions& in_msg, carma_v2x_msgs::msg::VehicleSafetyExtensions& out_msg)
{   
    // presence_vector
    out_msg.presence_vector = in_msg.presence_vector;

    // events
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_EVENTS){
        out_msg.events = in_msg.events;
    }

    // path_history
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY){
        convert(in_msg.path_history, out_msg.path_history);
    }

    // path_prediction
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION){
        convert(in_msg.path_prediction, out_msg.path_prediction);
    }

    // lights
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::VehicleSafetyExtensions::HAS_LIGHTS){
        out_msg.lights = in_msg.lights;
    }
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::SupplementalVehicleExtensions& in_msg, carma_v2x_msgs::msg::SupplementalVehicleExtensions& out_msg)
{   
    // presence_vector
    out_msg.presence_vector = in_msg.presence_vector;

    // classification
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION){
        out_msg.classification = in_msg.classification;
    }

    // class_details
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS){
        out_msg.class_details = in_msg.class_details;
    }

    // vehicle_data
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA){
        // presence_vector
        out_msg.vehicle_data.presence_vector = in_msg.vehicle_data.presence_vector;

        // vehicle_data.height
        if(in_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_HEIGHT){
            if(in_msg.vehicle_data.height.vehicle_height == j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE){
                out_msg.vehicle_data.height.unavailable = true;
            }
            else{
                out_msg.vehicle_data.height.vehicle_height = in_msg.vehicle_data.height.vehicle_height / units::TWENTIETH_M_PER_M;
            }        
        }

        // vehicle_data.bumpers
        if(in_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_BUMPERS){
            out_msg.vehicle_data.bumpers.front.bumper_height = in_msg.vehicle_data.bumpers.front.bumper_height / units::CM_PER_M;
            out_msg.vehicle_data.bumpers.rear.bumper_height = in_msg.vehicle_data.bumpers.rear.bumper_height / units::CM_PER_M;
        }

        // vehicle_data.mass
        if(in_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_MASS){
            if(in_msg.vehicle_data.mass.vehicle_mass == j2735_v2x_msgs::msg::VehicleMass::MASS_UNAVAILABLE){
                out_msg.vehicle_data.mass.unavailable = true;
            }
            else{
                // NOTE: Refer to J2735 ASN Specification for more details regarding this conversion.
                if(in_msg.vehicle_data.mass.vehicle_mass <= 80){
                    out_msg.vehicle_data.mass.vehicle_mass = in_msg.vehicle_data.mass.vehicle_mass * 50;
                }
                else if(in_msg.vehicle_data.mass.vehicle_mass <= 200){
                    out_msg.vehicle_data.mass.vehicle_mass = 4000 + (in_msg.vehicle_data.mass.vehicle_mass - 80) * 500;
                }
                else if(in_msg.vehicle_data.mass.vehicle_mass <= 253){
                    out_msg.vehicle_data.mass.vehicle_mass = 54000 + (in_msg.vehicle_data.mass.vehicle_mass - 200) * 2000;
                }
                else{
                    out_msg.vehicle_data.mass.vehicle_mass = carma_v2x_msgs::msg::VehicleMass::MASS_MAX;
                }
            }            
        }

        // vehicle_data.trailer_weight
        if(in_msg.vehicle_data.presence_vector & j2735_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT){
            out_msg.vehicle_data.trailer_weight.trailer_weight = in_msg.vehicle_data.trailer_weight.trailer_weight * units::TWO_KG;
        }
    }

    // weather_report
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT){
        // presence_vector
        out_msg.weather_report.presence_vector = in_msg.weather_report.presence_vector;

        // is_raining
        out_msg.weather_report.is_raining.precip_yes_no = in_msg.weather_report.is_raining.precip_yes_no;

        // weather_report.rain_rate
        if(in_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE){
            out_msg.weather_report.rain_rate.precip_rate = in_msg.weather_report.rain_rate.precip_rate / units::TENTH_GRAM_PER_GRAM;
        }    

        // weather_report.precip_situation
        if(in_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION){
            out_msg.weather_report.precip_situation.ess_precip_situation = in_msg.weather_report.precip_situation.ess_precip_situation;
        }    

        // weather_report.solar_radiation
        if(in_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION){
            out_msg.weather_report.solar_radiation.ess_solar_radiation = in_msg.weather_report.solar_radiation.ess_solar_radiation;
        }   

        // weather_report.friction
        if(in_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_FRICTION){
            out_msg.weather_report.friction.ess_mobile_friction = in_msg.weather_report.friction.ess_mobile_friction;
        }    

        // weather_report.road_friction
        if(in_msg.weather_report.presence_vector & j2735_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION){
            if(in_msg.weather_report.road_friction.coefficient == j2735_v2x_msgs::msg::CoefficientOfFriction::COEFFICIENT_UNAVAILABLE){
                out_msg.weather_report.road_friction.unavailable = true;
            }
            else{
                out_msg.weather_report.road_friction.coefficient = in_msg.weather_report.road_friction.coefficient * units::TWO_TENTHS_MICRO;
            }
        }    
    }

    // weather_probe
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE){
        // presence_vector
        out_msg.weather_probe.presence_vector = in_msg.weather_probe.presence_vector;

        // weather_probe.air_temp
        if(in_msg.weather_probe.presence_vector & j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP){
            if(in_msg.weather_probe.air_temp.temperature == j2735_v2x_msgs::msg::AmbientAirTemperature::TEMPERATURE_UNKNOWN){
                out_msg.weather_probe.air_temp.unavailable = true;
            }
            else{
                out_msg.weather_probe.air_temp.temperature = in_msg.weather_probe.air_temp.temperature - units::FORTY_DEGREES_C;
            }
        }       

        // weather_probe.air_pressure
        if(in_msg.weather_probe.presence_vector & j2735_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE){
            if(in_msg.weather_probe.air_pressure.pressure == j2735_v2x_msgs::msg::AmbientAirPressure::PRESSURE_UNAVAILABLE){
                out_msg.weather_probe.air_pressure.unavailable = true;
            }
            else{
                out_msg.weather_probe.air_pressure.pressure = in_msg.weather_probe.air_pressure.pressure / units::HPA_PER_TWO_PA;
            }
        }    

        // weather_probe.rain_rates
        if(in_msg.weather_probe.presence_vector & j2735_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES){
            // presence_vector
            out_msg.weather_probe.rain_rates.presence_vector = in_msg.weather_probe.rain_rates.presence_vector;

            // weather_probe.rain_rates.status_front
            out_msg.weather_probe.rain_rates.status_front.wiper_status = in_msg.weather_probe.rain_rates.status_front.wiper_status;

            // weather_probe.rain_rates.rate_front
            out_msg.weather_probe.rain_rates.rate_front.wiper_rate = in_msg.weather_probe.rain_rates.rate_front.wiper_rate / units::SEC_PER_MIN;

            // weather_probe.rain_rates.status_rear
            if(in_msg.weather_probe.rain_rates.presence_vector & j2735_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR){
                out_msg.weather_probe.rain_rates.status_rear.wiper_status = in_msg.weather_probe.rain_rates.status_rear.wiper_status;
            }

            // weather_probe.rain_rates.rate_rear
            if(in_msg.weather_probe.rain_rates.presence_vector & j2735_v2x_msgs::msg::WiperSet::HAS_RATE_REAR){
                out_msg.weather_probe.rain_rates.rate_rear.wiper_rate = in_msg.weather_probe.rain_rates.rate_rear.wiper_rate / units::SEC_PER_MIN;
            }
        }    
    }

    // obstacle
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE){
        // presence_vector
        out_msg.obstacle.presence_vector = in_msg.obstacle.presence_vector;

        // obstacle.ob_dist
        out_msg.obstacle.ob_dist.distance = in_msg.obstacle.ob_dist.distance / units::UNCHANGED;

        // obstacle.ob_direct
        if(in_msg.obstacle.ob_direct.direction.angle == j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
            out_msg.obstacle.ob_direct.direction.unavailable = true;
        }
        else{
            out_msg.obstacle.ob_direct.direction.angle = in_msg.obstacle.ob_direct.direction.angle / units::EIGHTIETH_DEG_PER_DEG;
        }

        // obstacle.date_time
        out_msg.obstacle.date_time = in_msg.obstacle.date_time;

        // obstacle.description
        if(in_msg.obstacle.presence_vector & j2735_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION){
            out_msg.obstacle.description = in_msg.obstacle.description;
        }

        // obstacle.location_details
        if(in_msg.obstacle.presence_vector & j2735_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS){
            out_msg.obstacle.location_details = in_msg.obstacle.location_details;
        }

        // obstacle.vert_event
        if(in_msg.obstacle.presence_vector & j2735_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT){
            out_msg.obstacle.vert_event = in_msg.obstacle.vert_event;
        }
    }

    // status
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS){
        // presence_vector
        out_msg.status.presence_vector = in_msg.status.presence_vector;

        // status_details
        out_msg.status.status_details = in_msg.status.status_details;

        // location_details
        out_msg.status.location_details = in_msg.status.location_details;
    }

    // speed_profile
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE){
        for(size_t i = 0; i < in_msg.speed_profile.size(); i++){
            carma_v2x_msgs::msg::GrossSpeed out_speed;

            if(in_msg.speed_profile[i].speed == j2735_v2x_msgs::msg::GrossSpeed::SPEED_UNAVAILABLE){
                out_speed.unavailable = true;
                out_msg.speed_profile.push_back(out_speed);
            }
            else{
                out_speed.speed = in_msg.speed_profile[i].speed;
                out_msg.speed_profile.push_back(out_speed);
            }
        }
    }
}

void BSMConvertor::convert(const j2735_v2x_msgs::msg::SpecialVehicleExtensions& in_msg, carma_v2x_msgs::msg::SpecialVehicleExtensions& out_msg)
{   
    // presence_vector
    out_msg.presence_vector = in_msg.presence_vector;

    // vehicle_alerts
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS){
        // Both 'vehicle_alerts' fields are j2735_v2x_msgs::msg::EmergencyDetails 
        out_msg.vehicle_alerts = in_msg.vehicle_alerts;
    }

    // description
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION){
        // Both 'description' fields are j2735_v2x_msgs::msg::EventDescription 
        out_msg.description = in_msg.description;
    }

    // trailers
    if(in_msg.presence_vector & j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS){
        // ssp_index
        out_msg.trailers.ssp_index = in_msg.trailers.ssp_index;

        // connection.pivot_offset.offset
        if(in_msg.trailers.connection.pivot_offset.offset == j2735_v2x_msgs::msg::OffsetB11::OFFSET_UNKNOWN){
            out_msg.trailers.connection.pivot_offset.unavailable = true;
        }
        else{
            out_msg.trailers.connection.pivot_offset.offset = in_msg.trailers.connection.pivot_offset.offset / units::CM_PER_M;
        }

        // connection.pivot_angle.angle
        if(in_msg.trailers.connection.pivot_angle.angle == j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
            out_msg.trailers.connection.pivot_angle.unavailable = true;
        }
        else{
            out_msg.trailers.connection.pivot_angle.angle = in_msg.trailers.connection.pivot_angle.angle / units::EIGHTIETH_DEG_PER_DEG;
        }

        // connection.pivots.pivoting_allowed
        out_msg.trailers.connection.pivots.pivoting_allowed = in_msg.trailers.connection.pivots.pivoting_allowed;

        // units
        for(size_t i = 0; i < in_msg.trailers.units.trailer_unit_descriptions.size(); i++){
            j2735_v2x_msgs::msg::TrailerUnitDescription in_trailer_unit_description = in_msg.trailers.units.trailer_unit_descriptions[i];
            carma_v2x_msgs::msg::TrailerUnitDescription out_trailer_unit_description;

            // presence_vector
            out_trailer_unit_description.presence_vector = in_trailer_unit_description.presence_vector;
            
            // is_dolly
            out_trailer_unit_description.is_dolly.is_dolly = in_trailer_unit_description.is_dolly.is_dolly;

            // width
            if(in_trailer_unit_description.width.vehicle_width == j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_UNAVAILABLE){
                out_trailer_unit_description.width.unavailable = true;
            }
            else{
                out_trailer_unit_description.width.vehicle_width = in_trailer_unit_description.width.vehicle_width / units::CM_PER_M;
            }

            // length
            if(in_trailer_unit_description.length.vehicle_length == j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_UNAVAILABLE){
                out_trailer_unit_description.length.unavailable = true;
            }
            else{
                out_trailer_unit_description.length.vehicle_length = in_trailer_unit_description.length.vehicle_length / units::CM_PER_M;
            }

            // height
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT){
                if(in_trailer_unit_description.height.vehicle_height == j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE){
                    out_trailer_unit_description.height.unavailable = true;
                }
                else{
                    out_trailer_unit_description.height.vehicle_height = in_trailer_unit_description.height.vehicle_height / units::TWENTIETH_M_PER_M;
                }
            }

            // mass
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS){
                if(in_trailer_unit_description.mass.trailer_mass == j2735_v2x_msgs::msg::TrailerMass::TRAILER_MASS_UNKNOWN){
                    out_trailer_unit_description.mass.unavailable = true;
                }
                else{
                    out_trailer_unit_description.mass.trailer_mass = in_trailer_unit_description.mass.trailer_mass * units::KG_PER_HALF_METRIC_TON;
                }
            }

            // bumper_heights
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS){
                out_trailer_unit_description.bumper_heights.front.bumper_height = in_msg.trailers.units.trailer_unit_descriptions[i].bumper_heights.front.bumper_height / units::CM_PER_M;
                out_trailer_unit_description.bumper_heights.rear.bumper_height = in_msg.trailers.units.trailer_unit_descriptions[i].bumper_heights.rear.bumper_height / units::CM_PER_M;
            }

            // center_of_gravity
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY){
                if(in_trailer_unit_description.center_of_gravity.vehicle_height == j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE){
                    out_trailer_unit_description.center_of_gravity.unavailable = true;
                }
                else{
                    out_trailer_unit_description.center_of_gravity.vehicle_height = in_trailer_unit_description.center_of_gravity.vehicle_height / units::TWENTIETH_M_PER_M;
                }
            }

            // front_pivot.pivot_offset (OffsetB11)
            if(in_trailer_unit_description.front_pivot.pivot_offset.offset == j2735_v2x_msgs::msg::OffsetB11::OFFSET_UNKNOWN){
                out_trailer_unit_description.front_pivot.pivot_offset.unavailable = true;
            }
            else{
                out_trailer_unit_description.front_pivot.pivot_offset.offset = in_trailer_unit_description.front_pivot.pivot_offset.offset / units::CM_PER_M;
            }

            // front_pivot.pivot_angle 
            if(in_trailer_unit_description.front_pivot.pivot_angle.angle == j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                out_trailer_unit_description.front_pivot.pivot_offset.unavailable = true;
            }
            else{
                out_trailer_unit_description.front_pivot.pivot_angle.angle = in_trailer_unit_description.front_pivot.pivot_angle.angle / units::EIGHTIETH_DEG_PER_DEG;
            }

            // front_pivot.pivots
            out_trailer_unit_description.front_pivot.pivots.pivoting_allowed = in_trailer_unit_description.front_pivot.pivots.pivoting_allowed;


            // rear_pivot
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT){
                // rear_pivot.pivot_offset (OffsetB11)
                if(in_trailer_unit_description.rear_pivot.pivot_offset.offset == j2735_v2x_msgs::msg::OffsetB11::OFFSET_UNKNOWN){
                    out_trailer_unit_description.rear_pivot.pivot_offset.unavailable = true;
                }
                else{
                    out_trailer_unit_description.rear_pivot.pivot_offset.offset = in_trailer_unit_description.rear_pivot.pivot_offset.offset / units::CM_PER_M;
                }

                // rear_pivot.pivot_angle 
                if(in_trailer_unit_description.rear_pivot.pivot_angle.angle == j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                    out_trailer_unit_description.rear_pivot.pivot_offset.unavailable = true;
                }
                else{
                    out_trailer_unit_description.rear_pivot.pivot_angle.angle = in_trailer_unit_description.rear_pivot.pivot_angle.angle / units::EIGHTIETH_DEG_PER_DEG;
                }

                // rear_pivot.pivots
                out_trailer_unit_description.rear_pivot.pivots.pivoting_allowed = in_trailer_unit_description.rear_pivot.pivots.pivoting_allowed;
            }

            // rear_wheel_offset
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET){
                if(in_trailer_unit_description.rear_wheel_offset.offset == j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN){
                    out_trailer_unit_description.rear_wheel_offset.unavailable = true;
                }
                else{
                    out_trailer_unit_description.rear_wheel_offset.offset = in_trailer_unit_description.rear_wheel_offset.offset / units::CM_PER_M;
                }
            }

            // position_offset.x
            if(in_trailer_unit_description.position_offset.x == j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN){
                out_trailer_unit_description.position_offset.x.unavailable = true;
            }
            else{
                out_trailer_unit_description.position_offset.x.offset = in_trailer_unit_description.position_offset.x / units::CM_PER_M;
            }

            // position_offset.y
            if(in_trailer_unit_description.position_offset.y == j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN){
                out_trailer_unit_description.position_offset.y.unavailable = true;
            }
            else{
                out_trailer_unit_description.position_offset.y.offset = in_trailer_unit_description.position_offset.y / units::CM_PER_M;
            }

            // elevation_offset
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET){
                if(in_trailer_unit_description.elevation_offset.offset == j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_UNAVAILABLE){
                    out_trailer_unit_description.elevation_offset.unavailable = true;
                }
                else{
                    out_trailer_unit_description.elevation_offset.offset = in_trailer_unit_description.elevation_offset.offset / units::DECI_M_PER_M;
                }
            }

            // crumb_data
            if(in_trailer_unit_description.presence_vector & j2735_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA){
                for(size_t j = 0; j < in_trailer_unit_description.crumb_data.trailer_history_points.size(); j++){
                    j2735_v2x_msgs::msg::TrailerHistoryPoint in_trailer_history_point = in_trailer_unit_description.crumb_data.trailer_history_points[j];
                    carma_v2x_msgs::msg::TrailerHistoryPoint out_trailer_history_point;

                    // crumb_data.presence_vector
                    out_trailer_history_point.presence_vector = in_trailer_history_point.presence_vector;

                    // crumb_data.angle
                    if(in_trailer_history_point.pivot_angle.angle == j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE){
                        out_trailer_history_point.pivot_angle.unavailable = true;
                    }
                    else{
                        out_trailer_history_point.pivot_angle.angle = in_trailer_history_point.pivot_angle.angle / units::EIGHTIETH_DEG_PER_DEG;
                    }

                    // crumb_data.time_offset
                    if(in_trailer_history_point.time_offset.offset == j2735_v2x_msgs::msg::TimeOffset::UNAVAILABLE){
                        out_trailer_history_point.time_offset.unavailable = true;
                    }
                    else{
                        out_trailer_history_point.time_offset.offset = in_trailer_history_point.time_offset.offset / units::CENTI_S_PER_S;
                    }

                    // crumb_data.position_offset.x
                    if(in_trailer_history_point.position_offset.x == j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN){
                        out_trailer_history_point.position_offset.x.unavailable = true;
                    }
                    else{
                        out_trailer_history_point.position_offset.x.offset = in_trailer_history_point.position_offset.x / units::CM_PER_M;
                    }

                    // crumb_data.position_offset.y
                    if(in_trailer_history_point.position_offset.y == j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN){
                        out_trailer_history_point.position_offset.y.unavailable = true;
                    }
                    else{
                        out_trailer_history_point.position_offset.y.offset = in_trailer_history_point.position_offset.y / units::CM_PER_M;
                    }

                    // crumb_data.elevation_offset
                    if(in_trailer_history_point.presence_vector & carma_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET){
                        if(in_trailer_history_point.elevation_offset.offset == j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_UNAVAILABLE){
                            out_trailer_history_point.elevation_offset.unavailable = true;
                        }
                        else{
                            out_trailer_history_point.elevation_offset.offset = in_trailer_history_point.elevation_offset.offset / units::DECI_M_PER_M;
                        }
                    }

                    // crumb_data.heading
                    if(in_trailer_history_point.presence_vector & j2735_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING){
                        if(in_trailer_history_point.heading.heading == j2735_v2x_msgs::msg::CoarseHeading::UNAVAILABLE){
                            out_trailer_history_point.heading.unavailable = true;
                        }
                        else{
                            out_trailer_history_point.heading.heading = in_trailer_history_point.heading.heading * units::ONE_AND_A_HALF_DEG;
                        }
                    }

                    out_trailer_unit_description.crumb_data.trailer_history_points.push_back(out_trailer_history_point);
                }
            }
            
            out_msg.trailers.units.trailer_unit_descriptions.push_back(out_trailer_unit_description);
        }

    }
}

void BSMConvertor::convert(const std::vector<j2735_v2x_msgs::msg::BSMRegionalExtension>& in_msg, std::vector<carma_v2x_msgs::msg::BSMRegionalExtension>& out_msg)
{
    for(size_t i = 0; i < in_msg.size(); i++){
        carma_v2x_msgs::msg::BSMRegionalExtension out_regional_extension;
        out_regional_extension.regional_extension_id = in_msg[i].regional_extension_id;

        if(in_msg[i].regional_extension_id == j2735_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS){
            convert(in_msg[i].route_destination_points, out_regional_extension.route_destination_points);
        }

        out_msg.push_back(out_regional_extension);
    }
}

void BSMConvertor::convert(const std::vector<j2735_v2x_msgs::msg::BSMPartIIExtension>& in_msg, std::vector<carma_v2x_msgs::msg::BSMPartIIExtension>& out_msg)
{
    for(size_t i = 0; i < in_msg.size(); i++){
        carma_v2x_msgs::msg::BSMPartIIExtension out_part_ii_extension;
        out_part_ii_extension.part_ii_id = in_msg[i].part_ii_id;

        if(in_msg[i].part_ii_id == j2735_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT ){
            carma_v2x_msgs::msg::VehicleSafetyExtensions out_vehicle_safety_extensions;
            convert(in_msg[i].vehicle_safety_extensions, out_part_ii_extension.vehicle_safety_extensions);
        }
        if(in_msg[i].part_ii_id == j2735_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT){
            carma_v2x_msgs::msg::SpecialVehicleExtensions out_special_vehicle_extensions;
            convert(in_msg[i].special_vehicle_extensions, out_part_ii_extension.special_vehicle_extensions);
        }
        else if(in_msg[i].part_ii_id == j2735_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT){
            carma_v2x_msgs::msg::SupplementalVehicleExtensions out_supplemental_vehicle_extensions;
            convert(in_msg[i].supplemental_vehicle_extensions, out_part_ii_extension.supplemental_vehicle_extensions);
        }

        out_msg.push_back(out_part_ii_extension);
    }
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
  out_msg.presence_vector = in_msg.presence_vector;

  convert(in_msg.core_data, out_msg.core_data);

  if(in_msg.presence_vector & j2735_v2x_msgs::msg::BSM::HAS_PART_II){
    convert(in_msg.part_ii, out_msg.part_ii);
  }


  if(in_msg.presence_vector & j2735_v2x_msgs::msg::BSM::HAS_REGIONAL){
    convert(in_msg.regional, out_msg.regional);
  }
}

////
// Convert carma_v2x_msgs to j2735_msgs
////

void BSMConvertor::convert(const std::vector<carma_v2x_msgs::msg::Position3D>& in_msg, std::vector<j2735_v2x_msgs::msg::Position3D>& out_msg)
{
    for(size_t i = 0; i < in_msg.size(); i++){
        carma_v2x_msgs::msg::Position3D in_position_3d = in_msg[i];
        j2735_v2x_msgs::msg::Position3D out_position_3d;
                
        // latitude
        if(in_position_3d.latitude == carma_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE){
            out_position_3d.latitude = j2735_v2x_msgs::msg::Position3D::LATITUDE_UNAVAILABLE;
        }
        else{
            out_position_3d.latitude = in_position_3d.latitude * units::TENTH_MICRO_DEG_PER_DEG;
        }

        // longitude
        if(in_position_3d.longitude == carma_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE){
            out_position_3d.longitude = j2735_v2x_msgs::msg::Position3D::LONGITUDE_UNAVAILABLE;
        }
        else{
            out_position_3d.longitude = in_position_3d.longitude * units::TENTH_MICRO_DEG_PER_DEG;
        }                

        // elevation
        if(in_position_3d.elevation_exists){
            out_position_3d.elevation = in_position_3d.elevation * units::DECI_M_PER_M;
        }
        else{
            out_position_3d.elevation = j2735_v2x_msgs::msg::Position3D::ELEVATION_UNAVAILABLE;
        }

        out_msg.push_back(out_position_3d);
    }
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::PathPrediction& in_msg, j2735_v2x_msgs::msg::PathPrediction& out_msg)
{
  out_msg.radius_of_curvature = in_msg.radius_of_curvature * units::CENTI_DEG_PER_DEG;
  out_msg.confidence = in_msg.confidence * 200;
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::Heading& in_msg, j2735_v2x_msgs::msg::Heading& out_msg) {
      if(!in_msg.unavailable){
        out_msg.heading = in_msg.heading * units::EIGHTIETH_DEG_PER_DEG;
      }
      else{
        out_msg.heading = j2735_v2x_msgs::msg::Heading::HEADING_UNAVAILABLE;
      }
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::Velocity& in_msg, j2735_v2x_msgs::msg::Velocity& out_msg) {
      if(!in_msg.unavailable){
        out_msg.velocity = in_msg.velocity * units::FIFTIETH_M_PER_M;
      }
      else{
        out_msg.velocity = j2735_v2x_msgs::msg::Velocity::UNAVAILABLE;
      }
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::PathHistory& in_msg, j2735_v2x_msgs::msg::PathHistory& out_msg)
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

void BSMConvertor::convert(const carma_v2x_msgs::msg::VehicleSafetyExtensions& in_msg, j2735_v2x_msgs::msg::VehicleSafetyExtensions& out_msg)
{   
    // presence_vector
    out_msg.presence_vector = in_msg.presence_vector;

    // events
    if(in_msg.presence_vector & carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_EVENTS){
        out_msg.events = in_msg.events;
    }

    // path_history
    if(in_msg.presence_vector & carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_HISTORY){
        convert(in_msg.path_history, out_msg.path_history);
    }

    // path_prediction
    if(in_msg.presence_vector & carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_PATH_PREDICTION){
        convert(in_msg.path_prediction, out_msg.path_prediction);
    }

    // lights
    if(in_msg.presence_vector & carma_v2x_msgs::msg::VehicleSafetyExtensions::HAS_LIGHTS){
        out_msg.lights = in_msg.lights;
    }
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::SupplementalVehicleExtensions& in_msg, j2735_v2x_msgs::msg::SupplementalVehicleExtensions& out_msg)
{   
    // presence_vector
    out_msg.presence_vector = in_msg.presence_vector;

    // classification
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASSIFICATION){
        out_msg.classification = in_msg.classification;
    }

    // class_details
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_CLASS_DETAILS){
        out_msg.class_details = in_msg.class_details;
    }

    // vehicle_data
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_VEHICLE_DATA){
        // presence_vector
        out_msg.vehicle_data.presence_vector = in_msg.vehicle_data.presence_vector;

        // vehicle_data.height
        if(in_msg.vehicle_data.presence_vector & carma_v2x_msgs::msg::VehicleData::HAS_HEIGHT){
            if(in_msg.vehicle_data.height.unavailable){
                out_msg.vehicle_data.height.vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE;
            }
            else{
                out_msg.vehicle_data.height.vehicle_height = in_msg.vehicle_data.height.vehicle_height * units::TWENTIETH_M_PER_M;
            }        
        }

        // vehicle_data.bumpers
        if(in_msg.vehicle_data.presence_vector & carma_v2x_msgs::msg::VehicleData::HAS_BUMPERS){
            out_msg.vehicle_data.bumpers.front.bumper_height = in_msg.vehicle_data.bumpers.front.bumper_height * units::CM_PER_M;
            out_msg.vehicle_data.bumpers.rear.bumper_height = in_msg.vehicle_data.bumpers.rear.bumper_height * units::CM_PER_M;
        }

        // vehicle_data.mass
        if(in_msg.vehicle_data.presence_vector & carma_v2x_msgs::msg::VehicleData::HAS_MASS){
            if(in_msg.vehicle_data.mass.unavailable){
                out_msg.vehicle_data.mass.vehicle_mass = j2735_v2x_msgs::msg::VehicleMass::MASS_UNAVAILABLE;
            }
            else{
                // NOTE: Refer to J2735 ASN Specification for more details regarding this conversion.
                if(in_msg.vehicle_data.mass.vehicle_mass <= 4000){
                    out_msg.vehicle_data.mass.vehicle_mass = in_msg.vehicle_data.mass.vehicle_mass / 50;
                }
                else if(in_msg.vehicle_data.mass.vehicle_mass <= 54500){
                    out_msg.vehicle_data.mass.vehicle_mass = 80 + ((in_msg.vehicle_data.mass.vehicle_mass - 4000) / 500);
                }
                else if(in_msg.vehicle_data.mass.vehicle_mass <= 170000){
                    out_msg.vehicle_data.mass.vehicle_mass = 180 + ((in_msg.vehicle_data.mass.vehicle_mass - 54500) / 2000);
                }
                else{
                    out_msg.vehicle_data.mass.vehicle_mass = 254;
                }
            }            
        }

        // vehicle_data.trailer_weight
        if(in_msg.vehicle_data.presence_vector & carma_v2x_msgs::msg::VehicleData::HAS_TRAILER_WEIGHT){
            out_msg.vehicle_data.trailer_weight.trailer_weight = in_msg.vehicle_data.trailer_weight.trailer_weight / units::TWO_KG;
        }
    }

    // weather_report
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_REPORT){
        // presence_vector
        out_msg.weather_report.presence_vector = in_msg.weather_report.presence_vector;

        // is_raining
        out_msg.weather_report.is_raining.precip_yes_no = in_msg.weather_report.is_raining.precip_yes_no;

        // weather_report.rain_rate
        if(in_msg.weather_report.presence_vector & carma_v2x_msgs::msg::WeatherReport::HAS_RAIN_RATE){
            out_msg.weather_report.rain_rate.precip_rate = in_msg.weather_report.rain_rate.precip_rate * units::TENTH_GRAM_PER_GRAM;
        }    

        // weather_report.precip_situation
        if(in_msg.weather_report.presence_vector & carma_v2x_msgs::msg::WeatherReport::HAS_PRECIP_SITUATION){
            out_msg.weather_report.precip_situation.ess_precip_situation = in_msg.weather_report.precip_situation.ess_precip_situation;
        }    

        // weather_report.solar_radiation
        if(in_msg.weather_report.presence_vector & carma_v2x_msgs::msg::WeatherReport::HAS_SOLAR_RADIATION){
            out_msg.weather_report.solar_radiation.ess_solar_radiation = in_msg.weather_report.solar_radiation.ess_solar_radiation;
        }   

        // weather_report.friction
        if(in_msg.weather_report.presence_vector & carma_v2x_msgs::msg::WeatherReport::HAS_FRICTION){
            out_msg.weather_report.friction.ess_mobile_friction = in_msg.weather_report.friction.ess_mobile_friction;
        }    

        // weather_report.road_friction
        if(in_msg.weather_report.presence_vector & carma_v2x_msgs::msg::WeatherReport::HAS_ROAD_FRICTION){
            if(in_msg.weather_report.road_friction.unavailable){
                out_msg.weather_report.road_friction.coefficient = j2735_v2x_msgs::msg::CoefficientOfFriction::COEFFICIENT_UNAVAILABLE;
            }
            else{
                out_msg.weather_report.road_friction.coefficient = in_msg.weather_report.road_friction.coefficient / units::TWO_TENTHS_MICRO;
            }
        }    
    }

    // weather_probe
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_WEATHER_PROBE){
        // presence_vector
        out_msg.weather_probe.presence_vector = in_msg.weather_probe.presence_vector;

        // weather_probe.air_temp
        if(in_msg.weather_probe.presence_vector & carma_v2x_msgs::msg::WeatherProbe::HAS_AIR_TEMP){
            if(in_msg.weather_probe.air_temp.unavailable){
                out_msg.weather_probe.air_temp.temperature = j2735_v2x_msgs::msg::AmbientAirTemperature::TEMPERATURE_UNKNOWN;
            }
            else{
                out_msg.weather_probe.air_temp.temperature = in_msg.weather_probe.air_temp.temperature + units::FORTY_DEGREES_C;
            }
        }       

        // weather_probe.air_pressure
        if(in_msg.weather_probe.presence_vector & carma_v2x_msgs::msg::WeatherProbe::HAS_AIR_PRESSURE){
            if(in_msg.weather_probe.air_pressure.unavailable){
                out_msg.weather_probe.air_pressure.pressure = j2735_v2x_msgs::msg::AmbientAirPressure::PRESSURE_UNAVAILABLE;
            }
            else{
                out_msg.weather_probe.air_pressure.pressure = in_msg.weather_probe.air_pressure.pressure * units::HPA_PER_TWO_PA;
            }
        }    

        // weather_probe.rain_rates
        if(in_msg.weather_probe.presence_vector & carma_v2x_msgs::msg::WeatherProbe::HAS_RAIN_RATES){
            // presence_vector
            out_msg.weather_probe.rain_rates.presence_vector = in_msg.weather_probe.rain_rates.presence_vector;

            // weather_probe.rain_rates.status_front
            out_msg.weather_probe.rain_rates.status_front.wiper_status = in_msg.weather_probe.rain_rates.status_front.wiper_status;

            // weather_probe.rain_rates.rate_front
            out_msg.weather_probe.rain_rates.rate_front.wiper_rate = in_msg.weather_probe.rain_rates.rate_front.wiper_rate * units::SEC_PER_MIN;

            // weather_probe.rain_rates.status_rear
            if(in_msg.weather_probe.rain_rates.presence_vector & carma_v2x_msgs::msg::WiperSet::HAS_STATUS_REAR){
                out_msg.weather_probe.rain_rates.status_rear.wiper_status = in_msg.weather_probe.rain_rates.status_rear.wiper_status;
            }

            // weather_probe.rain_rates.rate_rear
            if(in_msg.weather_probe.rain_rates.presence_vector & carma_v2x_msgs::msg::WiperSet::HAS_RATE_REAR){
                out_msg.weather_probe.rain_rates.rate_rear.wiper_rate = in_msg.weather_probe.rain_rates.rate_rear.wiper_rate * units::SEC_PER_MIN;
            }
        }    
    }

    // obstacle
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_OBSTACLE){
        // presence_vector
        out_msg.obstacle.presence_vector = in_msg.obstacle.presence_vector;

        // obstacle.ob_dist
        out_msg.obstacle.ob_dist.distance = in_msg.obstacle.ob_dist.distance * units::UNCHANGED;

        // obstacle.ob_direct
        if(in_msg.obstacle.ob_direct.direction.unavailable){
            out_msg.obstacle.ob_direct.direction.angle = j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE;
        }
        else{
            out_msg.obstacle.ob_direct.direction.angle = in_msg.obstacle.ob_direct.direction.angle * units::EIGHTIETH_DEG_PER_DEG;
        }

        // obstacle.date_time
        out_msg.obstacle.date_time = in_msg.obstacle.date_time;

        // obstacle.description
        if(in_msg.obstacle.presence_vector & carma_v2x_msgs::msg::ObstacleDetection::HAS_DESCRIPTION){
            out_msg.obstacle.description = in_msg.obstacle.description;
        }

        // obstacle.location_details
        if(in_msg.obstacle.presence_vector & carma_v2x_msgs::msg::ObstacleDetection::HAS_LOCATION_DETAILS){
            out_msg.obstacle.location_details = in_msg.obstacle.location_details;
        }

        // obstacle.vert_event
        if(in_msg.obstacle.presence_vector & carma_v2x_msgs::msg::ObstacleDetection::HAS_VERT_EVENT){
            out_msg.obstacle.vert_event = in_msg.obstacle.vert_event;
        }
    }

    // status
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_STATUS){
        // presence_vector
        out_msg.status.presence_vector = in_msg.status.presence_vector;

        // status_details
        out_msg.status.status_details = in_msg.status.status_details;

        // location_details
        out_msg.status.location_details = in_msg.status.location_details;
    }

    // speed_profile
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SupplementalVehicleExtensions::HAS_SPEED_PROFILE){
        for(size_t i = 0; i < in_msg.speed_profile.size(); i++){
            j2735_v2x_msgs::msg::GrossSpeed out_speed;

            if(in_msg.speed_profile[i].unavailable){
                out_speed.speed = j2735_v2x_msgs::msg::GrossSpeed::SPEED_UNAVAILABLE;
                out_msg.speed_profile.push_back(out_speed);
            }
            else{
                out_speed.speed = in_msg.speed_profile[i].speed;
                out_msg.speed_profile.push_back(out_speed);
            }
        }
    }
}

void BSMConvertor::convert(const carma_v2x_msgs::msg::SpecialVehicleExtensions& in_msg, j2735_v2x_msgs::msg::SpecialVehicleExtensions& out_msg)
{   
    // presence_vector
    out_msg.presence_vector = in_msg.presence_vector;

    // vehicle_alerts
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS){
        // Both 'vehicle_alerts' fields are j2735_v2x_msgs::msg::EmergencyDetails 
        out_msg.vehicle_alerts = in_msg.vehicle_alerts;
    }

    // description
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_DESCRIPTION){
        // Both 'description' fields are j2735_v2x_msgs::msg::EventDescription 
        out_msg.description = in_msg.description;
    }

    // trailers
    if(in_msg.presence_vector & carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_TRAILERS){
        // ssp_index
        out_msg.trailers.ssp_index = in_msg.trailers.ssp_index;

        // connection.pivot_offset.offset
        if(in_msg.trailers.connection.pivot_offset.unavailable){
            out_msg.trailers.connection.pivot_offset.offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_UNKNOWN;
        }
        else{
            out_msg.trailers.connection.pivot_offset.offset = in_msg.trailers.connection.pivot_offset.offset * units::CM_PER_M;
        }

        // connection.pivot_angle.angle
        if(in_msg.trailers.connection.pivot_angle.unavailable){
            out_msg.trailers.connection.pivot_angle.angle = j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE;
        }
        else{
            out_msg.trailers.connection.pivot_angle.angle = in_msg.trailers.connection.pivot_angle.angle * units::EIGHTIETH_DEG_PER_DEG;
        }

        // connection.pivots.pivoting_allowed
        out_msg.trailers.connection.pivots.pivoting_allowed = in_msg.trailers.connection.pivots.pivoting_allowed;

        // units
        for(size_t i = 0; i < in_msg.trailers.units.trailer_unit_descriptions.size(); i++){
            carma_v2x_msgs::msg::TrailerUnitDescription in_trailer_unit_description = in_msg.trailers.units.trailer_unit_descriptions[i];
            j2735_v2x_msgs::msg::TrailerUnitDescription out_trailer_unit_description;

            // presence_vector
            out_trailer_unit_description.presence_vector = in_trailer_unit_description.presence_vector;
            
            // is_dolly
            out_trailer_unit_description.is_dolly.is_dolly = in_trailer_unit_description.is_dolly.is_dolly;

            // width
            if(in_trailer_unit_description.width.unavailable){
                out_trailer_unit_description.width.vehicle_width = j2735_v2x_msgs::msg::VehicleWidth::VEHICLE_WIDTH_UNAVAILABLE;
            }
            else{
                out_trailer_unit_description.width.vehicle_width = in_trailer_unit_description.width.vehicle_width * units::CM_PER_M;
            }

            // length
            if(in_trailer_unit_description.length.unavailable){
                out_trailer_unit_description.length.vehicle_length = j2735_v2x_msgs::msg::VehicleLength::VEHICLE_LENGTH_UNAVAILABLE;
            }
            else{
                out_trailer_unit_description.length.vehicle_length = in_trailer_unit_description.length.vehicle_length * units::CM_PER_M;
            }

            // height
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_HEIGHT){
                if(in_trailer_unit_description.height.unavailable){
                    out_trailer_unit_description.height.vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE;
                }
                else{
                    out_trailer_unit_description.height.vehicle_height = in_trailer_unit_description.height.vehicle_height * units::TWENTIETH_M_PER_M;
                }
            }

            // mass
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_MASS){
                if(in_trailer_unit_description.mass.unavailable){
                    out_trailer_unit_description.mass.trailer_mass = j2735_v2x_msgs::msg::TrailerMass::TRAILER_MASS_UNKNOWN;
                }
                else{
                    out_trailer_unit_description.mass.trailer_mass = in_trailer_unit_description.mass.trailer_mass / units::KG_PER_HALF_METRIC_TON;
                }
            }

            // bumper_heights
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_BUMPER_HEIGHTS){
                out_trailer_unit_description.bumper_heights.front.bumper_height = in_msg.trailers.units.trailer_unit_descriptions[i].bumper_heights.front.bumper_height * units::CM_PER_M;
                out_trailer_unit_description.bumper_heights.rear.bumper_height = in_msg.trailers.units.trailer_unit_descriptions[i].bumper_heights.rear.bumper_height * units::CM_PER_M;
            }

            // center_of_gravity
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_CENTER_OF_GRAVITY){
                if(in_trailer_unit_description.center_of_gravity.unavailable){
                    out_trailer_unit_description.center_of_gravity.vehicle_height = j2735_v2x_msgs::msg::VehicleHeight::VEHICLE_HEIGHT_UNAVAILABLE;
                }
                else{
                    out_trailer_unit_description.center_of_gravity.vehicle_height = in_trailer_unit_description.center_of_gravity.vehicle_height * units::TWENTIETH_M_PER_M;
                }
            }

            // front_pivot.pivot_offset (OffsetB11)
            if(in_trailer_unit_description.front_pivot.pivot_offset.unavailable){
                out_trailer_unit_description.front_pivot.pivot_offset.offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_UNKNOWN;
            }
            else{
                out_trailer_unit_description.front_pivot.pivot_offset.offset = in_trailer_unit_description.front_pivot.pivot_offset.offset * units::CM_PER_M;
            }

            // front_pivot.pivot_angle 
            if(in_trailer_unit_description.front_pivot.pivot_offset.unavailable){
                out_trailer_unit_description.front_pivot.pivot_angle.angle = j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE;
            }
            else{
                out_trailer_unit_description.front_pivot.pivot_angle.angle = in_trailer_unit_description.front_pivot.pivot_angle.angle * units::EIGHTIETH_DEG_PER_DEG;
            }

            // front_pivot.pivots
            out_trailer_unit_description.front_pivot.pivots.pivoting_allowed = in_trailer_unit_description.front_pivot.pivots.pivoting_allowed;


            // rear_pivot
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_PIVOT){
                // rear_pivot.pivot_offset (OffsetB11)
                if(in_trailer_unit_description.rear_pivot.pivot_offset.unavailable){
                    out_trailer_unit_description.rear_pivot.pivot_offset.offset = j2735_v2x_msgs::msg::OffsetB11::OFFSET_UNKNOWN;
                }
                else{
                    out_trailer_unit_description.rear_pivot.pivot_offset.offset = in_trailer_unit_description.rear_pivot.pivot_offset.offset * units::CM_PER_M;
                }

                // rear_pivot.pivot_angle 
                if(in_trailer_unit_description.rear_pivot.pivot_offset.unavailable){
                    out_trailer_unit_description.rear_pivot.pivot_angle.angle = j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE;
                }
                else{
                    out_trailer_unit_description.rear_pivot.pivot_angle.angle = in_trailer_unit_description.rear_pivot.pivot_angle.angle * units::EIGHTIETH_DEG_PER_DEG;
                }

                // rear_pivot.pivots
                out_trailer_unit_description.rear_pivot.pivots.pivoting_allowed = in_trailer_unit_description.rear_pivot.pivots.pivoting_allowed;
            }

            // rear_wheel_offset
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_REAR_WHEEL_OFFSET){
                if(in_trailer_unit_description.rear_wheel_offset.unavailable){
                    out_trailer_unit_description.rear_wheel_offset.offset = j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN;
                }
                else{
                    out_trailer_unit_description.rear_wheel_offset.offset = in_trailer_unit_description.rear_wheel_offset.offset * units::CM_PER_M;
                }
            }

            // position_offset.x
            if(in_trailer_unit_description.position_offset.x.unavailable){
                out_trailer_unit_description.position_offset.x = j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN;
            }
            else{
                out_trailer_unit_description.position_offset.x = in_trailer_unit_description.position_offset.x.offset * units::CM_PER_M;
            }

            // position_offset.y
            if(in_trailer_unit_description.position_offset.y.unavailable){
                out_trailer_unit_description.position_offset.y = j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN;
            }
            else{
                out_trailer_unit_description.position_offset.y = in_trailer_unit_description.position_offset.y.offset * units::CM_PER_M;
            }

            // elevation_offset
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_ELEVATION_OFFSET){
                if(in_trailer_unit_description.elevation_offset.unavailable){
                    out_trailer_unit_description.elevation_offset.offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_UNAVAILABLE;
                }
                else{
                    out_trailer_unit_description.elevation_offset.offset = in_trailer_unit_description.elevation_offset.offset * units::DECI_M_PER_M;
                }
            }

            // crumb_data
            if(in_trailer_unit_description.presence_vector & carma_v2x_msgs::msg::TrailerUnitDescription::HAS_CRUMB_DATA){
                for(size_t j = 0; j < in_trailer_unit_description.crumb_data.trailer_history_points.size(); j++){
                    carma_v2x_msgs::msg::TrailerHistoryPoint in_trailer_history_point = in_trailer_unit_description.crumb_data.trailer_history_points[j];
                    j2735_v2x_msgs::msg::TrailerHistoryPoint out_trailer_history_point;

                    // crumb_data.presence_vector
                    out_trailer_history_point.presence_vector = in_trailer_history_point.presence_vector;

                    // crumb_data.angle
                    if(in_trailer_history_point.pivot_angle.unavailable){
                        out_trailer_history_point.pivot_angle.angle = j2735_v2x_msgs::msg::Angle::ANGLE_UNAVAILABLE;
                    }
                    else{
                        out_trailer_history_point.pivot_angle.angle = in_trailer_history_point.pivot_angle.angle * units::EIGHTIETH_DEG_PER_DEG;
                    }

                    // crumb_data.time_offset
                    if(in_trailer_history_point.time_offset.unavailable){
                        out_trailer_history_point.time_offset.offset = j2735_v2x_msgs::msg::TimeOffset::UNAVAILABLE;
                    }
                    else{
                        out_trailer_history_point.time_offset.offset = in_trailer_history_point.time_offset.offset * units::CENTI_S_PER_S;
                    }

                    // crumb_data.position_offset.x
                    if(in_trailer_history_point.position_offset.x.unavailable){
                        out_trailer_history_point.position_offset.x = j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN;
                    }
                    else{
                        out_trailer_history_point.position_offset.x = in_trailer_history_point.position_offset.x.offset * units::CM_PER_M;
                    }

                    // crumb_data.position_offset.y
                    if(in_trailer_history_point.position_offset.y.unavailable){
                        out_trailer_history_point.position_offset.y = j2735_v2x_msgs::msg::OffsetB12::OFFSET_UNKNOWN;
                    }
                    else{
                        out_trailer_history_point.position_offset.y = in_trailer_history_point.position_offset.y.offset * units::CM_PER_M;
                    }

                    // crumb_data.elevation_offset
                    if(in_trailer_history_point.presence_vector & carma_v2x_msgs::msg::TrailerHistoryPoint::HAS_ELEVATION_OFFSET){
                        if(in_trailer_history_point.elevation_offset.unavailable){
                            out_trailer_history_point.elevation_offset.offset = j2735_v2x_msgs::msg::VertOffsetB07::OFFSET_UNAVAILABLE;
                        }
                        else{
                            out_trailer_history_point.elevation_offset.offset = in_trailer_history_point.elevation_offset.offset * units::DECI_M_PER_M;
                        }
                    }

                    // crumb_data.heading
                    if(in_trailer_history_point.presence_vector & carma_v2x_msgs::msg::TrailerHistoryPoint::HAS_HEADING){
                        if(in_trailer_history_point.heading.unavailable){
                            out_trailer_history_point.heading.heading = j2735_v2x_msgs::msg::CoarseHeading::UNAVAILABLE;
                        }
                        else{
                            out_trailer_history_point.heading.heading = in_trailer_history_point.heading.heading / units::ONE_AND_A_HALF_DEG;
                        }
                    }

                    out_trailer_unit_description.crumb_data.trailer_history_points.push_back(out_trailer_history_point);
                }
            }
            
            out_msg.trailers.units.trailer_unit_descriptions.push_back(out_trailer_unit_description);
        }

    }
}

void BSMConvertor::convert(const std::vector<carma_v2x_msgs::msg::BSMRegionalExtension>& in_msg, std::vector<j2735_v2x_msgs::msg::BSMRegionalExtension>& out_msg)
{
    for(size_t i = 0; i < in_msg.size(); i++){
        j2735_v2x_msgs::msg::BSMRegionalExtension out_regional_extension;
        out_regional_extension.regional_extension_id = in_msg[i].regional_extension_id;

        if(in_msg[i].regional_extension_id == carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS){
            convert(in_msg[i].route_destination_points, out_regional_extension.route_destination_points);
        }

        out_msg.push_back(out_regional_extension);
    }
}

void BSMConvertor::convert(const std::vector<carma_v2x_msgs::msg::BSMPartIIExtension>& in_msg, std::vector<j2735_v2x_msgs::msg::BSMPartIIExtension>& out_msg)
{
    for(size_t i = 0; i < in_msg.size(); i++){
        j2735_v2x_msgs::msg::BSMPartIIExtension out_part_ii_extension;
        out_part_ii_extension.part_ii_id = in_msg[i].part_ii_id;

        if(in_msg[i].part_ii_id == carma_v2x_msgs::msg::BSMPartIIExtension::VEHICLE_SAFETY_EXT ){
            j2735_v2x_msgs::msg::VehicleSafetyExtensions out_vehicle_safety_extensions;
            convert(in_msg[i].vehicle_safety_extensions, out_part_ii_extension.vehicle_safety_extensions);
        }
        else if(in_msg[i].part_ii_id == carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT){
            j2735_v2x_msgs::msg::SpecialVehicleExtensions out_special_vehicle_extensions;
            convert(in_msg[i].special_vehicle_extensions, out_part_ii_extension.special_vehicle_extensions);
        }
        else if(in_msg[i].part_ii_id == carma_v2x_msgs::msg::BSMPartIIExtension::SUPPLEMENTAL_VEHICLE_EXT){
            j2735_v2x_msgs::msg::SupplementalVehicleExtensions out_supplemental_vehicle_extensions;
            convert(in_msg[i].supplemental_vehicle_extensions, out_part_ii_extension.supplemental_vehicle_extensions);
        }

        out_msg.push_back(out_part_ii_extension);
    }
}

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
  out_msg.presence_vector = in_msg.presence_vector;

  convert(in_msg.core_data, out_msg.core_data);
  
  if(in_msg.presence_vector & carma_v2x_msgs::msg::BSM::HAS_PART_II){
    convert(in_msg.part_ii, out_msg.part_ii);
  }

  if(in_msg.presence_vector & carma_v2x_msgs::msg::BSM::HAS_REGIONAL){
    convert(in_msg.regional, out_msg.regional);
  }
}

}  // namespace j2735_convertor
