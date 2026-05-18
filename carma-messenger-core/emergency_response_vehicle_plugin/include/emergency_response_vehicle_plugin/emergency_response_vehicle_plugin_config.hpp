#pragma once
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
#include <iostream>
#include <vector>

namespace emergency_response_vehicle_plugin
{
  /**
   * \brief Enum for Emergency Vehicle Class types that can be published in BSM messages
   *        Follows the J2735 standard for emergency vehicle classification
   */
  enum class EmergencyVehicleClass : int
  {
    EMERGENCY_TYPE_UNKNOWN = 60,           // Default emergency vehicle type
    EMERGENCY_TYPE_OTHER = 61,             // Includes federal users
    EMERGENCY_FIRE_LIGHT_VEHICLE = 62,     // Light fire vehicle
    EMERGENCY_FIRE_HEAVY_VEHICLE = 63,     // Heavy fire vehicle
    EMERGENCY_FIRE_PARAMEDIC_VEHICLE = 64, // Fire paramedic vehicle
    EMERGENCY_FIRE_AMBULANCE_VEHICLE = 65, // Fire ambulance vehicle
    EMERGENCY_POLICE_LIGHT_VEHICLE = 66,   // Light police vehicle
    EMERGENCY_POLICE_HEAVY_VEHICLE = 67,   // Heavy police vehicle
    EMERGENCY_OTHER_RESPONDER = 68,        // Other emergency responder
    EMERGENCY_OTHER_AMBULANCE = 69         // Other ambulance vehicle
  };

  enum class BasicVehicleRole : int
  {
    BASIC_VEHICLE = 0,      // Light duty passenger vehicle type
    PUBLIC_TRANSPORT = 1,   // Used in EU for Transit us
    SPECIAL_TRANSPORT = 2,  // Used in EU (e.g., heavy load)
    DANGEROUS_GOODS = 3,    // Used in EU for any HAZMAT
    ROAD_WORK = 4,          // Used in EU for State and Local DOT uses
    ROAD_RESCUE = 5,        // Used in EU and in the US to include tow trucks.
    EMERGENCY = 6,          // Used in EU for Police, Fire and Ambulance units
    SAFETY_CAR = 7,         // Used in EU for Escort vehicles
    NONE_UNKNOWN = 8,       // added to follow current SAE style guidelines
    TRUCK = 9,              // Heavy trucks with additional BSM rights and obligations
    MOTORCYCLE = 10, 
    ROAD_SIDE_SOURCE = 11,  // For infrastructure generated calls such as
    POLICE = 12,
    FIRE = 13,
    AMBULANCE = 14,         // (does not include private para-transit etc.)
    DOT = 15,               // all roadwork vehicles
    TRANSIT = 16,           // all transit vehicles
    SLOW_MOVING = 17,       // to also include oversize etc.
    STOP_NGO = 18,          // to include trash trucks, school buses and others
    CYCLIST = 19,           // 
    PEDESTRIAN = 20,        // also includes those with mobility limitations
    NON_MOTORIZED = 21,     // other, horse drawn, etc.
    MILITARY = 22,
  };

  /**
   * \brief Struct containing the algorithm configuration values for emergency_response_vehicle_plugin
   */
  struct Config
  {
    bool enable_emergency_response_vehicle_plugin = true;           //  A flag indicating whether this plugin shall be activated. If activated, this plugin will publish
                                                                     //  the Emergency Response Vehicle's BSMs and process incoming UDP packets on the local port provided
                                                                     //  in the 'listening_port' parameter.
    double bsm_generation_frequency = 10.0;                          // (Hz) The frequency at which BSMs will be generated and published by this plugin.
    double min_distance_to_next_destination_point = 30.0;            // (Meters) The distance that the ERV must be from its next route destination point before the point is removed from
                                                                     // the list of future route destination points.
    std::string emergency_route_file_name = "DEFAULT-FILE-NAME.csv"; // The name of the .csv file on the host PC that contains the pre-defined points existing along the ERV's route.
    std::string route_file_folder = "DEFAULT-FOLDER-PATH";           // The path to the directoy on the host PC that contains the .csv file with the ERV's route destination points.
    int listening_port = 5005;                                       // The listening port that this node's UDP socket will bind to in order to receive data related to the status
                                                                     // of the ERV's emergency sirens and lights.
    int bsm_message_id = 8;                                          // The BSM message ID for the Emergency Response Vehicle. The value will be converted to a 4 element array of uint8_t
                                                                     // where each byte of the parameter becomes one element of the array.
    int emergency_vehicle_class = static_cast<int>(EmergencyVehicleClass::EMERGENCY_TYPE_UNKNOWN); // The emergency vehicle class to be published in BSM messages

    int emergency_vehicle_role =  static_cast<int>(BasicVehicleRole::NONE_UNKNOWN);  // default to unknown role

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "emergency_response_vehicle_plugin::Config { " << std::endl
           << "enable_emergency_response_vehicle_plugin: " << c.enable_emergency_response_vehicle_plugin << std::endl
           << "bsm_generation_frequency: " << c.bsm_generation_frequency << std::endl
           << "min_distance_to_next_destination_point: " << c.min_distance_to_next_destination_point << std::endl
           << "emergency_route_file_name: " << c.emergency_route_file_name << std::endl
           << "route_file_folder: " << c.route_file_folder << std::endl
           << "listening_port: " << c.listening_port << std::endl
           << "bsm_message_id: " << c.bsm_message_id << std::endl
           << "emergency_vehicle_class: " << c.emergency_vehicle_class << std::endl
           << "emergency_vehicle_role: " << c.emergency_vehicle_role << std::endl
           << "}" << std::endl;
      return output;
    }
  };
} // emergency_response_vehicle_plugin
