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
   * \brief Struct containing the algorithm configuration values for emergency_response_vehicle_plugin
   */
  struct Config
  {
    bool enable_emergency_response_vehicle_plugin = false;          //  A flag indicating whether this plugin shall be activated. If activated, this plugin will publish
                                                                    //  the Emergency Response Vehicle's BSMs and process incoming UDP packets on the local port provided
                                                                    //  in the 'listening_port' parameter.

    double bsm_generation_frequency = 10.0;                         // (Hz) The frequency at which BSMs will be generated and published by this plugin. 

    double min_distance_to_next_destination_point = 30.0;           // (Meters) The distance that the ERV must be from its next route destination point before the point is removed from
                                                                    // the list of future route destination points.

    std::string emergency_route_file_path = "DEFAULT_PATH";         // The path to the file on the host PC for the .csv file that contains the pre-defined points existing along active emergency vehicle's route.
                                                                    // The folder containing this .csv file is expected to be /opt/carma/routes (Example: "/opt/carma/routes/example_route.csv").

    int listening_port = 1610;                                      // The listening port that this node’s UDP socket will bind to in order to receive data related to the status 
                                                                    // of the ERV’s emergency sirens and lights.

    int bsm_message_id = 8;                                         // The BSM message ID for the Emergency Response Vehicle. The value will be converted to a 4 element array of uint8_t
                                                                    // where each byte of the parameter becomes one element of the array.

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "emergency_response_vehicle_plugin::Config { " << std::endl
           << "enable_emergency_response_vehicle_plugin: " << c.enable_emergency_response_vehicle_plugin << std::endl
           << "bsm_generation_frequency: " << c.bsm_generation_frequency << std::endl
           << "min_distance_to_next_destination_point: " << c.min_distance_to_next_destination_point << std::endl
           << "emergency_route_file_path: " << c.emergency_route_file_path << std::endl
           << "listening_port: " << c.listening_port << std::endl
           << "bsm_message_id: " << c.bsm_message_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // emergency_response_vehicle_plugin
