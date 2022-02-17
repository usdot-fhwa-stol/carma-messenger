#pragma once

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

#include <iostream>
#include <vector>

namespace truck_inspection_plugin_ros2
{

  /**
   * \brief Stuct containing the algorithm configuration values for truck_inspection_plugin_ros2
   */
  struct Config
  {
    //! Example parameter
    int num_of_entries = 13;

    // Stream operator for this config
    // TODO for USER: Update prints for the added parameters
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "truck_inspection_plugin_ros2::Config { " << std::endl
           << "num_of_entries: " << c.num_of_entries << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // truck_inspection_plugin_ros2