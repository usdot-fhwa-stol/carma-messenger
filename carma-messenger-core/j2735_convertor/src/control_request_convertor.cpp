/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <j2735_convertor/control_request_convertor.h>
#include <math.h>

namespace j2735_convertor
{
namespace geofence_request
{
////
// Convert j2735_msgs to cav_msgs
////


void convert(const j2735_msgs::ControlRequest& in_msg, cav_msgs::ControlRequest& out_msg) {

  out_msg.version = in_msg.version;

  for (auto bound : in_msg.bounds) {
    cav_msgs::ControlBounds out_bound;
    convert(bound, out_bound, in_msg.scale);
    out_msg.bounds.emplace_back(out_bound);
  }
}

void convert(const j2735_msgs::ControlBounds& in_msg, cav_msgs::ControlBounds& out_msg, const int8_t scale) {
  out_msg.oldest = out_msg.oldest.fromNSec(in_msg.oldest * units::NS_PER_MS_INT);
  out_msg.latitude = (double)in_msg.latitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.longitude = (double)in_msg.longitude / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.offsets[0] = (double)(in_msg.offsets[0]) * pow(10, scale);
  out_msg.offsets[1] = (double)(in_msg.offsets[1]) * pow(10, scale);
  out_msg.offsets[2] = (double)(in_msg.offsets[2]) * pow(10, scale);
}

////
// Convert cav_msgs to j2735_msgs
////

// The use case for the control request is to grab large sets of geofences at once. So error under a meter is unlikely to matter
// Therefore it is safe to truncate and then use the modulo operator
// Therefore control requests sent from this component cannot have accuracy better than 1m

bool isIntegerDivisable(const std::vector<double>& data, int16_t divisor) {
  for (auto element : data) {
    int64_t int_element = (int64_t)element; // Cast to integer to allow use of modulo
    if (int_element % divisor != 0) {
      return false;
    }
  }

  return true;
}

bool isLessThan(const std::vector<double>& data, int16_t threshold) {
  for (auto element : data) {
    if (element > fabs(threshold)) {
      return false;
    }
  }

  return true;
}

void convert(const cav_msgs::ControlBounds& in_msg, j2735_msgs::ControlBounds& out_msg, const int8_t scale) {
  out_msg.oldest = (in_msg.oldest.sec * units::MS_PER_S) + (in_msg.oldest.nsec / units::NS_PER_MS_INT);
  out_msg.latitude = in_msg.latitude * units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.longitude = in_msg.longitude * units::TENTH_MICRO_DEG_PER_DEG;
  
  double scaled_x = in_msg.offsets[0] / pow(10, scale);
  double scaled_y = in_msg.offsets[1] / pow(10, scale);
  double scaled_z = in_msg.offsets[2] / pow(10, scale);
  // Check bounds of message storage ability
  constexpr int INT_16_MIN = -32768;
  constexpr int INT_16_MAX = 32767; 
  if (scaled_x < INT_16_MIN || INT_16_MAX < scaled_x ||
      scaled_y < INT_16_MIN || INT_16_MAX < scaled_y ||
      scaled_z < INT_16_MIN || INT_16_MAX < scaled_z) 
  {
    throw std::invalid_argument("cav_msgs::ControlBounds cannot be converted because the provided bounds cannot be scaled into the 16 bit integer range without reducing precision below 1m.");
  }

  out_msg.offsets[0] = scaled_x;
  out_msg.offsets[1] = scaled_y;
  out_msg.offsets[2] = scaled_z;
}

void convert(const cav_msgs::ControlRequest& in_msg, j2735_msgs::ControlRequest& out_msg) {
  out_msg.version = in_msg.version;

  bool canScale_neg_1 = true;
  bool canScale_neg_2 = true;
  bool canScale_neg_3 = true;

  for (auto bound : in_msg.bounds) {
    std::vector<double> offsets(bound.offsets.begin(), bound.offsets.end());
    canScale_neg_1 = canScale_neg_1 && isLessThan(offsets, 1000);
    canScale_neg_2 = canScale_neg_2 && isLessThan(offsets, 100);
    canScale_neg_3 = canScale_neg_3 && isLessThan(offsets, 10);
  }

  if (canScale_neg_3) {
    out_msg.scale = -3;
  } else if (canScale_neg_2) {
    out_msg.scale = -2;
  } else if (canScale_neg_1) {
    out_msg.scale = -1;
  } else {
    out_msg.scale = 0;
  }     

  if (out_msg.scale == 0) {
    bool canScale_1 = true;
    bool canScale_2 = true;
    bool canScale_3 = true;

    for (auto bound : in_msg.bounds) {
      std::vector<double> offsets(bound.offsets.begin(), bound.offsets.end());
      canScale_1 = canScale_1 && isIntegerDivisable(offsets, 10);
      canScale_2 = canScale_2 && isIntegerDivisable(offsets, 100);
      canScale_3 = canScale_3 && isIntegerDivisable(offsets, 1000); 
    }

    if (canScale_3) {
      out_msg.scale = 3;
    } else if (canScale_2) {
      out_msg.scale = 2;
    } else if (canScale_1) {
      out_msg.scale = 1;
    }
  }

  for (auto bound : in_msg.bounds) {

    j2735_msgs::ControlBounds out_bound;
    convert(bound, out_bound, out_msg.scale);
    out_msg.bounds.emplace_back(out_bound);
  }                                            
}
}  // namespace geofence_control
}  // namespace j2735_convertor