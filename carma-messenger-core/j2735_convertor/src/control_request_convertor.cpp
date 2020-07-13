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
/////
// Convert j2735_msgs to cav_msgs
/////

void convert(const j2735_msgs::OffsetPoint& in_msg, cav_msgs::OffsetPoint& out_msg,const int8_t scale)
{
  out_msg.deltax = (double)in_msg.deltax* pow(10, scale);
  out_msg.deltay = (double)in_msg.deltay* pow(10, scale);
}

void convert(const j2735_msgs::TrafficControlBounds& in_msg, cav_msgs::TrafficControlBounds& out_msg, const int8_t scale) 
{
  out_msg.oldest = ros::Time(in_msg.oldest * units::SEC_PER_MIN, 0);
  out_msg.reflat = (double)in_msg.reflat / units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.reflon = (double)in_msg.reflon / units::TENTH_MICRO_DEG_PER_DEG;

  for(int i = 0; i < 3; i++)
  {
    convert(in_msg.offsets[i], out_msg.offsets[i],scale);
  }
}

void convert(const j2735_msgs::TrafficControlRequestV01& in_msg, cav_msgs::TrafficControlRequestV01& out_msg)
{
  // # reqid ::= Id64b
  // j2735_msgs/Id64b reqid
  out_msg.reqid = in_msg.reqid;

  // # reqseq ::= INTEGER (0..255)
  // uint8 reqseq
  out_msg.reqseq = in_msg.reqseq;

  //# Bounds SEQUENCE (SIZE(1..63)) OF TrafficControlBounds

  for (auto in_bound : in_msg.bounds)
  {
    cav_msgs::TrafficControlBounds out_bound;
    convert(in_bound, out_bound, in_msg.scale);
    out_msg.bounds.emplace_back(out_bound);
  }

} 

void convert(const j2735_msgs::TrafficControlRequest& in_msg, cav_msgs::TrafficControlRequest& out_msg) 
{
  // uint8 choice
  out_msg.choice = in_msg.choice;

  switch(in_msg.choice)
  {
    case j2735_msgs::TrafficControlRequest::RESERVED : 
      break;
    case j2735_msgs::TrafficControlRequest::TCRV01 : 
      convert(in_msg.tcrV01, out_msg.tcrV01);
      break;
    default:
      break;
  }

}

////
// Convert cav_msgs to j2735_msgs
////

// The use case for the control request is to grab large sets of geofences at once. So error under a meter is unlikely to matter
// Therefore it is safe to truncate and then use the modulo operator
// Therefore control requests sent from this component cannot have accuracy better than 1m

bool isIntegerDivisable(const std::vector<double>& data, int16_t divisor) 
{
  for (auto element : data) 
  {
    int64_t int_element = (int64_t)element; // Cast to integer to allow use of modulo
    if (int_element % divisor != 0) 
    {
      return false;
    }
  }

  return true;
}

bool isLessThan(const std::vector<double>& data, int16_t threshold) 
{
  for (auto element : data) 
  {
    if (element > fabs(threshold))
    {
      return false;
    }
  }

  return true;
}

void convert(const cav_msgs::TrafficControlBounds& in_msg, j2735_msgs::TrafficControlBounds& out_msg, const int8_t scale) 
{
  out_msg.oldest = in_msg.oldest.toSec() / units::SEC_PER_MIN;
  out_msg.reflat = in_msg.reflat * units::TENTH_MICRO_DEG_PER_DEG;
  out_msg.reflon = in_msg.reflon * units::TENTH_MICRO_DEG_PER_DEG;

  double scaled_deltax0 = in_msg.offsets[0].deltax / pow(10, scale);
  double scaled_deltay0 = in_msg.offsets[0].deltay / pow(10, scale);
  double scaled_deltax1 = in_msg.offsets[1].deltax / pow(10, scale);
  double scaled_deltay1 = in_msg.offsets[1].deltay / pow(10, scale);
  double scaled_deltax2 = in_msg.offsets[2].deltax / pow(10, scale);
  double scaled_deltay2 = in_msg.offsets[2].deltay / pow(10, scale);

  // Check bounds of message storage ability
  constexpr int INT_16_MIN = -32768;
  constexpr int INT_16_MAX = 32767; 
  if (scaled_deltax0 < INT_16_MIN || INT_16_MAX < scaled_deltax0 ||
      scaled_deltay0 < INT_16_MIN || INT_16_MAX < scaled_deltay0 ||
      scaled_deltax1 < INT_16_MIN || INT_16_MAX < scaled_deltax1 ||
      scaled_deltay1 < INT_16_MIN || INT_16_MAX < scaled_deltay1 ||
      scaled_deltax2 < INT_16_MIN || INT_16_MAX < scaled_deltay2 ||
      scaled_deltay2 < INT_16_MIN || INT_16_MAX < scaled_deltay2 ) 
  {
    throw std::invalid_argument("cav_msgs::ControlBounds cannot be converted because the provided bounds cannot be scaled into the 16 bit integer range without reducing precision below 1m.");
  }

  out_msg.offsets[0].deltax=scaled_deltax0;
  out_msg.offsets[0].deltay=scaled_deltay0;
  out_msg.offsets[1].deltax=scaled_deltax1;
  out_msg.offsets[1].deltay=scaled_deltay1;
  out_msg.offsets[2].deltax=scaled_deltax2;
  out_msg.offsets[2].deltay=scaled_deltay2;

}

void convert(const cav_msgs::TrafficControlRequestV01& in_msg, j2735_msgs::TrafficControlRequestV01& out_msg)
{
  // # reqid ::= Id64b
  // j2735_msgs/Id64b reqid
  out_msg.reqid = in_msg.reqid;

  // # reqseq ::= INTEGER (0..255)
  // uint8 reqseq
  out_msg.reqseq = in_msg.reqseq;

  bool canScale_neg_1_x = true;
  bool canScale_neg_2_x = true;
  bool canScale_neg_3_x = true;

  bool canScale_neg_1_y = true;
  bool canScale_neg_2_y = true;
  bool canScale_neg_3_y = true;

  for (auto bound : in_msg.bounds) 
  {

  	std::vector<double> offsetsx; 
  	std::vector<double> offsetsy;
  
  	for (auto offset : bound.offsets) 
  	{
  		offsetsx.push_back(offset.deltax);
  		offsetsy.push_back(offset.deltay);
 	}

    canScale_neg_1_x = canScale_neg_1_x && isLessThan(offsetsx, 1000);
    canScale_neg_2_x = canScale_neg_2_x && isLessThan(offsetsx, 100);
    canScale_neg_3_x = canScale_neg_3_x && isLessThan(offsetsx, 10);

    canScale_neg_1_y = canScale_neg_1_y && isLessThan(offsetsy, 1000);
    canScale_neg_2_y = canScale_neg_2_y && isLessThan(offsetsy, 100);
    canScale_neg_3_y = canScale_neg_3_y && isLessThan(offsetsy, 10);
  }

  if (canScale_neg_3_x && canScale_neg_3_y )
  {
    out_msg.scale = -3;
  } else if (canScale_neg_2_x && canScale_neg_2_y )
  {
    out_msg.scale = -2;
  } else if (canScale_neg_1_x && canScale_neg_1_y)
  {
    out_msg.scale = -1;
  } else 
  {
    out_msg.scale = 0;
  }     

  if (out_msg.scale == 0) 
  {
    bool canScale_1_x = true;
    bool canScale_2_x = true;
    bool canScale_3_x = true;

    bool canScale_1_y = true;
    bool canScale_2_y = true;
    bool canScale_3_y = true;

  	for (auto bound : in_msg.bounds) 
  	{
      std::vector<double> offsetsx;
      std::vector<double> offsetsy;
  
  	  for (auto offset : bound.offsets) 
  		{  	
  		offsetsx.push_back(offset.deltax);
  		offsetsy.push_back(offset.deltay);
  		}
     // std::vector<cav_msgs::OffsetPoint> offsets(bound.offsets.begin(), bound.offsets.end());
      canScale_1_x = canScale_1_x && isIntegerDivisable(offsetsx, 10);
      canScale_2_x = canScale_2_x && isIntegerDivisable(offsetsx, 100);
      canScale_3_x = canScale_3_x && isIntegerDivisable(offsetsx, 1000); 

      canScale_1_y = canScale_1_y && isIntegerDivisable(offsetsy, 10);
      canScale_2_y = canScale_2_y && isIntegerDivisable(offsetsy, 100);
      canScale_3_y = canScale_3_y && isIntegerDivisable(offsetsy, 1000); 
    }

    if (canScale_3_x && canScale_3_y) 
    {
      out_msg.scale = 3;
    } else if (canScale_2_x && canScale_3_y) 
    {
      out_msg.scale = 2;
    } else if (canScale_1_x && canScale_1_y)
    {
      out_msg.scale = 1;
    }
  }

  for (auto bound : in_msg.bounds)
  {
    j2735_msgs::TrafficControlBounds out_bound;
    convert(bound, out_bound, out_msg.scale);
    out_msg.bounds.emplace_back(out_bound);
  }
}

   void convert(const cav_msgs::TrafficControlRequest& in_msg, j2735_msgs::TrafficControlRequest& out_msg) 
  {
     // uint8 choice
     out_msg.choice = in_msg.choice;

     switch(in_msg.choice)
     {
       case cav_msgs::TrafficControlRequest::RESERVED : 
       break;
       case cav_msgs::TrafficControlRequest::TCRV01 : 
       convert(in_msg.tcrV01, out_msg.tcrV01);
       break;
       default : 
      break;
     }
  }

}// namespace geofence_control
}// namespace j2735_convertor
