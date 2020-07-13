/*
 * Copyright (C) 2020 LEIDOS.
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

#include <gmock/gmock.h>
#include <j2735_convertor/control_request_convertor.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace j2735_convertor
{
 
TEST(ControlRequest, convertControlRequestToCAV)
{
  j2735_msgs::TrafficControlRequestV01 in_msg;
  in_msg.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  in_msg.reqseq = 77;
  in_msg.scale = 2;
  
  j2735_msgs::TrafficControlBounds b1;
  b1.oldest = 500;
  b1.reflat = 450000000;// 45 deg
  b1.reflon = 400000000;// 40 deg
  for(int i = 0; i < 3; i++)
    {
      b1.offsets[i].deltax = 1;
      b1.offsets[i].deltay = 1;
    }

  j2735_msgs::TrafficControlBounds b2;
  b2.oldest = 500;
  b2.reflat = 450000000;// 45 deg
  b2.reflon = 400000000;// 40 deg
  for(int i = 0; i < 3; i++)
    {
      b2.offsets[i].deltax = 2;
      b2.offsets[i].deltay = 1;
    }

  j2735_msgs::TrafficControlBounds b3;
  b3.oldest = 500;
  b3.reflat = 450000000;// 45 deg
  b3.reflon = 400000000;// 40 deg
  for(int i = 0; i < 3; i++)
    {
      b3.offsets[i].deltax = 2;
      b3.offsets[i].deltay = 2;
    }

  j2735_msgs::TrafficControlBounds b4;
  b4.oldest = 500;
  b4.reflat = 450000000;// 45 deg
  b4.reflon = 400000000;// 40 deg
  for(int i = 0; i < 3; i++)
    {
      b4.offsets[i].deltax = 1;
      b4.offsets[i].deltay = 2;
    }

  in_msg.bounds.push_back(b1);
  in_msg.bounds.push_back(b2);
  in_msg.bounds.push_back(b3);
  in_msg.bounds.push_back(b4);

  cav_msgs::TrafficControlRequestV01 out_msg;
  j2735_convertor::geofence_request::convert(in_msg, out_msg);

   ASSERT_EQ(in_msg.reqid.id, out_msg.reqid.id);
  ASSERT_EQ(in_msg.reqseq, out_msg.reqseq);

  ASSERT_EQ(out_msg.bounds.size(), 4); // Verify the correct number of bounds exist
  // Verify bounds are converted. No need to check everything as there is a separate bounds unit test.

  ASSERT_EQ(out_msg.bounds[0].offsets[0].deltax, 100);
  ASSERT_EQ(out_msg.bounds[0].offsets[0].deltay, 100); 

  ASSERT_EQ(out_msg.bounds[0].offsets[1].deltax, 100);
  ASSERT_EQ(out_msg.bounds[0].offsets[1].deltay, 100);

  ASSERT_EQ(out_msg.bounds[0].offsets[2].deltax, 100);
  ASSERT_EQ(out_msg.bounds[0].offsets[2].deltay, 100);


  ASSERT_EQ(out_msg.bounds[1].offsets[0].deltax, 200);
  ASSERT_EQ(out_msg.bounds[1].offsets[0].deltay, 100); 

  ASSERT_EQ(out_msg.bounds[1].offsets[1].deltax, 200);
  ASSERT_EQ(out_msg.bounds[1].offsets[1].deltay, 100);

  ASSERT_EQ(out_msg.bounds[1].offsets[2].deltax, 200);
  ASSERT_EQ(out_msg.bounds[1].offsets[2].deltay, 100);

}

TEST(ControlRequest, convertControlBoundsToCAV)
{
  int8_t scale = 2; // Using scaling exponenet of 2. Applied as 10^n.
  j2735_msgs::TrafficControlBounds in_msg;
  in_msg.oldest = 500;
  in_msg.reflat = 450000000;// 45 deg
  in_msg.reflon = 400000000;// 40 deg
  for(int i = 0; i < 3; i++)
    {
      in_msg.offsets[i].deltax = 5;
      in_msg.offsets[i].deltay = 6;
    }

  cav_msgs::TrafficControlBounds out_msg;  
  j2735_convertor::geofence_request::convert(in_msg, out_msg, scale);
  
  ASSERT_EQ(out_msg.oldest.sec, 0);
  ASSERT_EQ(out_msg.oldest.nsec, 500000000);
  ASSERT_NEAR(out_msg.reflat, 45.0, 0.00000001);
  ASSERT_NEAR(out_msg.reflon, 40.0, 0.00000001);

  ASSERT_NEAR(out_msg.offsets[0].deltax, 500.0, 0.000001);
  ASSERT_NEAR(out_msg.offsets[0].deltay,600.0, 0.000001); 

  ASSERT_NEAR(out_msg.offsets[1].deltax, 500.0, 0.000001);
  ASSERT_NEAR(out_msg.offsets[1].deltay, 600.0, 0.000001);

  ASSERT_NEAR(out_msg.offsets[2].deltax, 500.0, 0.000001);
  ASSERT_NEAR(out_msg.offsets[2].deltay, 600.0, 0.000001);

}

TEST(ControlRequest, convertControlRequestToJ2735)
{

  // Unscaled scenario
  cav_msgs::TrafficControlRequestV01 in_msg;
  in_msg.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  in_msg.reqseq = 77;

  cav_msgs::TrafficControlBounds b1;
  b1.oldest = b1.oldest.fromNSec(500000000);
  b1.reflat = 45.0;// 45 deg
  b1.reflon = 40.0;// 40 deg
  
  for(int i = 0; i < 3; i++)
    {
      b1.offsets[i].deltax = 1000;
      b1.offsets[i].deltay = 100;
    }

  cav_msgs::TrafficControlBounds b2;
  b2.oldest = b1.oldest.fromNSec(500000000);
  b2.reflat = 45.0;// 45 deg
  b2.reflon = 40.0;// 40 deg

  for(int i = 0; i < 3; i++)
    {
      b2.offsets[i].deltax = 2000;
      b2.offsets[i].deltay = 100;
    }

  cav_msgs::TrafficControlBounds b3;
  b3.oldest = b1.oldest.fromNSec(500000000);
  b3.reflat = 45.0;// 45 deg
  b3.reflon = 40.0;// 40 deg
  
  for(int i = 0; i < 3; i++)
    {
      b3.offsets[i].deltax = 2000;
      b3.offsets[i].deltay = 200;
    }

  cav_msgs::TrafficControlBounds b4;
  b4.oldest = b1.oldest.fromNSec(500000000);
  b4.reflat = 45.0;// 45 deg
  b4.reflon = 40.0;// 40 deg

  for(int i = 0; i < 3; i++)
    {
      b3.offsets[i].deltax = 1005;
      b3.offsets[i].deltay = 200;
    }

  in_msg.bounds.push_back(b1);
  in_msg.bounds.push_back(b2);
  in_msg.bounds.push_back(b3);
  in_msg.bounds.push_back(b4);

  j2735_msgs::TrafficControlRequestV01 out_msg;
  j2735_convertor::geofence_request::convert(in_msg, out_msg);

  ASSERT_EQ(in_msg.reqid.id, out_msg.reqid.id);
  ASSERT_EQ(in_msg.reqseq, out_msg.reqseq);

  ASSERT_EQ(out_msg.bounds.size(), 4); // Verify the correct number of bounds exist
  // Verify bounds are converted. No need to check everything as there is a separate bounds unit test.
  ASSERT_EQ(out_msg.bounds[0].offsets[0].deltax, 1000);
  ASSERT_EQ(out_msg.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg.bounds[0].offsets[1].deltay, 100);

  ASSERT_EQ(out_msg.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg.bounds[0].offsets[2].deltay, 0);

  ASSERT_EQ(out_msg.bounds[1].offsets[0].deltax, 2000);
  ASSERT_EQ(out_msg.bounds[1].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg.bounds[1].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg.bounds[1].offsets[1].deltay, 100);

  ASSERT_EQ(out_msg.bounds[1].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg.bounds[1].offsets[2].deltay, 0);

  ASSERT_EQ(out_msg.scale, 0); 

  // +1 Scaling
  cav_msgs::TrafficControlRequestV01 in_msg2;
  in_msg2.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  
  cav_msgs::TrafficControlBounds b5;
  b5.offsets[0].deltax = 1010;
  b5.offsets[0].deltay = 200;
  b5.offsets[1].deltax = 100;
  b5.offsets[1].deltay = 200;
  b5.offsets[2].deltax = 1005;
  b5.offsets[2].deltay = 200;
  
  in_msg2.bounds.push_back(b5);

  j2735_msgs::TrafficControlRequestV01 out_msg2;
  j2735_convertor::geofence_request::convert(in_msg2, out_msg2);

  ASSERT_EQ(out_msg2.bounds[0].offsets[0].deltax, 1010);
  ASSERT_EQ(out_msg2.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg2.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg2.bounds[0].offsets[1].deltay, 200);

  ASSERT_EQ(out_msg2.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg2.bounds[0].offsets[2].deltay, 0);


  // +2 Scaling
  cav_msgs::TrafficControlRequestV01 in_msg3;
  in_msg3.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  
  cav_msgs::TrafficControlBounds b6;
  b6.offsets[0].deltax = 1100;
  b6.offsets[0].deltay = 100;
  b6.offsets[1].deltax = 100;
  b6.offsets[1].deltay = 200;
  b6.offsets[2].deltax = 1005;
  b6.offsets[2].deltay = 200;
  
  in_msg3.bounds.push_back(b6);

  j2735_msgs::TrafficControlRequestV01 out_msg3;
  j2735_convertor::geofence_request::convert(in_msg3, out_msg3);

  ASSERT_EQ(out_msg3.bounds[0].offsets[0].deltax, 1100);
  ASSERT_EQ(out_msg3.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg3.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg3.bounds[0].offsets[1].deltay, 200);

  ASSERT_EQ(out_msg3.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg3.bounds[0].offsets[2].deltay, 0);

  // +3 Scaling
  cav_msgs::TrafficControlRequestV01 in_msg4;
  in_msg4.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  
  cav_msgs::TrafficControlBounds b7;
  b7.offsets[0].deltax = 15000;
  b7.offsets[0].deltay = 5000;
  b7.offsets[1].deltax = 1000;
  b7.offsets[1].deltay = 2000;
  b7.offsets[2].deltax = 1005;
  b7.offsets[2].deltay = 2000;
  
  in_msg4.bounds.push_back(b7);

  j2735_msgs::TrafficControlRequestV01 out_msg4;
  j2735_convertor::geofence_request::convert(in_msg4, out_msg4);

  ASSERT_EQ(out_msg4.bounds[0].offsets[0].deltax, 15000);
  ASSERT_EQ(out_msg4.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg4.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg4.bounds[0].offsets[1].deltay, 2000);

  ASSERT_EQ(out_msg4.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg4.bounds[0].offsets[2].deltay, 0);

  // -1 Scaling
  cav_msgs::TrafficControlRequestV01 in_msg5;
  in_msg5.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  
  cav_msgs::TrafficControlBounds b8;
  b8.offsets[0].deltax = 999.9;
  b8.offsets[0].deltay = 101.4;
  b8.offsets[1].deltax = 999.9;
  b8.offsets[1].deltay = 101.4;
  b8.offsets[2].deltax = 999.9;
  b8.offsets[2].deltay = 101.4;
  
  in_msg5.bounds.push_back(b8);

  j2735_msgs::TrafficControlRequestV01 out_msg5;
  j2735_convertor::geofence_request::convert(in_msg5, out_msg5);

  ASSERT_EQ(out_msg5.bounds[0].offsets[0].deltax, 9999);
  ASSERT_EQ(out_msg5.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg5.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg5.bounds[0].offsets[1].deltay, 1014);

  ASSERT_EQ(out_msg5.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg5.bounds[0].offsets[2].deltay, 0);

  // -2 Scaling
  cav_msgs::TrafficControlRequestV01 in_msg6;
  in_msg6.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  
  cav_msgs::TrafficControlBounds b10;
  b10.offsets[0].deltax = 99.99;
  b10.offsets[0].deltay = 10.14;
  b10.offsets[1].deltax = 99.99;
  b10.offsets[1].deltay = 10.14;
  b10.offsets[2].deltax = 99.99;
  b10.offsets[2].deltay = 10.14;
  
  in_msg6.bounds.push_back(b10);

  j2735_msgs::TrafficControlRequestV01 out_msg6;
  j2735_convertor::geofence_request::convert(in_msg6, out_msg6);

  ASSERT_EQ(out_msg6.bounds[0].offsets[0].deltax, 9999);
  ASSERT_EQ(out_msg6.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg6.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg6.bounds[0].offsets[1].deltay, 1014);

  ASSERT_EQ(out_msg6.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg6.bounds[0].offsets[2].deltay, 0);

  // -3 Scaling
  cav_msgs::TrafficControlRequestV01 in_msg7;
  in_msg7.reqid.id = {0, 1, 2, 3, 4, 5, 6, 7};
  
  cav_msgs::TrafficControlBounds b11;
  b11.offsets[0].deltax = 99.99;
  b11.offsets[0].deltay = 10.14;
  b11.offsets[1].deltax = 99.99;
  b11.offsets[1].deltay = 10.14;
  b11.offsets[2].deltax = 99.99;
  b11.offsets[2].deltay = 10.14;
  
  in_msg7.bounds.push_back(b11);

  j2735_msgs::TrafficControlRequestV01 out_msg7;
  j2735_convertor::geofence_request::convert(in_msg7, out_msg7);

  ASSERT_EQ(out_msg7.bounds[0].offsets[0].deltax, 9999);
  ASSERT_EQ(out_msg7.bounds[0].offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg7.bounds[0].offsets[1].deltax, 0);
  ASSERT_EQ(out_msg7.bounds[0].offsets[1].deltay, 1014);

  ASSERT_EQ(out_msg7.bounds[0].offsets[2].deltax, 0);
  ASSERT_EQ(out_msg7.bounds[0].offsets[2].deltay, 0);
}


TEST(ControlRequest, convertControlBoundsToJ2735)
{
  int8_t scale = 2; // Using scaling exponenet of 2. Applied as 10^n.
  cav_msgs::TrafficControlBounds in_msg;
  in_msg.oldest = in_msg.oldest.fromNSec(500000000);
  in_msg.reflat = 45.0;// 45 deg
  in_msg.reflon = 40.0;// 40 deg

  in_msg.offsets[0].deltax = 500.0;
  in_msg.offsets[0].deltay = 500.0;
  in_msg.offsets[1].deltax = 600.0;
  in_msg.offsets[1].deltay = 600.0;
  in_msg.offsets[2].deltax = 200.0;
  in_msg.offsets[2].deltay = 200.0;

  j2735_msgs::TrafficControlBounds out_msg;  
  j2735_convertor::geofence_request::convert(in_msg, out_msg, scale);
  
  ASSERT_EQ(out_msg.offsets[0].deltax, 5);
  ASSERT_EQ(out_msg.offsets[0].deltay, 0); 

  ASSERT_EQ(out_msg.offsets[1].deltax, 0);
  ASSERT_EQ(out_msg.offsets[1].deltay, 6);

  ASSERT_EQ(out_msg.offsets[2].deltax, 0);
  ASSERT_EQ(out_msg.offsets[2].deltay, 0);

  // Check exception
  ASSERT_THROW(j2735_convertor::geofence_request::convert(in_msg, out_msg, -6);, std::invalid_argument);
}

}// namespace j2735_convertor*/