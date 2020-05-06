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
  j2735_msgs::ControlRequest in_msg;
  in_msg.version = "1.2.3.4";
  in_msg.scale = 2;
  
  j2735_msgs::ControlBounds b1;
  b1.oldest = 500;
  b1.latitude = 450000000;// 45 deg
  b1.longitude = 400000000;// 40 deg
  b1.offsets[0] = 1;
  b1.offsets[1] = 1;
  b1.offsets[2] = 0;

  j2735_msgs::ControlBounds b2;
  b2.oldest = 500;
  b2.latitude = 450000000;// 45 deg
  b2.longitude = 400000000;// 40 deg
  b2.offsets[0] = 2;
  b2.offsets[1] = 1;
  b2.offsets[2] = 0;

  j2735_msgs::ControlBounds b3;
  b3.oldest = 500;
  b3.latitude = 450000000;// 45 deg
  b3.longitude = 400000000;// 40 deg
  b3.offsets[0] = 2;
  b3.offsets[1] = 2;
  b3.offsets[2] = 0;

  j2735_msgs::ControlBounds b4;
  b4.oldest = 500;
  b4.latitude = 450000000;// 45 deg
  b4.longitude = 400000000;// 40 deg
  b4.offsets[0] = 1;
  b4.offsets[1] = 2;
  b4.offsets[2] = 0;

  in_msg.bounds.push_back(b1);
  in_msg.bounds.push_back(b2);
  in_msg.bounds.push_back(b3);
  in_msg.bounds.push_back(b4);

  cav_msgs::ControlRequest out_msg;
  j2735_convertor::geofence_request::convert(in_msg, out_msg);

  ASSERT_STREQ(out_msg.version.c_str(), in_msg.version.c_str());
  ASSERT_EQ(out_msg.bounds.size(), 4); // Verify the correct number of bounds exist
  // Verify bounds are converted. No need to check everything as there is a separate bounds unit test.
  ASSERT_EQ(out_msg.bounds[0].offsets[0], 100);
  ASSERT_EQ(out_msg.bounds[1].offsets[0], 200); 
  ASSERT_EQ(out_msg.bounds[2].offsets[0], 200); 
  ASSERT_EQ(out_msg.bounds[3].offsets[0], 100); 

  ASSERT_EQ(out_msg.bounds[0].offsets[1], 100);
  ASSERT_EQ(out_msg.bounds[1].offsets[1], 100); 
  ASSERT_EQ(out_msg.bounds[2].offsets[1], 200); 
  ASSERT_EQ(out_msg.bounds[3].offsets[1], 200); 
}

TEST(ControlRequest, convertControlBoundsToCAV)
{
  int8_t scale = 2; // Using scaling exponenet of 2. Applied as 10^n.
  j2735_msgs::ControlBounds in_msg;
  in_msg.oldest = 500;
  in_msg.latitude = 450000000;// 45 deg
  in_msg.longitude = 400000000;// 40 deg
  in_msg.offsets[0] = 5;
  in_msg.offsets[1] = 6;
  in_msg.offsets[2] = 2;

  cav_msgs::ControlBounds out_msg;  
  j2735_convertor::geofence_request::convert(in_msg, out_msg, scale);
  
  ASSERT_EQ(out_msg.oldest.sec, 0);
  ASSERT_EQ(out_msg.oldest.nsec, 500000000);
  ASSERT_NEAR(out_msg.latitude, 45.0, 0.00000001);
  ASSERT_NEAR(out_msg.longitude, 40.0, 0.00000001);
  ASSERT_NEAR(out_msg.offsets[0], 500.0, 0.000001);
  ASSERT_NEAR(out_msg.offsets[1], 600.0, 0.000001);
  ASSERT_NEAR(out_msg.offsets[2], 200.0, 0.000001);
}

TEST(ControlRequest, convertControlRequestToJ2735)
{
  // Unscaled scenario
  cav_msgs::ControlRequest in_msg;
  in_msg.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b1;
  b1.oldest = b1.oldest.fromNSec(500000000);
  b1.latitude = 45.0;// 45 deg
  b1.longitude = 40.0;// 40 deg
  b1.offsets[0] = 1000;
  b1.offsets[1] = 100;
  b1.offsets[2] = 0;

  cav_msgs::ControlBounds b2;
  b2.oldest = b1.oldest.fromNSec(500000000);
  b2.latitude = 45.0;// 45 deg
  b2.longitude = 40.0;// 40 deg
  b2.offsets[0] = 2000;
  b2.offsets[1] = 100;
  b2.offsets[2] = 0;

  cav_msgs::ControlBounds b3;
  b3.oldest = b1.oldest.fromNSec(500000000);
  b3.latitude = 45.0;// 45 deg
  b3.longitude = 40.0;// 40 deg
  b3.offsets[0] = 2000;
  b3.offsets[1] = 200;
  b3.offsets[2] = 0;

  cav_msgs::ControlBounds b4;
  b4.oldest = b1.oldest.fromNSec(500000000);
  b4.latitude = 45.0;// 45 deg
  b4.longitude = 40.0;// 40 deg
  b4.offsets[0] = 1005;
  b4.offsets[1] = 200;
  b4.offsets[2] = 0;

  in_msg.bounds.push_back(b1);
  in_msg.bounds.push_back(b2);
  in_msg.bounds.push_back(b3);
  in_msg.bounds.push_back(b4);

  j2735_msgs::ControlRequest out_msg;
  j2735_convertor::geofence_request::convert(in_msg, out_msg);

  ASSERT_STREQ(out_msg.version.c_str(), in_msg.version.c_str());
  ASSERT_EQ(out_msg.bounds.size(), 4); // Verify the correct number of bounds exist
  // Verify bounds are converted. No need to check everything as there is a separate bounds unit test.
  ASSERT_EQ(out_msg.bounds[0].offsets[0], 1000);
  ASSERT_EQ(out_msg.bounds[1].offsets[0], 2000); 
  ASSERT_EQ(out_msg.bounds[2].offsets[0], 2000); 
  ASSERT_EQ(out_msg.bounds[3].offsets[0], 1005); 

  ASSERT_EQ(out_msg.bounds[0].offsets[1], 100);
  ASSERT_EQ(out_msg.bounds[1].offsets[1], 100); 
  ASSERT_EQ(out_msg.bounds[2].offsets[1], 200); 
  ASSERT_EQ(out_msg.bounds[3].offsets[1], 200); 
  ASSERT_EQ(out_msg.scale, 0);

  // +1 Scaling
  cav_msgs::ControlRequest in_msg2;
  in_msg2.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b5;
  b5.offsets[0] = 1010;
  b5.offsets[1] = 100;
  b5.offsets[2] = 0;

  in_msg2.bounds.push_back(b5);

  j2735_msgs::ControlRequest out_msg2;
  j2735_convertor::geofence_request::convert(in_msg2, out_msg2);

  ASSERT_EQ(out_msg2.bounds[0].offsets[0], 101);
  ASSERT_EQ(out_msg2.bounds[0].offsets[1], 10);
  ASSERT_EQ(out_msg2.bounds[0].offsets[2], 0);



  // +2 Scaling
  cav_msgs::ControlRequest in_msg3;
  in_msg2.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b6;
  b6.offsets[0] = 1100;
  b6.offsets[1] = 100;
  b6.offsets[2] = 0;

  in_msg3.bounds.push_back(b6);


  j2735_msgs::ControlRequest out_msg3;
  j2735_convertor::geofence_request::convert(in_msg3, out_msg3);

  ASSERT_EQ(out_msg3.bounds[0].offsets[0], 11);
  ASSERT_EQ(out_msg3.bounds[0].offsets[1], 1);
  ASSERT_EQ(out_msg3.bounds[0].offsets[2], 0);

  // +3 Scaling
  cav_msgs::ControlRequest in_msg4;
  in_msg2.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b7;
  b7.offsets[0] = 15000;
  b7.offsets[1] = 5000;
  b7.offsets[2] = 0;

  in_msg4.bounds.push_back(b7);


  j2735_msgs::ControlRequest out_msg4;
  j2735_convertor::geofence_request::convert(in_msg4, out_msg4);

  ASSERT_EQ(out_msg4.bounds[0].offsets[0], 15);
  ASSERT_EQ(out_msg4.bounds[0].offsets[1], 5);
  ASSERT_EQ(out_msg4.bounds[0].offsets[2], 0);

  // -1 Scaling
  cav_msgs::ControlRequest in_msg5;
  in_msg5.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b8;
  b8.offsets[0] = 999.9;
  b8.offsets[1] = 101.4;
  b8.offsets[2] = 0;

  in_msg5.bounds.push_back(b8);


  j2735_msgs::ControlRequest out_msg5;
  j2735_convertor::geofence_request::convert(in_msg5, out_msg5);

  ASSERT_EQ(out_msg5.bounds[0].offsets[0], 9999);
  ASSERT_EQ(out_msg5.bounds[0].offsets[1], 1014);
  ASSERT_EQ(out_msg5.bounds[0].offsets[2], 0);

  // -2 Scaling
  cav_msgs::ControlRequest in_msg6;
  in_msg6.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b9;
  b9.offsets[0] = 99.99;
  b9.offsets[1] = 11.5;
  b9.offsets[2] = 0;

  in_msg6.bounds.push_back(b9);


  j2735_msgs::ControlRequest out_msg6;
  j2735_convertor::geofence_request::convert(in_msg6, out_msg6);

  ASSERT_EQ(out_msg6.bounds[0].offsets[0], 9999);
  ASSERT_EQ(out_msg6.bounds[0].offsets[1], 1150);
  ASSERT_EQ(out_msg6.bounds[0].offsets[2], 0);

  // -3 Scaling
  cav_msgs::ControlRequest in_msg7;
  in_msg7.version = "1.2.3.4";
  
  cav_msgs::ControlBounds b10;
  b10.offsets[0] = 9.999;
  b10.offsets[1] = 1.156;
  b10.offsets[2] = 0;

  in_msg7.bounds.push_back(b10);


  j2735_msgs::ControlRequest out_msg7;
  j2735_convertor::geofence_request::convert(in_msg7, out_msg7);

  ASSERT_EQ(out_msg7.bounds[0].offsets[0], 9999);
  ASSERT_EQ(out_msg7.bounds[0].offsets[1], 1156);
  ASSERT_EQ(out_msg7.bounds[0].offsets[2], 0);
  //FAIL() << "NOT IMPLEMENTED";
}

TEST(ControlRequest, convertControlBoundsToJ2735)
{
  int8_t scale = 2; // Using scaling exponenet of 2. Applied as 10^n.
  cav_msgs::ControlBounds in_msg;
  in_msg.oldest = in_msg.oldest.fromNSec(500000000);
  in_msg.latitude = 45.0;// 45 deg
  in_msg.longitude = 40.0;// 40 deg
  in_msg.offsets[0] = 500.0;
  in_msg.offsets[1] = 600.0;
  in_msg.offsets[2] = 200.0;

  j2735_msgs::ControlBounds out_msg;  
  j2735_convertor::geofence_request::convert(in_msg, out_msg, scale);
  
  ASSERT_EQ(out_msg.oldest, 500);
  ASSERT_EQ(out_msg.latitude, 450000000);
  ASSERT_EQ(out_msg.longitude, 400000000);
  ASSERT_EQ(out_msg.offsets[0], 5);
  ASSERT_EQ(out_msg.offsets[1], 6);
  ASSERT_EQ(out_msg.offsets[2], 2);

  // Check exception
  ASSERT_THROW(j2735_convertor::geofence_request::convert(in_msg, out_msg, -6);, std::invalid_argument);
}

}  // namespace j2735_convertor