
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

------------------------------------------------------------------------------*/

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>

#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityHeader.h>
#include "truck_inspection_plugin.h"

class TestSubscriber
{
    public:
        bool receivedMessage;
        std_msgs::StringConstPtr message;

        TestSubscriber():receivedMessage(false) {}

        void callback(const std_msgs::StringConstPtr& newMessage)
        {
            ROS_INFO_STREAM("TestSubscriber callback..");
            receivedMessage = true;
            message = newMessage;
        }
};

//TO BE DELETED when upload to GitHub
TEST(TruckInspectionTest,TestMobilityOperationInBound){
    ros::NodeHandle nh = ros::NodeHandle();
    TestSubscriber subscriber;

    //publisher 
    ros::Publisher mobility_operation_inbound_pub = nh.advertise<cav_msgs::MobilityOperation>("mobility_operation_inbound",0);

    // truck info
    std::string vin_number_ = "1FUJGBDV8CLBP8898";
    std::string license_plate_ ="DOT-10002";
    std::string carrier_name_ = "FMCSA Tech Division";
    std::string carrier_id_ ="DOT 1";
    std::string ads_software_version_ ="CARMA Platform v3.3.0";
    std::string date_of_last_state_inspection_ ="2020.01.20";
    std::string date_of_last_ads_calibration_="2020.02.20";
    std::string pre_trip_ads_health_check_ ="red";
    std::string ads_health_status_ ="red";
    std::string state_short_name_ = "VA";
    std::string weight_= "15654";
    std::string iss_score_ = "75";
    std::string permit_required_= "yes";
    std::string ads_health_status = "engaged"; //ads_health_status
    std::string ads_auto_status_ = "1"; //CAUTION
    
    cav_msgs::MobilityOperation msg_out;
    cav_msgs::MobilityHeader msg_out_header;
    msg_out.strategy = "TruckInspection";
    msg_out.strategy_params = "'vin_number:"+vin_number_
                              +",carrier_name:"+carrier_name_
                              +",date_of_last_state_inspection:"+date_of_last_state_inspection_
                              +",date_of_last_ads_calibration:"+date_of_last_ads_calibration_
                              +",license_plate:"+license_plate_
                              +",weight:"+weight_
                              +",carrier_id:"+carrier_id_
                              +",permit_required:"+permit_required_
                              +",iss_score:"+iss_score_
                              +",pre_trip_ads_health_check:"+pre_trip_ads_health_check_
                              +",ads_health_status:"+ads_health_status_
                              +",ads_auto_status:"+ads_auto_status_
                              +",ads_software_version:"+ads_software_version_
                              +",state_short_name:"+state_short_name_+"'";
    msg_out_header.sender_id =  "''";
    msg_out_header.recipient_id="''";
    msg_out_header.sender_bsm_id="'5055'";
    msg_out_header.plan_id= "";
    msg_out_header.timestamp= 1586291827962;
    msg_out.header = msg_out_header;

    mobility_operation_inbound_pub.publish(msg_out);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Spin so that publication can get to subscription
    ros::spinOnce(); 
   
    //ASSERTION
    /**Require running: roslaunch truck_inspection_plugin test_launch.test and rosrun truck_inspection_plugin truck_inspection_plugin_test 
     * Before changing 0 to 1 making sure there is a subscriber to this topic***/
    EXPECT_EQ(1,mobility_operation_inbound_pub.getNumSubscribers());
}


TEST(TruckInspectionTest,TestTruckIdentified){
    ros::NodeHandle nh = ros::NodeHandle();
    TestSubscriber subscriber_Iden;

    // Subscriber
    ros::Subscriber cav_truck_identified_sub = nh.subscribe("cav_truck_identified", 5,&TestSubscriber::callback, &subscriber_Iden); 
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Spin so that publication can get to subscription
    ros::spinOnce(); 

    //ASSERTION
    EXPECT_EQ(1, cav_truck_identified_sub.getNumPublishers()); 
    
    //Require topic mobility_operation_inbound publishes data (cav_truck_identified will then publish the data) before run below test
    if(subscriber_Iden.receivedMessage)
        EXPECT_EQ("vin_number:1FUJGHDV0CLBP8896,license_plate:DOT-1003,state_short_name:VA",subscriber_Iden.message->data);
}


TEST(TruckInspectionTest,TestTruckSafetyInfo){
    ros::NodeHandle nh = ros::NodeHandle();
    TestSubscriber subscriber;

    // Subscriber
    ros::Subscriber truck_safety_info_sub = nh.subscribe("truck_safety_info", 0,&TestSubscriber::callback, &subscriber);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
   
    //ASSERTION
    EXPECT_EQ(1, truck_safety_info_sub.getNumPublishers());
    //Require topic mobility_operation_inbound publishes data (truck_safety_info will then publish the data) before run below test
    if(subscriber.receivedMessage)
            EXPECT_EQ("vin_number:1FUJGBDV8CLBP8898,carrier_name:FMCSA Tech Division,date_of_last_state_inspection:2020.01.20,date_of_last_ads_calibration:2020.02.20,license_plate:DOT-10002,weight:15654,carrier_id:DOT 1,permit_required:Yes,iss_score:75,pre_trip_ads_health_check:red,ads_health_status:1,ads_auto_status:engaged,ads_software_version:CARMA Platform v3.3.0,state_short_name:VA,timestamp:1586291827962"
             ,subscriber.message->data);   
}

int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "truck_inspection_plugin_node_test");
    std::thread spinner([] {while (ros::ok()) ros::spin();});
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}