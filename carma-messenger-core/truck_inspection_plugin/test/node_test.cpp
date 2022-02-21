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


#include <gtest/gtest.h>
#include <thread>
#include <chrono>


#include "truck_inspection_plugin/truck_inspection_plugin_node.hpp"

namespace std_ph = std::placeholders;

TEST(TruckInspectionTest,TestMobilityOperationInBound){

    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<truck_inspection_plugin::Node>(options);
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_inbound_pub;
    //publisher 
    mobility_operation_inbound_pub = worker_node->create_publisher<carma_v2x_msgs::msg::MobilityOperation>("mobility_operation_inbound", 5);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime


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
    
    carma_v2x_msgs::msg::MobilityOperation msg_out;
    carma_v2x_msgs::msg::MobilityHeader msg_out_header;
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
    msg_out.m_header = msg_out_header;

    mobility_operation_inbound_pub->publish(msg_out);

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    EXPECT_EQ(1,mobility_operation_inbound_pub->get_subscription_count());
}


TEST(TruckInspectionTest,TestTruckIdentified){

    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<truck_inspection_plugin::Node>(options);

    int counter = 0;
    std_msgs::msg::String::ConstSharedPtr received_msg;

    std::promise<void> sub_called;
    std::shared_future<void> sub_called_future(sub_called.get_future());
    auto callback =
        [&counter, &sub_called, &received_msg](std_msgs::msg::String::ConstSharedPtr msg) -> void
        {
        received_msg = msg;
        ++counter;
        sub_called.set_value();
        };

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    auto cav_truck_identified_sub = worker_node->create_subscription<std_msgs::msg::String>("cav_truck_identified", 5, callback);

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    if(counter > 0)
        EXPECT_EQ("vin_number:1FUJGHDV0CLBP8896,license_plate:DOT-1003,state_short_name:VA", received_msg->data);

    //ASSERTION
    EXPECT_EQ(1, cav_truck_identified_sub->get_publisher_count()); 

}


TEST(TruckInspectionTest,TestTruckSafetyInfo){
    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<truck_inspection_plugin::Node>(options);

    int counter = 0;
    std_msgs::msg::String::ConstSharedPtr received_msg;

    std::promise<void> sub_called;
    std::shared_future<void> sub_called_future(sub_called.get_future());
    auto callback =
        [&counter, &sub_called, &received_msg](std_msgs::msg::String::ConstSharedPtr msg) -> void
        {
        ++counter;
        received_msg = msg;
        sub_called.set_value();
        };

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    auto truck_safety_info_sub = worker_node->create_subscription<std_msgs::msg::String>("truck_safety_info", 5, callback);
    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    if(counter > 0)
        EXPECT_EQ("vin_number:1FUJGBDV8CLBP8898,carrier_name:FMCSA Tech Division,date_of_last_state_inspection:2020.01.20,date_of_last_ads_calibration:2020.02.20,license_plate:DOT-10002,weight:15654,carrier_id:DOT 1,permit_required:Yes,iss_score:75,pre_trip_ads_health_check:red,ads_health_status:1,ads_auto_status:engaged,ads_software_version:CARMA Platform v3.3.0,state_short_name:VA,timestamp:1586291827962",received_msg->data);


    //ASSERTION
    EXPECT_EQ(1, truck_safety_info_sub->get_publisher_count());   
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 

