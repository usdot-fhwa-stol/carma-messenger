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


#include <gtest/gtest.h>

#include "emergency_response_vehicle_plugin/emergency_response_vehicle_plugin_node.hpp"

namespace emergency_response_vehicle_plugin{

    TEST(EmergencyResponseVehiclePluginTest, testGetDistanceBetween){

        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->config_.enable_emergency_response_vehicle_plugin = true;
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Position 1 latitude/longitude coordinates (degrees)
        double lat_1_deg = 38.95612;
        double lon_1_deg = -77.15101;

        // Position 2 latitude/longitude coordinates (degrees)
        double lat_2_deg = 38.95506;
        double lon_2_deg = -77.14740;

        double distance_between_points_meters = worker_node->getDistanceBetween(lat_1_deg, lon_1_deg, lat_2_deg, lon_2_deg);
        
        // Verify distance output is within 10.0 meters of the distance reported the Google Maps measurement tool (334.97 meters for these points)
        ASSERT_NEAR(distance_between_points_meters, 334.97, 10.0);

        worker_node->handle_on_shutdown();
    }

    TEST(EmergencyResponseVehiclePluginTest, testRemoveRouteDestinationPoint){

        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->config_.enable_emergency_response_vehicle_plugin = true;
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Current ERV position latitude/longitude coordinates (degrees)
        gps_msgs::msg::GPSFix current_pose;
        current_pose.latitude = 38.95612;
        current_pose.longitude = -77.15101;

        // Future route destination point 1 latitude/longitude coordinates (degrees)
        // Note: ~335 meters from current ERV Position according to Google Maps
        carma_v2x_msgs::msg::Position3D future_destination_point_1;
        future_destination_point_1.latitude = 38.95506;
        future_destination_point_1.longitude = -77.14740;

        // Future route destination point 2 latitude/longitude coordinates (degrees)
        // Note: ~160 meters from previous point according to Google Maps
        carma_v2x_msgs::msg::Position3D future_destination_point_2;
        future_destination_point_2.latitude = 38.95612;
        future_destination_point_2.longitude = -77.14614;

        // Future route destination point 3 latitude/longitude coordinates (degrees)
        // Note: ~120 meters from previous point according to Google Maps
        carma_v2x_msgs::msg::Position3D future_destination_point_3;
        future_destination_point_3.latitude = 38.95696;
        future_destination_point_3.longitude = -77.14527;

        // Add future route destination points to node's internal list
        worker_node->route_destination_points_.push_back(future_destination_point_1);
        worker_node->route_destination_points_.push_back(future_destination_point_2);
        worker_node->route_destination_points_.push_back(future_destination_point_3);
        
        // Set configuration parameter so that ERV must be <= 200 meters to next point for the next point to be removed from route_destination_points_
        worker_node->config_.min_distance_to_next_destination_point = 200.0; // Meters

        // Trigger pose callback with current location ~335 meters from first point in route_destination_points_
        std::unique_ptr<gps_msgs::msg::GPSFix> current_pose_ptr = std::make_unique<gps_msgs::msg::GPSFix>(current_pose);
        worker_node->poseCallback(std::move(current_pose_ptr));

        // Verify route_destination_points_ does not change in size (the first point was not removed)
        ASSERT_EQ(worker_node->route_destination_points_.size(), 3);

        // Set configuration parameter so that ERV must be <= 500 meters to next point for the next point to be removed from route_destination_points_
        worker_node->config_.min_distance_to_next_destination_point = 500.0; // Meters
        
        // Trigger pose callback with current location ~335 meters from first point in route_destination_points_
        std::unique_ptr<gps_msgs::msg::GPSFix> current_pose_ptr2 = std::make_unique<gps_msgs::msg::GPSFix>(current_pose);
        worker_node->poseCallback(std::move(current_pose_ptr2));
        
        // Verify route_destination_points_ is reduced in size by 1 since ERV is within 500 meters of the next point in route_destination_points_
        ASSERT_EQ(worker_node->route_destination_points_.size(), 2);
        ASSERT_NEAR(worker_node->route_destination_points_[0].latitude, future_destination_point_2.latitude, 0.1);
        ASSERT_NEAR(worker_node->route_destination_points_[0].longitude, future_destination_point_2.longitude, 0.1);

        worker_node->handle_on_shutdown();
    }

    TEST(EmergencyResponseVehiclePluginTest, testGenerateBsm){

        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin>(options);

        worker_node->configure(); //Call configure state transition
        worker_node->config_.enable_emergency_response_vehicle_plugin = true;
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Set node's member objects that are used for generating the BSM
        worker_node->config_.bsm_message_id = 9;
        worker_node->prev_msg_count_ = 126;
        worker_node->current_latitude_ = 38.95612;
        worker_node->current_longitude_ = -77.15101;
        worker_node->current_velocity_ = 10.0;
        worker_node->emergency_lights_active_ = false;
        worker_node->emergency_sirens_active_ = true;

        // Add 3 points to node's future route destinations vector
        carma_v2x_msgs::msg::Position3D future_destination_point_1;
        future_destination_point_1.latitude = 38.95506;
        future_destination_point_1.longitude = -77.14740;

        carma_v2x_msgs::msg::Position3D future_destination_point_2;
        future_destination_point_2.latitude = 38.95612;
        future_destination_point_2.longitude = -77.14614;

        carma_v2x_msgs::msg::Position3D future_destination_point_3;
        future_destination_point_3.latitude = 38.95696;
        future_destination_point_3.longitude = -77.14527;

        worker_node->route_destination_points_.push_back(future_destination_point_1);
        worker_node->route_destination_points_.push_back(future_destination_point_2);
        worker_node->route_destination_points_.push_back(future_destination_point_3);

        // Generate BSM
        carma_v2x_msgs::msg::BSM bsm_msg = worker_node->generateBSM();

        // Verify contents of BSM's core_data
        ASSERT_EQ(worker_node->bsm_id_string_, "09000000");
        ASSERT_EQ(bsm_msg.core_data.msg_count, 127);
        ASSERT_TRUE(bsm_msg.core_data.presence_vector && carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE);
        ASSERT_NEAR(bsm_msg.core_data.latitude, 38.95612, 0.1);
        ASSERT_TRUE(bsm_msg.core_data.presence_vector && carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE);
        ASSERT_NEAR(bsm_msg.core_data.longitude, -77.15101, 0.1);
        ASSERT_TRUE(bsm_msg.core_data.presence_vector && carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE);
        ASSERT_NEAR(bsm_msg.core_data.speed, 10.0, 0.1);

        // Verify BSM's Part II Content
        ASSERT_TRUE(bsm_msg.presence_vector && carma_v2x_msgs::msg::BSM::HAS_PART_II);
        ASSERT_EQ(bsm_msg.part_ii.size(), 1);
        ASSERT_EQ(bsm_msg.part_ii[0].part_ii_id, carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT);
        ASSERT_TRUE(bsm_msg.part_ii[0].special_vehicle_extensions.presence_vector && carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS);
        ASSERT_EQ(bsm_msg.part_ii[0].special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use, j2735_v2x_msgs::msg::SirenInUse::IN_USE);
        ASSERT_EQ(bsm_msg.part_ii[0].special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use, j2735_v2x_msgs::msg::LightbarInUse::UNAVAILABLE);

        // Verify BSM's Regional Extension Content
        ASSERT_TRUE(bsm_msg.presence_vector && carma_v2x_msgs::msg::BSM::HAS_REGIONAL);
        ASSERT_EQ(bsm_msg.regional.size(), 1);
        ASSERT_EQ(bsm_msg.regional[0].regional_extension_id, carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points.size(), 3);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points[0].latitude, 38.95506);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points[0].longitude, -77.14740);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points[1].latitude, 38.95612);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points[1].longitude, -77.14614);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points[2].latitude, 38.95696);
        ASSERT_EQ(bsm_msg.regional[0].route_destination_points[2].longitude, -77.14527);

        // Generate BSM in order to verify that msg_count is properly adjusted
        bsm_msg = worker_node->generateBSM();

        // Verify that BSM's msg_count is reset to 0 since it would have surpassed max value of 127
        ASSERT_EQ(bsm_msg.core_data.msg_count, 0);
        ASSERT_EQ(worker_node->bsm_id_string_, "09000000");

        // Update statuses of lights and sirens and regenerate BSM
        worker_node->emergency_lights_active_ = true;
        worker_node->emergency_sirens_active_ = false;

        bsm_msg = worker_node->generateBSM();

        ASSERT_EQ(bsm_msg.part_ii[0].special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use, j2735_v2x_msgs::msg::SirenInUse::UNAVAILABLE);
        ASSERT_EQ(bsm_msg.part_ii[0].special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use, j2735_v2x_msgs::msg::LightbarInUse::IN_USE);

        // Update statuses of lights and sirens (again) and regenerate BSM
        worker_node->emergency_lights_active_ = true;
        worker_node->emergency_sirens_active_ = true;
        
        bsm_msg = worker_node->generateBSM();

        ASSERT_EQ(bsm_msg.part_ii[0].special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use, j2735_v2x_msgs::msg::SirenInUse::IN_USE);
        ASSERT_EQ(bsm_msg.part_ii[0].special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use, j2735_v2x_msgs::msg::LightbarInUse::IN_USE);

        worker_node->handle_on_shutdown();
    }

    TEST(EmergencyResponseVehiclePluginTest, testLoadRouteDestinationPointsFromFile){
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin>(options);
        
        worker_node->configure(); //Call configure state transition
        worker_node->config_.enable_emergency_response_vehicle_plugin = true;
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that getEmergencyRouteCallback() response is not successful since a route has not been loaded by the plugin yet
        auto req = std::make_shared<carma_planning_msgs::srv::GetEmergencyRoute::Request>();
        auto resp = std::make_shared<carma_planning_msgs::srv::GetEmergencyRoute::Response>();
        auto header_srv = std::make_shared<rmw_request_id_t>();
        worker_node->getEmergencyRouteCallback(header_srv, req, resp);

        ASSERT_EQ(resp->is_successful, false);

        // Provide file path to getRouteDestinationPointsFromFile() to extract route destination points
        worker_node->loadRouteDestinationPointsFromFile("../../install_ros2/emergency_response_vehicle_plugin/share/emergency_response_vehicle_plugin/resource/example_route.csv");

        // Verify size and contents of route_destination_points
        ASSERT_EQ(worker_node->route_destination_points_.size(), 3);

        ASSERT_NEAR(worker_node->route_destination_points_[0].longitude, -77.1474, 0.1);
        ASSERT_NEAR(worker_node->route_destination_points_[0].latitude, 38.95506, 0.1);

        ASSERT_NEAR(worker_node->route_destination_points_[1].longitude, -77.14614, 0.1);
        ASSERT_NEAR(worker_node->route_destination_points_[1].latitude, 38.95612, 0.1);

        ASSERT_NEAR(worker_node->route_destination_points_[2].longitude, -77.14527, 0.1);
        ASSERT_NEAR(worker_node->route_destination_points_[2].latitude, 38.95696, 0.1);

        ASSERT_EQ(worker_node->route_final_destination_name_, "FINAL-DEST");

        // Verify that getEmergencyRouteCallback() response is successful (and a route name is provided) now that a route has been loaded by the plugin
        auto req2 = std::make_shared<carma_planning_msgs::srv::GetEmergencyRoute::Request>();
        auto resp2 = std::make_shared<carma_planning_msgs::srv::GetEmergencyRoute::Response>();
        auto header_srv2 = std::make_shared<rmw_request_id_t>();
        worker_node->getEmergencyRouteCallback(header_srv2, req2, resp2);

        ASSERT_EQ(resp2->is_successful, true);
        ASSERT_EQ(resp2->route_name, "FINAL-DEST");

        // Verify that arrivedAtEmergencyDestination() response is successful and that plugin's route member objects are cleared
        auto req3 = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto resp3 = std::make_shared<std_srvs::srv::Trigger::Response>();
        auto header_srv3 = std::make_shared<rmw_request_id_t>();
        worker_node->arrivedAtEmergencyDestinationCallback(header_srv3, req3, resp3);

        ASSERT_EQ(resp3->success, true);
        ASSERT_EQ(worker_node->route_destination_points_.size(), 0);
        ASSERT_EQ(worker_node->route_final_destination_name_, "");

        worker_node->handle_on_shutdown();
    }

    TEST(EmergencyResponseVehiclePluginTest, testProcessIncomingUdpBinary){
        rclcpp::NodeOptions options;
        auto worker_node = std::make_shared<emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin>(options);
        
        worker_node->configure(); //Call configure state transition
        worker_node->config_.enable_emergency_response_vehicle_plugin = true;
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that node is initially constructed with sirens and lights both inactive
        ASSERT_FALSE(worker_node->emergency_sirens_active_);
        ASSERT_FALSE(worker_node->emergency_lights_active_);

        // Send mock UDP binary vector with first byte set to '1' to indicate sirens and lights are inactive
        std::vector<uint8_t> binary_data;
        binary_data.push_back(1);
        std::shared_ptr<std::vector<uint8_t>> binary_data_ptr = std::make_shared<std::vector<uint8_t>>(binary_data);
        worker_node->processIncomingUdpBinary(binary_data_ptr);

        ASSERT_FALSE(worker_node->emergency_sirens_active_);
        ASSERT_FALSE(worker_node->emergency_lights_active_);

        // Send mock UDP binary vector with first byte set to '2' to indicate sirens active and lights inactive
        binary_data[0] = 2;
        std::shared_ptr<std::vector<uint8_t>> binary_data_ptr2 = std::make_shared<std::vector<uint8_t>>(binary_data);
        worker_node->processIncomingUdpBinary(binary_data_ptr2);

        ASSERT_TRUE(worker_node->emergency_sirens_active_);
        ASSERT_FALSE(worker_node->emergency_lights_active_);

        // Send mock UDP binary vector with first byte set to '3' to indicate sirens inactive and lights active
        binary_data[0] = 3;
        std::shared_ptr<std::vector<uint8_t>> binary_data_ptr3 = std::make_shared<std::vector<uint8_t>>(binary_data);
        worker_node->processIncomingUdpBinary(binary_data_ptr3);

        ASSERT_FALSE(worker_node->emergency_sirens_active_);
        ASSERT_TRUE(worker_node->emergency_lights_active_);

        // Send mock UDP binary vector with first byte set to '4' to indicate sirens active and lights active
        binary_data[0] = 4;
        std::shared_ptr<std::vector<uint8_t>> binary_data_ptr4 = std::make_shared<std::vector<uint8_t>>(binary_data);
        worker_node->processIncomingUdpBinary(binary_data_ptr4);

        ASSERT_TRUE(worker_node->emergency_sirens_active_);
        ASSERT_TRUE(worker_node->emergency_lights_active_);

        worker_node->handle_on_shutdown();
    }

} // namespace emergency_response_vehicle_plugin


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

