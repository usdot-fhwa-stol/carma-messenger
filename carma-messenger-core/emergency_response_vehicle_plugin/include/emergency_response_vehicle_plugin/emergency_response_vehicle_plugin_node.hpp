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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <thread>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <gtest/gtest_prod.h>
#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "emergency_response_vehicle_plugin/emergency_response_vehicle_plugin_config.hpp"
#include "carma_v2x_msgs/msg/bsm.hpp"
#include "carma_v2x_msgs/msg/emergency_vehicle_ack.hpp"
#include "carma_v2x_msgs/msg/emergency_vehicle_response.hpp"
#include "j2735_v2x_msgs/msg/special_vehicle_extensions.hpp"
#include "carma_msgs/msg/ui_instructions.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "emergency_response_vehicle_plugin/udp_listener.hpp"

namespace emergency_response_vehicle_plugin
{

  // Constant for approximating the earth's radius; used to estimate the distance between latitude/longitude coordinates
  constexpr double EARTH_RADIUS_METERS = 6371000.0;

  //! @brief Enum describing the value of the first byte in a UDP packet from a connected Raspberry Pi device
  //         configured to send the status of the emergency response vehicle's sirens and lights.
  enum SirensAndLightsStatus : uint8_t
  {
    SIRENS_AND_LIGHTS_INACTIVE = 49,  // Value of char '1'
    ONLY_SIRENS_ACTIVE = 50,          // Value of char '2'
    ONLY_LIGHTS_ACTIVE = 51,          // Value of char '3'
    SIRENS_AND_LIGHTS_ACTIVE = 52     // Value of char '4'
  };

  class EmergencyResponseVehiclePlugin : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::EmergencyVehicleResponse> incoming_emergency_vehicle_response_sub_;
    carma_ros2_utils::SubPtr<gps_msgs::msg::GPSFix> pose_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::BSM> outgoing_bsm_pub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::EmergencyVehicleAck> outgoing_emergency_vehicle_ack_pub_;
    carma_ros2_utils::PubPtr<carma_msgs::msg::UIInstructions> emergency_vehicle_ui_warnings_pub_;

    // Service Servers
    carma_ros2_utils::ServicePtr<std_srvs::srv::Trigger> arrived_at_emergency_destination_server_;

    // Node configuration
    Config config_;

    // Timer for generating and publishing a BSM for the Emergency Response Vehicle
    rclcpp::TimerBase::SharedPtr bsm_generation_timer_;

    // Ordered future route destination points for the Emergency Response Vehicle; final point is the destination point
    std::vector<carma_v2x_msgs::msg::Position3D> route_destination_points_;

    // The BSM ID of the Emergency Response Vehicle as a string
    std::string bsm_id_string_;

    // The trackec msg_count field for the previously published BSM
    uint8_t prev_msg_count_ = 0;

    // The current latitude, longitude, and velocity of the Emergency Response Vehicle
    double current_latitude_;
    double current_longitude_;
    double current_velocity_;

    // Flags to indicate whether the Emergency Response Vehicle's lights and sirens are active
    bool emergency_lights_active_ = false;
    bool emergency_sirens_active_ = false;

    // The name used for ROS logging purposes
    std::string logger_name_ = "emergency_response_vehicle_plugin_node";

    // Objects required to enable asychronous UDP packet receiving by udp_listener_ object
    std::shared_ptr<boost::asio::io_service::work> work_;
    std::shared_ptr<std::thread> io_thread_;
    std::unique_ptr<boost::asio::io_service> io_;

    // Object for handling aynchronous UDP packet receiving
    std::unique_ptr<UDPListener> udp_listener_;

    // Unit Test Accessors
    FRIEND_TEST(EmergencyResponseVehiclePluginTest, testGetDistanceBetween);
    FRIEND_TEST(EmergencyResponseVehiclePluginTest, testRemoveRouteDestinationPoint);
    FRIEND_TEST(EmergencyResponseVehiclePluginTest, testGenerateBsm);
    FRIEND_TEST(EmergencyResponseVehiclePluginTest, testLoadRouteDestinationPointsFromFile);
    FRIEND_TEST(EmergencyResponseVehiclePluginTest, testProcessIncomingUdpBinary);

  public:
    /**
     * \brief EmergencyResponseVehiclePlugin constructor 
     */
    explicit EmergencyResponseVehiclePlugin(const rclcpp::NodeOptions &);

    /**
     * \brief callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Function for connecting the udp_listener_ member object to a local port to enable it to process incoming UDP packets
     * from a device that provides the activation status of the Emergency Response Vehicle's emergency lights and emergency sirens.
     * A warning statement is logged if an error occurs while building and running the udp_listener_.
     * \param local_port The local port that udp_listener_ will listen to UDP packets on.
     */
    void connect(unsigned short local_port);

    /**
     * \brief Function to process the incoming UDP packet binary data from the device that provides the activation status of the 
     * Emergency Response Vehicle's emergency lights and emergency sirens. UDP packets are received by the udp_listener_ object and the 
     * binary data is passed to this function.
     * \param data The binary data associated with the received UDP packet.
     */
    void processIncomingUdpBinary(const std::shared_ptr<const std::vector<uint8_t>>& data);

    /**
     * \brief Function to extract the route destination points from a .csv file stored on the Host PC at the file path
     * provided in the route_file_path argument. Route destination points are stored sequentially within the plugin's
     * route_destination_points_ member object.
     * \param route_file_path The file path on the Host PC that contains the .csv file for the Emergency Response Vehicle's route destination points.
     * \return A vector containing the sequential route destination points that describe the Emergency Response Vehicle's route at startup.
     */
    void loadRouteDestinationPointsFromFile(const std::string& route_file_path);

    /**
     * \brief Helper function to get the next value associated with a generated BSM's 'msgcount' field based on the 
     * provided msgcount of the previously generated BSM.
     * \param old_msg_count The value of the previously generated BSM's 'msgcount' field. 
     * \return The value to be used for the next generated BSM's 'msgcount' field.
     */
    uint8_t getNextMsgCount(uint8_t old_msg_count);

    /**
     * \brief Function for generating a BSM for the Emergency Response Vehicle based on the latest status of this object's 
     * member variables. This function also updates the value of prev_msg_count_.
     * \return The generated carma_v2x_msgs::msg::BSM message.
     */
    carma_v2x_msgs::msg::BSM generateBSM();

    /**
     * \brief Function for publishing a BSM for the Emergency Response Vehicle. Makes a call to generateBSM() to obtain
     * the BSM that will be published.
     */
    void publishBSM();

    /**
     * \brief Service callback for a service call that indicates the Emergency Response Vehicle has arrived at its destination.
     * Service is called by the Emergency Response Web UI Widget, and results in this plugin clearing its route_destination_points_
     * so that future route destination points will not be included in regional extension of the BSMs generated within generateBSM().
     * \param req An empty std_srvs::srv::Trigger::Request.
     * \param resp A std_srvs::srv::Trigger::Response with the 'success' field set to true.
     * Emergency Resonse Vehicle's route.
     */
    void arrivedAtEmergencyDestinationCallback(
      std::shared_ptr<rmw_request_id_t>, 
      std_srvs::srv::Trigger::Request::SharedPtr req, 
      std_srvs::srv::Trigger::Response::SharedPtr resp);

    /**
     * \brief A callback to process a received EmergencyVehicleResponse message, which indicates that a CDA vehicle
     * is unable to change lanes out of the path of an approaching Emergency Response Vehicle. If the message is intended
     * for the ego Emergency Response Vehicle, then a call to broadcastEmergencyVehicleAck() will be made, and message will
     * be published using emergency_vehicle_ui_warnings_pub_ to provide the Emergency Response Web UI widget with information
     * to display in a warning message to the user.
     * \param msg The incoming EmergencyVehicleResponse message.
     */
    void incomingEmergencyVehicleResponseCallback(carma_v2x_msgs::msg::EmergencyVehicleResponse::UniquePtr msg);

    /**
     * \brief A function for publishing an outgoing EmergencyVehicleAck message in response to the CDA vehicle that broadcasted 
     * an EmergencyVehicleResponse message to this Emergency Resposne Vehicle. 
     * \param recipient_id The vehicle id of the CDA vehicle that this EmergencyVehicleAck message is intended for.
     */
    void broadcastEmergencyVehicleAck(const std::string& recipient_id);

    /**
     * \brief A helpfer function to approximate the distance (in meters) between two pairs of latitude/longitude coordinates.
     * This function uses the Haversine Formula and assumes Earth as a spherical shape to approximate this measurement.
     * \param lat_1_deg The latitude (in degrees) of the first coordinate pair.
     * \param lon_1_deg The longitude (in degrees) of the first coordinate pair.
     * \param lat_2_deg The latitude (in degrees) of the second coordinate pair.
     * \param lon_2_deg The longitude (in degrees) of the second coordinate pair.
     * \return The approximate distance (in meters) between the two provided latitude/longitude coordinate pairs.
     */
    double getDistanceBetween(const double& lat_1_deg, const double& lon_1_deg, 
                                      const double& lat_2_deg, const double& lon_2_deg);

    /**
     * \brief Message callback for the Emergency Response Vehicle's current latitude and longitude.
     * \param msg The incoming gps_msgs::msg::GPSFix message.
     */
    void poseCallback(gps_msgs::msg::GPSFix::UniquePtr msg);

    /**
     * \brief Message callback for the Emergency Response Vehicle's current velocity.
     * \param msg The incoming geometry_msgs::msg::TwistStamped message.
     */
    void twistCallback(geometry_msgs::msg::TwistStamped::UniquePtr msg);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
    virtual void handle_on_shutdown();
  };

} // emergency_response_vehicle_plugin
