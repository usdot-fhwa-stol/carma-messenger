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
#include "emergency_response_vehicle_plugin/emergency_response_vehicle_plugin_node.hpp"

namespace emergency_response_vehicle_plugin
{
  namespace std_ph = std::placeholders;

  EmergencyResponseVehiclePlugin::EmergencyResponseVehiclePlugin(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.enable_emergency_response_vehicle_plugin = declare_parameter<bool>("enable_emergency_response_vehicle_plugin", config_.enable_emergency_response_vehicle_plugin);
    config_.bsm_generation_frequency = declare_parameter<double>("bsm_generation_frequency", config_.bsm_generation_frequency);
    config_.min_distance_to_next_destination_point = declare_parameter<double>("min_distance_to_next_destination_point", config_.min_distance_to_next_destination_point);
    config_.emergency_route_file_path = declare_parameter<std::string>("emergency_route_file_path", config_.emergency_route_file_path);
    config_.listening_port = declare_parameter<int>("listening_port", config_.listening_port);
    config_.bsm_message_id = declare_parameter<int>("bsm_message_id", config_.bsm_message_id);
  }

  rcl_interfaces::msg::SetParametersResult EmergencyResponseVehiclePlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_1 = update_params<bool>({{"enable_emergency_response_vehicle_plugin", config_.enable_emergency_response_vehicle_plugin}}, parameters);

    auto error_2 = update_params<double>({
        {"bsm_generation_frequency", config_.bsm_generation_frequency},
        {"min_distance_to_next_destination_point", config_.min_distance_to_next_destination_point},
    }, parameters);

    auto error_3 = update_params<std::string>({
        {"emergency_route_file_path", config_.emergency_route_file_path}
    }, parameters);
    rcl_interfaces::msg::SetParametersResult result;

    auto error_4 = update_params<int>({
        {"listening_port", config_.listening_port},
        {"bsm_message_id", config_.bsm_message_id},
    }, parameters);

    result.successful = !error_1 && !error_2 && !error_3 && !error_4;

    return result;
  }

  carma_ros2_utils::CallbackReturn EmergencyResponseVehiclePlugin::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "EmergencyResponseVehiclePlugin trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<bool>("enable_emergency_response_vehicle_plugin", config_.enable_emergency_response_vehicle_plugin);
    get_parameter<double>("bsm_generation_frequency", config_.bsm_generation_frequency);
    get_parameter<double>("min_distance_to_next_destination_point", config_.min_distance_to_next_destination_point);
    get_parameter<std::string>("emergency_route_file_path", config_.emergency_route_file_path);
    get_parameter<int>("listening_port", config_.listening_port);
    get_parameter<int>("bsm_message_id", config_.bsm_message_id);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name_), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&EmergencyResponseVehiclePlugin::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    incoming_emergency_vehicle_response_sub_ = create_subscription<carma_v2x_msgs::msg::EmergencyVehicleResponse>("incoming_emergency_vehicle_response", 5,
                                                              std::bind(&EmergencyResponseVehiclePlugin::incomingEmergencyVehicleResponseCallback, this, std_ph::_1));

    pose_sub_ = create_subscription<gps_msgs::msg::GPSFix>("vehicle_pose", 5,
                                                              std::bind(&EmergencyResponseVehiclePlugin::poseCallback, this, std_ph::_1));

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("velocity", 5,
                                                              std::bind(&EmergencyResponseVehiclePlugin::twistCallback, this, std_ph::_1));

    // Setup publishers
    outgoing_bsm_pub_ = create_publisher<carma_v2x_msgs::msg::BSM>("outgoing_bsm", 5);

    outgoing_emergency_vehicle_ack_pub_ = create_publisher<carma_v2x_msgs::msg::EmergencyVehicleAck>("outgoing_emergency_vehicle_ack", 5);

    emergency_vehicle_ui_warnings_pub_ = create_publisher<carma_msgs::msg::UIInstructions>("emergency_vehicle_ui_warning", 5);

    // Setup service servers
    get_emergency_route_server_ = create_service<carma_planning_msgs::srv::GetEmergencyRoute>("get_emergency_route", 
                                                              std::bind(&EmergencyResponseVehiclePlugin::getEmergencyRouteCallback, this, std_ph::_1, std_ph::_2, std_ph::_3));

    arrived_at_emergency_destination_server_ = create_service<std_srvs::srv::Trigger>("arrived_at_emergency_destination",
                                                              std::bind(&EmergencyResponseVehiclePlugin::arrivedAtEmergencyDestinationCallback, this, std_ph::_1, std_ph::_2, std_ph::_3));

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn EmergencyResponseVehiclePlugin::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    // Enable BSM generation, UDP listener, etc. based on the setting of config_.enable_emergency_response_vehicle_plugin
    if(config_.enable_emergency_response_vehicle_plugin){
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Plugin has been enabled by its configuration parameter settings.");

      // Create timer for plugin to generate and publish a new BSM
      int bsm_generation_period_ms = (1 / config_.bsm_generation_frequency) * 1000; // Conversion from frequency (Hz) to milliseconds time period
      bsm_generation_timer_ = create_timer(get_clock(),
                            std::chrono::milliseconds(bsm_generation_period_ms),
                            std::bind(&EmergencyResponseVehiclePlugin::publishBSM, this));

      // Load route destination points from file path provided by the configuration parameters
      loadRouteDestinationPointsFromFile(config_.emergency_route_file_path);

      // Connect udp_listener_ to process incoming UDP packets that provide the status of this ERV's emergency lights and sirens
      connect(config_.listening_port);
    }
    else{
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Plugin has been disabled by its configuration parameter settings.");
    }

    return CallbackReturn::SUCCESS;
  }

  void EmergencyResponseVehiclePlugin::connect(unsigned short local_port)
  {
    io_.reset(new boost::asio::io_service());

    // Build UDPListener object, which listens to UDP packets on local port defined by local_port
    try{
      if(udp_listener_){
        udp_listener_.reset(nullptr);
      }
      udp_listener_.reset(new UDPListener(*io_, local_port));
    }
    catch(boost::system::system_error e){
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Error occurred when building UDPListener!");
    }
    catch(std::exception e){
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Exception thrown when building UDPListener: " << e.what());
    };

    // Connect udp_listener_'s onReceive() to this object's processIncomingUdpBinary() function
    udp_listener_->onReceive.connect([this](const std::shared_ptr<const std::vector<uint8_t>>& data){processIncomingUdpBinary(data);});

    // Called to enable asynchronous processing of UDP packets when io->run is called
    work_.reset(new boost::asio::io_service::work(*io_));

    udp_listener_->start();

    // Run the io service
    io_thread_.reset(new std::thread([this](){
      boost::system::error_code error;
      io_->run(error);
      if(error)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Error occurred when running io service. UDPListener will not be functional!");
      }
    }));
  }

  void EmergencyResponseVehiclePlugin::handle_on_shutdown()
  {
    work_.reset();
    io_->stop();
    io_thread_->join();
  }

  void EmergencyResponseVehiclePlugin::processIncomingUdpBinary(const std::shared_ptr<const std::vector<uint8_t>>& data)
  {
    auto data_vector = *data;

    if(!data_vector.empty()){
      switch(data_vector[0]){
        case 1:
          // 1: Emergency sirens and emergency lights are both inactive
          emergency_sirens_active_ = false;
          emergency_lights_active_ = false;
          break;
        case 2:
          // 2: Emergency sirens are active, emergency lights are inactive
          emergency_sirens_active_ = true;
          emergency_lights_active_ = false;
          break;
        case 3:
          // 3: Emergency sirens are inactive, emergency lights are active
          emergency_sirens_active_ = false;
          emergency_lights_active_ = true;
          break;
        case 4:
          // 4: Emergency sirens and emergency lights are both active
          emergency_sirens_active_ = true;
          emergency_lights_active_ = true;
          break;
        default:
          RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Unsupported initial UDP byte of " << data_vector[0] << " received; packet will not be processed");
          break;
      }
    }
    else{
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Empty UDP payload received; packet will not be processed");
    }
  }

  void EmergencyResponseVehiclePlugin::loadRouteDestinationPointsFromFile(const std::string& route_file_path)
  {   
    // Only load destination points if the file is a .csv; assumes file name ends with ".csv"
    if(route_file_path.find(".csv") == std::string::npos){
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Route file located at " << route_file_path << " is not a .csv, destination points will not be loaded");
    }
    else{
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Loading route destination points from " << route_file_path);
      std::ifstream fs(route_file_path);
      std::string line;

      // Read each line in route file (if any) and generate a route destination point
      // NOTE: Each line of the .csv follows the format "LONGITUDE<float>,LATITUDE<float>,ELEVATION<float>,DESTINATION-NAME<string>"
      std::string final_destination_name;
      while(std::getline(fs, line)){
        carma_v2x_msgs::msg::Position3D destination_point;

        // lat lon and elev is seperated by comma
        auto comma = line.find(",");
        // convert lon value in degrees from string
        destination_point.longitude = std::stof(line.substr(0, comma));
        line.erase(0, comma + 1);
        comma = line.find(",");
        // convert lat value in degrees from string
        destination_point.latitude = std::stof(line.substr(0, comma));
        // elevation is in meters
        line.erase(0, comma + 1);
        comma = line.find(",");
        destination_point.elevation = std::stof(line.substr(0, comma));
        destination_point.elevation_exists = true;

        route_destination_points_.push_back(destination_point);

        // Set route_final_destination_name_ using the descriptive name associated with this destination point
        line.erase(0, comma + 1);
        route_final_destination_name_ = line;
      }

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Number of route destination points loaded: " << route_destination_points_.size());
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "Final destination of route: " << route_final_destination_name_);

      if(route_destination_points_.empty()){
        RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "No route destination points were loaded by plugin!");
      }
    }
  }

  uint8_t EmergencyResponseVehiclePlugin::getNextMsgCount(uint8_t old_msg_count){
    old_msg_count++;
    if(old_msg_count > carma_v2x_msgs::msg::BSMCoreData::MSG_COUNT_MAX)
    {
        old_msg_count = 0;
    }
    return old_msg_count;
  }

  void EmergencyResponseVehiclePlugin::publishBSM()
  {
    carma_v2x_msgs::msg::BSM outgoing_bsm_msg = generateBSM();

    // Publish BSM
    outgoing_bsm_pub_->publish(outgoing_bsm_msg);
  }

  carma_v2x_msgs::msg::BSM EmergencyResponseVehiclePlugin::generateBSM()
  {
    carma_v2x_msgs::msg::BSM bsm_msg;

    bsm_msg.header.stamp = this->now();

    // Set msg_count
    uint8_t new_msg_count = getNextMsgCount(prev_msg_count_);
    bsm_msg.core_data.msg_count = new_msg_count;
    prev_msg_count_ = new_msg_count;

    // Set BSM ID
    std::vector<uint8_t> id(4);
    for(int i = 0; i < id.size(); ++i){
      id[i] = config_.bsm_message_id >> (8 * i);
    }
    bsm_msg.core_data.id = id;

    // Store string representation of BSM ID
    std::stringstream ss;
    for(size_t i = 0; i < bsm_msg.core_data.id.size(); ++i){
      ss << std::setfill('0') << std::setw(2) << std::hex << (unsigned)bsm_msg.core_data.id.at(i);
    }
    bsm_id_string_ = ss.str();

    // Set current latitude, longitude, and velocity
    bsm_msg.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE;
    bsm_msg.core_data.latitude = current_latitude_;

    bsm_msg.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE;
    bsm_msg.core_data.longitude = current_longitude_;

    bsm_msg.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE;
    bsm_msg.core_data.speed = current_velocity_;

    // Set lights status and/or siren status
    if(emergency_lights_active_ || emergency_sirens_active_){
      bsm_msg.presence_vector |= carma_v2x_msgs::msg::BSM::HAS_PART_II;

      // BSMPartIIExtension.special_vehicle_extensions
      carma_v2x_msgs::msg::BSMPartIIExtension part_ii_special;
      part_ii_special.part_ii_id = carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT;

      // BSMPartIIExtension.special_vehicle_extensions.vehicle_alerts
      part_ii_special.special_vehicle_extensions.presence_vector |= j2735_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS;

      if(emergency_sirens_active_){
        part_ii_special.special_vehicle_extensions.vehicle_alerts.siren_use.siren_in_use = j2735_v2x_msgs::msg::SirenInUse::IN_USE;
      }

      if(emergency_lights_active_){
        part_ii_special.special_vehicle_extensions.vehicle_alerts.lights_use.lightbar_in_use = j2735_v2x_msgs::msg::LightbarInUse::IN_USE;
      }

      bsm_msg.part_ii.push_back(part_ii_special);
    }

    // Set route destination points
    if(!route_destination_points_.empty()){
      // Create BSM regional extension that ERV's route destination points will be added to
      carma_v2x_msgs::msg::BSMRegionalExtension regional_ext;
      regional_ext.regional_extension_id = carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS;

      // Add route destination points to regional extension
      for(size_t i = 0; i < route_destination_points_.size(); ++i){
          regional_ext.route_destination_points.push_back(route_destination_points_[i]);
      }

      bsm_msg.regional.push_back(regional_ext);
    }

    return bsm_msg;
  }

  void EmergencyResponseVehiclePlugin::getEmergencyRouteCallback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::GetEmergencyRoute::Request::SharedPtr req, 
    carma_planning_msgs::srv::GetEmergencyRoute::Response::SharedPtr resp)
  {
    // Include the name of the route in the response if route destination points have been loaded into this plugin
    if(!route_destination_points_.empty()){
      resp->route_name = route_final_destination_name_;
      resp->is_successful = true;
    }
    else{
      resp->is_successful = false;
    }
  }

  void EmergencyResponseVehiclePlugin::arrivedAtEmergencyDestinationCallback(
    std::shared_ptr<rmw_request_id_t>, 
    std_srvs::srv::Trigger::Request::SharedPtr req, 
    std_srvs::srv::Trigger::Response::SharedPtr resp)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "ERV has arrived destination. Removing " << route_destination_points_.size() << " route destination points from plugin.");

    // Clear plugin's route member objects
    route_destination_points_.clear();
    route_final_destination_name_ = "";

    resp->success = true;
  }

  void EmergencyResponseVehiclePlugin::incomingEmergencyVehicleResponseCallback(carma_v2x_msgs::msg::EmergencyVehicleResponse::UniquePtr msg)
  {
    // Only process message if it is intended for the ego vehicle and the sending vehicle is unable to change lanes
    if((msg->m_header.recipient_id == bsm_id_string_) && !msg->can_change_lanes){

      // Generate warning message and send to UI so the UI can display an alert for the user
      carma_msgs::msg::UIInstructions ui_warning_msg;

      ui_warning_msg.msg = "Downstream vehicle " + msg->m_header.sender_id + " is unable to change lanes!";
      ui_warning_msg.type = carma_msgs::msg::UIInstructions::INFO;

      emergency_vehicle_ui_warnings_pub_->publish(ui_warning_msg);

      // Trigger publication of EmergencyVehicleAck to indicate ERV has received the EmergencyVehicleResponse message
      broadcastEmergencyVehicleAck(msg->m_header.sender_id);
    }
  }

  void EmergencyResponseVehiclePlugin::broadcastEmergencyVehicleAck(const std::string& recipient_id)
  {
    carma_v2x_msgs::msg::EmergencyVehicleAck ack_msg;

    ack_msg.m_header.sender_id = bsm_id_string_;
    ack_msg.m_header.recipient_id = recipient_id;
    ack_msg.acknowledgement = true;

    outgoing_emergency_vehicle_ack_pub_->publish(ack_msg);
  }

  double EmergencyResponseVehiclePlugin::getDistanceBetween(const double& lat_1_deg, const double& lon_1_deg, 
                                    const double& lat_2_deg, const double& lon_2_deg)
  {
    // Convert latitude and longitude values from degrees to radians for point 1
    double lat_1_rad = lat_1_deg * (M_PI / 180.0);
    double lon_1_rad = lon_1_deg * (M_PI / 180.0);

    // Convert latitude and longitude values from degrees to radians for point 2
    double lat_2_rad = lat_2_deg * (M_PI / 180.0);
    double lon_2_rad = lon_2_deg * (M_PI / 180.0);

    // Calculate deltas for inputted latitude and longitude values
    double delta_lat_rad = lat_2_rad - lat_1_rad;
    double delta_lon_rad = lon_2_rad - lon_1_rad;

    // Split calculation into two steps for readability
    double a = std::pow(std::sin(delta_lat_rad / 2.0), 2) + std::pow(std::sin(delta_lon_rad / 2.0), 2) * std::cos(lat_1_rad) * std::cos(lat_2_rad);
    double distance = EARTH_RADIUS_METERS * 2.0 * std::asin(sqrt(a));

    // Return distance between (lat_1, lon_1) and (lat_2, lon_2) in meters
    return distance;
  }

  void EmergencyResponseVehiclePlugin::poseCallback(gps_msgs::msg::GPSFix::UniquePtr msg)
  {
    current_latitude_ = msg->latitude;
    current_longitude_ = msg->longitude;

    if(!route_destination_points_.empty()){
      // Get distance between current ERV location and first destination point
      double distance_to_next_destination_point = getDistanceBetween(current_latitude_, current_longitude_, route_destination_points_[0].latitude, route_destination_points_[0].longitude);

      // Remove first destination point if ERV is within configurable distance of that point
      if(distance_to_next_destination_point <= config_.min_distance_to_next_destination_point){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name_), "ERV is " << distance_to_next_destination_point << " meters from next destination point. Point will be removed.");
        route_destination_points_.erase(route_destination_points_.begin());
      }
    }
  }

  void EmergencyResponseVehiclePlugin::twistCallback(geometry_msgs::msg::TwistStamped::UniquePtr msg)
  {
    current_velocity_ = msg->twist.linear.x;
  }


} // emergency_response_vehicle_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin)
