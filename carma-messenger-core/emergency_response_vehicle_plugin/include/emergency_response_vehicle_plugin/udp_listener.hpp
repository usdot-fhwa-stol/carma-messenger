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

#include <boost/asio/ip/udp.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/asio/io_service.hpp>

#include <rclcpp/rclcpp.hpp>

namespace emergency_response_vehicle_plugin
{
    /**
    * @brief UDPListener is a helper class that manages listening on a UDP socket
    * and passing received packets to subscribers.
    * NOTE: Class is loosely based off of UDPListener class implemented in the usdot-fhwa-stol GitHub organization's
    *       carma-cohda-dsrc-driver repository: https://github.com/usdot-fhwa-stol/carma-cohda-dsrc-driver
    */
    class UDPListener{
    
    private:
        boost::asio::io_service& io_; // Object used to enable IO services on a UDP Socket

        boost::asio::ip::udp::socket socket_; // UDP Socket object

        std::string logger_name_ = "emergency_response_vehicle_plugin::UDPListener";

        /**
        * @brief Starts asynchronous listening on udp socket.
        */
        void startReceiveUdp();

        /**
        * @brief Handles received packets
        * @param error_code
        * @param bytes_transferred
        */
        void handleReceivedUdp(const boost::system::error_code& error_code, std::size_t bytes_transferred);

    public:

        /**
        * @brief Constructs a UDPListener
        * @param io - an io_service object to service the asynchronous IO
        * @param port - the port to listen on
        */
        UDPListener(boost::asio::io_service& io, unsigned short port);

        /**
        * @brief Called on packet received
        */
        boost::signals2::signal<void (const std::shared_ptr< const std::vector<uint8_t>>&)> onReceive;

        /**
        * @brief Starts the
        * @return
        */
        bool start();

    };

} // namespace emergency_response_vehicle_plugin