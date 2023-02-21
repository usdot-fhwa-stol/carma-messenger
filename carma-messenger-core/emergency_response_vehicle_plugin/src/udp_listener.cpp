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

#include "emergency_response_vehicle_plugin/udp_listener.hpp"

namespace emergency_response_vehicle_plugin
{

    UDPListener::UDPListener(boost::asio::io_service& io, unsigned short port) : io_(io), socket_(io_)
    {
        socket_.open(boost::asio::ip::udp::v4());
        socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
    }

    void UDPListener::startReceiveUdp()
    {
        auto read_handler = [this](const boost::system::error_code& error_code, size_t bytes_transferred){
                handleReceivedUdp(error_code, bytes_transferred);};
            
        // Pass read_handler so that handleReceivedUdp() is called after the async_receive operation completes.
        socket_.async_receive(
            boost::asio::null_buffers(),
            read_handler);
    }

    void UDPListener::handleReceivedUdp(const boost::system::error_code& error_code, std::size_t bytes_transferred)
    {
        if(!error_code)
        {
            // Get the number of bytes available for reading
            size_t buffer_size = socket_.available();

            // Create buffer based on number of bytes available for reading
            std::shared_ptr<std::vector<uint8_t>> buffer = std::make_shared<std::vector<uint8_t>>(buffer_size);

            boost::system::error_code receive_error_code;

            // Receive data on UDP socket; data is stored in 'buffer'
            socket_.receive(boost::asio::buffer(*buffer), 0, receive_error_code);

            if(!receive_error_code)
            {
                // Trigger the onReceive() function with 'buffer' (containing the received data) as an argument
                io_.post([this, buffer](){onReceive(buffer);});
            }

            startReceiveUdp();
        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name_), "Error occurred while reading data from UDP socket!");
        }
    }

    bool UDPListener::start()
    {
        startReceiveUdp();
        return true;
    }
} // namespace emergency_response_vehicle_plugin