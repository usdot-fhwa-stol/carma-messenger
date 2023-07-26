#pragma once
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

#include <cstdint>
#include <j3224_v2x_msgs/msg/sensor_data_sharing_message.hpp>
#include <carma_v2x_msgs/msg/sensor_data_sharing_message.hpp>
#include <j2735_v2x_msgs/msg/yaw_rate.hpp>
#include <carma_v2x_msgs/msg/yaw_rate.hpp>
#include <carma_msgs/msg/system_alert.hpp>
#include "units.hpp"
#include "value_convertor.hpp"

namespace j2735_convertor
{
/**
 * @class SDSMConvertor
 * @brief Is the class responsible for converting J2735 SDSMs to CARMA usable SDSMs
 *
 * Handles conversion between SDSMs in the j3224_v2x_msgs and carma_v2x_msgs packages with additional conversions for j2735_v2x_msgs.
 * Unit conversions and presence flags are also handled
 */
class SDSMConvertor
{
public:


    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::SensorDataSharingMessage into a carma_v2x_msgs::msg::SensorDataSharingMessage
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::SensorDataSharingMessage& in_msg, carma_v2x_msgs::msg::SensorDataSharingMessage& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::SensorDataSharingMessage into a j3224_v2x_msgs::msg::SensorDataSharingMessage
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::SensorDataSharingMessage& in_msg, j3224_v2x_msgs::msg::SensorDataSharingMessage& out_msg);


private:
    ////
    // Convert j2735_msgs
    ////

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::YawRate into a carma_v2x_msgs::msg::YawRate
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::YawRate& in_msg, carma_v2x_msgs::msg::YawRate& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::YawRate into a j2735_v2x_msgs::msg::YawRate
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::YawRate& in_msg, j2735_v2x_msgs::msg::YawRate& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::Position3D into a carma_v2x_msgs::msg::Position3D
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::Position3D& in_msg, carma_v2x_msgs::msg::Position3D& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::Position3D into a j2735_v2x_msgs::msg::Position3D
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::Position3D& in_msg, j2735_v2x_msgs::msg::Position3D& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::PositionalAccuracy into a carma_v2x_msgs::msg::PositionalAccuracy
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::PositionalAccuracy& in_msg, carma_v2x_msgs::msg::PositionalAccuracy& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PositionalAccuracy into a j2735_v2x_msgs::msg::PositionalAccuracy
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::PositionalAccuracy& in_msg, j2735_v2x_msgs::msg::PositionalAccuracy& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::Speed into a carma_v2x_msgs::msg::Speed
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::Speed& in_msg, carma_v2x_msgs::msg::Speed& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::Speed into a j2735_v2x_msgs::msg::Speed
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::Speed& in_msg, j2735_v2x_msgs::msg::Speed& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::Heading into a carma_v2x_msgs::msg::Heading
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::Heading& in_msg, carma_v2x_msgs::msg::Heading& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::Heading into a j2735_v2x_msgs::msg::Heading
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::Heading& in_msg, j2735_v2x_msgs::msg::Heading& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::AccelerationSet4Way into a carma_v2x_msgs::msg::AccelerationSet4Way
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::AccelerationSet4Way& in_msg, carma_v2x_msgs::msg::AccelerationSet4Way& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AccelerationSet4Way into a j2735_v2x_msgs::msg::AccelerationSet4Way
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::AccelerationSet4Way& out_msg, j2735_v2x_msgs::msg::AccelerationSet4Way& in_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::VehicleSize into a carma_v2x_msgs::msg::VehicleSize
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::VehicleSize& in_msg, carma_v2x_msgs::msg::VehicleSize& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::VehicleSize into a j2735_v2x_msgs::msg::VehicleSize
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::VehicleSize& in_msg, j2735_v2x_msgs::msg::VehicleSize& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::VehicleHeight into a carma_v2x_msgs::msg::VehicleHeight
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::VehicleHeight& in_msg, carma_v2x_msgs::msg::VehicleHeight& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::VehicleHeight into a j2735_v2x_msgs::msg::VehicleHeight
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::VehicleHeight& in_msg, j2735_v2x_msgs::msg::VehicleHeight& out_msg);

    /**
    * @brief Convert the contents of a j2735_v2x_msgs::msg::AttachmentRadius into a carma_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j2735_v2x_msgs::msg::AttachmentRadius& in_msg, carma_v2x_msgs::msg::AttachmentRadius& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AttachmentRadius into a j2735_v2x_msgs::msg::AttachmentRadius
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::AttachmentRadius& in_msg, j2735_v2x_msgs::msg::AttachmentRadius& out_msg);


    ////
    // Convert j3224_msgs
    ////

    // static void convert(const j3224_v2x_msgs::msg::DetectedObjectList& in_msg, carma_v2x_msgs::msg::DetectedObjectList& out_msg);
    // static void convert(const carma_v2x_msgs::msg::DetectedObjectList& in_msg, j3224_v2x_msgs::msg::DetectedObjectList& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::DetectedObjectData into a carma_v2x_msgs::msg::DetectedObjectData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::DetectedObjectData& in_msg, carma_v2x_msgs::msg::DetectedObjectData& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::DetectedObjectData into a j3224_v2x_msgs::msg::DetectedObjectData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::DetectedObjectData& in_msg, j3224_v2x_msgs::msg::DetectedObjectData& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::DetectedObjectCommonData into a carma_v2x_msgs::msg::DetectedObjectCommonData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::DetectedObjectCommonData& in_msg, carma_v2x_msgs::msg::DetectedObjectCommonData& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::DetectedObjectCommonData into a j3224_v2x_msgs::msg::DetectedObjectCommonData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::DetectedObjectCommonData& in_msg, j3224_v2x_msgs::msg::DetectedObjectCommonData& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::DetectedObjectOptionalData into a carma_v2x_msgs::msg::DetectedObjectOptionalData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::DetectedObjectOptionalData& in_msg, carma_v2x_msgs::msg::DetectedObjectOptionalData& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::DetectedObjectOptionalData into a j3224_v2x_msgs::msg::DetectedObjectOptionalData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::DetectedObjectOptionalData& in_msg, j3224_v2x_msgs::msg::DetectedObjectOptionalData& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::MeasurementTimeOffset into a carma_v2x_msgs::msg::MeasurementTimeOffset
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::MeasurementTimeOffset& in_msg, carma_v2x_msgs::msg::MeasurementTimeOffset& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::MeasurementTimeOffset into a j3224_v2x_msgs::msg::MeasurementTimeOffset
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::MeasurementTimeOffset& in_msg, j3224_v2x_msgs::msg::MeasurementTimeOffset& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::PositionOffsetXYZ into a carma_v2x_msgs::msg::PositionOffsetXYZ
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::PositionOffsetXYZ& in_msg, carma_v2x_msgs::msg::PositionOffsetXYZ& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PositionOffsetXYZ into a j3224_v2x_msgs::msg::PositionOffsetXYZ
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::PositionOffsetXYZ& in_msg, j3224_v2x_msgs::msg::PositionOffsetXYZ& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::ObjectDistance into a carma_v2x_msgs::msg::ObjectDistance
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::ObjectDistance& in_msg, carma_v2x_msgs::msg::ObjectDistance& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::ObjectDistance into a j3224_v2x_msgs::msg::ObjectDistance
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::ObjectDistance& in_msg, j3224_v2x_msgs::msg::ObjectDistance& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::DetectedVehicleData into a carma_v2x_msgs::msg::DetectedVehicleData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::DetectedVehicleData& in_msg, carma_v2x_msgs::msg::DetectedVehicleData& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::DetectedVehicleData into a j3224_v2x_msgs::msg::DetectedVehicleData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::DetectedVehicleData& in_msg, j3224_v2x_msgs::msg::DetectedVehicleData& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::Attitude into a carma_v2x_msgs::msg::Attitude
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::Attitude& in_msg, carma_v2x_msgs::msg::Attitude& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::Attitude into a j3224_v2x_msgs::msg::Attitude
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::Attitude& in_msg, j3224_v2x_msgs::msg::Attitude& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::PitchDetected into a carma_v2x_msgs::msg::PitchDetected
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::PitchDetected& in_msg, carma_v2x_msgs::msg::PitchDetected& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PitchDetected into a j3224_v2x_msgs::msg::PitchDetected
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::PitchDetected& in_msg, j3224_v2x_msgs::msg::PitchDetected& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::RollDetected into a carma_v2x_msgs::msg::RollDetected
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::RollDetected& in_msg, carma_v2x_msgs::msg::RollDetected& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::RollDetected into a j3224_v2x_msgs::msg::RollDetected
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::RollDetected& in_msg, j3224_v2x_msgs::msg::RollDetected& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::YawDetected into a carma_v2x_msgs::msg::YawDetected
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::YawDetected& in_msg, carma_v2x_msgs::msg::YawDetected& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::YawDetected into a j3224_v2x_msgs::msg::YawDetected
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::YawDetected& in_msg, j3224_v2x_msgs::msg::YawDetected& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::AngularVelocity into a carma_v2x_msgs::msg::AngularVelocity
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::AngularVelocity& in_msg, carma_v2x_msgs::msg::AngularVelocity& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::AngularVelocity into a j3224_v2x_msgs::msg::AngularVelocity
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::AngularVelocity& in_msg, j3224_v2x_msgs::msg::AngularVelocity& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::PitchRate into a carma_v2x_msgs::msg::PitchRate
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::PitchRate& in_msg, carma_v2x_msgs::msg::PitchRate& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::PitchRate into a j3224_v2x_msgs::msg::PitchRate
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::PitchRate& in_msg, j3224_v2x_msgs::msg::PitchRate& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::RollRate into a carma_v2x_msgs::msg::RollRate
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::RollRate& in_msg, carma_v2x_msgs::msg::RollRate& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::RollRate into a j3224_v2x_msgs::msg::RollRate
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::RollRate& in_msg, j3224_v2x_msgs::msg::RollRate& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::DetectedVRUData into a carma_v2x_msgs::msg::DetectedVRUData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::DetectedVRUData& in_msg, carma_v2x_msgs::msg::DetectedVRUData& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::DetectedVRUData into a j3224_v2x_msgs::msg::DetectedVRUData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::DetectedVRUData& in_msg, j3224_v2x_msgs::msg::DetectedVRUData& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::DetectedObstacleData into a carma_v2x_msgs::msg::DetectedObstacleData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::DetectedObstacleData& in_msg, carma_v2x_msgs::msg::DetectedObstacleData& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::DetectedObstacleData into a j3224_v2x_msgs::msg::DetectedObstacleData
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::DetectedObstacleData& in_msg, j3224_v2x_msgs::msg::DetectedObstacleData& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::ObstacleSize into a carma_v2x_msgs::msg::ObstacleSize
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::ObstacleSize& in_msg, carma_v2x_msgs::msg::ObstacleSize& out_msg);

    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::ObstacleSize into a j3224_v2x_msgs::msg::ObstacleSize
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::ObstacleSize& in_msg, j3224_v2x_msgs::msg::ObstacleSize& out_msg);

    /**
    * @brief Convert the contents of a j3224_v2x_msgs::msg::SizeValue into a carma_v2x_msgs::msg::SizeValue
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const j3224_v2x_msgs::msg::SizeValue& in_msg, carma_v2x_msgs::msg::SizeValue& out_msg);
    
    /**
    * @brief Convert the contents of a carma_v2x_msgs::msg::SizeValue into a j3224_v2x_msgs::msg::SizeValue
    *
    * @param in_msg The message to be converted
    * @param out_msg The message to store the output
    *
    * Unit conversions and presence flags are handled
    */
    static void convert(const carma_v2x_msgs::msg::SizeValue& in_msg, j3224_v2x_msgs::msg::SizeValue& out_msg);

};
} // namespace j2735_convertor
