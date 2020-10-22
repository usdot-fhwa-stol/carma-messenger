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

/**
 * CPP File containing BSM Message method implementations
 */

#include "BSM_Message.h"

namespace cpp_message
{
    boost::optional<j2735_msgs::BSM> BSM_Message::decode_bsm_message(std::vector<uint8_t>& binary_array){
        
        j2735_msgs::BSM output;
        //decode results - stored in binary_array
        asn_dec_rval_t rval;
        MessageFrame_t* message = nullptr;
        
        //copy from vector to array         
        size_t len=binary_array.size();    
        
        uint8_t buf[len];             
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
         
        //if decode success
        if(rval.code==RC_OK)
        {
            ROS_WARN_STREAM("decoding");
            BSMcoreData_t core_data_msg = message->value.choice.BasicSafetyMessage.coreData;
            ROS_WARN_STREAM("decoding");
            output.core_data.msg_count = core_data_msg.msgCnt; 
            auto id_len = core_data_msg.id.size;
            ROS_WARN_STREAM("decoding");
            for(auto i = 0; i < id_len; i++)
            {
                output.core_data.id.push_back(core_data_msg.id.buf[i]);
            }
            ROS_WARN_STREAM("decoding0");
            output.core_data.sec_mark = core_data_msg.secMark;
            output.core_data.latitude = core_data_msg.lat;
            output.core_data.longitude = core_data_msg.Long; 
            output.core_data.elev = core_data_msg.elev;
            output.core_data.accuracy.orientation = core_data_msg.accuracy.orientation;
            output.core_data.accuracy.semiMajor = core_data_msg.accuracy.semiMajor;
            output.core_data.accuracy.semiMinor = core_data_msg.accuracy.semiMinor;
            output.core_data.transmission.transmission_state = core_data_msg.transmission;
            output.core_data.speed = core_data_msg.speed;
            output.core_data.heading = core_data_msg.heading;
            output.core_data.angle = core_data_msg.angle;
            ROS_WARN_STREAM("decoding1");

            output.core_data.accelSet.lateral = core_data_msg.accelSet.lat;
            output.core_data.accelSet.longitudinal =core_data_msg.accelSet.Long;
            output.core_data.accelSet.vert = core_data_msg.accelSet.vert;
            output.core_data.accelSet.yaw_rate = core_data_msg.accelSet.yaw;
            output.core_data.brakes.wheelBrakes.brake_applied_status = core_data_msg.brakes.wheelBrakes.buf[5];
            ROS_WARN_STREAM("orinting inside brakes");
            for (auto i = 0; i < 6; i++)
                ROS_WARN_STREAM((int)core_data_msg.brakes.wheelBrakes.buf[i]);
            output.core_data.brakes.traction.traction_control_status = core_data_msg.brakes.traction;
            output.core_data.brakes.abs.anti_lock_brake_status = core_data_msg.brakes.abs;
            output.core_data.brakes.scs.stability_control_status = core_data_msg.brakes.scs;
            output.core_data.brakes.brakeBoost.brake_boost_applied = core_data_msg.brakes.brakeBoost;
            output.core_data.brakes.auxBrakes.auxiliary_brake_status = core_data_msg.brakes.auxBrakes;            
            output.core_data.size.vehicle_length = core_data_msg.size.length;
            output.core_data.size.vehicle_width = core_data_msg.size.width;
            ROS_WARN_STREAM("decoding2");
            
            return boost::optional<j2735_msgs::BSM>(output);
        }
        ROS_WARN_STREAM("BasicSafetyMessage decoding failed");
        return boost::optional<j2735_msgs::BSM>{};

    }

    boost::optional<std::vector<uint8_t>> BSM_Message::encode_bsm_message(const j2735_msgs::BSM& plain_msg)
    {
        FILE *fp;
        fp = fopen("/home/misheel/Desktop/log_C.txt", "w");
        fprintf(fp, "encodeBSM function is called\n");
        
        //encode result placeholder
        uint8_t buffer[544];
        size_t buffer_size=sizeof(buffer);
        ROS_WARN_STREAM("size MessageFrame_t:" << sizeof(MessageFrame_t));
        asn_enc_rval_t ec;
        MessageFrame_t* message;
        message = (MessageFrame_t*) calloc(1, sizeof(MessageFrame_t));
        

        //if mem allocation fails
        if(!message)
        {
            ROS_WARN_STREAM("Cannot allocate mem for BasicSafetyMessage encoding");
            return boost::optional<std::vector<uint8_t>>{};
        }

        //set message type to BasicSafetyMessage
        message->messageId = 20;  
        message->value.present = MessageFrame__value_PR_BasicSafetyMessage;

        BasicSafetyMessage* bsm_msg;
        bsm_msg = (BasicSafetyMessage*) calloc(1, sizeof(BasicSafetyMessage));

        // Encode coreData
        BSMcoreData_t* core_data;
        core_data = (BSMcoreData_t*) calloc(1, sizeof(BSMcoreData_t));
        core_data->msgCnt = plain_msg.core_data.msg_count;
        //ROS_WARN_STREAM("size msgCnt" << sizeof(core_data->msgCnt));
        ROS_WARN_STREAM("size msgCnt DSRC_MsgCount_t:" << sizeof(DSRC_MsgCount_t));
        ROS_WARN_STREAM("size msgCnt long:" << sizeof(long));
        ROS_WARN_STREAM("size TemporaryID_t:" << sizeof(TemporaryID_t));
        ROS_WARN_STREAM("size DSecond_t:" << sizeof(DSecond_t));
        ROS_WARN_STREAM("size Latitude_t:" << sizeof(Latitude_t));
        ROS_WARN_STREAM("size Longitude_t:" << sizeof(Longitude_t));
        ROS_WARN_STREAM("size DSRC_Elevation_t:" << sizeof(DSRC_Elevation_t));
        ROS_WARN_STREAM("size PositionalAccuracy_t:" << sizeof(PositionalAccuracy_t));
        ROS_WARN_STREAM("size TransmissionState_t:" << sizeof(TransmissionState_t));
        ROS_WARN_STREAM("size Speed_t:" << sizeof(Speed_t));
        ROS_WARN_STREAM("size Heading_t:" << sizeof(Heading_t));
        ROS_WARN_STREAM("size SteeringWheelAngle_t:" << sizeof(SteeringWheelAngle_t));
        ROS_WARN_STREAM("size AccelerationSet4Way_t:" << sizeof(AccelerationSet4Way_t));
        ROS_WARN_STREAM("size BrakeSystemStatus_t:" << sizeof(BrakeSystemStatus_t));
        ROS_WARN_STREAM("size VehicleSize_t:" << sizeof(VehicleSize_t));
        ROS_WARN_STREAM("size VehicleWidth_t:" << sizeof(VehicleWidth_t));
        ROS_WARN_STREAM("size VehicleLength_t:" << sizeof(VehicleLength_t));
        ROS_WARN_STREAM("size BrakeAppliedStatus_t:" << sizeof(BrakeAppliedStatus_t));
        ROS_WARN_STREAM("size TractionControlStatus_t:" << sizeof(TractionControlStatus_t));
        ROS_WARN_STREAM("size AntiLockBrakeStatus_t:" << sizeof(AntiLockBrakeStatus_t));
        ROS_WARN_STREAM("size StabilityControlStatus_t:" << sizeof(StabilityControlStatus_t));
        ROS_WARN_STREAM("size BrakeBoostApplied_t:" << sizeof(BrakeBoostApplied_t));
        ROS_WARN_STREAM("size AuxiliaryBrakeStatus_t:" << sizeof(AuxiliaryBrakeStatus_t));
        ROS_WARN_STREAM("size Acceleration_t:" << sizeof(Acceleration_t));
        ROS_WARN_STREAM("size Acceleration_t:" << sizeof(Acceleration_t));
        ROS_WARN_STREAM("size VerticalAcceleration_t:" << sizeof(VerticalAcceleration_t));
        ROS_WARN_STREAM("size YawRate_t:" << sizeof(YawRate_t));
        ROS_WARN_STREAM("size SemiMajorAxisAccuracy_t:" << sizeof(SemiMajorAxisAccuracy_t));
        ROS_WARN_STREAM("size SemiMinorAxisAccuracy_t:" << sizeof(SemiMinorAxisAccuracy_t));
        ROS_WARN_STREAM("size SemiMajorAxisOrientation_t:" << sizeof(SemiMajorAxisOrientation_t));
        //Set the fields
        uint8_t id_content[4] = {0};
        for(auto i = 0; i < 4; i++)
        {
            id_content[i] = (char) plain_msg.core_data.id[i];
        }
        ROS_WARN_STREAM("CHECKPOINT 1");
        TemporaryID_t* temp_id;
        temp_id = (TemporaryID_t*) calloc(1, sizeof(TemporaryID_t));
        ROS_WARN_STREAM("CHECKPOINT 2");
        temp_id->buf = id_content; 
        temp_id->size = 4;
        core_data->id = *temp_id;
        core_data->secMark = plain_msg.core_data.sec_mark;

        core_data->lat = plain_msg.core_data.latitude;
        core_data->Long = plain_msg.core_data.longitude;
        core_data->elev = plain_msg.core_data.elev;
        PositionalAccuracy_t* pos_acc;
        pos_acc = (PositionalAccuracy_t*) calloc(1, sizeof(PositionalAccuracy_t));
        pos_acc->orientation = plain_msg.core_data.accuracy.orientation;
        pos_acc->semiMajor = plain_msg.core_data.accuracy.semiMajor;
        pos_acc->semiMinor = plain_msg.core_data.accuracy.semiMinor;
        core_data->accuracy = *pos_acc;
        core_data->transmission = plain_msg.core_data.transmission.transmission_state;
        core_data->speed = plain_msg.core_data.speed;
        core_data->heading = plain_msg.core_data.heading;
        core_data->angle = plain_msg.core_data.angle;
        AccelerationSet4Way_t* accel;
        accel = (AccelerationSet4Way_t*) calloc(1, sizeof(AccelerationSet4Way_t));
        accel->lat = plain_msg.core_data.accelSet.lateral;
        accel->Long = plain_msg.core_data.accelSet.longitudinal;
        accel->vert= plain_msg.core_data.accelSet.vert;
        accel->yaw = plain_msg.core_data.accelSet.yaw_rate;
        core_data->accelSet = *accel;

        VehicleSize_t* vehicle_size;
        vehicle_size = (VehicleSize_t*) calloc(1, sizeof(VehicleSize_t));
        vehicle_size->length = plain_msg.core_data.size.vehicle_length;
        vehicle_size->width = plain_msg.core_data.size.vehicle_width;
        core_data->size = *vehicle_size;


        // DEBUG
        BrakeSystemStatus_t* brakes;
        brakes = (BrakeSystemStatus_t*) calloc(1, sizeof(BrakeSystemStatus_t));
    
        brakes->traction = plain_msg.core_data.brakes.traction.traction_control_status;
        brakes->abs = plain_msg.core_data.brakes.abs.anti_lock_brake_status;
        brakes->scs = plain_msg.core_data.brakes.scs.stability_control_status;
        brakes->brakeBoost = plain_msg.core_data.brakes.brakeBoost.brake_boost_applied;
        brakes->auxBrakes = plain_msg.core_data.brakes.auxBrakes.auxiliary_brake_status;
        
        // Buggy part starting
        BrakeAppliedStatus_t* brake_applied_status;
        brake_applied_status = (BrakeAppliedStatus_t*) calloc(1, sizeof(BrakeAppliedStatus_t));
        uint8_t* wheel_brake;
        wheel_brake = (uint8_t*) calloc(1, sizeof(uint8_t)); //size 1
        // brakes->wheelBrakes = encode_wheel_brakes(plain_msg.core_data.brakes.wheelBrakes);
        // for now hardcode it
        wheel_brake[0] = 1;
        brake_applied_status->buf = wheel_brake;
        brake_applied_status->size = 1;
        //brake_applied_status->bits_unused = 3; // java code does this, but not sure

        ROS_WARN_STREAM("CHECKPOINT 3");
        // = encode_wheel_brakes(plain_msg.core_data.brakes.wheelBrakes).buf;
        //brake_applied_status->size = encode_wheel_brakes(plain_msg.core_data.brakes.wheelBrakes).size;
        //brake_applied_status->bits_unused = encode_wheel_brakes(plain_msg.core_data.brakes.wheelBrakes).bits_unused;
        //brake_applied_status->_asn_ctx = encode_wheel_brakes(plain_msg.core_data.brakes.wheelBrakes)._asn_ctx;

        
        ROS_WARN_STREAM("Check if the element is correctly set before encoding");
        for (auto i = 0; i < 1; i++)
            ROS_WARN_STREAM((int)brake_applied_status->buf[i]);
        
        brakes->wheelBrakes = *brake_applied_status;
        core_data->brakes = *brakes;

        bsm_msg->coreData = *core_data;
        message->value.choice.BasicSafetyMessage = *bsm_msg;
        ROS_WARN_STREAM("Check again if the element is correctly set before encoding");
        for (auto i = 0; i < 1; i++)
            ROS_WARN_STREAM((int)message->value.choice.BasicSafetyMessage.coreData.brakes.wheelBrakes.buf[i]);
        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
        asn_fprint(fp, &asn_DEF_MessageFrame, message);
        //log a warning if that fails
        if(ec.encoded == -1) {
            ROS_WARN_STREAM("Encoding for BasicSafetyMessage has failed");
            std::cout << "Failed: " << ec.failed_type->name << std::endl;
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //copy to byte array msg
        size_t array_length=ec.encoded / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
                
        //Debugging/Unit Testing
        for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }


} 
