/*
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
            BSMcoreData_t core_data_msg = message->value.choice.BasicSafetyMessage.coreData;
            output.core_data.msg_count = core_data_msg.msgCnt; 
            auto id_len = core_data_msg.id.size;
            for(auto i = 0; i < id_len; i++)
            {
                output.core_data.id.push_back(core_data_msg.id.buf[i]);
            }
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
            output.core_data.accelSet.lateral = core_data_msg.accelSet.lat;
            output.core_data.accelSet.longitudinal =core_data_msg.accelSet.Long;
            output.core_data.accelSet.vert = core_data_msg.accelSet.vert;
            output.core_data.accelSet.yaw_rate = core_data_msg.accelSet.yaw;
            // brake_applied_status decoding
            // e.g. make 0b0100000 to 0b0000100
            uint8_t binary = core_data_msg.brakes.wheelBrakes.buf[0] >> 3;
            unsigned int brake_applied_status_type = 4;
            // e.g. shift the binary right until it equals to 1 (0b00000001) to determine the location of the non-zero bit
            
            for (int i = 0; i < 4; i ++)
            {
                if ((int)binary == 1) 
                {
                    output.core_data.brakes.wheelBrakes.brake_applied_status = brake_applied_status_type;
                    break;
                }
                else
                {
                    brake_applied_status_type -= 1;
                    binary = binary >> 1;
                }
            }
            output.core_data.brakes.traction.traction_control_status = core_data_msg.brakes.traction;
            output.core_data.brakes.abs.anti_lock_brake_status = core_data_msg.brakes.abs;
            output.core_data.brakes.scs.stability_control_status = core_data_msg.brakes.scs;
            output.core_data.brakes.brakeBoost.brake_boost_applied = core_data_msg.brakes.brakeBoost;
            output.core_data.brakes.auxBrakes.auxiliary_brake_status = core_data_msg.brakes.auxBrakes;            
            output.core_data.size.vehicle_length = core_data_msg.size.length;
            output.core_data.size.vehicle_width = core_data_msg.size.width;
            
            return boost::optional<j2735_msgs::BSM>(output);
        }
        ROS_WARN_STREAM("BasicSafetyMessage decoding failed");
        return boost::optional<j2735_msgs::BSM>{};

    }

    boost::optional<std::vector<uint8_t>> BSM_Message::encode_bsm_message(const j2735_msgs::BSM& plain_msg)
    {
        //Uncomment below (and one line at the end of the function) to print the message in human readable form
        //FILE *fp;
        //fp = fopen("/absolute/directory/log_C.txt", "w");
        //fprintf(fp, "encodeBSM function is called\n");
        
        //encode result placeholder
        uint8_t buffer[544];
        size_t buffer_size=sizeof(buffer);
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
        //Set the fields
        uint8_t id_content[4] = {0};
        for(auto i = 0; i < 4; i++)
        {
            id_content[i] = (char) plain_msg.core_data.id[i];
        }
        TemporaryID_t* temp_id;
        temp_id = (TemporaryID_t*) calloc(1, sizeof(TemporaryID_t));
        temp_id->buf = id_content; 
        temp_id->size = 4;
        core_data->id = *temp_id;
        free(temp_id);
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
        free(pos_acc);
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
        free(accel);
        VehicleSize_t* vehicle_size;
        vehicle_size = (VehicleSize_t*) calloc(1, sizeof(VehicleSize_t));
        vehicle_size->length = plain_msg.core_data.size.vehicle_length;
        vehicle_size->width = plain_msg.core_data.size.vehicle_width;
        core_data->size = *vehicle_size;
        free(vehicle_size);
        BrakeSystemStatus_t* brakes;
        brakes = (BrakeSystemStatus_t*) calloc(1, sizeof(BrakeSystemStatus_t));
    
        brakes->traction = plain_msg.core_data.brakes.traction.traction_control_status;
        brakes->abs = plain_msg.core_data.brakes.abs.anti_lock_brake_status;
        brakes->scs = plain_msg.core_data.brakes.scs.stability_control_status;
        brakes->brakeBoost = plain_msg.core_data.brakes.brakeBoost.brake_boost_applied;
        brakes->auxBrakes = plain_msg.core_data.brakes.auxBrakes.auxiliary_brake_status;
        
        BrakeAppliedStatus_t* brake_applied_status;
        brake_applied_status = (BrakeAppliedStatus_t*) calloc(1, sizeof(BrakeAppliedStatus_t));
        
        uint8_t wheel_brake[1] = {8}; // dummy 8 value

        // there are 3 unused bits in the end: 0b000
        // which makes every possible encoded value to be multiples of 8: 0b00001000 (8), 0b00010000 (16), 0b00011000 (24) etc
        // so num in brackets indicate the position in the bit string:
        // unavailable: 0b10000000, leftFront: 0b01000000 etc
        wheel_brake[0] = (char) (8 << (4 - plain_msg.core_data.brakes.wheelBrakes.brake_applied_status)); 
        brake_applied_status->buf = wheel_brake;
        brake_applied_status->size = 1;
        brake_applied_status->bits_unused = 3;
        brakes->wheelBrakes = *brake_applied_status;
        
        core_data->brakes = *brakes;
        free(brakes);
        bsm_msg->coreData = *core_data;
        free(core_data);
        message->value.choice.BasicSafetyMessage = *bsm_msg;
        free(bsm_msg);
        //encode message
        ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
        // Uncomment below to enable logging in human readable form
        //asn_fprint(fp, &asn_DEF_MessageFrame, message);
        free(message);
        
        //log a warning if that fails
        if(ec.encoded == -1) {
            ROS_WARN_STREAM("Encoding for BasicSafetyMessage has failed");
            std::cout << "Failed: " << ec.failed_type->name << std::endl;
            return boost::optional<std::vector<uint8_t>>{};
        }
        
        //copy to byte array msg
        size_t array_length=(ec.encoded + 7) / 8;
        std::vector<uint8_t> b_array(array_length);
        for(size_t i=0;i<array_length;i++)b_array[i]=buffer[i];
                
        //Debugging/Unit Testing
        //for(size_t i = 0; i < array_length; i++) std::cout<< int(b_array[i])<< ", ";
        return boost::optional<std::vector<uint8_t>>(b_array);
    }


} 
