/*
 * Copyright (C) 2021 LEIDOS.
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
 * CPP File containing SPAT Message method implementations
 */

#include "SPAT_Message.h"

namespace cpp_message
{
    //Convert the SPAT j2735 message to cav_msgs
    boost::optional<j2735_msgs::SPAT> SPAT_Message::decode_spat_message(std::vector<uint8_t>& binary_array){
        //Decode the binary message into SPAT message
        j2735_msgs::SPAT output;
        //decode results - stored in binary array
        asn_dec_rval_t rval;
        MessageFrame_t* message = nullptr;

        //copy from vector to array
        size_t len = binary_array.size();

        uint8_t buf[len];
        std::copy(binary_array.begin(), binary_array.end(), buf);
        //use asn1c lib to decode

        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **) &message, buf, len, 0, 0);

        //if decode success
        if(rval.code == RC_OK){

        
        //1. Decode time stamp - Minute of the year
        uint64_t minute_of_the_year;
        bool time_stamp_exists = message->value.choice.SPAT.time_stamp_exists;
        if(time_stamp_exists){
            minute_of_the_year = message->value.choice.SPAT.time_span;
        }
        else{
            minute_of_the_year = DEFAULT_MINUTE_OF_YEAR_;
        }
        
        output.time_span_exists = time_stamp_exists;
        output.time_span = minute_of_the_year;
    
        //2. Decode name
        std::string name;
        size_t str_len = message->value.choice.SPAT.name.size();
        for(size_t i = 0; i< str_len; i++){
            name +=message->value.choice.SPAT.name.buf[i];
        }
        output.name = name;

        //3. Decode Intersection

           
            
        }

        boost::optional<j2735_msgs::SPAT> random_message;
        return random_message;
    }

    boost::optional<std::vector<uint8_t>> SPAT_Message::encode_spat_message(const j2735_msgs::SPAT& plainMessage){


        boost::optional<std::vector<uint8_t>> random_message;
        return random_message;
    }

}