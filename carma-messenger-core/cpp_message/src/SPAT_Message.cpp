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
        long int* minute_of_the_year = new long int;
        bool is_time_stamp_exists;
        if(message->value.choice.SPAT.timeStamp){
            is_time_stamp_exists = true;
            minute_of_the_year = message->value.choice.SPAT.timeStamp;
        }
        else{
            is_time_stamp_exists = false;
            *minute_of_the_year = DEFAULT_MINUTE_OF_YEAR_;
        }
        
        output.time_stamp_exists = is_time_stamp_exists;
        output.time_stamp = *minute_of_the_year;

        //2. Decode name
        std::string name;
        size_t str_len = message->value.choice.SPAT.name->size;
        for(size_t i = 0; i< str_len; i++){
            name +=message->value.choice.SPAT.name->buf[i];
        }
        output.name = name;

        //3. Decode Intersection
        j2735_msgs::IntersectionStateList intersections;

        //IntersectionStateList is an array of intersectionState - of size 1-32
        for(size_t i = 0; i < message->value.choice.SPAT.intersections.list.size; i++){
            j2735_msgs::IntersectionState intersection;
            IntersectionState_t* state = new IntersectionState_t;
            state = message->value.choice.SPAT.intersections.list.array[i];
            //encode intersection name
            if(state->name){
                intersection.name_exists = true;

                for(size_t j = 0; j < state->name->size; j++){
                    intersection.name += state->name->buf[i];
                }
            }
            else{
                intersection.name_exists = false;
                intersection.name = DEFAULT_STRING_;
            }

            //Encode id
            j2735_msgs::IntersectionReferenceID id;
            id.id = state->id.id;
            if(state->id.region){
                id.region_exists = true;
                id.region = *state->id.region;
            }
            else{
                id.region_exists = false;
                id.region = j2735_msgs::IntersectionReferenceID::REGION_UNAVAILABLE;
            }
           
            intersection.id = id;

            //Encode Msg count
            intersection.revision = state->revision;
            

            //Encode status
            //Convert IA5String to uint16
            
            //intersection.status.intersection_status_object = state->status;

            //Minute of the year
            
            bool moy_exists = false;
            if(state->moy){
                MinuteOfTheYear_t* moy = new MinuteOfTheYear_t;
                moy = state->moy;
                intersection.moy = *moy;
                intersection.moy_exists = true;
            }
            else{
                intersection.moy_exists = false;
                intersection.moy = j2735_msgs::IntersectionState::MOY_INVALID;
            }


            //Time stamp
            if(state->timeStamp){
                DSecond_t* time_stamp = new DSecond_t;
                time_stamp = state->timeStamp;
                intersection.time_stamp = *time_stamp;
                intersection.time_stamp_exists = true;
                
            }
            else{
                intersection.time_stamp = j2735_msgs::IntersectionState::TIME_STAMP_UNAVAILABLE;
                intersection.time_stamp_exists = false;
            }

            //Enabled lanes list
            j2735_msgs::EnabledLaneList enabled_lanes_list;
            bool enabled_lanes_exists = false;
            if(!state->enabledLanes->list.size > 0){
                enabled_lanes_exists = true;
                for(size_t j = 0;j < state->enabledLanes->list.size ; j++){
                    LaneID_t* enabled_lane_id = new LaneID_t;
                    enabled_lane_id = state->enabledLanes->list.array[j];
                    enabled_lanes_list.lane_id_list.push_back(*enabled_lane_id);
                }
            }

            intersection.enabled_lanes_exists = enabled_lanes_exists;
            intersection.enabled_lanes = enabled_lanes_list;
            
            //MovementList 
            j2735_msgs::MovementList movement_states;
            //movement states is an array of Movement states
            for(size_t j = 0;j < state->states.list.size; j++){
                j2735_msgs::MovementState movement_state;
                movement_state.movement_name_exists = false;
                if(state->states.list.array[j]->movementName){
                    movement_state.movement_name_exists = true;
                    //movement_state.movement_name = state->states.list.array[j]->movementName;
                    size_t len = state->states.list.array[j]->movementName->size;
                    for( int k = 0 ; k < len ; k++){
                        movement_state.movement_name += state->states.list.array[j]->movementName->buf[k];
                    }
                }
                
                //Signal Group ID
                movement_state.signal_group = SIGNAL_GROUP_UNAVAILBALE_;
                if(state->states.list.array[j]->signalGroup){
                    movement_state.signal_group = state->states.list.array[j]->signalGroup;
                }
                
                //State Time Speed Movement Event List
                j2735_msgs::MovementEventList movement_event_list;
                for(int k = 0; k<state->states.list.array[j]->state_time_speed.list.size ; k++){
                    j2735_msgs::MovementEvent movement_event;
                    //movement_event.event_state = state->states.list.array[j]->state_time_speed.list.array[j]->eventState;
                    //Decode movement event
                    //1. MovementPhaseState
                    movement_event.event_state.movement_phase_state =  state->states.list.array[j]->state_time_speed.list.array[k]->eventState;

                    //2. TimeChangeDetails
                    movement_event.timing_exists = false;
                    if(state->states.list.array[j]->state_time_speed.list.array[k]->timing){
                        movement_event.timing_exists = true;

                        j2735_msgs::TimeChangeDetails timing;
                        timing.start_time_exists = false;
                        if(state->states.list.array[j]->state_time_speed.list.array[k]->timing->startTime){
                            timing.start_time_exists = true;
                            DSRC_TimeMark_t* start_time = new DSRC_TimeMark_t;
                            start_time = state->states.list.array[j]->state_time_speed.list.array[k]->timing->startTime;
                            timing.start_time = *start_time;
                        }
                        
                        timing.min_end_time =  state->states.list.array[j]->state_time_speed.list.array[k]->timing->minEndTime;
                        
                        timing.max_end_time_exists = false;
                        if(state->states.list.array[j]->state_time_speed.list.array[k]->timing->maxEndTime){
                            timing.max_end_time_exists = true;
                            DSRC_TimeMark_t* end_time = new DSRC_TimeMark_t;
                            end_time = state->states.list.array[j]->state_time_speed.list.array[k]->timing->maxEndTime; 
                            timing.max_end_time = *end_time;
                        }

                        timing.likely_time_exists = false;
                        if (state->states.list.array[j]->state_time_speed.list.array[k]->timing->likelyTime){
                            timing.likely_time_exists = true;
                            DSRC_TimeMark_t* likely_time = new DSRC_TimeMark_t;
                            likely_time = state->states.list.array[j]->state_time_speed.list.array[k]->timing->likelyTime;
                            timing.likely_time = *likely_time;
                        }

                        timing.confidence_exists = false;
                        if(state->states.list.array[j]->state_time_speed.list.array[j]->timing->confidence){
                            timing.confidence_exists = true;
                            TimeIntervalConfidence_t* confidence = new TimeIntervalConfidence_t;
                            confidence = state->states.list.array[j]->state_time_speed.list.array[k]->timing->confidence;
                            timing.confidence = *confidence;
                        }

                        timing.next_time_exists = false;
                        if(state->states.list.array[j]->state_time_speed.list.array[k]->timing->nextTime){
                            timing.next_time_exists = true;
                            DSRC_TimeMark_t* next_time = new DSRC_TimeMark_t;
                            next_time = state->states.list.array[j]->state_time_speed.list.array[k]->timing->nextTime;
                            timing.next_time = *next_time;
                        }

                        movement_event.timing = timing;
                    }      

                    //3. Advisory Speed List
                    movement_event.speeds_exists = false;
                    if(state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.size > 0){
                        movement_event.speeds_exists = true;
                        j2735_msgs::AdvisorySpeedList advisory_speed_list;
                        for(size_t l = 0; l < state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.size; l++){
                            j2735_msgs::AdvisorySpeed advisory_speed;

                            if(state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->type){
                                advisory_speed.type.advisory_speed_type = state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->type;
                            }
                            else{
                                advisory_speed.type.advisory_speed_type = ADVISORY_SPEED_TYPE_NONE_;
                            }

                            advisory_speed.speed_exists = false;
                            if(state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->speed){
                                advisory_speed.speed_exists = true; 
                                SpeedAdvice_t* speed_advice = new SpeedAdvice_t;
                                speed_advice = state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->speed;
                                advisory_speed.speed = *speed_advice;
                            }
                            else{
                                advisory_speed.speed = j2735_msgs::AdvisorySpeed::SPEED_UNAVAILABLE;
                            }
                            
                            SpeedConfidence_t* confidence = new SpeedConfidence_t;
                            confidence = state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->confidence;
                            advisory_speed.confidence.speed_confidence = *confidence;

                            advisory_speed.distance_exists = false;
                            if(state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->distance){
                                advisory_speed.distance_exists = true;
                                ZoneLength_t* distance = new ZoneLength_t;
                                distance = state->states.list.array[j]->state_time_speed.list.array[k]->speeds->list.array[l]->distance;
                                advisory_speed.distance = *distance;
                            }
                            else{
                                advisory_speed.distance = j2735_msgs::AdvisorySpeed::DISTANCE_UNKNOWN;
                            }

                            // RESTRICTION CLASS ID not DEFINED in incoming state message

                            

                            movement_event.speeds.advisory_speed_list.push_back(advisory_speed);
                        }

                    }

                    movement_event_list.movement_event_list.push_back(movement_event);
                    
                }
                movement_state.state_time_speed.movement_event_list = movement_event_list.movement_event_list;
                
            }

            intersection.states = movement_states;

            //ManeuverAssistList
            j2735_msgs::ManeuverAssistList maneuver_assist_list;
            bool maneuver_assist_list_exists = false;
            if(state->maneuverAssistList->list.size > 0){
                maneuver_assist_list_exists = true;
                //maneuver_assist_list.connection_maneuver_assist_list.push_back(state->maneuverAssistList->list.array[i]);
            }

            intersection.maneuever_assist_list_exists = maneuver_assist_list_exists;
            intersection.maneuever_assist_list = maneuver_assist_list;

        }

        output.intersections = intersections;
        

        //4. Regional - not implemented yet
            
        }

        boost::optional<j2735_msgs::SPAT> random_message;
        return random_message;
    }

    boost::optional<std::vector<uint8_t>> SPAT_Message::encode_spat_message(const j2735_msgs::SPAT& plainMessage){


        boost::optional<std::vector<uint8_t>> random_message;
        return random_message;
    }

}