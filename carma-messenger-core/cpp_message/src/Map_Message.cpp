
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

#include "Map_Message.h"

namespace cpp_message
{
    boost::optional<j2735_msgs::MapData> Map_Message::decode_map_message(std::vector<uint8_t>& binary_array)
    {
        j2735_msgs::MapData output;

        // decode results
        asn_dec_rval_t rval;
        MessageFrame_t* message = 0;
        
         //copy from vector to array         
        size_t len=binary_array.size();    
        
        uint8_t buf[len];             
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
        if(rval.code == RC_OK)
        {


            auto map_msg = message->value.choice.MapData;

            IntersectionGeometry_t *map_msg_intersections = new IntersectionGeometry_t;


            //Intersection Handling
            if(map_msg.intersections)
             {   

                output.intersections_exists = true;
                auto v = map_msg.intersections;
                //Map Intersections
                for(size_t i = 0; i < map_msg.intersections->list.count; i++)
                {
                    map_msg_intersections = message->value.choice.MapData.intersections->list.array[i];
                    j2735_msgs::IntersectionGeometry new_intersection;
                    new_intersection.id.id = map_msg_intersections->id.id;
                    new_intersection.id.region_exists = true;

                    //Lane Set 
                    for(size_t l = 0; l < 0; l++)
                    {

                        GenericLane_t *lane = new GenericLane_t;

                        lane = map_msg_intersections->laneSet.list.array[l];
                        j2735_msgs::GenericLane ln;

                        if(lane->egressApproach)
                        {
                            ln.egress_approach_exists = true;
                            ln.egress_approach = *lane->egressApproach;
                        }
                        else
                        {
                            ln.egress_approach_exists = false;
                        }

                        if(lane->ingressApproach)
                        {
                            ln.ingress_approach_exists = true;
                            ln.ingress_approach = *lane->ingressApproach;
                        }
                        else
                        {
                            ln.ingress_approach_exists = false;
                        }

                        if(lane->maneuvers)
                        {
                            ln.maneuvers_exists = true;
                            ln.maneuvers.allowed_maneuvers = *lane->maneuvers->buf;
                        }
                        else
                        {
                            ln.maneuvers_exists = false;
                        }

                        if(lane->connectsTo)
                        {
                            ln.connects_to_exists = true;
                            Connection_t *ct = new Connection_t;

                            for(size_t c = 0; c< lane->connectsTo->list.size; c++)
                            {
                                ct = lane->connectsTo->list.array[c];

                                j2735_msgs::Connection entry;

                                //Connection ID
                                if(ct->connectionID)
                                {
                                    entry.connection_id_exists = true;
                                    entry.connection_id = *ct->connectionID;
                                }
                                else
                                {
                                    entry.connection_id_exists = false;
                                }

                                //Connecting Lane
                                entry.connecting_lane.lane = ct->connectingLane.lane;
                                if(ct->connectingLane.maneuver)
                                {
                                    entry.connecting_lane.maneuver_exists = true;
                                    entry.connecting_lane.maneuver.allowed_maneuvers = *ct->connectingLane.maneuver->buf;
                                }
                                else
                                {
                                    entry.connecting_lane.maneuver_exists=false;
                                }
                                
                                //Remote Intersection
                                if(ct->remoteIntersection)
                                {
                                    entry.remote_intersection_exists = true;
                                    
                                    //Remote Intersection Region
                                    if(ct->remoteIntersection->region)
                                    {
                                        entry.remote_intersection.region_exists = true;
                                        entry.remote_intersection.region = *ct->remoteIntersection->region;
                                    }
                                    else
                                    {
                                        entry.remote_intersection.region_exists = false;
                                    }

                                    entry.remote_intersection.id = ct->remoteIntersection->id;

                                }
                                else
                                {
                                    entry.remote_intersection_exists = false;
                                }

                                //Signal Group
                                if(ct->signalGroup)
                                {
                                    entry.signal_group_exists = true;
                                    entry.signal_group = *ct->signalGroup;
                                }
                                else
                                {
                                    entry.signal_group_exists = false;
                                }

                                //User Class
                                if (ct->userClass)
                                {
                                    entry.user_class_exists = true;
                                    entry.user_class = *ct->userClass;
                                }

                                ln.connects_to.connect_to_list.push_back(entry);

                            }

                        }
                        else
                        {
                            ln.egress_approach_exists = false;
                        }

                        new_intersection.lane_set.lane_list.push_back(ln);
                    }

                    if(map_msg_intersections->name)
                    {
                        new_intersection.name_exists=true;
                        for (size_t n = 0 ; n < map_msg_intersections->name->size; n++)
                        {
                            new_intersection.name.push_back(n);
                        }
                    }
                    else
                    {
                        new_intersection.name_exists = false;
                    }

                    if(map_msg_intersections->laneWidth)
                    {
                        new_intersection.lane_width_exists = true;
                        auto width = map_msg_intersections->laneWidth;
                        new_intersection.lane_width = *width;
                    }
                    else
                    {
                        new_intersection.lane_width_exists = false;
                    }        

                    if(map_msg_intersections->preemptPriorityData)
                    {
                        new_intersection.preempt_priority_data_exists = true;

                        for(size_t p = 0; i<map_msg_intersections->preemptPriorityData->list.size; p++)
                        {
                            SignalControlZone_t *sig = new SignalControlZone_t;
                            sig = map_msg_intersections->preemptPriorityData->list.array[p];
                            
                            j2735_msgs::SignalControlZone s;

                            /*SignalControlZone.msg states that RegionalExtension has not yet been implemented*/

                            new_intersection.preempt_priority_data.preempt_priority_list.push_back(s);
                        }
                        
                    }
                    else
                    {
                        new_intersection.preempt_priority_data_exists = false;
                    }

                    if(map_msg_intersections->speedLimits)
                    {
                        new_intersection.speed_limits_exists = true;
                        
                        for(size_t s = 0; s < map_msg_intersections->speedLimits->list.size ; s++)
                        {
                            RegulatorySpeedLimit_t *speedLimit = new RegulatorySpeedLimit_t;

                            speedLimit = map_msg_intersections->speedLimits->list.array[s];

                            j2735_msgs::RegulatorySpeedLimit rsl;
                            rsl.speed = speedLimit->speed;
                            rsl.type.speed_limit_type = speedLimit->type;
                            new_intersection.speed_limits.speed_limits.push_back(rsl);
                        }
                    }
                    else
                    {
                        new_intersection.speed_limits_exists = false;
                    }

                    if(map_msg_intersections->refPoint.elevation) 
                    {
                        new_intersection.ref_point.elevation_exists = true;
                        new_intersection.ref_point.latitude = map_msg_intersections->refPoint.lat;
                        new_intersection.ref_point.longitude = map_msg_intersections->refPoint.Long;


                        DSRC_Elevation_t *dsrc_el = new DSRC_Elevation_t;
                        dsrc_el = map_msg_intersections->refPoint.elevation;

                        new_intersection.ref_point.elevation = *dsrc_el;
                    }
                    else
                    {
                        new_intersection.ref_point.elevation_exists = false;
                    }

                    new_intersection.revision = map_msg_intersections->revision;
                

                    output.intersections.push_back(new_intersection);
                }

            }//end intersection handling
            else
            {
                output.intersections_exists = false;
            }


            output.layer_type.layer_type = *map_msg.layerType;

            //Layer ID
            if(map_msg.layerID)
            {
                output.layer_id_exists = true;
                output.layer_id = *map_msg.layerID;

            }//end layer ID
            else
            {
                output.layer_id_exists = false;
            }

            //Restriction List
            if(map_msg.restrictionList)
            {
                output.restriction_list_exists = true;

                RestrictionClassAssignment_t *rca = new RestrictionClassAssignment_t;

                for(size_t i = 0; i < map_msg.restrictionList->list.size; i++)
                {
                    rca = map_msg.restrictionList->list.array[i];

                    j2735_msgs::RestrictionClassAssignment rclass;
                    rclass.id = rca->id;
                    output.restriction_list.restriction_class_list.push_back(rclass);
                }
            }//end Restriction List
            else
            {
                output.restriction_list_exists = false;
            }


            //Road Segment
            if(map_msg.roadSegments)
            {

                output.road_segments_exists = true;

                RoadSegment_t *rseg = new RoadSegment_t;

                for(size_t i = 0; i< map_msg.roadSegments->list.size;i++)
                {
                    rseg = map_msg.roadSegments->list.array[i];

                    j2735_msgs::RoadSegment rs;

                    if(rseg->laneWidth)
                    {
                        rs.lane_width_exists = true;
                        rs.lane_width = *rseg->laneWidth;
                    }
                    else
                    {
                        rs.lane_width_exists = false;
                    }

                    //Road Segment Name
                    if(rseg->name)
                    {
                        rs.name_exists = true;
                        
                        DescriptiveName_t *nm = new DescriptiveName_t;

                        nm = rseg->name;
                        std::string n;
                        
                        for (size_t i = 0; i < nm->size;i++)
                        {
                            n.push_back(nm->buf[i]);
                        }
                        rs.name = n;
                    }
                    else
                    {
                        rs.name_exists = false;
                    }

                    if (rseg->laneWidth)
                    {
                        rs.lane_width_exists = true;

                        rs.lane_width = *rseg->laneWidth;
                    }
                    else
                    {
                        rs.lane_width_exists = false;
                    }

                    if(rseg->speedLimits)
                    {
                        rs.speed_limits_exists = true;

                        RegulatorySpeedLimit_t *speedL = new RegulatorySpeedLimit_t;

                        for (size_t i = 0; i < rseg->speedLimits->list.size;i++)
                        {
                            j2735_msgs::RegulatorySpeedLimit sl;

                            speedL = rseg->speedLimits->list.array[i];

                            sl.speed = speedL->speed;
                            sl.type.speed_limit_type = speedL->type;
                            rs.speed_limits.speed_limits.push_back(sl);
                        }
                    }//end speed limits
                    else
                    {
                        rs.speed_limits_exists = false;
                    }

                    if(rseg->refPoint.elevation)
                    {
                        rs.ref_point.elevation_exists = true;
                        rs.ref_point.elevation = *rseg->refPoint.elevation;
                        rs.ref_point.latitude = rseg->refPoint.lat;
                        rs.ref_point.longitude = rseg->refPoint.Long;
                    }
                    else
                    {
                        rs.ref_point.elevation_exists = false;
                    }

                    //RLS
                    GenericLane_t *g_lane = new GenericLane_t;
                    for(size_t i =0; i < rseg->roadLaneSet.list.size;i++)
                    {
                        g_lane = rseg->roadLaneSet.list.array[i];
                        j2735_msgs::GenericLane gl;
                        ////////
                        if(g_lane->egressApproach)
                        {
                            gl.egress_approach_exists = true;
                            gl.egress_approach = *g_lane->egressApproach;
                        }
                        else
                        {
                            gl.egress_approach_exists = false;
                        }

                        if(g_lane->ingressApproach)
                        {
                            gl.ingress_approach_exists = true;
                            gl.ingress_approach = *g_lane->ingressApproach;
                        }
                        else
                        {
                            gl.ingress_approach_exists = false;
                        }

                        if(g_lane->maneuvers)
                        {
                            gl.maneuvers_exists = true;
                            gl.maneuvers.allowed_maneuvers = *g_lane->maneuvers->buf;
                        }
                        else
                        {
                            gl.maneuvers_exists = false;
                        }

                        if(g_lane->connectsTo)
                        {
                            gl.connects_to_exists = true;
                            Connection_t *ct = new Connection_t;

                            for(size_t c = 0; c< g_lane->connectsTo->list.size; c++)
                            {
                                ct = g_lane->connectsTo->list.array[c];

                                j2735_msgs::Connection entry;

                                //Connection ID
                                if(ct->connectionID)
                                {
                                    entry.connection_id_exists = true;
                                    entry.connection_id = *ct->connectionID;
                                }
                                else
                                {
                                    entry.connection_id_exists = false;
                                }

                                //Connecting Lane
                                entry.connecting_lane.lane = ct->connectingLane.lane;
                                if(ct->connectingLane.maneuver)
                                {
                                    entry.connecting_lane.maneuver_exists = true;
                                    entry.connecting_lane.maneuver.allowed_maneuvers = *ct->connectingLane.maneuver->buf;
                                }
                                else
                                {
                                    entry.connecting_lane.maneuver_exists=false;
                                }
                                
                                //Remote Intersection
                                if(ct->remoteIntersection)
                                {
                                    entry.remote_intersection_exists = true;
                                    
                                    //Remote Intersection Region
                                    if(ct->remoteIntersection->region)
                                    {
                                        entry.remote_intersection.region_exists = true;
                                        entry.remote_intersection.region = *ct->remoteIntersection->region;
                                    }
                                    else
                                    {
                                        entry.remote_intersection.region_exists = false;
                                    }

                                    entry.remote_intersection.id = ct->remoteIntersection->id;

                                }
                                else
                                {
                                    entry.remote_intersection_exists = false;
                                }

                                //Signal Group
                                if(ct->signalGroup)
                                {
                                    entry.signal_group_exists = true;
                                    entry.signal_group = *ct->signalGroup;
                                }
                                else
                                {
                                    entry.signal_group_exists = false;
                                }

                                //User Class
                                if (ct->userClass)
                                {
                                    entry.user_class_exists = true;
                                    entry.user_class = *ct->userClass;
                                }

                                gl.connects_to.connect_to_list.push_back(entry);

                            }//end connects to

                        }
                        if (g_lane->overlays)
                        {
                            gl.overlay_lane_list_exists = true;
                            
                            for(size_t o = 0; o < g_lane->overlays->list.size;o++)
                            {
                                LaneID_t *ln = new LaneID_t;
                                ln = g_lane->overlays->list.array[o];
                                
                                
                                gl.overlay_lane_list.overlay_lane_list.push_back(*ln);
                            }

                        }


                        rs.road_lane_set.road_lane_set_list.push_back(gl);
                    }
                    


                    output.road_segments.road_segment_list.push_back(rs);
                }
            }//end Road Segments



            if(map_msg.dataParameters)
            {
                output.data_parameters_exists = true;


                for(size_t i = 0; i < map_msg.dataParameters->geoidUsed->size; i++)
                {
                    output.data_parameters.geoid_used.push_back(map_msg.dataParameters->geoidUsed->buf[i]);

                }

                for(size_t i = 0; i < map_msg.dataParameters->lastCheckedDate->size; i++)
                {
                    output.data_parameters.last_checked_date.push_back(map_msg.dataParameters->lastCheckedDate->buf[i]);
                }

                for(size_t i = 0; i < map_msg.dataParameters->processAgency->size; i++)
                {
                    output.data_parameters.process_agency.push_back(map_msg.dataParameters->processAgency->buf[i]);
                }

                for(size_t i = 0; i < map_msg.dataParameters->processMethod->size; i++)
                {
                    output.data_parameters.last_checked_date.push_back(map_msg.dataParameters->processMethod->buf[i]);
                }



            }

            else
            {
                output.data_parameters_exists = false;
            }

            if(map_msg.timeStamp)
            {
                output.time_stamp_exists = true;

                output.time_stamp = *map_msg.timeStamp;
            }
            else
            {
                output.time_stamp_exists = false;
            }

            return boost::optional<j2735_msgs::MapData>(output);


        }


    }


    boost::optional<std::vector<uint8_t>>Map_Message::encode_map_message(const j2735_msgs::MapData& plainMessage)
    {
        
            //encode result placeholder
            uint8_t buffer[544];
            size_t buffer_size=sizeof(buffer);
            asn_enc_rval_t ec;
            MessageFrame_t* message;
            message = (MessageFrame_t*) calloc(1, sizeof(MessageFrame_t));

            uint8_t id_content[4] = {0};
        for(auto i = 0; i < 4; i++)
        {
            id_content[i] = (char) plainMessage.layer_id;
        }
        

            //if mem allocation fails
            if(!message)
            {
                ROS_WARN_STREAM("Cannot allocate mem for MapData encoding");
                return boost::optional<std::vector<uint8_t>>{};
            }

            //set message type to MapData
            //message->messageId = ;  

            message->value.present = MessageFrame__value_PR_MapData;

            MapData_t *map_data;

            map_data = (MapData_t*) calloc(1, sizeof(MapData_t));

            //Encode timestamp
            MinuteOfTheYear_t* timestamp = new MinuteOfTheYear_t;
            if(plainMessage.time_stamp_exists)
            {
                *timestamp = plainMessage.time_stamp;
            }
            else{
                    ROS_DEBUG_STREAM("Encoding, Assigning default timestamp");
                    //*timestamp = DEFAULT_TIME_STAMP;
                }
            map_data->timeStamp = timestamp;

            IntersectionGeometry_t* intersection = new IntersectionGeometry_t;

            if(plainMessage.intersections_exists)
            {

                for(size_t i =0; i< plainMessage.intersections.size(); i++)
                {

                    intersection->id.id = plainMessage.intersections[i].id.id;

                    if(plainMessage.intersections[i].id.region_exists)
                    {
                        for(size_t b =0; b < plainMessage.intersections.size(); b++)
                        {
                            *intersection->id.region = plainMessage.intersections[b].id.region;
                        }
                    }
                    else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection id region does not exist");
                        }


                    //Lane List
                    for(size_t j = 0; j < plainMessage.intersections[i].lane_set.lane_list.size(); j++)
                    {
                        if(plainMessage.intersections[i].lane_set.lane_list[j].egress_approach_exists)
                        {
                            *intersection->laneSet.list.array[j]->egressApproach = plainMessage.intersections[i].lane_set.lane_list[j].egress_approach;
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection  Lane List egress approach does not exist");
                        }


                        if(plainMessage.intersections[i].lane_set.lane_list[j].ingress_approach_exists)
                        {
                            *intersection->laneSet.list.array[j]->ingressApproach = plainMessage.intersections[i].lane_set.lane_list[j].ingress_approach_exists;
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection Lane List ingress approach does not exist");
                        }

                        if(plainMessage.intersections[i].lane_set.lane_list[j].maneuvers_exists)
                        {
                            intersection->laneSet.list.array[j]->maneuvers->buf[j] = plainMessage.intersections[i].lane_set.lane_list[j].maneuvers.allowed_maneuvers;
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection Lane List name does not exist");
                        }

                        if(plainMessage.intersections[i].lane_set.lane_list[j].name_exists)
                        {
                            std::string name = plainMessage.intersections[i].lane_set.lane_list[j].name;
                             uint8_t string_content[name.size()];
                            for(size_t k = 0; k < name.size(); k++){
                                string_content[k] = name[k];
                            }

                            //LOAD NAME VALUES INTO BUF
                            intersection->laneSet.list.array[j]->name->buf = string_content;
                            intersection->laneSet.list.array[j]->name->size = name.size();

                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection Lane List name does not exist");
                        }

                        if(plainMessage.intersections[i].lane_set.lane_list[j].connects_to_exists)
                        {
                            //Connection_t *ctl;
                            //ctl = (Connection_t) calloc(1, sizeof(Connection_t));
                            for(size_t k = 0; k < plainMessage.intersections[i].lane_set.lane_list[j].connects_to.connect_to_list.size(); k++)
                            {
                               auto ctl = plainMessage.intersections[i].lane_set.lane_list[j].connects_to.connect_to_list[k];
                                intersection->laneSet.list.array[j]->connectsTo->list.array[k]->connectingLane.lane = ctl.connecting_lane.lane;

                                if(ctl.connecting_lane.maneuver_exists)
                                {
                                    auto man = ctl.connecting_lane.maneuver;
                                    intersection->laneSet.list.array[j]->connectsTo->list.array[k]->connectingLane.maneuver->buf[0] = man.allowed_maneuvers;//COMEBACKTOTHIS
                                }
                                else
                                {
                                    ROS_DEBUG_STREAM("Encoding, Intersection ConnectTo Lane Maneuvers does not exist");
                                }

                            }
                            
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection ConnectTo does not exist");
                        }//end ConnectTo

                        if(plainMessage.intersections[i].lane_set.lane_list[j].overlay_lane_list_exists)
                        {
                            for(size_t k = 0; k < plainMessage.intersections[i].lane_set.lane_list[j].overlay_lane_list.overlay_lane_list.size(); k++)
                            {
                                auto elem = plainMessage.intersections[i].lane_set.lane_list[j].overlay_lane_list.overlay_lane_list[k];

                                *intersection->laneSet.list.array[j]->overlays->list.array[k] = elem;
                            }
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Encoding, Intersection Overlay Lane List does not exist");
                        }//end Overlay Lane List
                        
                        
                    }//end Lane List


                    if(plainMessage.intersections[i].lane_width_exists)
                    {
                        *intersection->laneWidth = plainMessage.intersections[i].lane_width;
                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Encoding, Lane Width does not exist");
                    }

                   if(plainMessage.intersections[i].preempt_priority_data_exists)
                   {
                       ROS_DEBUG_STREAM("Encoding, Preempt Priority has not been implemented yet");
                   }
                   else
                   {
                       ROS_DEBUG_STREAM("Encoding, Preempt Priority has not been implemented yet");
                   }

                   if(plainMessage.intersections[i].ref_point.elevation_exists)
                   {
                       *intersection->refPoint.elevation = plainMessage.intersections[i].ref_point.elevation;
                       intersection->refPoint.lat = plainMessage.intersections[i].ref_point.latitude;
                       intersection->refPoint.Long = plainMessage.intersections[i].ref_point.longitude;
                   }
                   else
                    {
                        ROS_DEBUG_STREAM("Encoding, Reference Point Elevation does not exist");
                    }

                  if (plainMessage.intersections[i].speed_limits_exists)
                  {
                    for(size_t k = 0; k < plainMessage.intersections[i].speed_limits.speed_limits.size(); k++)
                    {
                        auto elem = plainMessage.intersections[i].speed_limits.speed_limits[k];
                        intersection->speedLimits->list.array[k]->speed = elem.speed;
                        intersection->speedLimits->list.array[k]->type = elem.type.speed_limit_type;
                    }
                  }
                  else
                    {
                        ROS_DEBUG_STREAM("Encoding, Intersection Speed Limits do not exist");
                    }

                    intersection->revision = plainMessage.intersections[i].revision;

                    map_data->intersections->list.array[i] = intersection;
                }


            }//End Intersections
            
            //LayerID
            if(plainMessage.layer_id_exists)
             {
                LayerID_t *lid;
                *lid = plainMessage.layer_id;

                map_data->layerID = lid;
             }
             else
             {
                 ROS_DEBUG_STREAM("Encoding, layer ID does not exist");
             }//End LayerID
             
            //LayerType
            LayerType_t * type = new LayerType_t;
            *type = plainMessage.layer_type.layer_type;
            map_data->layerType = type;


            //Restriction List
            RestrictionClassAssignment_t* rca;
            rca = (RestrictionClassAssignment_t*) calloc(1, sizeof(RestrictionClassAssignment_t));
            for(size_t i = 0; i < plainMessage.restriction_list.restriction_class_list.size(); i++)
            {
                rca->id = plainMessage.restriction_list.restriction_class_list.at(i).id;
                map_data->restrictionList->list.array[i] = rca;
            }
            free(rca);

            RoadSegment_t *rs;
            rs = (RoadSegment_t*) calloc(1, sizeof(RoadSegment_t));
            plainMessage.road_segments.road_segment_list;
            for(size_t i = 0; i < plainMessage.road_segments.road_segment_list.size(); i++)
            {
                rs->id.id = plainMessage.road_segments.road_segment_list.at(i).id.id;
                 uint8_t string_content[plainMessage.road_segments.road_segment_list[i].name.size()];
                for(size_t j = 0; j < plainMessage.road_segments.road_segment_list[i].name.size(); j++)
                {
                    string_content[j] = plainMessage.road_segments.road_segment_list[i].name[j];
                }

                rs->name->buf = string_content;
                rs->name->size = plainMessage.road_segments.road_segment_list[i].name.size();

                map_data->roadSegments->list.array[i]->id.id = rs->id.id;
                map_data->roadSegments->list.array[i]->id.region = rs->id.region;
                
            }
            map_data->msgIssueRevision = plainMessage.msg_issue_revision;


 
            message->value.choice.MapData = *map_data;
            free(map_data);
        
            //encode message
            ec=uper_encode_to_buffer(&asn_DEF_MessageFrame, 0 , message , buffer , buffer_size);
            // Uncomment below to enable logging in human readable form
            //asn_fprint(fp, &asn_DEF_MessageFrame, message);
            free(message);
        
            //log a warning if that fails
            if(ec.encoded == -1)
            {
                ROS_WARN_STREAM("Encoding for MapMessage has failed");
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