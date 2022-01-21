
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
        MessageFrame_t* message = nullptr;
        
         //copy from vector to array         
        size_t len=binary_array.size();    
        
        uint8_t buf[len];             
        std::copy(binary_array.begin(),binary_array.end(),buf);
        //use asn1c lib to decode
        
        rval=uper_decode(0, &asn_DEF_MessageFrame,(void **) &message, buf, len, 0, 0);
        if(rval.code == RC_OK)
        {


            auto map_msg = message->value.choice.MapData;

            //Time Stamp
            if(map_msg.timeStamp)
            {
                output.time_stamp_exists = true;

                output.time_stamp = *map_msg.timeStamp;
            }
            else
            {
                output.time_stamp_exists = false;
            }

            output.msg_issue_revision = map_msg.msgIssueRevision;

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

            IntersectionGeometry_t *map_msg_intersections = new IntersectionGeometry_t;

            //Intersection Handling
            if(map_msg.intersections)
             {   
                 
                output.intersections_exists = true;
                //Map Intersections
                for(size_t i = 0; i < map_msg.intersections->list.count; i++)
                {
                    map_msg_intersections = map_msg.intersections->list.array[i];
                    j2735_msgs::IntersectionGeometry new_intersection;
                    new_intersection.id.id = map_msg_intersections->id.id;

                    if(map_msg_intersections->id.region)
                    {
                        new_intersection.id.region_exists = true; //TODO: Implement intersection ID Region
                        new_intersection.id.region = *map_msg_intersections->id.region;
                    }
                    else
                    {
                        new_intersection.id.region_exists = false;
                        new_intersection.id.region = j2735_msgs::IntersectionReferenceID::REGION_UNAVAILABLE;
                    }
                    ROS_ERROR_STREAM("Before entering lane ");

                    for(size_t i =0; i < map_msg_intersections->laneSet.list.count;i++)
                    {
                        
                        auto gl = decode_generic_lane(map_msg_intersections->laneSet.list.array[i]);
                        
                        new_intersection.lane_set.lane_list.push_back(gl);
                    }
                    
                    ROS_ERROR_STREAM("1/4 Here");
                    if(map_msg_intersections->name)
                    {
                        new_intersection.name_exists=true;

                        DescriptiveName_t *nm = new DescriptiveName_t;

                        nm = map_msg_intersections->name;
                        std::string n;
                        
                        for (size_t x = 0; x < nm->size;x++)
                        {
                            n.push_back(nm->buf[x]);
                        }
                        new_intersection.name = n;

                    }
                    else
                    {
                        new_intersection.name_exists = false;
                    }
                    ROS_ERROR_STREAM("1/4 1/4");
                    if(map_msg_intersections->laneWidth)
                    {
                        new_intersection.lane_width_exists = true;
                        new_intersection.lane_width = *map_msg_intersections->laneWidth;
                    }
                    else
                    {
                        new_intersection.lane_width_exists = false;
                    }        

                    if(map_msg_intersections->preemptPriorityData)
                    {
                        new_intersection.preempt_priority_data_exists = true;

                        for(size_t p = 0; i<map_msg_intersections->preemptPriorityData->list.count; p++)
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
                    ROS_ERROR_STREAM("1/4 Half");
                    if(map_msg_intersections->speedLimits)
                    {
                        new_intersection.speed_limits_exists = true;
                        
                        for(size_t s = 0; s < map_msg_intersections->speedLimits->list.count ; s++)
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
                        DSRC_Elevation_t *dsrc_el = new DSRC_Elevation_t;
                        dsrc_el = map_msg_intersections->refPoint.elevation;

                        new_intersection.ref_point.elevation = *dsrc_el;
                    }
                    else
                    {
                        new_intersection.ref_point.elevation_exists = false;
                    }
                    new_intersection.ref_point.latitude = map_msg_intersections->refPoint.lat;
                    new_intersection.ref_point.longitude = map_msg_intersections->refPoint.Long;
                    ROS_ERROR_STREAM("1/4 3/4");

                    new_intersection.revision = map_msg_intersections->revision;
                

                    output.intersections.push_back(new_intersection);
                }

            }//end intersection handling
            else
            {
                output.intersections_exists = false;
            }
            ROS_ERROR_STREAM("1/4");
            //Road Segment
            if(map_msg.roadSegments)
            {

                output.road_segments_exists = true;

                RoadSegment_t *rseg = new RoadSegment_t;

                for(size_t i = 0; i< map_msg.roadSegments->list.count;i++)
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

                    if(rseg->speedLimits)
                    {
                        rs.speed_limits_exists = true;

                        RegulatorySpeedLimit_t *speedL = new RegulatorySpeedLimit_t;

                        for (size_t i = 0; i < rseg->speedLimits->list.count;i++)
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
                       
                    }
                    else
                    {
                        rs.ref_point.elevation_exists = false;
                    } 
                    rs.ref_point.latitude = rseg->refPoint.lat;
                    rs.ref_point.longitude = rseg->refPoint.Long;

                    //RLS
                    
                    for(size_t i =0; i < rseg->roadLaneSet.list.count;i++)
                    {
                        auto gl = decode_generic_lane(rseg->roadLaneSet.list.array[i]);
                        rs.road_lane_set.road_lane_set_list.push_back(gl);
                    }
                    
                    output.road_segments.road_segment_list.push_back(rs);
                }
            }//end Road Segments
            ROS_ERROR_STREAM("HALF");
            //Data Parameters
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
            ROS_ERROR_STREAM("3/4");
            //Restriction List
            if(map_msg.restrictionList)
            {
                output.restriction_list_exists = true;

                RestrictionClassAssignment_t *rca = new RestrictionClassAssignment_t;

                for(size_t i = 0; i < map_msg.restrictionList->list.count; i++)
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

        }
        return boost::optional<j2735_msgs::MapData>(output);

    }


    j2735_msgs::GenericLane Map_Message::decode_generic_lane(GenericLane_t* g_lane)
    {
        j2735_msgs::GenericLane gl;
        //Egress Approach
        if(g_lane->egressApproach)
        {
            gl.egress_approach_exists = true;
            gl.egress_approach = *g_lane->egressApproach;
        }
        else
        {
            gl.egress_approach_exists = false;
        }

        //Ingress Approach
        if(g_lane->ingressApproach)
        {
            gl.ingress_approach_exists = true;
            gl.ingress_approach = *g_lane->ingressApproach;
        }
        else
        {
            gl.ingress_approach_exists = false;
        }

        //Lane Attributes
        // - Lane Direction
        size_t lane_direction_bits_to_shift = g_lane->laneAttributes.directionalUse.bits_unused;
        gl.lane_attributes.directional_use.lane_direction =  *g_lane->laneAttributes.directionalUse.buf >> lane_direction_bits_to_shift;

        // - LaneType
        uint8_t lane_type_choice = g_lane->laneAttributes.laneType.present;

        for (size_t i = 0; i < 8; i++) //8 for uint8. ASN1 is received as bits for lane type
        {
             ROS_ERROR_STREAM("lane_type_choice: "  << (int)lane_type_choice);
            if ((int)lane_type_choice == 1)
            {
                gl.lane_attributes.laneType.choice = i; 
                ROS_ERROR_STREAM("i: "  << i);
                break;
            }
            lane_type_choice = lane_type_choice >> 1;
        }

        gl.lane_attributes.laneType.crosswalk.lane_attributes_crosswalk = *g_lane->laneAttributes.laneType.choice.crosswalk.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.crosswalk.bits_unused;
        gl.lane_attributes.laneType.median.lane_attributes_barrier = *g_lane->laneAttributes.laneType.choice.median.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.median.bits_unused;
        gl.lane_attributes.laneType.parking.lane_attributes_parking = *g_lane->laneAttributes.laneType.choice.parking.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.parking.bits_unused;
        gl.lane_attributes.laneType.bikeLane.lane_attributes_bike = *g_lane->laneAttributes.laneType.choice.bikeLane.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.bikeLane.bits_unused;
        gl.lane_attributes.laneType.sidewalk.lane_attributes_sidewalk = *g_lane->laneAttributes.laneType.choice.sidewalk.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.sidewalk.bits_unused;
        gl.lane_attributes.laneType.striping.lane_attributes_striping = *g_lane->laneAttributes.laneType.choice.striping.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.striping.bits_unused;
        gl.lane_attributes.laneType.trackedVehicle.lane_attributes_trackedvehicle = *g_lane->laneAttributes.laneType.choice.trackedVehicle.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.trackedVehicle.bits_unused;
        gl.lane_attributes.laneType.vehicle.lane_attributes_vehicle = *g_lane->laneAttributes.laneType.choice.vehicle.buf >> 
                                                                            g_lane->laneAttributes.laneType.choice.vehicle.bits_unused;

        if(g_lane->maneuvers)
        {
            gl.maneuvers_exists = true;

            gl.maneuvers.allowed_maneuvers = *g_lane->maneuvers->buf >> g_lane->maneuvers->bits_unused;
        }

        gl.lane_attributes.shared_with.lane_sharing = *g_lane->laneAttributes.sharedWith.buf >> g_lane->laneAttributes.sharedWith.bits_unused;
        
        if(g_lane->connectsTo)
        {
            gl.connects_to_exists = true;
            Connection_t *ct = new Connection_t;

            for(size_t c = 0; c< g_lane->connectsTo->list.count; c++)
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
                    uint16_t binary = ct->connectingLane.maneuver->buf[0] >> 4;
                    unsigned int maneuver_type = 4;
                        // e.g. shift the binary right until it equals to 1 (0b00000001) to determine the location of the non-zero bit

                    for (int m = 0; m < ct->connectingLane.maneuver->size; m ++)
                    {
                        if ((int)binary == 1) 
                        {
                            entry.connecting_lane.maneuver.allowed_maneuvers = maneuver_type;
                            break;
                        }
                        else
                        {
                            maneuver_type -= 1;
                            binary = binary >> 1;
                        }
                    }


                }
                else
                {
                    entry.connecting_lane.maneuver_exists=false;
                    entry.connecting_lane.maneuver.allowed_maneuvers = 0;
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
                        entry.remote_intersection.region = j2735_msgs::IntersectionReferenceID::REGION_UNAVAILABLE;
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

            }

        }
        else
        {
            gl.connects_to_exists = false;
        }//end connects to
        
        if (g_lane->laneID)
        {
            gl.lane_id = g_lane->laneID;
        }
        ROS_ERROR_STREAM("15");
        //Node List
        gl.node_list.choice = g_lane->nodeList.present;
        gl.node_list.computed.offset_x_axis.large = g_lane->nodeList.choice.computed.offsetXaxis.choice.large;
        gl.node_list.computed.offset_x_axis.small = g_lane->nodeList.choice.computed.offsetXaxis.choice.small;

        gl.node_list.computed.offset_y_axis.large = g_lane->nodeList.choice.computed.offsetYaxis.choice.large;
        gl.node_list.computed.offset_y_axis.small = g_lane->nodeList.choice.computed.offsetYaxis.choice.small;

        //NodeList Scale Axis
        if(g_lane->nodeList.choice.computed.scaleXaxis)
        {
            gl.node_list.computed.scale_x_axis_exists = true;
            gl.node_list.computed.scale_x_axis = *g_lane->nodeList.choice.computed.scaleXaxis;

        }
        if(g_lane->nodeList.choice.computed.scaleYaxis)
        {
            gl.node_list.computed.scale_y_axis_exists = true;
            gl.node_list.computed.scale_y_axis = *g_lane->nodeList.choice.computed.scaleYaxis;

        }

        //Rotate XY
        if(g_lane->nodeList.choice.computed.rotateXY)
        {
            gl.node_list.computed.rotatexy_exists = true;
            gl.node_list.computed.rotateXY = *g_lane->nodeList.choice.computed.rotateXY;

        }
        
        //Scale Axis
        if(g_lane->nodeList.choice.computed.scaleXaxis)
        {
            gl.node_list.computed.scale_x_axis_exists = true;
            gl.node_list.computed.scale_x_axis = *g_lane->nodeList.choice.computed.scaleXaxis;

        }
        if(g_lane->nodeList.choice.computed.scaleYaxis)
        {
            gl.node_list.computed.scale_y_axis_exists = true;
            gl.node_list.computed.scale_y_axis = *g_lane->nodeList.choice.computed.scaleYaxis;

        }

        // Reference lat long
        
        gl.node_list.computed.reference_lane_id = g_lane->nodeList.choice.computed.referenceLaneId;
        //Nodes
        for(size_t n =0;n< g_lane->nodeList.choice.nodes.list.count; n++)
        {
            j2735_msgs::NodeXY node;

            //Attributes
            if(g_lane->nodeList.choice.nodes.list.array[n]->attributes)
            {
                auto attributes = g_lane->nodeList.choice.nodes.list.array[n]->attributes;
                node.attributes_exists = true;
                
                //Attribute Data
                if(attributes->data)
                {
                    node.attributes.data_exists = true;
                    j2735_msgs::LaneDataAttribute data;
                    for(size_t d =0; d< attributes->data->list.count; d++)
                    {
                        data.lane_angle = attributes->data->list.array[d]->choice.laneAngle;
                        data.lane_crown_point_center = attributes->data->list.array[d]->choice.laneCrownPointCenter;
                        data.lane_crown_point_left = attributes->data->list.array[d]->choice.laneCrownPointLeft;
                        data.lane_crown_point_right = attributes->data->list.array[d]->choice.laneCrownPointRight;
                        data.path_end_point_angle = attributes->data->list.array[d]->choice.pathEndPointAngle;

                        for(size_t sl = 0; sl < attributes->data->list.array[d]->choice.speedLimits.list.count; sl++)
                        {
                            j2735_msgs::RegulatorySpeedLimit speedLimit;
                            speedLimit.speed = attributes->data->list.array[d]->choice.speedLimits.list.array[sl]->speed;
                            speedLimit.type.speed_limit_type = attributes->data->list.array[d]->choice.speedLimits.list.array[sl]->type;
                                data.speed_limits.speed_limits.push_back(speedLimit);
                        }
                        
                        node.attributes.data.lane_attribute_list.push_back(data);
                    }
                }
                else
                {
                    node.attributes.data_exists = false;
                }//end Attribute Data

                //Attribute DElevation
                if(attributes->dElevation)
                {
                    node.attributes.dElevation_exists = true;
                    node.attributes.dElevation = *attributes->dElevation;

                }
                else
                {
                    node.attributes.dElevation_exists = false;
                }//end Attribute DElevation

                //Attributes Disabled
                if(attributes->disabled)
                {
                    node.attributes.disabled_exists = true;
                    SegmentAttributeXY_t *sa_xy = new SegmentAttributeXY_t;
                    for(size_t sa = 0; sa < attributes->disabled->list.count; sa++)
                    {
                        sa_xy = attributes->disabled->list.array[sa];
                        j2735_msgs::SegmentAttributeXY sxy;
                        sxy.segment_attribute_xy = *sa_xy;
                        node.attributes.disabled.segment_attribute_xy.push_back(sxy);
                    }
                }
                else
                {
                    node.attributes.disabled_exists = false;
                    
                }//end Attributes Disabled

                //Attributes DWidth
                if(attributes->dWidth)
                {
                    node.attributes.dWitdh_exists = true;
                    node.attributes.dWitdh = *attributes->dWidth;
                }
                else
                {
                    node.attributes.dWitdh_exists = false;
                }//end Attributes DWidth

                //Attributes Enabled
                if(attributes->enabled)
                {
                    node.attributes.enabled_exists = true;
                    
                    SegmentAttributeXY_t *sa_xy = new SegmentAttributeXY_t;
                    for(size_t en = 0; en < attributes->enabled->list.count; en++)
                    {
                        sa_xy = attributes->enabled->list.array[en];

                        j2735_msgs::SegmentAttributeXY enabled;
                        enabled.segment_attribute_xy = *sa_xy;
                        node.attributes.enabled.segment_attribute_xy.push_back(enabled);
                    }
                }
                else
                {
                    node.attributes.enabled_exists = false;
                }//end Attributes Enabled


                //Attribute Local Node
                if(attributes->localNode)
                {
                    node.attributes.local_node_exists = true;

                    NodeAttributeXY_t *na = new NodeAttributeXY_t;

                    for(size_t ln = 0; ln < attributes->localNode->list.count; ln++)
                    {
                        j2735_msgs::NodeAttributeXY n_att;
                        na = attributes->localNode->list.array[ln];
                        n_att.node_attribute_xy = *na;
                        node.attributes.local_node.node_attribute_xy_List.push_back(n_att);
                        
                    }
                }
                else
                {
                    node.attributes.local_node_exists = false;
                }//end Attribute Local Node

            }
            else
            {
                node.attributes_exists = false;
            } //end Attributes

            //Node Delta
            node.delta.choice = g_lane->nodeList.choice.nodes.list.array[n]->delta.present;
            
            node.delta.node_latlon.latitude = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_LatLon.lat;
            node.delta.node_latlon.longitude = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_LatLon.lon;

            node.delta.node_xy1.x = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY1.x;
            node.delta.node_xy1.y = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY1.y;

            node.delta.node_xy2.x = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY2.x;
            node.delta.node_xy2.y = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY2.y;

            node.delta.node_xy3.x = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY3.x;
            node.delta.node_xy3.y = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY3.y;

            node.delta.node_xy4.x = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY4.x;
            node.delta.node_xy4.y = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY4.y;

            node.delta.node_xy5.x = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY5.x;
            node.delta.node_xy5.y = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY5.y;

            node.delta.node_xy6.x = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY6.x;
            node.delta.node_xy6.y = g_lane->nodeList.choice.nodes.list.array[n]->delta.choice.node_XY6.y;

            gl.node_list.nodes.node_set_xy.push_back(node);
            //end Node Delta Handling
            
        }


        if (g_lane->overlays)
        {
            gl.overlay_lane_list_exists = true;
            
            for(size_t o = 0; o < g_lane->overlays->list.count;o++)
            {
                LaneID_t *ln = new LaneID_t;
                ln = g_lane->overlays->list.array[o];
                
                
                gl.overlay_lane_list.overlay_lane_list.push_back(*ln);
            }

        }

        return gl;
    }
                    
    



}