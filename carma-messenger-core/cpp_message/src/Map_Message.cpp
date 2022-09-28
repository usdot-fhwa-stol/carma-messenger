
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

#include "cpp_message/Map_Message.h"

namespace cpp_message
{
    template <typename T>
    T *create_store_shared(std::vector<std::shared_ptr<void>> &shared_pointers)
    {
        auto obj_shared = std::make_shared<T>();
        shared_pointers.push_back(obj_shared);
        return obj_shared.get();
    }

    boost::optional<j2735_v2x_msgs::msg::MapData> Map_Message::decode_map_message(std::vector<uint8_t> &binary_array)
    {
        j2735_v2x_msgs::msg::MapData output;

        // decode results
        asn_dec_rval_t rval;
        MessageFrame_t *message = nullptr;

        // copy from vector to array
        size_t len = binary_array.size();

        uint8_t buf[len];
        std::copy(binary_array.begin(), binary_array.end(), buf);
        // use asn1c lib to decode

        rval = uper_decode(0, &asn_DEF_MessageFrame, (void **)&message, buf, len, 0, 0);
        if (rval.code != RC_OK)
        {
            return boost::optional<j2735_v2x_msgs::msg::MapData>{};
        }

        auto map_msg = message->value.choice.MapData;

        // Time Stamp
        if (map_msg.timeStamp)
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

        // Layer ID
        if (map_msg.layerID)
        {
            output.layer_id_exists = true;
            output.layer_id = *map_msg.layerID;

        } // end layer ID
        else
        {
            output.layer_id_exists = false;
        }

        IntersectionGeometry_t *map_msg_intersections = nullptr;
        std::vector<std::shared_ptr<void>> shared_ptrs; // keep references to the objects until the encoding is complete

        // Intersection Handling
        if (map_msg.intersections)
        {
            output.intersections_exists = true;
            // Map Intersections
            for (size_t i = 0; i < map_msg.intersections->list.count; i++)
            {
                map_msg_intersections = map_msg.intersections->list.array[i];
                j2735_v2x_msgs::msg::IntersectionGeometry new_intersection;
                // auto new_intersection = *create_store_shared<j2735_v2x_msgs::msg::IntersectionGeometry>(shared_ptrs);

                new_intersection.id.id = map_msg_intersections->id.id;

                if (map_msg_intersections->id.region)
                {
                    new_intersection.id.region_exists = true; // TODO: Implement intersection ID Region
                    new_intersection.id.region = *map_msg_intersections->id.region;
                }
                else
                {
                    new_intersection.id.region_exists = false;
                    new_intersection.id.region = j2735_v2x_msgs::msg::IntersectionReferenceID::REGION_UNAVAILABLE;
                }

                for (size_t i = 0; i < map_msg_intersections->laneSet.list.count; i++)
                {
                    auto gl = *create_store_shared<j2735_v2x_msgs::msg::GenericLane>(shared_ptrs);
                    gl = decode_generic_lane(map_msg_intersections->laneSet.list.array[i], shared_ptrs);
                    new_intersection.lane_set.lane_list.push_back(gl);
                }

                if (map_msg_intersections->name)
                {
                    new_intersection.name_exists = true;

                    auto nm = create_store_shared<DescriptiveName_t>(shared_ptrs);
                    nm = map_msg_intersections->name;

                    auto n = *create_store_shared<std::string>(shared_ptrs);

                    for (size_t x = 0; x < nm->size; x++)
                    {
                        n.push_back(nm->buf[x]);
                    }
                    new_intersection.name = n;
                }
                else
                {
                    new_intersection.name_exists = false;
                }

                if (map_msg_intersections->laneWidth)
                {
                    new_intersection.lane_width_exists = true;
                    new_intersection.lane_width = *map_msg_intersections->laneWidth;
                }
                else
                {
                    new_intersection.lane_width_exists = false;
                }

                if (map_msg_intersections->preemptPriorityData)
                {
                    new_intersection.preempt_priority_data_exists = true;

                    for (size_t p = 0; i < map_msg_intersections->preemptPriorityData->list.count; p++)
                    {
                        auto signal_control_zone = *create_store_shared<j2735_v2x_msgs::msg::SignalControlZone>(shared_ptrs);

                        /*SignalControlZone.msg states that RegionalExtension has not yet been implemented*/
                        new_intersection.preempt_priority_data.preempt_priority_list.push_back(signal_control_zone);
                    }
                }
                else
                {
                    new_intersection.preempt_priority_data_exists = false;
                }

                if (map_msg_intersections->speedLimits)
                {
                    new_intersection.speed_limits_exists = true;

                    for (size_t s = 0; s < map_msg_intersections->speedLimits->list.count; s++)
                    {
                        auto regulatory_speed_limit = *create_store_shared<j2735_v2x_msgs::msg::RegulatorySpeedLimit>(shared_ptrs);

                        // j2735_v2x_msgs::msg::RegulatorySpeedLimit regulatory_speed_limit;
                        regulatory_speed_limit.speed = map_msg_intersections->speedLimits->list.array[s]->speed;
                        regulatory_speed_limit.type.speed_limit_type = map_msg_intersections->speedLimits->list.array[s]->type;
                        new_intersection.speed_limits.speed_limits.push_back(regulatory_speed_limit);
                    }
                }
                else
                {
                    new_intersection.speed_limits_exists = false;
                }

                if (map_msg_intersections->refPoint.elevation)
                    if (map_msg_intersections->refPoint.elevation)
                        if (map_msg_intersections->refPoint.elevation)
                        {
                            new_intersection.ref_point.elevation_exists = true;
                            new_intersection.ref_point.elevation_exists = true;
                            new_intersection.ref_point.elevation_exists = true;

                            auto dsrc_el = create_store_shared<DSRC_Elevation_t>(shared_ptrs);
                            dsrc_el = map_msg_intersections->refPoint.elevation;
                            new_intersection.ref_point.elevation = *dsrc_el;
                        }
                        else
                        {
                            new_intersection.ref_point.elevation_exists = false;
                        }
                new_intersection.ref_point.latitude = map_msg_intersections->refPoint.lat;
                new_intersection.ref_point.longitude = map_msg_intersections->refPoint.Long;

                new_intersection.revision = map_msg_intersections->revision;

                output.intersections.push_back(new_intersection);
            }

        } // end intersection handling
        else
        {
            output.intersections_exists = false;
        }

        // Road Segment
        if (map_msg.roadSegments)
        {
            output.road_segments_exists = true;
            auto rseg = create_store_shared<RoadSegment_t>(shared_ptrs);

            for (size_t i = 0; i < map_msg.roadSegments->list.count; i++)
            {
                rseg = map_msg.roadSegments->list.array[i];

                auto rs = *create_store_shared<j2735_v2x_msgs::msg::RoadSegment>(shared_ptrs);
                // j2735_v2x_msgs::msg::RoadSegment rs;

                if (rseg->laneWidth)
                {
                    rs.lane_width_exists = true;
                    rs.lane_width = *rseg->laneWidth;
                }
                else
                {
                    rs.lane_width_exists = false;
                }

                // Road Segment Name
                if (rseg->name)
                {
                    rs.name_exists = true;

                    auto nm = create_store_shared<DescriptiveName_t>(shared_ptrs);
                    nm = rseg->name;

                    auto n = *create_store_shared<std::string>(shared_ptrs);

                    for (size_t i = 0; i < nm->size; i++)
                    {
                        n.push_back(nm->buf[i]);
                    }
                    rs.name = n;
                }
                else
                {
                    rs.name_exists = false;
                }

                if (rseg->speedLimits)
                {
                    rs.speed_limits_exists = true;

                    for (size_t i = 0; i < rseg->speedLimits->list.count; i++)
                    {
                        auto sl = *create_store_shared<j2735_v2x_msgs::msg::RegulatorySpeedLimit>(shared_ptrs);

                        sl.speed = rseg->speedLimits->list.array[i]->speed;
                        sl.type.speed_limit_type = rseg->speedLimits->list.array[i]->type;
                        rs.speed_limits.speed_limits.push_back(sl);
                    }
                } // end speed limits
                else
                {
                    rs.speed_limits_exists = false;
                }

                if (rseg->refPoint.elevation)
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

                // RLS

                for (size_t i = 0; i < rseg->roadLaneSet.list.count; i++)
                {
                    auto gl = *create_store_shared<j2735_v2x_msgs::msg::GenericLane>(shared_ptrs);
                    gl = decode_generic_lane(rseg->roadLaneSet.list.array[i], shared_ptrs);
                    rs.road_lane_set.road_lane_set_list.push_back(gl);
                }

                output.road_segments.road_segment_list.push_back(rs);
            }
        } // end Road Segments

        // Data Parameters
        if (map_msg.dataParameters)
        {
            output.data_parameters_exists = true;

            for (size_t i = 0; i < map_msg.dataParameters->geoidUsed->size; i++)
            {
                output.data_parameters.geoid_used.push_back(map_msg.dataParameters->geoidUsed->buf[i]);
            }

            for (size_t i = 0; i < map_msg.dataParameters->lastCheckedDate->size; i++)
            {
                output.data_parameters.last_checked_date.push_back(map_msg.dataParameters->lastCheckedDate->buf[i]);
            }

            for (size_t i = 0; i < map_msg.dataParameters->processAgency->size; i++)
            {
                output.data_parameters.process_agency.push_back(map_msg.dataParameters->processAgency->buf[i]);
            }

            for (size_t i = 0; i < map_msg.dataParameters->processMethod->size; i++)
            {
                output.data_parameters.last_checked_date.push_back(map_msg.dataParameters->processMethod->buf[i]);
            }
        }
        else
        {
            output.data_parameters_exists = false;
        }

        // Restriction List
        if (map_msg.restrictionList)
        {
            output.restriction_list_exists = true;

            auto rca = create_store_shared<RestrictionClassAssignment_t>(shared_ptrs);

            for (size_t i = 0; i < map_msg.restrictionList->list.count; i++)
            {
                rca = map_msg.restrictionList->list.array[i];

                auto rclass = *create_store_shared<j2735_v2x_msgs::msg::RestrictionClassAssignment>(shared_ptrs);
                rclass.id = rca->id;
                output.restriction_list.restriction_class_list.push_back(rclass);
            }
        } // end Restriction List
        else
        {
            output.restriction_list_exists = false;
        }

        ASN_STRUCT_FREE(asn_DEF_MessageFrame, message);
        return output;
    }

    j2735_v2x_msgs::msg::GenericLane Map_Message::decode_generic_lane(GenericLane_t *g_lane, std::vector<std::shared_ptr<void>> &shared_ptrs)
    {
        j2735_v2x_msgs::msg::GenericLane gl;
        // Egress Approach
        if (g_lane->egressApproach)
        {
            gl.egress_approach_exists = true;
            gl.egress_approach = *g_lane->egressApproach;
        }
        else
        {
            gl.egress_approach_exists = false;
        }

        // Ingress Approach
        if (g_lane->ingressApproach)
        {
            gl.ingress_approach_exists = true;
            gl.ingress_approach = *g_lane->ingressApproach;
        }
        else
        {
            gl.ingress_approach_exists = false;
        }

        // Lane Attributes
        //  - Lane Direction
        size_t lane_direction_bits_to_shift = g_lane->laneAttributes.directionalUse.bits_unused;
        gl.lane_attributes.directional_use.lane_direction = *g_lane->laneAttributes.directionalUse.buf >> lane_direction_bits_to_shift;

        // - LaneType
        gl.lane_attributes.lane_type.choice = (int)g_lane->laneAttributes.laneType.present - 1; // j2735 ROS msg starts from index 0

        gl.lane_attributes.lane_type.crosswalk.lane_attributes_crosswalk = *g_lane->laneAttributes.laneType.choice.crosswalk.buf >>
                                                                           g_lane->laneAttributes.laneType.choice.crosswalk.bits_unused;
        gl.lane_attributes.lane_type.median.lane_attributes_barrier = *g_lane->laneAttributes.laneType.choice.median.buf >>
                                                                      g_lane->laneAttributes.laneType.choice.median.bits_unused;
        gl.lane_attributes.lane_type.parking.lane_attributes_parking = *g_lane->laneAttributes.laneType.choice.parking.buf >>
                                                                       g_lane->laneAttributes.laneType.choice.parking.bits_unused;
        gl.lane_attributes.lane_type.bike_lane.lane_attributes_bike = *g_lane->laneAttributes.laneType.choice.bikeLane.buf >>
                                                                      g_lane->laneAttributes.laneType.choice.bikeLane.bits_unused;
        gl.lane_attributes.lane_type.sidewalk.lane_attributes_sidewalk = *g_lane->laneAttributes.laneType.choice.sidewalk.buf >>
                                                                         g_lane->laneAttributes.laneType.choice.sidewalk.bits_unused;
        gl.lane_attributes.lane_type.striping.lane_attributes_striping = *g_lane->laneAttributes.laneType.choice.striping.buf >>
                                                                         g_lane->laneAttributes.laneType.choice.striping.bits_unused;
        gl.lane_attributes.lane_type.tracked_vehicle.lane_attributes_trackedvehicle = *g_lane->laneAttributes.laneType.choice.trackedVehicle.buf >>
                                                                                      g_lane->laneAttributes.laneType.choice.trackedVehicle.bits_unused;
        gl.lane_attributes.lane_type.vehicle.lane_attributes_vehicle = *g_lane->laneAttributes.laneType.choice.vehicle.buf >>
                                                                       g_lane->laneAttributes.laneType.choice.vehicle.bits_unused;

        if (g_lane->maneuvers)
        {
            gl.maneuvers_exists = true;

            gl.maneuvers.allowed_maneuvers = *g_lane->maneuvers->buf >> g_lane->maneuvers->bits_unused;
        }

        gl.lane_attributes.shared_with.lane_sharing = *g_lane->laneAttributes.sharedWith.buf >> g_lane->laneAttributes.sharedWith.bits_unused;

        if (g_lane->connectsTo)
        {
            gl.connects_to_exists = true;
            auto ct = create_store_shared<Connection_t>(shared_ptrs);

            for (size_t c = 0; c < g_lane->connectsTo->list.count; c++)
            {
                ct = g_lane->connectsTo->list.array[c];

                auto entry = *create_store_shared<j2735_v2x_msgs::msg::Connection>(shared_ptrs);

                // Connection ID
                if (ct->connectionID)
                {
                    entry.connection_id_exists = true;
                    entry.connection_id = *ct->connectionID;
                }
                else
                {
                    entry.connection_id_exists = false;
                }

                // Connecting Lane
                entry.connecting_lane.lane = ct->connectingLane.lane;
                if (ct->connectingLane.maneuver)
                {
                    entry.connecting_lane.maneuver_exists = true;
                    uint16_t binary = ct->connectingLane.maneuver->buf[0] >> 4;
                    auto maneuver_type = *create_store_shared<unsigned int>(shared_ptrs);
                    maneuver_type = 4;

                    // e.g. shift the binary right until it equals to 1 (0b00000001) to determine the location of the non-zero bit
                    for (int m = 0; m < ct->connectingLane.maneuver->size; m++)
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
                    entry.connecting_lane.maneuver_exists = false;
                    entry.connecting_lane.maneuver.allowed_maneuvers = 0;
                }

                // Remote Intersection
                if (ct->remoteIntersection)
                {
                    entry.remote_intersection_exists = true;

                    // Remote Intersection Region
                    if (ct->remoteIntersection->region)
                    {
                        entry.remote_intersection.region_exists = true;
                        entry.remote_intersection.region = *ct->remoteIntersection->region;
                    }
                    else
                    {
                        entry.remote_intersection.region_exists = false;
                        entry.remote_intersection.region = j2735_v2x_msgs::msg::IntersectionReferenceID::REGION_UNAVAILABLE;
                    }

                    entry.remote_intersection.id = ct->remoteIntersection->id;
                }
                else
                {
                    entry.remote_intersection_exists = false;
                }

                // Signal Group
                if (ct->signalGroup)
                {
                    entry.signal_group_exists = true;
                    entry.signal_group = *ct->signalGroup;
                }
                else
                {
                    entry.signal_group_exists = false;
                }

                // User Class
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
        } // end connects to

        if (g_lane->laneID)
        {
            gl.lane_id = g_lane->laneID;
        }

        // Node List
        gl.node_list.choice = (int)g_lane->nodeList.present - 1; // ROS j2735 msg starts from 0 index
        gl.node_list.computed.offset_x_axis.large = g_lane->nodeList.choice.computed.offsetXaxis.choice.large;
        gl.node_list.computed.offset_x_axis.small = g_lane->nodeList.choice.computed.offsetXaxis.choice.small;

        gl.node_list.computed.offset_y_axis.large = g_lane->nodeList.choice.computed.offsetYaxis.choice.large;
        gl.node_list.computed.offset_y_axis.small = g_lane->nodeList.choice.computed.offsetYaxis.choice.small;

        // NodeList Scale Axis
        if (g_lane->nodeList.choice.computed.scaleXaxis)
        {
            gl.node_list.computed.scale_x_axis_exists = true;
            gl.node_list.computed.scale_x_axis = *g_lane->nodeList.choice.computed.scaleXaxis;
        }
        if (g_lane->nodeList.choice.computed.scaleYaxis)
        {
            gl.node_list.computed.scale_y_axis_exists = true;
            gl.node_list.computed.scale_y_axis = *g_lane->nodeList.choice.computed.scaleYaxis;
        }

        // Rotate XY
        if (g_lane->nodeList.choice.computed.rotateXY)
        {
            gl.node_list.computed.rotatexy_exists = true;
            gl.node_list.computed.rotate_xy = *g_lane->nodeList.choice.computed.rotateXY;
        }

        // Scale Axis
        if (g_lane->nodeList.choice.computed.scaleXaxis)
        {
            gl.node_list.computed.scale_x_axis_exists = true;
            gl.node_list.computed.scale_x_axis = *g_lane->nodeList.choice.computed.scaleXaxis;
        }
        if (g_lane->nodeList.choice.computed.scaleYaxis)
        {
            gl.node_list.computed.scale_y_axis_exists = true;
            gl.node_list.computed.scale_y_axis = *g_lane->nodeList.choice.computed.scaleYaxis;
        }

        // Reference lat long

        gl.node_list.computed.reference_lane_id = g_lane->nodeList.choice.computed.referenceLaneId;
        // Nodes
        for (size_t n = 0; n < g_lane->nodeList.choice.nodes.list.count; n++)
        {
            auto node = *create_store_shared<j2735_v2x_msgs::msg::NodeXY>(shared_ptrs);

            // Attributes
            if (g_lane->nodeList.choice.nodes.list.array[n]->attributes)
            {
                auto attributes = create_store_shared<NodeAttributeSetXY>(shared_ptrs);
                attributes = g_lane->nodeList.choice.nodes.list.array[n]->attributes;
                node.attributes_exists = true;

                // Attribute Data
                if (attributes->data)
                {
                    node.attributes.data_exists = true;
                    auto data = *create_store_shared<j2735_v2x_msgs::msg::LaneDataAttribute>(shared_ptrs);
                    for (size_t d = 0; d < attributes->data->list.count; d++)
                    {
                        data.lane_angle = attributes->data->list.array[d]->choice.laneAngle;
                        data.lane_crown_point_center = attributes->data->list.array[d]->choice.laneCrownPointCenter;
                        data.lane_crown_point_left = attributes->data->list.array[d]->choice.laneCrownPointLeft;
                        data.lane_crown_point_right = attributes->data->list.array[d]->choice.laneCrownPointRight;
                        data.path_end_point_angle = attributes->data->list.array[d]->choice.pathEndPointAngle;

                        for (size_t sl = 0; sl < attributes->data->list.array[d]->choice.speedLimits.list.count; sl++)
                        {
                            auto speedLimit = *create_store_shared<j2735_v2x_msgs::msg::RegulatorySpeedLimit>(shared_ptrs);
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
                } // end Attribute Data

                // Attribute DElevation
                if (attributes->dElevation)
                {
                    node.attributes.d_elevation_exists = true;
                    node.attributes.d_elevation = *attributes->dElevation;
                }
                else
                {
                    node.attributes.d_elevation_exists = false;
                } // end Attribute DElevation

                // Attributes Disabled
                if (attributes->disabled)
                {
                    node.attributes.disabled_exists = true;
                    for (size_t sa = 0; sa < attributes->disabled->list.count; sa++)
                    {
                        auto sxy = *create_store_shared<j2735_v2x_msgs::msg::SegmentAttributeXY>(shared_ptrs);
                        sxy.segment_attribute_xy = *attributes->disabled->list.array[sa];
                        node.attributes.disabled.segment_attribute_xy.push_back(sxy);
                    }
                }
                else
                {
                    node.attributes.disabled_exists = false;
                } // end Attributes Disabled

                // Attributes DWidth
                if (attributes->dWidth)
                {
                    node.attributes.d_width_exists = true;
                    node.attributes.d_width = *attributes->dWidth;
                }
                else
                {
                    node.attributes.d_width_exists = false;
                } // end Attributes DWidth

                // Attributes Enabled
                if (attributes->enabled)
                {
                    node.attributes.enabled_exists = true;
                    for (size_t en = 0; en < attributes->enabled->list.count; en++)
                    {
                        auto enabled = *create_store_shared<j2735_v2x_msgs::msg::SegmentAttributeXY>(shared_ptrs);
                        enabled.segment_attribute_xy = *attributes->enabled->list.array[en];
                        node.attributes.enabled.segment_attribute_xy.push_back(enabled);
                    }
                }
                else
                {
                    node.attributes.enabled_exists = false;
                } // end Attributes Enabled

                // Attribute Local Node
                if (attributes->localNode)
                {
                    node.attributes.local_node_exists = true;
                    for (size_t ln = 0; ln < attributes->localNode->list.count; ln++)
                    {
                        auto n_att = *create_store_shared<j2735_v2x_msgs::msg::NodeAttributeXY>(shared_ptrs);
                        n_att.node_attribute_xy = *attributes->localNode->list.array[ln];
                        node.attributes.local_node.node_attribute_xy_list.push_back(n_att);
                    }
                }
                else
                {
                    node.attributes.local_node_exists = false;
                } // end Attribute Local Node
            }
            else
            {
                node.attributes_exists = false;
            } // end Attributes

            // Node Delta
            node.delta.choice = (int)g_lane->nodeList.choice.nodes.list.array[n]->delta.present - 1; // index starts from 0 on j2735 msg

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
            // end Node Delta Handling
        }

        if (g_lane->overlays)
        {
            gl.overlay_lane_list_exists = true;

            for (size_t o = 0; o < g_lane->overlays->list.count; o++)
            {
                auto ln = create_store_shared<LaneID_t>(shared_ptrs);
                ln = g_lane->overlays->list.array[o];
                gl.overlay_lane_list.overlay_lane_list.push_back(*ln);
            }
        }
        return gl;
    }
}
