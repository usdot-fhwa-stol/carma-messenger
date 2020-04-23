/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.message;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;

import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;

import cav_msgs.LocationECEF;
import cav_msgs.MobilityRequest;
import cav_msgs.PlanType;
import gov.dot.fhwa.saxton.carma.message.factory.MobilityRequestMessage;
import gov.dot.fhwa.saxton.carma.rosjava_utils.SaxtonLogger;

public class MobilityRequestDecodeTest {

    SaxtonLogger mockLogger;
    MessageFactory mockFactory;
    MobilityRequestMessage message;
    MobilityRequest request;
    
    @Before
    public void setup() {
        mockLogger = mock(SaxtonLogger.class);
        mockFactory = mock(MessageFactory.class);
        message = new MobilityRequestMessage(mockLogger, mockFactory);
        request = mock(MobilityRequest.class);
    }
    
    @Test
    public void decodeMobilityRequestWithoutOptionalField() {
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] strategy = new byte[50];
        byte[] locationTime = new byte[19];
        byte[] strategyParams = new byte[1000];
        byte[] trajectoryStartTime = new byte[19];
        int[][] offsets = new int[3][60];
        byte[] expiration = new byte[19];
        byte[] decodedMessage = {0, -16, -128, -125, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 2, -35, -42, 44, 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21, -84, -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88, -79, 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117, 38, 109, 26, -74, 110, -31, -56, 58, 30, 30, 91, 112, -81, -95, -77, 15, 77, -5, -9, 105, -35, -100, 62, 116, -62, -92, 74, -23, -123, -75, 23, -109, 12, 67, 50, -80, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -64, 17, -125, 74, 59, 23, 77, 87, 48, 88, -126, 13, 40, -20, -99, 49, 108, -71, -128};
        long start = System.currentTimeMillis();
        int res = message.callJniDecode(decodedMessage, request, senderId, targetId, bsmId, planId, timestamp, strategy,
                                        mock(PlanType.class), mock(LocationECEF.class), locationTime, strategyParams,
                                        mock(LocationECEF.class), trajectoryStartTime, offsets, expiration);
        long end = System.currentTimeMillis();
        System.out.println("decodeMobilityRequestWithoutOptionalField take " + (end - start) + "ms to finish.");
        assertEquals(0, res);
    }
    
    @Test
    public void decodeMobilityRequestWithOptionalField() {
        byte[] senderId = new byte[16];
        byte[] targetId = new byte[16];
        byte[] bsmId = new byte[8];
        byte[] planId = new byte[36];
        byte[] timestamp = new byte[19];
        byte[] strategy = new byte[50];
        byte[] locationTime = new byte[19];
        byte[] strategyParams = new byte[1000];
        byte[] trajectoryStartTime = new byte[19];
        int[][] offsets = new int[3][60];
        byte[] expiration = new byte[19];
        byte[] decodedMessage = {0, -16, -127, -110, 77, 90, 113, 39, -44, 90, -47, -85, 22, 12, 2, -35, -42, 44, 32, -62, -121, 18, 44, 102, 44, 88, -79, 98, -59, -117, 21, -84, -103, 50, 100, -75, -101, 54, 108, -42, -63, -125, 6, 10, -42, 44, 88, -79, 98, -59, -117, 22, 44, 88, -79, 96, -63, -125, 6, 12, 24, 48, 96, -63, -117, 38, 109, 26, -74, 110, -31, -49, 58, 30, 30, 91, 112, -81, -95, -77, 15, 77, -5, -9, 105, -35, -100, 62, 116, -62, -92, 74, -23, -123, -75, 23, -109, 12, 67, 50, -80, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -64, 17, -125, 74, 59, 23, 77, 87, 48, 88, -126, 13, 40, -20, -99, 49, 108, -71, -126, 96, -54, -119, -124, -63, -107, 19, 9, -125, 42, 38, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 107, -29, -24, -6, 62, -113, -85, -22, -6, -66, -49, -77, -20, -5, -66, -17, -69, -16, -4, 63, 15, -53, -14, -4, -65, 79, -45, -12, -3, -65, 111, -37, -8, -2, 63, -113, -21, -6, -2, -65, -49, -13, -4, -1, -65, -17, -4, 1, 0, 64, 16, 12, 3, 0, -64, 80, 20, 5, 1, -64, 112, 28, 9, 2, 64, -112, 44, 11, 2, -64, -48, 52, 13, 3, -64, -16, 60, 17, 4, 65, 16, 76, 19, 4, -63, 80, 84, 21, 5, -63, 112, 92, 25, 6, 65, -112, 108, 27, 6, -63, -48, 116, 29, 7, -63, -16, 124, 33, 8, 66, 16, -116, 35, 8, -62, 80, -108, 37, 9, -62, 112, -100, 41, 10, 66, -112, -84, 43, 10, -62, -48, -76, 45, 11, -62, -16, -68, 49, 12, 67, 16, -52, 51, 12, -61, 80, -44, 53, 13, -61, 112, -36, 57, 14, 67, -112, -20, 59, 14, -61, -48, -12, 61, 15, -61, -16, -4, 65, 16, 68, 17, 12, 67, 16, -60, 81, 20, 69, 17, -60, 113, 28, 73, 18, 68, -111, 44, 75, 18, -60, -47, 52, 77, 19, -60, -15, 60, 81, 20, 69, 17, 76, 83, 20, -59, 81, 84, 85, 21, -59, 113, 92, 89, 22, 69, -111, 108, 91, 22, -59, -47, 116, 93, 23, -59, -15, 123, 6, 12, 24, 48, 96, -63, -125, 6, 12, 24, 48, 96, -63, -125, 7, 12, -128};
        long start = System.currentTimeMillis();
        int res = message.callJniDecode(decodedMessage, request, senderId, targetId, bsmId, planId, timestamp, strategy,
                mock(PlanType.class), mock(LocationECEF.class), locationTime, strategyParams,
                mock(LocationECEF.class), trajectoryStartTime, offsets, expiration);
        long end = System.currentTimeMillis();
        System.out.println("decodeMobilityRequestWithOptionalField take " + (end - start) + "ms to finish.");
        assertEquals(0, res);
    }
}
