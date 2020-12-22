
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_CARMA2.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_MessageFrame_H_
#define	_MessageFrame_H_


#include <asn_application.h>

/* Including external dependencies */
#include "DSRCmsgID.h"
#include <ANY.h>
#include <asn_ioc.h>
#include "BasicSafetyMessage.h"
#include "MapData.h"
#include "SPAT.h"
#include "CommonSafetyRequest.h"
#include "EmergencyVehicleAlert.h"
#include "IntersectionCollision.h"
#include "NMEAcorrections.h"
#include "ProbeDataManagement.h"
#include "ProbeVehicleData.h"
#include "RoadSideAlert.h"
#include "RTCMcorrections.h"
#include "SignalRequestMessage.h"
#include "SignalStatusMessage.h"
#include "TravelerInformation.h"
#include "PersonalSafetyMessage.h"
#include "TestMessage00.h"
#include "TestMessage01.h"
#include "TestMessage02.h"
#include "TestMessage03.h"
#include "TestMessage04.h"
#include "TestMessage05.h"
#include "TestMessage06.h"
#include "TestMessage07.h"
#include "TestMessage08.h"
#include "TestMessage09.h"
#include "TestMessage10.h"
#include "TestMessage11.h"
#include "TestMessage12.h"
#include "TestMessage13.h"
#include "TestMessage14.h"
#include "TestMessage15.h"
#include <OPEN_TYPE.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MessageFrame__value_PR {
	MessageFrame__value_PR_NOTHING,	/* No components present */
	MessageFrame__value_PR_BasicSafetyMessage,
	MessageFrame__value_PR_MapData,
	MessageFrame__value_PR_SPAT,
	MessageFrame__value_PR_CommonSafetyRequest,
	MessageFrame__value_PR_EmergencyVehicleAlert,
	MessageFrame__value_PR_IntersectionCollision,
	MessageFrame__value_PR_NMEAcorrections,
	MessageFrame__value_PR_ProbeDataManagement,
	MessageFrame__value_PR_ProbeVehicleData,
	MessageFrame__value_PR_RoadSideAlert,
	MessageFrame__value_PR_RTCMcorrections,
	MessageFrame__value_PR_SignalRequestMessage,
	MessageFrame__value_PR_SignalStatusMessage,
	MessageFrame__value_PR_TravelerInformation,
	MessageFrame__value_PR_PersonalSafetyMessage,
	MessageFrame__value_PR_TestMessage00,
	MessageFrame__value_PR_TestMessage01,
	MessageFrame__value_PR_TestMessage02,
	MessageFrame__value_PR_TestMessage03,
	MessageFrame__value_PR_TestMessage04,
	MessageFrame__value_PR_TestMessage05,
	MessageFrame__value_PR_TestMessage06,
	MessageFrame__value_PR_TestMessage07,
	MessageFrame__value_PR_TestMessage08,
	MessageFrame__value_PR_TestMessage09,
	MessageFrame__value_PR_TestMessage10,
	MessageFrame__value_PR_TestMessage11,
	MessageFrame__value_PR_TestMessage12,
	MessageFrame__value_PR_TestMessage13,
	MessageFrame__value_PR_TestMessage14,
	MessageFrame__value_PR_TestMessage15
} MessageFrame__value_PR;

/* MessageFrame */
typedef struct MessageFrame {
	DSRCmsgID_t	 messageId;
	struct MessageFrame__value {
		MessageFrame__value_PR present;
		union MessageFrame__value_u {
			BasicSafetyMessage_t	 BasicSafetyMessage;
			MapData_t	 MapData;
			SPAT_t	 SPAT;
			CommonSafetyRequest_t	 CommonSafetyRequest;
			EmergencyVehicleAlert_t	 EmergencyVehicleAlert;
			IntersectionCollision_t	 IntersectionCollision;
			NMEAcorrections_t	 NMEAcorrections;
			ProbeDataManagement_t	 ProbeDataManagement;
			ProbeVehicleData_t	 ProbeVehicleData;
			RoadSideAlert_t	 RoadSideAlert;
			RTCMcorrections_t	 RTCMcorrections;
			SignalRequestMessage_t	 SignalRequestMessage;
			SignalStatusMessage_t	 SignalStatusMessage;
			TravelerInformation_t	 TravelerInformation;
			PersonalSafetyMessage_t	 PersonalSafetyMessage;
			TestMessage00_t	 TestMessage00;
			TestMessage01_t	 TestMessage01;
			TestMessage02_t	 TestMessage02;
			TestMessage03_t	 TestMessage03;
			TestMessage04_t	 TestMessage04;
			TestMessage05_t	 TestMessage05;
			TestMessage06_t	 TestMessage06;
			TestMessage07_t	 TestMessage07;
			TestMessage08_t	 TestMessage08;
			TestMessage09_t	 TestMessage09;
			TestMessage10_t	 TestMessage10;
			TestMessage11_t	 TestMessage11;
			TestMessage12_t	 TestMessage12;
			TestMessage13_t	 TestMessage13;
			TestMessage14_t	 TestMessage14;
			TestMessage15_t	 TestMessage15;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} value;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MessageFrame_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MessageFrame;

#ifdef __cplusplus
}
#endif

#endif	/* _MessageFrame_H_ */
#include <asn_internal.h>
