
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

#ifndef	_TravelerDataFrame_H_
#define	_TravelerDataFrame_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SSPindex.h"
#include "TravelerInfoType.h"
#include "DYear.h"
#include "MinuteOfTheYear.h"
#include "MinutesDuration.h"
#include "SignPrority.h"
#include "URL-Short.h"
#include "FurtherInfoID.h"
#include "RoadSignID.h"
#include <constr_CHOICE.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "ITIScodesAndText.h"
#include "WorkZone.h"
#include "GenericSignage.h"
#include "SpeedLimit.h"
#include "ExitService.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TravelerDataFrame__msgId_PR {
	TravelerDataFrame__msgId_PR_NOTHING,	/* No components present */
	TravelerDataFrame__msgId_PR_furtherInfoID,
	TravelerDataFrame__msgId_PR_roadSignID
} TravelerDataFrame__msgId_PR;
typedef enum TravelerDataFrame__content_PR {
	TravelerDataFrame__content_PR_NOTHING,	/* No components present */
	TravelerDataFrame__content_PR_advisory,
	TravelerDataFrame__content_PR_workZone,
	TravelerDataFrame__content_PR_genericSign,
	TravelerDataFrame__content_PR_speedLimit,
	TravelerDataFrame__content_PR_exitService
} TravelerDataFrame__content_PR;

/* Forward declarations */
struct GeographicalPath;

/* TravelerDataFrame */
typedef struct TravelerDataFrame {
	SSPindex_t	 sspTimRights;
	TravelerInfoType_t	 frameType;
	struct TravelerDataFrame__msgId {
		TravelerDataFrame__msgId_PR present;
		union TravelerDataFrame__msgId_u {
			FurtherInfoID_t	 furtherInfoID;
			RoadSignID_t	 roadSignID;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} msgId;
	DYear_t	*startYear	/* OPTIONAL */;
	MinuteOfTheYear_t	 startTime;
	MinutesDuration_t	 duratonTime;
	SignPrority_t	 priority;
	SSPindex_t	 sspLocationRights;
	struct TravelerDataFrame__regions {
		A_SEQUENCE_OF(struct GeographicalPath) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regions;
	SSPindex_t	 sspMsgRights1;
	SSPindex_t	 sspMsgRights2;
	struct TravelerDataFrame__content {
		TravelerDataFrame__content_PR present;
		union TravelerDataFrame__content_u {
			ITIScodesAndText_t	 advisory;
			WorkZone_t	 workZone;
			GenericSignage_t	 genericSign;
			SpeedLimit_t	 speedLimit;
			ExitService_t	 exitService;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} content;
	URL_Short_t	*url	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TravelerDataFrame_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TravelerDataFrame;
extern asn_SEQUENCE_specifics_t asn_SPC_TravelerDataFrame_specs_1;
extern asn_TYPE_member_t asn_MBR_TravelerDataFrame_1[13];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "GeographicalPath.h"

#endif	/* _TravelerDataFrame_H_ */
#include <asn_internal.h>
