
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
 * From ASN.1 module "AddGrpB"
 * 	found in "J2735_201603_CARMA2.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_MovementEvent_addGrpB_H_
#define	_MovementEvent_addGrpB_H_


#include <asn_application.h>

/* Including external dependencies */
#include "TimeRemaining.h"
#include "MinTimetoChange.h"
#include "MaxTimetoChange.h"
#include "TimeIntervalConfidence.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MovementEvent-addGrpB */
typedef struct MovementEvent_addGrpB {
	TimeRemaining_t	*startTime	/* OPTIONAL */;
	MinTimetoChange_t	 minEndTime;
	MaxTimetoChange_t	*maxEndTime	/* OPTIONAL */;
	TimeRemaining_t	*likelyTime	/* OPTIONAL */;
	TimeIntervalConfidence_t	*confidence	/* OPTIONAL */;
	TimeRemaining_t	*nextTime	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MovementEvent_addGrpB_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MovementEvent_addGrpB;

#ifdef __cplusplus
}
#endif

#endif	/* _MovementEvent_addGrpB_H_ */
#include <asn_internal.h>
