
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

#ifndef	_ConfidenceSet_H_
#define	_ConfidenceSet_H_


#include <asn_application.h>

/* Including external dependencies */
#include "TimeConfidence.h"
#include "SteeringWheelAngleConfidence.h"
#include "HeadingConfidence.h"
#include "ThrottleConfidence.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct AccelSteerYawRateConfidence;
struct SpeedandHeadingandThrottleConfidence;
struct PositionConfidenceSet;

/* ConfidenceSet */
typedef struct ConfidenceSet {
	struct AccelSteerYawRateConfidence	*accelConfidence	/* OPTIONAL */;
	struct SpeedandHeadingandThrottleConfidence	*speedConfidence	/* OPTIONAL */;
	TimeConfidence_t	*timeConfidence	/* OPTIONAL */;
	struct PositionConfidenceSet	*posConfidence	/* OPTIONAL */;
	SteeringWheelAngleConfidence_t	*steerConfidence	/* OPTIONAL */;
	HeadingConfidence_t	*headingConfidence	/* OPTIONAL */;
	ThrottleConfidence_t	*throttleConfidence	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ConfidenceSet_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ConfidenceSet;
extern asn_SEQUENCE_specifics_t asn_SPC_ConfidenceSet_specs_1;
extern asn_TYPE_member_t asn_MBR_ConfidenceSet_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "AccelSteerYawRateConfidence.h"
#include "SpeedandHeadingandThrottleConfidence.h"
#include "PositionConfidenceSet.h"

#endif	/* _ConfidenceSet_H_ */
#include <asn_internal.h>
