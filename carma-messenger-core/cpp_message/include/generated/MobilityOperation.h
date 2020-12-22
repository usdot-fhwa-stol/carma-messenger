
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

#ifndef	_MobilityOperation_H_
#define	_MobilityOperation_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MobilityStrategy.h"
#include "MobilityParameters.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MobilityOperation */
typedef struct MobilityOperation {
	MobilityStrategy_t	 strategy;
	MobilityParameters_t	 operationParams;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MobilityOperation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MobilityOperation;
extern asn_SEQUENCE_specifics_t asn_SPC_MobilityOperation_specs_1;
extern asn_TYPE_member_t asn_MBR_MobilityOperation_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _MobilityOperation_H_ */
#include <asn_internal.h>
