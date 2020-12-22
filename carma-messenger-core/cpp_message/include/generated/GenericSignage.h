
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

#ifndef	_GenericSignage_H_
#define	_GenericSignage_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include "ITIScodes.h"
#include "ITIStextPhrase.h"
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum GenericSignage__Member__item_PR {
	GenericSignage__Member__item_PR_NOTHING,	/* No components present */
	GenericSignage__Member__item_PR_itis,
	GenericSignage__Member__item_PR_text
} GenericSignage__Member__item_PR;

/* Forward definitions */
typedef struct GenericSignage__Member {
	struct GenericSignage__Member__item {
		GenericSignage__Member__item_PR present;
		union GenericSignage__Member__item_u {
			ITIScodes_t	 itis;
			ITIStextPhrase_t	 text;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} item;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} GenericSignage__Member;

/* GenericSignage */
typedef struct GenericSignage {
	A_SEQUENCE_OF(GenericSignage__Member) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} GenericSignage_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_GenericSignage;
extern asn_SET_OF_specifics_t asn_SPC_GenericSignage_specs_1;
extern asn_TYPE_member_t asn_MBR_GenericSignage_1[1];
extern asn_per_constraints_t asn_PER_type_GenericSignage_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _GenericSignage_H_ */
#include <asn_internal.h>
