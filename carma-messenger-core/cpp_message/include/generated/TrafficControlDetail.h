
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

#ifndef	_TrafficControlDetail_H_
#define	_TrafficControlDetail_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>
#include <NULL.h>
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TrafficControlDetail_PR {
	TrafficControlDetail_PR_NOTHING,	/* No components present */
	TrafficControlDetail_PR_signal,
	TrafficControlDetail_PR_stop,
	TrafficControlDetail_PR_yield,
	TrafficControlDetail_PR_notowing,
	TrafficControlDetail_PR_restricted,
	TrafficControlDetail_PR_closed,
	TrafficControlDetail_PR_chains,
	TrafficControlDetail_PR_direction,
	TrafficControlDetail_PR_lataffinity,
	TrafficControlDetail_PR_latperm,
	TrafficControlDetail_PR_parking,
	TrafficControlDetail_PR_minspeed,
	TrafficControlDetail_PR_maxspeed,
	TrafficControlDetail_PR_minhdwy,
	TrafficControlDetail_PR_maxvehmass,
	TrafficControlDetail_PR_maxvehheight,
	TrafficControlDetail_PR_maxvehwidth,
	TrafficControlDetail_PR_maxvehlength,
	TrafficControlDetail_PR_maxvehaxles,
	TrafficControlDetail_PR_minvehocc
	/* Extensions may appear below */
	
} TrafficControlDetail_PR;
typedef enum TrafficControlDetail__closed {
	TrafficControlDetail__closed_open	= 0,
	TrafficControlDetail__closed_closed	= 1,
	TrafficControlDetail__closed_taperleft	= 2,
	TrafficControlDetail__closed_taperright	= 3,
	TrafficControlDetail__closed_openleft	= 4,
	TrafficControlDetail__closed_openright	= 5
} e_TrafficControlDetail__closed;
typedef enum TrafficControlDetail__chains {
	TrafficControlDetail__chains_no	= 0,
	TrafficControlDetail__chains_permitted	= 1,
	TrafficControlDetail__chains_required	= 2
} e_TrafficControlDetail__chains;
typedef enum TrafficControlDetail__direction {
	TrafficControlDetail__direction_forward	= 0,
	TrafficControlDetail__direction_reverse	= 1
} e_TrafficControlDetail__direction;
typedef enum TrafficControlDetail__lataffinity {
	TrafficControlDetail__lataffinity_left	= 0,
	TrafficControlDetail__lataffinity_right	= 1
} e_TrafficControlDetail__lataffinity;
typedef enum TrafficControlDetail__latperm__Member {
	TrafficControlDetail__latperm__Member_none	= 0,
	TrafficControlDetail__latperm__Member_permitted	= 1,
	TrafficControlDetail__latperm__Member_passing_only	= 2,
	TrafficControlDetail__latperm__Member_emergency_only	= 3
} e_TrafficControlDetail__latperm__Member;
typedef enum TrafficControlDetail__parking {
	TrafficControlDetail__parking_no	= 0,
	TrafficControlDetail__parking_parallel	= 1,
	TrafficControlDetail__parking_angled	= 2
} e_TrafficControlDetail__parking;

/* TrafficControlDetail */
typedef struct TrafficControlDetail {
	TrafficControlDetail_PR present;
	union TrafficControlDetail_u {
		OCTET_STRING_t	 signal;
		NULL_t	 stop;
		NULL_t	 yield;
		NULL_t	 notowing;
		NULL_t	 restricted;
		long	 closed;
		long	 chains;
		long	 direction;
		long	 lataffinity;
		struct TrafficControlDetail__latperm {
			A_SEQUENCE_OF(long) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} latperm;
		long	 parking;
		long	 minspeed;
		long	 maxspeed;
		long	 minhdwy;
		long	 maxvehmass;
		long	 maxvehheight;
		long	 maxvehwidth;
		long	 maxvehlength;
		long	 maxvehaxles;
		long	 minvehocc;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TrafficControlDetail_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_closed_7;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_chains_14;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_direction_18;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_lataffinity_21;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_Member_25;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_parking_30;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_TrafficControlDetail;
extern asn_CHOICE_specifics_t asn_SPC_TrafficControlDetail_specs_1;
extern asn_TYPE_member_t asn_MBR_TrafficControlDetail_1[20];
extern asn_per_constraints_t asn_PER_type_TrafficControlDetail_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _TrafficControlDetail_H_ */
#include <asn_internal.h>
