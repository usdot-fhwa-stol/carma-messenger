/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2023-06-22.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_RoadLaneSetList_H_
#define	_RoadLaneSetList_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct GenericLane;

/* RoadLaneSetList */
typedef struct RoadLaneSetList {
	A_SEQUENCE_OF(struct GenericLane) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadLaneSetList_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadLaneSetList;
extern asn_SET_OF_specifics_t asn_SPC_RoadLaneSetList_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadLaneSetList_1[1];
extern asn_per_constraints_t asn_PER_type_RoadLaneSetList_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "GenericLane.h"

#endif	/* _RoadLaneSetList_H_ */
#include <asn_internal.h>
