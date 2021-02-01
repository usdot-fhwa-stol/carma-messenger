/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpB"
 * 	found in "J2735_201603_CARMA2.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_Node_LLdms_80b_H_
#define	_Node_LLdms_80b_H_


#include <asn_application.h>

/* Including external dependencies */
#include "LongitudeDMS2.h"
#include "LatitudeDMS2.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Node-LLdms-80b */
typedef struct Node_LLdms_80b {
	LongitudeDMS2_t	 lon;
	LatitudeDMS2_t	 lat;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Node_LLdms_80b_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Node_LLdms_80b;
extern asn_SEQUENCE_specifics_t asn_SPC_Node_LLdms_80b_specs_1;
extern asn_TYPE_member_t asn_MBR_Node_LLdms_80b_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Node_LLdms_80b_H_ */
#include <asn_internal.h>
