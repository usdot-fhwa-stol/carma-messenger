/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpB"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_LatitudeDMS2_H_
#define	_LatitudeDMS2_H_


#include <asn_application.h>

/* Including external dependencies */
#include "DegreesLat.h"
#include "MinutesAngle.h"
#include "SecondsAngle.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LatitudeDMS2 */
typedef struct LatitudeDMS2 {
	DegreesLat_t	 d;
	MinutesAngle_t	 m;
	SecondsAngle_t	 s;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} LatitudeDMS2_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_LatitudeDMS2;
extern asn_SEQUENCE_specifics_t asn_SPC_LatitudeDMS2_specs_1;
extern asn_TYPE_member_t asn_MBR_LatitudeDMS2_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _LatitudeDMS2_H_ */
#include <asn_internal.h>
