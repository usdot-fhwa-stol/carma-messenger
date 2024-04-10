/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "SDSM"
 * 	found in "J2735_201603_2023-06-22.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_ObstacleSizeConfidence_H_
#define	_ObstacleSizeConfidence_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SizeValueConfidence.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ObstacleSizeConfidence */
typedef struct ObstacleSizeConfidence {
	SizeValueConfidence_t	 widthConfidence;
	SizeValueConfidence_t	 lengthConfidence;
	SizeValueConfidence_t	*heightConfidence;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ObstacleSizeConfidence_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ObstacleSizeConfidence;
extern asn_SEQUENCE_specifics_t asn_SPC_ObstacleSizeConfidence_specs_1;
extern asn_TYPE_member_t asn_MBR_ObstacleSizeConfidence_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _ObstacleSizeConfidence_H_ */
#include <asn_internal.h>
