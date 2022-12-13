/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_MobilityECEFOffset_H_
#define	_MobilityECEFOffset_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MobilityECEFOffsetCoordinate.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MobilityECEFOffset */
typedef struct MobilityECEFOffset {
	MobilityECEFOffsetCoordinate_t	 offsetX;
	MobilityECEFOffsetCoordinate_t	 offsetY;
	MobilityECEFOffsetCoordinate_t	 offsetZ;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MobilityECEFOffset_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MobilityECEFOffset;
extern asn_SEQUENCE_specifics_t asn_SPC_MobilityECEFOffset_specs_1;
extern asn_TYPE_member_t asn_MBR_MobilityECEFOffset_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _MobilityECEFOffset_H_ */
#include <asn_internal.h>
