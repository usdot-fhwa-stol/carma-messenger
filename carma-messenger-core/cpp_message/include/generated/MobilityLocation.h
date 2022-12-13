/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_MobilityLocation_H_
#define	_MobilityLocation_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MobilityECEFCoordinate.h"
#include "MobilityTimestamp.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MobilityLocation */
typedef struct MobilityLocation {
	MobilityECEFCoordinate_t	 ecefX;
	MobilityECEFCoordinate_t	 ecefY;
	MobilityECEFCoordinate_t	 ecefZ;
	MobilityTimestamp_t	 timestamp;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MobilityLocation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MobilityLocation;
extern asn_SEQUENCE_specifics_t asn_SPC_MobilityLocation_specs_1;
extern asn_TYPE_member_t asn_MBR_MobilityLocation_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _MobilityLocation_H_ */
#include <asn_internal.h>
