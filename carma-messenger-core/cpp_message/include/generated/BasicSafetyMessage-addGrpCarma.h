/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpCarma"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_BasicSafetyMessage_addGrpCarma_H_
#define	_BasicSafetyMessage_addGrpCarma_H_


#include <asn_application.h>

/* Including external dependencies */
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Position3D_addGrpCarma;

/* BasicSafetyMessage-addGrpCarma */
typedef struct BasicSafetyMessage_addGrpCarma {
	struct BasicSafetyMessage_addGrpCarma__routeDestinationPoints {
		A_SEQUENCE_OF(struct Position3D_addGrpCarma) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *routeDestinationPoints;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BasicSafetyMessage_addGrpCarma_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BasicSafetyMessage_addGrpCarma;
extern asn_SEQUENCE_specifics_t asn_SPC_BasicSafetyMessage_addGrpCarma_specs_1;
extern asn_TYPE_member_t asn_MBR_BasicSafetyMessage_addGrpCarma_1[1];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "Position3D-addGrpCarma.h"

#endif	/* _BasicSafetyMessage_addGrpCarma_H_ */
#include <asn_internal.h>
