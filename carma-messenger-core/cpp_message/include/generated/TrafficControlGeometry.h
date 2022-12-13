/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_TrafficControlGeometry_H_
#define	_TrafficControlGeometry_H_


#include <asn_application.h>

/* Including external dependencies */
#include <IA5String.h>
#include "EpochMins.h"
#include "Longitude.h"
#include "Latitude.h"
#include "DSRC_Elevation.h"
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PathNode;

/* TrafficControlGeometry */
typedef struct TrafficControlGeometry {
	IA5String_t	 proj;
	IA5String_t	 datum;
	EpochMins_t	 reftime;
	Longitude_t	 reflon;
	Latitude_t	 reflat;
	DSRC_Elevation_t	 refelv;
	long	 refwidth;
	long	 heading;
	struct TrafficControlGeometry__nodes {
		A_SEQUENCE_OF(struct PathNode) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} nodes;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TrafficControlGeometry_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TrafficControlGeometry;
extern asn_SEQUENCE_specifics_t asn_SPC_TrafficControlGeometry_specs_1;
extern asn_TYPE_member_t asn_MBR_TrafficControlGeometry_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PathNode.h"

#endif	/* _TrafficControlGeometry_H_ */
#include <asn_internal.h>
