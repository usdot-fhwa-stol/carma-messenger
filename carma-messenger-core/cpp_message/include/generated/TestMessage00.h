/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/home/qswawrq/J2735_201603_CARMA2.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_TestMessage00_H_
#define	_TestMessage00_H_


#include <asn_application.h>

/* Including external dependencies */
#include "MobilityHeader.h"
#include "MobilityRequest.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TestMessage00 */
typedef struct TestMessage00 {
	MobilityHeader_t	 header;
	MobilityRequest_t	 body;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} TestMessage00_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TestMessage00;
extern asn_SEQUENCE_specifics_t asn_SPC_TestMessage00_specs_1;
extern asn_TYPE_member_t asn_MBR_TestMessage00_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _TestMessage00_H_ */
#include <asn_internal.h>