/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_CARMA2.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_NMEA_Revision_H_
#define	_NMEA_Revision_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NMEA_Revision {
	NMEA_Revision_unknown	= 0,
	NMEA_Revision_reserved	= 1,
	NMEA_Revision_rev1	= 2,
	NMEA_Revision_rev2	= 3,
	NMEA_Revision_rev3	= 4,
	NMEA_Revision_rev4	= 5,
	NMEA_Revision_rev5	= 6
	/*
	 * Enumeration is extensible
	 */
} e_NMEA_Revision;

/* NMEA-Revision */
typedef long	 NMEA_Revision_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_NMEA_Revision_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_NMEA_Revision;
extern const asn_INTEGER_specifics_t asn_SPC_NMEA_Revision_specs_1;
asn_struct_free_f NMEA_Revision_free;
asn_struct_print_f NMEA_Revision_print;
asn_constr_check_f NMEA_Revision_constraint;
ber_type_decoder_f NMEA_Revision_decode_ber;
der_type_encoder_f NMEA_Revision_encode_der;
xer_type_decoder_f NMEA_Revision_decode_xer;
xer_type_encoder_f NMEA_Revision_encode_xer;
oer_type_decoder_f NMEA_Revision_decode_oer;
oer_type_encoder_f NMEA_Revision_encode_oer;
per_type_decoder_f NMEA_Revision_decode_uper;
per_type_encoder_f NMEA_Revision_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _NMEA_Revision_H_ */
#include <asn_internal.h>
