/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_BrakeAppliedPressure_H_
#define	_BrakeAppliedPressure_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BrakeAppliedPressure {
	BrakeAppliedPressure_unavailable	= 0,
	BrakeAppliedPressure_minPressure	= 1,
	BrakeAppliedPressure_bkLvl_2	= 2,
	BrakeAppliedPressure_bkLvl_3	= 3,
	BrakeAppliedPressure_bkLvl_4	= 4,
	BrakeAppliedPressure_bkLvl_5	= 5,
	BrakeAppliedPressure_bkLvl_6	= 6,
	BrakeAppliedPressure_bkLvl_7	= 7,
	BrakeAppliedPressure_bkLvl_8	= 8,
	BrakeAppliedPressure_bkLvl_9	= 9,
	BrakeAppliedPressure_bkLvl_10	= 10,
	BrakeAppliedPressure_bkLvl_11	= 11,
	BrakeAppliedPressure_bkLvl_12	= 12,
	BrakeAppliedPressure_bkLvl_13	= 13,
	BrakeAppliedPressure_bkLvl_14	= 14,
	BrakeAppliedPressure_maxPressure	= 15
} e_BrakeAppliedPressure;

/* BrakeAppliedPressure */
typedef long	 BrakeAppliedPressure_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_BrakeAppliedPressure_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_BrakeAppliedPressure;
extern const asn_INTEGER_specifics_t asn_SPC_BrakeAppliedPressure_specs_1;
asn_struct_free_f BrakeAppliedPressure_free;
asn_struct_print_f BrakeAppliedPressure_print;
asn_constr_check_f BrakeAppliedPressure_constraint;
ber_type_decoder_f BrakeAppliedPressure_decode_ber;
der_type_encoder_f BrakeAppliedPressure_encode_der;
xer_type_decoder_f BrakeAppliedPressure_decode_xer;
xer_type_encoder_f BrakeAppliedPressure_encode_xer;
oer_type_decoder_f BrakeAppliedPressure_decode_oer;
oer_type_encoder_f BrakeAppliedPressure_encode_oer;
per_type_decoder_f BrakeAppliedPressure_decode_uper;
per_type_encoder_f BrakeAppliedPressure_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _BrakeAppliedPressure_H_ */
#include <asn_internal.h>
