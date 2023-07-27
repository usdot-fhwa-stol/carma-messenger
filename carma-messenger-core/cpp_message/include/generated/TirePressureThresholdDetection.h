/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2023-06-22.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_TirePressureThresholdDetection_H_
#define	_TirePressureThresholdDetection_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum TirePressureThresholdDetection {
	TirePressureThresholdDetection_noData	= 0,
	TirePressureThresholdDetection_overPressure	= 1,
	TirePressureThresholdDetection_noWarningPressure	= 2,
	TirePressureThresholdDetection_underPressure	= 3,
	TirePressureThresholdDetection_extremeUnderPressure	= 4,
	TirePressureThresholdDetection_undefined	= 5,
	TirePressureThresholdDetection_errorIndicator	= 6,
	TirePressureThresholdDetection_notAvailable	= 7
} e_TirePressureThresholdDetection;

/* TirePressureThresholdDetection */
typedef long	 TirePressureThresholdDetection_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_TirePressureThresholdDetection_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_TirePressureThresholdDetection;
extern const asn_INTEGER_specifics_t asn_SPC_TirePressureThresholdDetection_specs_1;
asn_struct_free_f TirePressureThresholdDetection_free;
asn_struct_print_f TirePressureThresholdDetection_print;
asn_constr_check_f TirePressureThresholdDetection_constraint;
ber_type_decoder_f TirePressureThresholdDetection_decode_ber;
der_type_encoder_f TirePressureThresholdDetection_encode_der;
xer_type_decoder_f TirePressureThresholdDetection_decode_xer;
xer_type_encoder_f TirePressureThresholdDetection_encode_xer;
oer_type_decoder_f TirePressureThresholdDetection_decode_oer;
oer_type_encoder_f TirePressureThresholdDetection_encode_oer;
per_type_decoder_f TirePressureThresholdDetection_decode_uper;
per_type_encoder_f TirePressureThresholdDetection_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _TirePressureThresholdDetection_H_ */
#include <asn_internal.h>
