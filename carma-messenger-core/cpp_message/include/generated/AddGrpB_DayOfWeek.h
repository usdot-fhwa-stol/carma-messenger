/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpB"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_AddGrpB_DayOfWeek_H_
#define	_AddGrpB_DayOfWeek_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AddGrpB_DayOfWeek {
	AddGrpB_DayOfWeek_unknown	= 0,
	AddGrpB_DayOfWeek_monday	= 1,
	AddGrpB_DayOfWeek_tuesday	= 2,
	AddGrpB_DayOfWeek_wednesday	= 3,
	AddGrpB_DayOfWeek_thursday	= 4,
	AddGrpB_DayOfWeek_friday	= 5,
	AddGrpB_DayOfWeek_saturday	= 6,
	AddGrpB_DayOfWeek_sunday	= 7
} e_AddGrpB_DayOfWeek;

/* AddGrpB_DayOfWeek */
typedef long	 AddGrpB_DayOfWeek_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_AddGrpB_DayOfWeek_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_AddGrpB_DayOfWeek;
extern const asn_INTEGER_specifics_t asn_SPC_AddGrpB_DayOfWeek_specs_1;
asn_struct_free_f AddGrpB_DayOfWeek_free;
asn_struct_print_f AddGrpB_DayOfWeek_print;
asn_constr_check_f AddGrpB_DayOfWeek_constraint;
ber_type_decoder_f AddGrpB_DayOfWeek_decode_ber;
der_type_encoder_f AddGrpB_DayOfWeek_encode_der;
xer_type_decoder_f AddGrpB_DayOfWeek_decode_xer;
xer_type_encoder_f AddGrpB_DayOfWeek_encode_xer;
oer_type_decoder_f AddGrpB_DayOfWeek_decode_oer;
oer_type_encoder_f AddGrpB_DayOfWeek_encode_oer;
per_type_decoder_f AddGrpB_DayOfWeek_decode_uper;
per_type_encoder_f AddGrpB_DayOfWeek_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _AddGrpB_DayOfWeek_H_ */
#include <asn_internal.h>
