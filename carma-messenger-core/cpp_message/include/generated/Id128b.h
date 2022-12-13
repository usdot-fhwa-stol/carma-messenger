/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_Id128b_H_
#define	_Id128b_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Id128b */
typedef OCTET_STRING_t	 Id128b_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_Id128b_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_Id128b;
asn_struct_free_f Id128b_free;
asn_struct_print_f Id128b_print;
asn_constr_check_f Id128b_constraint;
ber_type_decoder_f Id128b_decode_ber;
der_type_encoder_f Id128b_encode_der;
xer_type_decoder_f Id128b_decode_xer;
xer_type_encoder_f Id128b_encode_xer;
oer_type_decoder_f Id128b_decode_oer;
oer_type_encoder_f Id128b_encode_oer;
per_type_decoder_f Id128b_decode_uper;
per_type_encoder_f Id128b_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _Id128b_H_ */
#include <asn_internal.h>
