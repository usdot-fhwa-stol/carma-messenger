/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_2022-12-07.asn"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -no-gen-APER -no-gen-JER`
 */

#ifndef	_TireLocation_H_
#define	_TireLocation_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TireLocation */
typedef long	 TireLocation_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_TireLocation_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_TireLocation;
asn_struct_free_f TireLocation_free;
asn_struct_print_f TireLocation_print;
asn_constr_check_f TireLocation_constraint;
ber_type_decoder_f TireLocation_decode_ber;
der_type_encoder_f TireLocation_encode_der;
xer_type_decoder_f TireLocation_decode_xer;
xer_type_encoder_f TireLocation_encode_xer;
oer_type_decoder_f TireLocation_decode_oer;
oer_type_encoder_f TireLocation_encode_oer;
per_type_decoder_f TireLocation_decode_uper;
per_type_encoder_f TireLocation_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _TireLocation_H_ */
#include <asn_internal.h>
