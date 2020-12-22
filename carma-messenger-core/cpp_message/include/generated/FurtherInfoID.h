
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.

------------------------------------------------------------------------------*/

/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "J2735_201603_CARMA2.asn1"
 * 	`asn1c -pdu=MessageFrame -fcompound-names -gen-PER`
 */

#ifndef	_FurtherInfoID_H_
#define	_FurtherInfoID_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FurtherInfoID */
typedef OCTET_STRING_t	 FurtherInfoID_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_FurtherInfoID_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_FurtherInfoID;
asn_struct_free_f FurtherInfoID_free;
asn_struct_print_f FurtherInfoID_print;
asn_constr_check_f FurtherInfoID_constraint;
ber_type_decoder_f FurtherInfoID_decode_ber;
der_type_encoder_f FurtherInfoID_encode_der;
xer_type_decoder_f FurtherInfoID_decode_xer;
xer_type_encoder_f FurtherInfoID_encode_xer;
oer_type_decoder_f FurtherInfoID_decode_oer;
oer_type_encoder_f FurtherInfoID_encode_oer;
per_type_decoder_f FurtherInfoID_decode_uper;
per_type_encoder_f FurtherInfoID_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _FurtherInfoID_H_ */
#include <asn_internal.h>
