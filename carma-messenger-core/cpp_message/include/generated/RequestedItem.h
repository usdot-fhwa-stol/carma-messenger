
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

#ifndef	_RequestedItem_H_
#define	_RequestedItem_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RequestedItem {
	RequestedItem_reserved	= 0,
	RequestedItem_itemA	= 1,
	RequestedItem_itemB	= 2,
	RequestedItem_itemC	= 3,
	RequestedItem_itemD	= 4,
	RequestedItem_itemE	= 5,
	RequestedItem_itemF	= 6,
	RequestedItem_itemG	= 7,
	RequestedItem_itemI	= 8,
	RequestedItem_itemJ	= 9,
	RequestedItem_itemK	= 10,
	RequestedItem_itemL	= 11,
	RequestedItem_itemM	= 12,
	RequestedItem_itemN	= 13,
	RequestedItem_itemO	= 14,
	RequestedItem_itemP	= 15,
	RequestedItem_itemQ	= 16
	/*
	 * Enumeration is extensible
	 */
} e_RequestedItem;

/* RequestedItem */
typedef long	 RequestedItem_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_RequestedItem_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_RequestedItem;
extern const asn_INTEGER_specifics_t asn_SPC_RequestedItem_specs_1;
asn_struct_free_f RequestedItem_free;
asn_struct_print_f RequestedItem_print;
asn_constr_check_f RequestedItem_constraint;
ber_type_decoder_f RequestedItem_decode_ber;
der_type_encoder_f RequestedItem_encode_der;
xer_type_decoder_f RequestedItem_decode_xer;
xer_type_encoder_f RequestedItem_encode_xer;
oer_type_decoder_f RequestedItem_decode_oer;
oer_type_encoder_f RequestedItem_encode_oer;
per_type_decoder_f RequestedItem_decode_uper;
per_type_encoder_f RequestedItem_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _RequestedItem_H_ */
#include <asn_internal.h>
