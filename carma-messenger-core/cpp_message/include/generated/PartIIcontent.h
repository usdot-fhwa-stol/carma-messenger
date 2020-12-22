
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

#ifndef	_PartIIcontent_H_
#define	_PartIIcontent_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PartII-Id.h"
#include <ANY.h>
#include <asn_ioc.h>
#include "VehicleSafetyExtensions.h"
#include "SpecialVehicleExtensions.h"
#include "SupplementalVehicleExtensions.h"
#include <OPEN_TYPE.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PartIIcontent_126P0__partII_Value_PR {
	PartIIcontent_126P0__partII_Value_PR_NOTHING,	/* No components present */
	PartIIcontent_126P0__partII_Value_PR_VehicleSafetyExtensions,
	PartIIcontent_126P0__partII_Value_PR_SpecialVehicleExtensions,
	PartIIcontent_126P0__partII_Value_PR_SupplementalVehicleExtensions
} PartIIcontent_126P0__partII_Value_PR;

/* PartIIcontent */
typedef struct PartIIcontent_126P0 {
	PartII_Id_t	 partII_Id;
	struct PartIIcontent_126P0__partII_Value {
		PartIIcontent_126P0__partII_Value_PR present;
		union PartIIcontent_126P0__partII_Value_u {
			VehicleSafetyExtensions_t	 VehicleSafetyExtensions;
			SpecialVehicleExtensions_t	 SpecialVehicleExtensions;
			SupplementalVehicleExtensions_t	 SupplementalVehicleExtensions;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} partII_Value;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PartIIcontent_126P0_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PartIIcontent_126P0;
extern asn_SEQUENCE_specifics_t asn_SPC_PartIIcontent_126P0_specs_1;
extern asn_TYPE_member_t asn_MBR_PartIIcontent_126P0_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _PartIIcontent_H_ */
#include <asn_internal.h>
