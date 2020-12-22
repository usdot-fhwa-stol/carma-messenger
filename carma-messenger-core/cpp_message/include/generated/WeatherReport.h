
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

#ifndef	_WeatherReport_H_
#define	_WeatherReport_H_


#include <asn_application.h>

/* Including external dependencies */
#include "EssPrecipYesNo.h"
#include "EssPrecipRate.h"
#include "EssPrecipSituation.h"
#include "EssSolarRadiation.h"
#include "EssMobileFriction.h"
#include "CoefficientOfFriction.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* WeatherReport */
typedef struct WeatherReport {
	EssPrecipYesNo_t	 isRaining;
	EssPrecipRate_t	*rainRate	/* OPTIONAL */;
	EssPrecipSituation_t	*precipSituation	/* OPTIONAL */;
	EssSolarRadiation_t	*solarRadiation	/* OPTIONAL */;
	EssMobileFriction_t	*friction	/* OPTIONAL */;
	CoefficientOfFriction_t	*roadFriction	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} WeatherReport_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_WeatherReport;
extern asn_SEQUENCE_specifics_t asn_SPC_WeatherReport_specs_1;
extern asn_TYPE_member_t asn_MBR_WeatherReport_1[6];

#ifdef __cplusplus
}
#endif

#endif	/* _WeatherReport_H_ */
#include <asn_internal.h>
