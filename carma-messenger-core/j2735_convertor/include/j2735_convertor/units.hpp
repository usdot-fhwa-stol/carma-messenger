#pragma once
/*
 * Copyright (C) 2018-2021 LEIDOS.
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
 */

namespace j2735_convertor
{
/**
 * Defined the units namespace which contains many unit conversion factors
 * These are primarily meant to be used in conversions between j2735_msgs and cav_msgs
 */

namespace units
{
constexpr double DECI_MPS_PER_MPS = 10.0;
constexpr double DECI_S_PER_S = 10.0;
constexpr double CENTI_S_PER_S = 100.0;
constexpr double MS_PER_S = 1000.0;
constexpr double CM_PER_M = 100.0;
constexpr double TENTH_MICRO_DEG_PER_DEG = 10000000.0;
constexpr double DECI_M_PER_M = 10.0;
constexpr double TWENTIETH_M_PER_M = 20.0;
constexpr double FIFTIETH_M_PER_M = 50.0;
constexpr double FIFTIETH_G_PER_M_PER_SEC_SQR = 5.10204081633;
constexpr double ONE_AND_A_HALF_DEG = 1.5;
constexpr double ONE_AND_A_HALF_DEG_PER_DEG = 0.666666666666;
constexpr double CENTI_DEG_PER_DEG = 100.0;
constexpr double THREE_TENTHS_DEG = 0.3;
constexpr double EIGHTIETH_DEG_PER_DEG = 80.0;
constexpr double DEG_360_OVER_65535_PER_DEG = 182.041666097;
constexpr double UNCHANGED = 1.0;
constexpr double SEC_PER_MIN = 60.0;
constexpr double DECA_DEG_PER_DEG = 10.0;
constexpr double KG_PER_HALF_METRIC_TON = 500.0;
constexpr double TWO_KG = 2.0;
constexpr double TENTH_GRAM_PER_GRAM = 10;
constexpr double TWO_TENTHS_MICRO = 0.02;
constexpr double FORTY_DEGREES_C = 40.0;
constexpr double HPA_PER_TWO_PA = 0.002;
constexpr double HALF_PERCENT_PER_HUNDRED_PERCENT = 200.0;


// Integer Varients
constexpr uint64_t NS_PER_MS_INT = 1000000;


}  // namespace units
}  // namespace j2735_convertor
