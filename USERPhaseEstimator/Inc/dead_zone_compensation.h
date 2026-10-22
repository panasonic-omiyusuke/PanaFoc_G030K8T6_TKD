/******************************************************************
<File Name> ：dead_zone_compensation.h
<Function> 	：死区补偿
<Program> 	：
<Author> 	：曾祥海
<Date> 		：2026/8/1
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/

#ifndef DEAD_ZONE_COMPENSATION_H
#define DEAD_ZONE_COMPENSATION_H

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"

#define HYST_MAX_PLUS     950
#define HYST_MAX_MINUS   -950
#define HYST_MIN_PLUS     235
#define HYST_MIN_MINUS   -235
#define HYST_CURR_DIFF    715
#define VOLT_COMP_MAX     150           /* 220*1.3*1.414/702*32767*1.5/200 */
#define COMP_FACTOR_MAX   16384
#define COMP_FACTOR_DIFF  11469
#define COMP_FACTOR_MIN   4915    /* COMP_FACTOR_MIN = 0.3* COMP_FACTOR_MAX */

typedef struct
{
int16_t hDead_Zone_Gain;
int16_t hyst_high_max;
int16_t hyst_low_max;
int16_t hyst_high_min;
int16_t hyst_low_min;
int16_t hyst_curr_diff;
int16_t V_dead_unit;
int16_t hGain_Ia;
int16_t hGain_Ib;
int16_t hGain_Ic;
int16_t hSign_Ia;
int16_t hSign_Ib;
int16_t hSign_Ic;
int16_t hGain_max;
int16_t hGain_mid;
int16_t hGain_min;
int16_t hGain_diff;
}DEAD_ZONE_Handle_t;

extern DEAD_ZONE_Handle_t dead_zone_comp_M1;
void dead_zone_comp_Init(DEAD_ZONE_Handle_t *pHandle);
abc_t dead_zone_compensation(DEAD_ZONE_Handle_t *pHandle,ab_t Iab,uint16_t Vbus);
alphabeta_t voltage_limit(alphabeta_t alphabeta_comp,alphabeta_t Valphabeta);

#endif /* DEAD_ZONE_COMPENSATION_H */
