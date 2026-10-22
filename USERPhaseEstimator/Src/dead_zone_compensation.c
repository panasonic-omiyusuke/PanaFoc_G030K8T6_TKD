/******************************************************************
<File Name> ：dead_zone_compensation.c
<Function> 	：死区补偿
<Program> 	：
<Author> 	：曾祥海
<Date> 		：2026/8/1
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/
#include "dead_zone_compensation.h"


DEAD_ZONE_Handle_t dead_zone_comp_M1;
static inline int16_t curr_abs(int16_t Iabc); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */


void dead_zone_comp_Init(DEAD_ZONE_Handle_t *pHandle)
{

pHandle->hyst_high_max = HYST_MAX_PLUS;
pHandle->hyst_low_max = HYST_MAX_MINUS;
pHandle->hyst_high_min = HYST_MIN_PLUS;
pHandle->hyst_low_min = HYST_MIN_MINUS;
pHandle->hGain_max = COMP_FACTOR_MAX;
pHandle->hGain_min = COMP_FACTOR_MIN;
pHandle->hyst_curr_diff = HYST_CURR_DIFF;
pHandle->hGain_diff = COMP_FACTOR_DIFF;
pHandle->hSign_Ia = 1;
pHandle->hSign_Ib = 1;
pHandle->hSign_Ic = 1;
pHandle->hGain_Ia = COMP_FACTOR_MAX;
pHandle->hGain_Ib = COMP_FACTOR_MAX;
pHandle->hGain_Ic = COMP_FACTOR_MAX;

}


abc_t dead_zone_compensation(DEAD_ZONE_Handle_t *pHandle,ab_t Iab,uint16_t Vbus)
{

alphabeta_t Valphabeta;
abc_t Vabc;
int32_t wAux;
int32_t ratioAux;
int32_t hGaindiffAux;
int16_t Iadiff;
int16_t Ibdiff;
int16_t Icdiff;
int16_t Ic = -Iab.a-Iab.b;
int16_t hVbus = Vbus >> 8;   /* 200*2/1.5=266 */
pHandle->V_dead_unit =  -hVbus;        
if(pHandle->V_dead_unit >VOLT_COMP_MAX)
{
pHandle->V_dead_unit = VOLT_COMP_MAX;
}

if(Iab.a>pHandle->hyst_high_min)
{
  pHandle->hSign_Ia = 1;
}
else if(Iab.a<pHandle->hyst_low_min)
{
  pHandle->hSign_Ia = - 1;
}
else
{
}
if(Iab.b>pHandle->hyst_high_min)
{
  pHandle->hSign_Ib = 1;
}
else if(Iab.b<pHandle->hyst_low_min)
{
  pHandle->hSign_Ib = - 1;
}
else
{
}
if(Ic>pHandle->hyst_high_min)
{
  pHandle->hSign_Ic = 1;
}
else if(Ic<pHandle->hyst_low_min)
{
  pHandle->hSign_Ic = - 1;
}
else
{
}

if(curr_abs(Iab.a)>=pHandle->hyst_high_max)
{
  pHandle->hGain_Ia = pHandle->hGain_max;
}
else if(curr_abs(Iab.a)>pHandle->hyst_high_min)
{
  Iadiff = curr_abs(Iab.a) - pHandle->hyst_high_min;
  ratioAux = (int32_t)Iadiff << 14;
  ratioAux = ratioAux/pHandle->hyst_curr_diff;
  hGaindiffAux = ratioAux * pHandle->hGain_diff;
  hGaindiffAux = hGaindiffAux >> 14;
  pHandle->hGain_Ia = pHandle->hGain_min+(int16_t)hGaindiffAux;
}
else
{
  pHandle->hGain_Ia = pHandle->hGain_min;
}
if(curr_abs(Iab.b)>=pHandle->hyst_high_max)
{
  pHandle->hGain_Ib = pHandle->hGain_max;
}
else if(curr_abs(Iab.b)>pHandle->hyst_high_min)
{
  Ibdiff = curr_abs(Iab.b) - pHandle->hyst_high_min;
  ratioAux = (int32_t)Ibdiff << 14;
  ratioAux = ratioAux/pHandle->hyst_curr_diff;
  hGaindiffAux = ratioAux * pHandle->hGain_diff;
  hGaindiffAux = hGaindiffAux >> 14;
  pHandle->hGain_Ib = pHandle->hGain_min+(int16_t)hGaindiffAux;
}
else
{
  pHandle->hGain_Ib = pHandle->hGain_min;
}

if(curr_abs(Ic)>=pHandle->hyst_high_max)
{
  pHandle->hGain_Ic = pHandle->hGain_max;
}
else if(curr_abs(Ic)>pHandle->hyst_high_min)
{
  Icdiff = curr_abs(Ic) - pHandle->hyst_high_min;
  ratioAux = (int32_t)Icdiff << 14;
  ratioAux = ratioAux/pHandle->hyst_curr_diff;
  hGaindiffAux = ratioAux * pHandle->hGain_diff;
  hGaindiffAux = hGaindiffAux >> 14;
  pHandle->hGain_Ic = pHandle->hGain_min+(int16_t)hGaindiffAux;
}
else
{
  pHandle->hGain_Ic = pHandle->hGain_min;
}
wAux = (int32_t)pHandle->hGain_Ia * pHandle->V_dead_unit;
wAux = wAux * pHandle->hSign_Ia;
wAux = wAux>>14;
Vabc.a =(int16_t)wAux;

wAux = (int32_t)pHandle->hGain_Ib * pHandle->V_dead_unit;
wAux = wAux * pHandle->hSign_Ib;
wAux = wAux>>14;
Vabc.b =(int16_t)wAux;

wAux = (int32_t)pHandle->hGain_Ic * pHandle->V_dead_unit;
wAux = wAux * pHandle->hSign_Ic;
wAux = wAux>>14;
Vabc.c =(int16_t)wAux;

return Vabc;

}
alphabeta_t voltage_limit(alphabeta_t alphabeta_comp,alphabeta_t Valphabeta)
{
  int32_t wValpha = Valphabeta.alpha+alphabeta_comp.alpha;
  int32_t wVbeta = Valphabeta.beta+alphabeta_comp.beta;
  alphabeta_t hValphabeta;
if (wValpha>INT16_MAX)
{
 hValphabeta.alpha = INT16_MAX;
}
else if(wValpha<-INT16_MAX)
{
 hValphabeta.alpha = -INT16_MAX;
}
else
{
hValphabeta.alpha = (int16_t)wValpha;
}
if (wVbeta>INT16_MAX)
{
 hValphabeta.beta = INT16_MAX;
}
else if(wVbeta<-INT16_MAX)
{
 hValphabeta.beta = -INT16_MAX;
}
else
{
hValphabeta.beta = (int16_t)wVbeta;
}
return hValphabeta;
}
static inline int16_t curr_abs(int16_t Iabc) /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
{
   return Iabc >= 0 ? Iabc:-Iabc;
}
