/**************************************************************************
<Title>			:adc.h
<Description>	:AD変換関連のヘッダファイル（関数宣言）
<Author/Date>	:Takeoka / 2020-05-20
<Update>		:
 **************************************************************************/
#ifndef ADC_H
#define ADC_H
#include "Sys_type.h"



void InitDtcAd(void);
void InitAdPort(void);

UI08 IsStoppedAdc(void);
void StartAdcForced(void);
void StopAdcForced(void);
void ClearAdcSetting(void);
void SelectAdcCurrentTrans(void);
void ReinitAdc(void);
void UpdateAdcSequence(uint32_t index);
void ReEnableAdConvertIfNeed(void);

#ifdef UNIT_TEST
UI08 GetPd0UseAd(void);
UI08 GetPd1UseAd(void);
UI08 GetAdSelShunt(void);
UI08 GetRestartNum(void);
UI08 CallGetReAdStart(void);
UI08 CallGetTrdAdStart1(void);

void InitAdc(void);
#endif

#endif
