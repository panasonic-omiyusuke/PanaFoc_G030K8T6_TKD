#ifndef PWM_H
#define PWM_H
#include "Sys_type.h"
#include "vipdc.h"

/*******************************************
		マクロ定義
 *******************************************/
#define SWITCHING_HEAD (0)
#define SWITCHING_TAIL (1)

#define D_STOP_INDEX     (6)
#define D_STANDBY_INDEX  (7)

#define OFF_DETECT_REQ_OFF_TM_US (45)									/* pwm.cから移動 *//* @263swDOD[23/07/26] */
#define OFF_DETECT_REQ_OFF_TM_TRD  (OFF_DETECT_REQ_OFF_TM_US * D_fTRD)	/* pwm.cから移動 *//* @263swDOD[23/07/26] */
#define HYSTERESIS_DUTY 10 /* [%] */									/* pwm.cから移動 *//* @263swDOD[23/07/26] */
#define HYSTERESIS_DIV_NUM (100/HYSTERESIS_DUTY) /* 100/n[%] eg.hys=10% 100/10=10, Hys=5% 100/5=20 *//* pwm.cから移動 *//* @263swDOD[23/07/26] */

#define D_POS_DTECT_TM_US	(43)			/* ※TIMERD_ADDRV 変更時はD_POS_DTECT_TM_USも変更のこと *//* @220swDOD[22/06/01] *//* pwm.cから移動 *//* @263swDOD[23/07/26] */
#define D_POS_DTECT_TM_TRD	(D_POS_DTECT_TM_US * D_fTRD)				/* pwm.cから移動 *//* @263swDOD[23/07/26] */
#define  D_DUTYFULL 1000												/* pwm.cから移動 *//* @263swDOD[23/07/26] */
/*******************************************
		プロトタイプ宣言
 *******************************************/
void SelectSwitchingHeadOrTail(UI08 head_or_tail);

//void CorrectNowFullPhase(SI16 correction_time);
void ResetFirstHalfTimer(void);
void InitDtcPwm(void);
//void NoCorrectNowFullPhase(void);
void InitPwmCompare(void);
void InitPwmCompareReg(void);
void InitPwm(void);
void UpdatePwm(void);
void UpdatePwm2(void);
void SetPwmCarrier(enum eCarrierModeTag mode);
UI08 IsPwmUpdated(void);
UI08 IsPwmFreqVector(void);
UI08 GetTRNS_PWMfreq(void);

void SelectLowArmPolarity(void);
void init_addtct_trdgr(void);
void setDTC_CHGCRR(void);
UI16 unPMDc_GetPWMprd(void);
void PWM_wait(void);
void vPMDc_UpdateDuty(UI16 new_duty);
void vPMDc_UpdateDutyPdcless(UI16 new_duty);
void DisablePwmOut(void);					/* PWM output disable */
void StopPwmTimer(void);
void StartPwmTimer(void);
void SetStandbyDuty(void);

void AddInitPwmTimer(void);

extern UI08 ucPMD_mdidx;
extern UI08 ucTYPE_carrier_freq;	/* @250swDOD[23/05/18] */
extern UI16 unPWMPeriodHi;			/* @257swDOD[23/06/27] */

#ifdef UNIT_TEST
UI16 GetPwmOnMin(void);
UI16 GetAdConversionTime(void);
UI16 GetAdConversion3Times(void);
UI16 GetOffDetectReqOffTimeTrd(void);
UI16 GetHysterisysDuty(void);
UI16 GetDoubleSamplingMinTime(void);
UI16 GetDefaultNonPwmDutyInBoost(void);
UI16 GetDefaultPwmPeriod(void);
UI16 GetTrdCompareStop(void);
UI16 GetForcedHighRpsPwmPeriod(void);
UI16 GetLowRpsPwmPeriod(void);
UI16 GetHighRpsPwmPeriod(void);
UI16 GetTorquCtrlPwmPeriod(void);
UI16 GetTorquCheckPwmPeriod(void);
void unPMDc_3kjdg(unsigned8 buf);
unsigned16 unPMDc_SetPWM_nrm(unsigned8 buf, unsigned16 unPWMPrd);
unsigned16 unPMDc_SetPWM(void);
void SetPwmCompareStop(void);
void SetPwmCompareStandby(void);
void SetPwmComparePosition(void);
void SetPwmCompareForced(void);
void SetPwmCompareDefault(void);
UI08 IsMdoutPosition1st(void);
UI08 IsMdoutPosition2nd(void);
UI08 IsMdoutStop(void);
void CalcHysteresisDutyDetectPwmMode(UI16 pwm_period_trd);
void SelectDetectPwmModeCalc(UI16 duty);
#endif

#endif //PWM_H
