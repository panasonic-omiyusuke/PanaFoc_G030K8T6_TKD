#ifndef ADPDC_H
#define ADPDC_H

#include "adc.h" /* for test */

#define NOW_DATA (0)
#define PAST_DATA (1)

#define ADSTNUM_SHUNT (0)                                               /* DTCでのPN間電圧A/D変換結果の格納配列番号の先頭 *//* @267swRRD[23/09/22] */
#define ADC_SERIES_NUM (8)

#define ADSTNUM_VT	(NUM_DTCSHUNT)       /* @215swDOD[22/05/20] */                               /* DTCでの端子電圧A/D変換結果の格納配列番号の先頭 */
#define ADSTNUM_VPN (NUM_DTCSHUNT + 1)   /* @215swDOD[22/05/20] */                       /* DTCでのPN間電圧A/D変換結果の格納配列番号の先頭 */

#define REVERSE_JUDGE (100) 			/* 逆回転判定ｶｳﾝﾀ：位置検出レベル逆行回数(通常増加/低下していく誘起電圧レベルが、低下/増加していった回数) */
										/* 差圧起動用アルゴリズムでない場合は無効化のために100に設定 *//* @216swDOD[22/05/25] */

#define FORCED_THRESHOLD_SHIFT (2)	/* Forced中の閾値SHIFT量 2設定の場合⇒1/8 PN 遅れ方向へ *//* @155swDOD[21/07/01] */
#define KYUYU_THRESHOLD_SHIFT (3) /* 給油回転数中の閾値SHIFT量 9/16 PN */
#define THRESHOLD_SHIFT_NUM (8) /* for running 1/2 PN */
#define THRESHOLD_SPIKE_END_SHIFT_NUM (1) /* スパイク終了判定の閾値用  Ｔｈｒｅｓｈｏｌｄ = Ｖpn - ((1/2 * Ｖpn) >> THRESHOLD_SPIKE_END_SHIFT_NUM) */

/* 配列参照用 */
#define SELTU 0
#define SELTV 1
#define SELTW 2
#define SELPN 3

#define D_FULLON	(0xffff)					/* ｷｬﾘｱ周波数に関わらずベタONとなるON時間 */

/* #define SAMPLINGTIME	(5.6) */
#define TIMERC_SMPL1	(UI08)(SAMPLINGTIME * 0 * D_fTRC)
#define TIMERC_SMPL2	(UI08)(SAMPLINGTIME * 1 * D_fTRC)						/*  */
#define TIMERC_SMPL3	(UI08)(SAMPLINGTIME * 2 * D_fTRC)						/*  */


/* 差圧起動 */
/* #define D_FIRSTHALF	(D_PWMPERIOD + 1)			 @007MzDOD[16/12/21]  差圧起動 */
/* #define D_CUR_THRESHOLD 175			*/		/* 1.0A 相当 */		/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_CUR_THRESHOLD 185			*/		/* 1.3A 相当 */
#define D_CUR_THRESHOLD 200						/* 1.8A 相当 */
/* #define D_CUR_THRESHOLD 215			*/		/* 2.2A 相当 */


/********************************************/
/*		型宣言  							*/
/********************************************/
enum StateCheckSpike {
	kWaitZeroCross,
	kWaitCommutaion,
	kWaitSpikeEdge,
	kStopReverse,
	kOnlyAdConvert,
	kWaitFreeRunZeroCross,
	kStopCheckSpike,
};	/* @025yhOTT[18/03/01] */


/********************************************/
/*		公開関数							*/
/********************************************/
void InitAdc(void);
void InitDtc(void);
void InitDifferentialActivation(void);
void set_dtc_enspkpdc(UI16);
UI08 get_dtc_vpn(void);
UI16 get_dtc_vpn12b(void);		/* @318swROD[24/11/14] */
void ResetDTC2(UI08 ad_port_num, UI08 pdc_expect_hilo);
void ReinitDma(void);
void INT_ADDTC(void);

void SetSkipAdSpikeIfNeed(void);/* @263swDOD[23/07/26] */

UI08 CurrentOffset(void);						/* @304swDOD[24/09/02] */

UI08 IsFinishCurrentOffset(void);				/* @304swDOD[24/09/02] */

void StoreCurrentBuf(SI16 hold_cur);			/* @304swDOD[24/09/02] */
UI16 Calc_current_ave(void);					/* @304swDOD[24/09/02] */
/********************************************/
/*		外部参照							*/
/********************************************/
extern void ClearCurrentBuf(void);
extern UI16 Calc_current_ave(void);
extern void INT_ADDTC_BRAKE(void);
extern UI08 ucPDC_portsel; /* pmd1g:vPMD_PdcPtnSet */
extern UI08 adconv_cnt;  /* 1転流区間の中でad変換した回数 */
extern UI08	ucPMD_PDjudge;
extern uint16_t ad_data_arr[ADC_SERIES_NUM]; /* adc データの格納先 */
extern enum StateCheckSpike chk_spike; /* vPMD_start */
													/* Spike検出												*/
													/* 	0:位置検出許可中(ｽﾊﾟｲｸ検出済み) 						*/
													/* 	1:ｽﾊﾟｲｸ検出許可待ち(ｽﾊﾟｲｸ未検出)ｽﾊﾟｲｸonまでにﾀｲﾑﾗｸﾞ発生する際への対応*/
													/* 	2:ｽﾊﾟｲｸ検出許可中(ｽﾊﾟｲｸ未検出) 							*/


#endif	/* #ifndef ADPDC_H */

