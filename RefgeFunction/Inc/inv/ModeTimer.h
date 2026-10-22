/*
 * ModeTimer.h
 *
 *  Created on: 2020/10/22
 *      Author: 2993017
 */

#ifndef INV_MODETIMER_H_
#define INV_MODETIMER_H_

#include "Sys_type.h"
#include "vipdc.h"
#include "PMD1GPar.h"
#include "angle_control.h"
#include "pwm_i.h"
#include "Com_def.h"

#define WEIGHTING_BIT_SHIFT (1) //速度のLPFの現在と過去分の重みづけのためのビットシフト数
#define MAX_PERIOD (0xffff - D_MTBUFSZ / 2) //転流の最大周期
#define D_INSTANTint	(15)				/*即TRC割込発生用CLK数=10*/  							/* @008MzDOD[16/12/21] */
#define TRCGRA_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRBの設定時の最小値 オーバフロー対策は未実施 */
#define TRCGRB_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRBの設定時の最小値 オーバフロー対策は未実施 */
#define TRCGRC_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRCの設定時の最小値 オーバフロー対策は未実施 */
#define TRCGRD_MIN (unCPU_TRC + D_INSTANTint) /* TRCGRDの設定時の最小値 オーバフロー対策は未実施 */
#ifndef ClearModeTimerIntRequest
#define ClearModeTimerIntRequest(TIM_IT_CCx_Or_TIM_IT_UPDATE) (__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CCx_Or_TIM_IT_UPDATE))
#endif



extern UI08	ucPMDi_mtbufpt;				/* Mode time ﾊﾞｯﾌｧﾎﾟｲﾝﾀ. {0  --  (D_MTBUFSZ - 1)} */
extern TIM_HandleTypeDef htim3;


void InitializeCommutatePeriodRotation(void);
void InitializeCommutatePeriodLowPassFilter(void);
UI16 CalcCommutatePeriodRotationAvg(void);    /* 位置検出間隔÷1回転の転流数の合計で1回転の位置検出周期の平均を算出 */
void SetDummyCntBuf(UI16 dummy_cnt);			/* @263swDOD[23/07/26] */
UI16 CalcSumDummyCnt(void);						/* @263swDOD[23/07/26] */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

static inline  void ClearTrcgrd(void){
	htim3.Instance->CCR3 = 0xffff;
	ClearModeTimerIntRequest(TIM_IT_CC3);
}

/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void CommutatePeriodLowPassFilter(UI16 period){
	extern UI16 unPMDi_MCAPdumy;
	UI16 now_avg;
    now_avg = unPMDi_MCAPdumy;
    now_avg -= (now_avg >> WEIGHTING_BIT_SHIFT);
    now_avg += (period >> WEIGHTING_BIT_SHIFT);
    unPMDi_MCAPdumy = now_avg;
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:2023/07/26 除算切捨値の積算による実回転数算出ズレの修正  @276swRRD[23/11/21]
                :コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void UpdateCommutatePeriodRotation(UI16 period){        /* 1回転の平均速度計算 */
	extern UI16 unPMDi_mtbuf[D_MTBUFSZ];

	/* 一回転の位置のインデックスの更新 */
    if( ucPMDi_mtbufpt >= D_MTBUFSZ - 1 ){ ucPMDi_mtbufpt = 0;}
    else                                 { ucPMDi_mtbufpt++;}

    /* 転流タイミングの最大（最低速）のリミット処理 */
    if( period > MAX_PERIOD ){
        period = MAX_PERIOD;
    } else {}
    unPMDi_mtbuf[ucPMDi_mtbufpt] = period;	/* 位置検出毎に転流間隔を記録。 CalcCommutatePeriodRotationAvg()で1回転分の総和から平均を算出 *//* @276swRRD[23/11/21] */
											/* 四捨五入のための D_MTBUFSZ / 2 の加算はD_MTBUFSZで除算する時のみ必要なので、ここでは削除 */	/* @276swRRD[23/11/21] */
    SetDummyCntBuf(0);										/* @263swDOD[23/07/26] */
}
/**************************************************************************
 **************************************************************************/
static inline void MatchTrcgrbInstant(void){
	ClearModeTimerIntRequest(TIM_IT_CC1);
	htim3.Instance->CCR1 = 1;
	htim3.Instance->EGR |= TIM_EGR_CC1G;
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline UI16 TimeOverlapStartSensorless(void){
    extern sAngleDataTag angle;
	extern UI16 unPMDi_MCAPdumy;
//    return (UI16)(((UI32)unPMDi_MCAPdumy * angle.lap_start) >> (DEGREE_60_RESOLUTION_BIT + 1)); /* 全体を2倍したので、計算結果を更に1/2 */
    return (Deg60RatioAngle2ModeTime(angle.lap_start, unPMDi_MCAPdumy) >> 1); /* lap_startが2倍なので、計算結果を1/2 */
}
/**************************************************************************
 **************************************************************************/
static inline void UpdateTrcgrbAndPwmOutIfNeeded(UI16 time_lap_start){
    UI16 trcgrb_tmp;

    trcgrb_tmp = mMDTM2TRC(time_lap_start); /* モードタイムをタイマ値に換算 */


	ClearModeTimerIntRequest(TIM_IT_CC1);
	htim3.Instance->CCR1 = trcgrb_tmp;

	if(htim3.Instance->CNT >= htim3.Instance->CCR1 ){
		htim3.Instance->EGR |= TIM_EGR_CC1G;
        PWM_out();
	}
}
/**************************************************************************
 **************************************************************************/
static inline void UpdateTrcgrcAndPwmOutIfNeeded(UI16 time_nonlap_start){
    UI16 trcgrc_tmp;

    trcgrc_tmp= mMDTM2TRC(time_nonlap_start); /* モードタイムをタイマ値に換算 */

	ClearModeTimerIntRequest(TIM_IT_CC2);
	htim3.Instance->CCR2 = trcgrc_tmp;
	if(htim3.Instance->CNT >= htim3.Instance->CCR2 ){
		htim3.Instance->EGR |= TIM_EGR_CC2G;
        PWM_out();
	}
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void UpdateTrcgra(UI16 mode_time){ //Not INLINE : TrcgraはVectorで利用のため別ファイルから参照
	UI16 trcgra_tmp;
	trcgra_tmp = mMDTM2TRC(mode_time); /* モードタイムをタイマ値に換算 */

	ClearModeTimerIntRequest(TIM_IT_UPDATE);
	htim3.Instance->ARR = trcgra_tmp;

	if(htim3.Instance->CNT >= htim3.Instance->ARR ){
		htim3.Instance->EGR |= TIM_EGR_UG;
	}
}

/**************************************************************************
<Title>			:ClearTrcgra
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:竹岡
<Date>			:
<Update>		:コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void ClearTrcgra(void){
	htim3.Instance->ARR = 0xffff;
/*	htim3.Instance->SR &= ~TIM_IT_UPDATE; */
	ClearModeTimerIntRequest(TIM_IT_UPDATE);

}


/**************************************************************************
 **************************************************************************/
static inline void ClearTrcgrb(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24) */
	htim3.Instance->CCR1 = 0xffff;
/*	htim3.Instance->SR &= ~TIM_IT_CC1; */
	ClearModeTimerIntRequest(TIM_IT_CC1);
}




/**************************************************************************
 **************************************************************************/
static inline void ClearTrcgrc(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24) */
	htim3.Instance->CCR2 = 0xffff;
/*	htim3.Instance->SR &= ~TIM_IT_CC2; */
	ClearModeTimerIntRequest(TIM_IT_CC2);
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void UpdateTrcgrd(UI16 mode_time){
    UI16 trcgrd_tmp;

    trcgrd_tmp = mMDTM2TRC(mode_time);

	ClearModeTimerIntRequest(TIM_IT_CC3);
	htim3.Instance->CCR3 = trcgrd_tmp;
	if(htim3.Instance->CNT >= htim3.Instance->CCR3 ){
		htim3.Instance->EGR |= TIM_EGR_CC3G;
	}
}

/**************************************************************************
 **************************************************************************/
static inline void MatchTrcgrcInstant(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要(26.9.24) */
	ClearModeTimerIntRequest(TIM_IT_CC2);
	htim3.Instance->CCR2 = 1;
	htim3.Instance->EGR |= TIM_EGR_CC2G;
}
/**************************************************************************
 **************************************************************************/
static inline void MatchTrcgrdInstant(void){
	ClearModeTimerIntRequest(TIM_IT_CC3);
	htim3.Instance->CCR3 = 1;
	htim3.Instance->EGR |= TIM_EGR_CC3G;

} /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

static inline void UpdateTrcgrf(UI16 mode_time){
    UI16 trcgr_tmp;

    trcgr_tmp = mMDTM2TRC(mode_time);

	ClearModeTimerIntRequest(TIM_IT_CC4);
	htim3.Instance->CCR4 = trcgr_tmp;
	if(htim3.Instance->CNT >= htim3.Instance->CCR4 ){
		htim3.Instance->EGR |= TIM_EGR_CC4G;
	} /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
}
static inline  void ClearTrcgrf(void){
	htim3.Instance->CCR4 = 0xffff;
	ClearModeTimerIntRequest(TIM_IT_CC4);
} /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

static inline void mDisableLapTimerInt(void){
	htim3.Instance->DIER &= ~MDTM_INT_LAP_ENABLE; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
}
static inline void mEnableLapTimerInt(void){
	ClearModeTimerIntRequest(TIM_IT_CC1);
	htim3.Instance->DIER |= MDTM_INT_LAP_ENABLE;
} /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

static inline void mDisableDmummyPdcTimerInt(void){
	htim3.Instance->DIER &= ~MDTM_INT_DUMMY_PDC_ENABLE; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
}
static inline void mEnableDmummyPdcTimerInt(void){
	ClearModeTimerIntRequest(TIM_IT_CC4);
	htim3.Instance->DIER |= MDTM_INT_DUMMY_PDC_ENABLE;
}


void ReinitModeTimer(void);

#endif /* INV_MODETIMER_H_ */
