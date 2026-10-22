#ifndef PWM_I_H
#define PWM_I_H
#include "Sys_type.h"
#include "vipdc.h"
#include "PMD1GPar.h"
#include "Adpdc.h"
#include "angle_control.h"
#include "Pmd1G.h"
#include "adc_i.h"

#include "main.h" /* for test DA出力関数呼び出し用 */
#include "Hogo.h" /* for vPMD_PwmDisable() AD停止時は停止用 */

/*#include "debug_inv.h"*/

#define POS_DETECT_PWM_OFF  (0)
#define POS_DETECT_PWM_ON   (1)

#define unCPU_TRDGR_PWM0	htim1.Instance->ARR
#define unCPU_TRDGR_PWM1	htim1.Instance->ARR

#define unCPU_TRDGR_U 	(TIM1->CCR1)
#define unCPU_TRDGR_V 	(TIM1->CCR2)
#define unCPU_TRDGR_W 	(TIM1->CCR3)
#define unCPU_TRDGR_X 	(TIM1->CCR1)
#define unCPU_TRDGR_Y 	(TIM1->CCR2)
#define unCPU_TRDGR_Z 	(TIM1->CCR3)

#define CC1_ON   (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0) /*TIM_CCMR1_OC1M_Msk*/
#define CC1_PWM  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) /*TIM_CCMR1_OC1M_Msk*/
#define CC2_ON   (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0) /*TIM_CCMR1_OC2M_Msk*/
#define CC2_PWM  (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1) /*TIM_CCMR1_OC2M_Msk*/
#define CC3_ON   (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0) /*TIM_CCMR2_OC3M_Msk*/
#define CC3_PWM  (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) /*TIM_CCMR2_OC3M_Msk*/
#define CC4_PWM  (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1) /*TIM_CCMR2_OC4M_Msk*/
#define OUTPUT_MODE_MASK12 (TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk)
#define OUTPUT_MODE_MASK3  (TIM_CCMR2_OC3M_Msk)

#define PWMOUT_MASK (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)

#define MR1_U_X_PWM CC1_PWM
#define MR1_W_Z_PWM 0
#define MR1_V_Y_PWM CC2_PWM
#define MR2_U_X_PWM 0
#define MR2_W_Z_PWM CC3_PWM
#define MR2_V_Y_PWM 0

#define MR1_U_X_ON CC1_ON
#define MR1_W_Z_ON 0
#define MR1_V_Y_ON CC2_ON
#define MR2_U_X_ON 0
#define MR2_W_Z_ON CC3_ON
#define MR2_V_Y_ON 0
#define TIM_CCER_CCE_U  TIM_CCER_CC1E
#define TIM_CCER_CCE_V  TIM_CCER_CC2E
#define TIM_CCER_CCE_W  TIM_CCER_CC3E
#define TIM_CCER_CCNE_X TIM_CCER_CC1NE
#define TIM_CCER_CCNE_Y TIM_CCER_CC2NE
#define TIM_CCER_CCNE_Z TIM_CCER_CC3NE


extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern const uint32_t output_mode_table12[8];
extern const uint32_t output_mode_table3[8];
extern const uint32_t output_mode_all_pwm_12;
extern const uint32_t output_mode_all_pwm_3;
extern const uint32_t output_mode_table12[8];
extern const uint32_t output_mode_table3[8];
extern const uint32_t output_mode_lap_table12[8];
extern const uint32_t output_mode_lap_table3[8];
extern const uint32_t T_MDOUTptn[8];
extern const uint32_t T_MDOUTlap[8];
extern const uint32_t T_MDOUTgap[8];

extern UI32 next_mdout;
extern UI32 next_ccmr1;
extern UI32 next_ccmr2;



#if DETECT_PWM_OFF == ACTIVE
static inline  UI08 IsDetectPwmOff(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    extern UI08 detect_pwm_mode;
    return (!detect_pwm_mode);
}
#endif




/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
#ifdef RENESAS_TO_ST32
#if 1
/*#define    mStopPwmPD	(hdma_adc1.Instance->CCR &= ~DMA_CCR_EN)*/
/*#define    mStartPwmPD  {DMA1->IFCR |= 0x0000000f; hdma_adc1.Instance->CCR |= DMA_CCR_EN;} set CTEIF1*/
static inline  void StartPwmPositionDetect(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI16 addtct_trdgr[2];
	htim1.Instance->CCR4 = addtct_trdgr[0];
}
#define    mStopPwmPD	vPMD_PwmDisable()
#define    mStartPwmPD  vPMD_PwmEnable() /*set CTEIF1*/
#else
#define    mStopPwmPD
#define    mStartPwmPD
#endif
#else
#define    mStopPwmPD	{ucCPU_TRDIER0 &= 0x01;\
						 ucCPU_TRDIER1 = 0x00;\
						 ucCPU_TRD1IC = 0x00;} /*要検討：TRD1ICは常時0x00のため削除可能？*/
#endif
#ifdef RENESAS_TO_ST32
#define vPMD_ChangeCarrierDisable()
#else
static inline  void vPMD_ChangeCarrierDisable(void){
    ucCPU_TRDIER0 &= ~0x01;												/* 一旦割り込みは禁止 */
}
#endif

static inline  void PresetMdout(UI08 index){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    if( IsAngleGap() ){
    	next_mdout = T_MDOUTgap[index];
    	next_ccmr1 = output_mode_lap_table12[index];
    	next_ccmr2 = output_mode_lap_table3[index];
    }else if( IsAngleLap() ){
    	next_mdout = T_MDOUTlap[index];
    	next_ccmr1 = output_mode_lap_table12[index];
    	next_ccmr2 = output_mode_lap_table3[index];
    } else {
    	next_mdout = T_MDOUTptn[index];
    	next_ccmr1 = output_mode_table12[index];
    	next_ccmr2 = output_mode_table3[index];
    }
}

static inline  void PresetNormalMdout(UI08 index){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    	next_mdout = T_MDOUTptn[index];
    	next_ccmr1 = output_mode_table12[index];
    	next_ccmr2 = output_mode_table3[index];
}
static inline  void PresetShortCircuitBrakingMdout(UI08 index){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    	next_mdout = T_MDOUTptn[D_STANDBY_INDEX];
    	next_ccmr1 = output_mode_all_pwm_12;
    	next_ccmr2 = output_mode_all_pwm_3;
}
static inline  void PresetPositionMdout(UI08 index){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	if( index == D_POS1ST_IDX ){
	    	next_mdout = T_MDOUTlap[D_POS2ND_IDX]; /*lap と通常パタンを切り替え どちらも2ndのindexを利用*/
	} else {
	    	next_mdout = T_MDOUTptn[D_POS2ND_IDX]; /*lap と通常パタンを切り替え どちらも2ndのindexを利用*/
	}
	next_ccmr1 = output_mode_all_pwm_12;
	next_ccmr2 = output_mode_all_pwm_3;

}
static inline  void UpdateMdout(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI32 unPMD_MDOUT;
	extern UI32 ulPMD_CCMR1;
	extern UI32 ulPMD_CCMR2;

	unPMD_MDOUT = next_mdout;							/*�[�d�p�^��01/03/16*/
	ulPMD_CCMR1 = next_ccmr1;
	ulPMD_CCMR2 = next_ccmr2;
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
				:
<Copyright> 	:
 **************************************************************************/
static inline  void SelectPwmPort(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#ifdef RENESAS_TO_ST32
	extern UI32 unPMD_MDOUT;				/* 位置検出の期待値と通電パターン格納用 */
	extern UI32 ulPMD_CCMR1;
	extern UI32 ulPMD_CCMR2;
	uint32_t ccer_tmp;
	uint32_t ccmr_tmp;
	/* commutate */
	ccer_tmp = htim1.Instance->CCER;
	htim1.Instance->CCER =  ((ccer_tmp & ~PWMOUT_MASK)        | unPMD_MDOUT);  /* Port出力 */
	ccmr_tmp = htim1.Instance->CCMR1;
	htim1.Instance->CCMR1 = ((ccmr_tmp & ~OUTPUT_MODE_MASK12) | ulPMD_CCMR1); /*PWM出力設定*/
	ccmr_tmp = htim1.Instance->CCMR2;
	htim1.Instance->CCMR2 = ((ccmr_tmp & ~OUTPUT_MODE_MASK3)  | ulPMD_CCMR2); /*PWM出力設定*/
#else
	extern UI16	unPMD_MDOUT;				/* 位置検出の期待値と通電パターン格納用 */
	extern UI08 ucPMD_p2ptn;
    ucPMD_p2ptn = (0xff ^ ((UI08)(unPMD_MDOUT) & mask));
#endif
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
				:
<Copyright> 	:
 **************************************************************************/
static inline  void SelectAllPwmPort(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#ifdef RENESAS_TO_ST32
	extern UI32	unPMD_MDOUT;				/* 位置検出の期待値と通電パターン格納用 */
	uint32_t ccer_tmp;
	uint32_t ccmr_tmp;
	/* commutate */
	ccer_tmp = htim1.Instance->CCER;
	htim1.Instance->CCER =  ((ccer_tmp & ~PWMOUT_MASK)        | unPMD_MDOUT);  /* Port出力 */
	ccmr_tmp = htim1.Instance->CCMR1;
	htim1.Instance->CCMR1 = ((ccmr_tmp & ~OUTPUT_MODE_MASK12) | output_mode_all_pwm_12); /*PWM出力設定*/
	ccmr_tmp = htim1.Instance->CCMR2;
	htim1.Instance->CCMR2 = ((ccmr_tmp & ~OUTPUT_MODE_MASK3)  | output_mode_all_pwm_3); /*PWM出力設定*/
#else
	extern UI16	unPMD_MDOUT;				/* 位置検出の期待値と通電パターン格納用 */
	extern UI08 ucPMD_p2ptn;
    ucPMD_p2ptn = (0xff ^ ((UI08)(unPMD_MDOUT) & mask));
#endif
}

/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
				:
<Copyright> 	:
 **************************************************************************/
static inline  void SelectOutPort(UI08 mask){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI32	unPMD_MDOUT;				/* 位置検出の期待値と通電パターン格納用 */
	extern UI08 uc_PMC_p2ptnL_pol;
	extern UI08 ucPMD_p2ptn_full;
    UI08 out_port = (UI08)unPMD_MDOUT;
    out_port = (((out_port & 0x02) << 1) | ((out_port & 0x04) >> 1) | (out_port & 0xe8));
    ucPMD_p2ptn_full = ((out_port & mask) ^ uc_PMC_p2ptnL_pol);
}
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
				:
<Copyright> 	:
 **************************************************************************/
static inline  void PwmWaitLap(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

#ifdef RENESAS_TO_ST32
    SelectPwmPort();
#else
	extern UI08	ucPMD_mdidx;				/* ﾓｰﾄﾞｲﾝﾃﾞｯｸｽ６通り. {0 -- 5} 位置検出毎に増or減. */
	extern const UI08 cls_mask_sw_tbl_tail[6];
	extern const UI08 cls_mask_full_tbl_tail[6];
    SelectPwmPort(cls_mask_sw_tbl_tail[ucPMD_mdidx]);
    SelectOutPort(cls_mask_full_tbl_tail[ucPMD_mdidx]);
#endif
}


extern UI16 first_half_timer;
extern volatile UI32 *crr_full;
extern volatile UI16	unPMD_CMPU;							/*U相PWMｵﾝ時間格納変数 */
#define mWaitCompareTimerFull()  {asm(" nop "); asm(" nop "); asm(" nop ");asm(" nop "); asm(" nop "); asm(" nop "); asm(" nop "); asm(" nop ");}
#define D_PWMPERIOD    ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_DEFAULT)-1)	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define D_PWMPERIODLo  ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_LOW)-1)		/* @007MzDOD[16/12/21] */ /* 差圧起動 */
/* #define D_PWMPERIODHi  ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_HIGH)-1) */		/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define D_PWMPERIODHi0  ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_HIGH0)-1)		/* @007MzDOD[16/12/21] */ /* 差圧起動 *//* @250swDOD[23/05/18] */
#define D_PWMPERIODHi1  ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_HIGH1)-1)		/* @007MzDOD[16/12/21] */ /* 差圧起動 *//* @250swDOD[23/05/18] */
#define D_PWMPERIODfrc ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_VECTOR)-1)	/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define D_PWMPERIOD_ECO ((UI16)(((UI32)1000000 * D_fTRD) / D_PWM_FREQUENCY_ECO)-1)
#define D_FIRSTHALF	(D_PWMPERIOD + 1)			/* @007MzDOD[16/12/21] */ /* 差圧起動 */
#define TPROC_ADDRV	 (UI16)(SAMPLINGTIME * (NUM_DTCVT + NUM_DTCSHUNT + NUM_DTCVPN) ) 	/* 駆動処理に必要なAD変換完了までの時間 *//* @219swDOD[22/06/01] */
#define TPROC_ADALL	 (UI16)(SAMPLINGTIME * (NUM_DTCVT + NUM_DTCSHUNT + NUM_DTCVPN + NUM_DTCETC) ) 		/* 全AD変換完了までの時間 *//* @219swDOD[22/06/01] */
#define TIMERD_ADDRV (TPROC_ADDRV * D_fTRD)												/* 駆動処理に必要なAD変換のみを考慮したPWMオフ開始からの戻り時間 ※D_POS_DTECT_TM_US 変更時はTIMERD_ADも変更のこと*//* @219swDOD[22/06/01] */
#define TIMERD_ADALL (TPROC_ADALL * D_fTRD)												/* 全AD変換を考慮したPWMオフ開始からの戻り時間 *//* @219swDOD[22/06/01] */
#define D_PWMONMIN	 (D_REMVRING + TIMERD_ADDRV)   /* PWMｵﾝ幅許容最小時間 */  /* @006axOOM[09/05/11] */

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
static inline  void NoCorrectNowFullPhase(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	*crr_full = first_half_timer;       /* 前回の補正結果をそのまま使用 */
}

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
static inline  UI16 CalcFirstHalfTimer(SI16 correct){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
/*    extern UI16 unPMD_CMPU;*/

    /*limit上限 */
    if( correct > (SI16)D_FIRSTHALF - (SI16)first_half_timer ){					/* 100% -現在のコンペア値(前回算出したコンペア値) 100%を越えるなら100%にする */
        return D_FIRSTHALF;
    }
    /* 下限チェック */
    if( correct < (SI16)unPMD_CMPU - (SI16)first_half_timer ){					/* 補正してPWMON幅以下までは減らさない */
		return unPMD_CMPU + D_PWMONMIN;                                         /* 要検討 unPMD_CMPU + D_PWMONMIN == D_FIRSTHALF */
    }
    /* 通常 */
    return (UI16)((SI16)first_half_timer + correct);						/*correct 補正量（電流抑えるときはマイナス値、電流増やすときは＋値) */
}

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
static inline  void CorrectNowFullPhase(SI16 correction_time){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    /* ベタON相のタイマコンペアを計算 */
    first_half_timer = CalcFirstHalfTimer(correction_time);

#ifdef RENESAS_TO_ST32
#else
    /* 補正結果がTRDタイマを跨ぐ場合は(first_half_timer < TRD0 < *crr_full=今のコンペア値)かチェック */
    if( (*crr_full > unCPU_TRD0) && (first_half_timer < (unCPU_TRD0 + 4*16 + 24)) ){
		/* フル相のコンペア値がTRDを跨ぐ */
        *crr_full = unCPU_TRD0 + 24;        /* コンペア一致する最小値を格納 */
        mWaitCompareTimerFull();			/*TRDが一致するまで待つ(nop = 24に相当)*/
    }

    /*(状態) TRDが現在のコンペア値以上になりました*/
#endif
    *crr_full = first_half_timer;           /* ベタON相の補正結果を格納 */ /* 次のｷｬﾘｱで補正値が反映されて電流 */
}

static inline  void EnableTimerSyncAdc(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
/*hadc1.Instance->CR |= ADC_CR_ADSTART; 連続モードだが、毎回AD停止のため、スタート必要*/
}

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
static inline void vPMD_PwmDisable(void){ /*org inline*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	/* 要検討：AD変換完了割込みの禁止も必要？名前もDisableIntAdStart()みたいな名前に変更？ */
	/*         ADの割込みも禁止したらPwmEnableでAd変換完了割込み許可が必要。ほかに許可必要ないか確認必要 */
#ifdef RENESAS_TO_ST32
	void vPMD_Brake(void);
	UI32 tmp_cr = hadc1.Instance->CR;
	if( (tmp_cr &  (ADC_CR_ADDIS | ADC_CR_ADEN)) != ADC_CR_ADEN) {
		vPMD_Brake();
		return;
	}
	if( (hadc1.Instance->CR & (ADC_CR_ADSTP | ADC_CR_ADSTART)) == ADC_CR_ADSTART){
		hadc1.Instance->CR |= ADC_CR_ADSTP;  /* a要検討 */
	}
	if(bStopAD() == 0){ /*AD変換異常なら 基本的には発生しない*/
		vPMD_Brake();
		return;
	}
	hdma_adc1.Instance->CCR &= ~DMA_CCR_EN;
	hdma_adc1.Instance->CCR &= ~DMA_CCR_TCIE;

/*	htim1.Instance->CCR4 = 0xffff;*/
#else
    ucCPU_TRDIER0 &= 0x01;												/* ��U���荞�݂͋֎~ */
    ucCPU_TRDIER1 = 0x00;												/* ��U���荞�݂͋֎~ */
#endif
}
/**************************************************************************
<Title>			:vPMD_PwmEnable 		;INTTMR
<Function>		:PWM出力割込許可設定
<Description>	:信号検出開始時機に本処理を実行しPWM関連割込を許可
<Input>			:D_PWMon_INT_ENABLSEL0 or D_PWMon_INT_ENABLSEL1
<Return>		:trdier0,trdgrb1,trd0ic,trd1ic
<Author>		:-Outi	作成
<Date>			:2008-0411
<Update>		:Ver REF081-0.00 2008-04-11
				:Ver REF081-0.02 2008-06-05 -Outi 信号検出処理修正 	@000ahOTM[08/06/05](発覚08/05/21)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void vPMD_PwmEnable(void){	/*org inline*/						/*PWM出力割込許可設定	@000ahOTM[08/06/05](発覚08/05/21)*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#ifdef RENESAS_TO_ST32
	/*	extern uint16_t ad_data_arr[8];*/
	/*	while(hadc1.Instance->CR & ADC_CR_ADSTART){*/
	/*	hadc1.Instance->CR |= ADC_CR_ADSTP;*/
	/*	if(hadc1.Instance->CR & ADC_CR_ADSTART){*/
	/*	}*/
	DMA1->IFCR |= DMA_IFCR_CGIF1; /*0 割込み要求クリア*/

	hdma_adc1.Instance->CNDTR = ADC_SERIES_NUM;
	/*	hdma_adc1.Instance->CMAR = (__IO uint32_t)ad_data_arr;			   0 メモリの転送先アドレス*/
	hdma_adc1.Instance->CCR |= DMA_CCR_TCIE;
	hdma_adc1.Instance->CCR |= DMA_CCR_EN;
	hadc1.Instance->CR |= ADC_CR_ADSTART; /* 連続モードだが、毎回AD停止のため、スタート必要 */ 

#else
	extern UI08 pwmint_enable_trd0;
	extern UI08 pwmint_enable_trd1;

	ucCPU_TRDSR0;/*ucCPU_TRDSR0を一度読みだすことで、クリア可能になるため、読み出しを実施*/
	ucCPU_TRDSR0 &= 0x01;
	ucCPU_TRDSR1;                                                  /*ucCPU_TRDSR1を一度読みだすことで、クリア可能になるため、読み出しを実施*/
	ucCPU_TRDSR1 = 0;
	ucCPU_TRDIER0 = (pwmint_enable_trd0 | (ucCPU_TRDIER0 & 0x01)); /*要注意：本来であれば(pwmint_enable_trd0 & ~0x01)が良いが、処理時間短縮のため省略。元データで確実にbit0を0にすること*/
	ucCPU_TRDIER1 = pwmint_enable_trd1;

#endif
}


/**************************************************************************
<Title>			:PWM_out 		;vPMD_Brake(),vPMD_start(),vPMDc_TC1(),vPMDi_Setcomtptn(),INTTMR3(),INTPDC(),INTTMR
<Function>		:駆動信号出力処理
<Description>	:駆動出力portのラッチ(PWM信号ラッチ含む)にptnを転送
<Input>			:ucPMD_p2ptn
<Return>		:p2,trdoer1
<Author>		:-Outi	作成
<Date>			:2008-0411
<Update>		:Ver REF081-0.00 2008-04-11
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline void PWM_out(void){			/*org inline*/				/*駆動信号出力処理*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#ifdef RENESAS_TO_ST32
	extern volatile UI32 *nxt_phase_sw;				/* 次のオン素子 上	 */
	extern volatile UI32 *crr_phase_sw;				/* 現在のオン素子 上 */
	extern volatile UI32 *nxt_full;
	extern volatile UI32 *crr_full;
	extern UI16  unPMDc_PWMprd_calc;
	extern eStateTag  ePMD_drvmode;					/* State of Pmd1G */
	extern UI16 first_half_timer;
	/*	extern UI32 unPMD_MDOUT;*/
	/*	extern UI32 ulPMD_CCMR1;*/
	/*	extern UI32 ulPMD_CCMR2;*/
	extern volatile UI16	unPMD_CMPU;
	/*	htim1.Instance->CCER = unPMD_MDOUT;*/
	/*	htim1.Instance->CCMR1 = ulPMD_CCMR1;*/
	/*	htim1.Instance->CCMR2 = ulPMD_CCMR2;*/

	htim1.Instance->EGR |= TIM_EGR_COMG; /*update pwm out*/
	crr_phase_sw   = nxt_phase_sw;
	if( ePMD_drvmode >= D_running ){return;} /*上下同時PWM必要なら下の処理必要*/

    crr_full = nxt_full;														/* @007MzDOD[16/12/21] */ /* 差圧起動 */
    *crr_phase_sw = unPMD_CMPU;													/* @007MzDOD[16/12/21] */ /* 差圧起動 */
    *crr_full = first_half_timer;												/* @007MzDOD[16/12/21] */ /* 差圧起動 */
	htim1.Instance->ARR = unPMDc_PWMprd_calc; /* carrier frequency */
#else
	extern UI08 ucPMD_p2ptn;
	extern UI08 ucPMD_p2ptn_full;
	extern volatile UI16 *nxt_phase_sw;				/* 次のオン素子 上	 */
	extern volatile UI16 *crr_phase_sw;				/* 現在のオン素子 上 */
	extern volatile UI16 *nxt_full;
	extern volatile UI16 *crr_full;
	extern eStateTag  ePMD_drvmode;					/* State of Pmd1G */
	extern UI16 first_half_timer;
	extern volatile UI16	unPMD_CMPU;

    ucCPU_TRDOER1 = ucPMD_p2ptn;
    ucCPU_P2 = ucPMD_p2ptn_full;

	crr_phase_sw   = nxt_phase_sw;

	if( ePMD_drvmode >= D_running ){return;} /*上下同時PWM必要なら下の処理必要*/

    crr_full = nxt_full;														/* @007MzDOD[16/12/21] */ /* 差圧起動 */
    /* pre_call_pwm_flg = call_setpwm_flg; */ /* setDTC_CHGCRR呼び出し後～DTC実行までの間に呼ばれることが無いため処理不要 */
    *crr_phase_sw = unPMD_CMPU;													/* @007MzDOD[16/12/21] */ /* 差圧起動 */
    *crr_full = first_half_timer;												/* @007MzDOD[16/12/21] */ /* 差圧起動 */
    /* setDTC_CHGCRR呼び出し後～DTC実行までの間に呼ばれることが無いため処理不要 */
    /* if( call_setpwm_flg !=  pre_call_pwm_flg ){
         *crr_phase_sw = unPMD_CMPU;
         *crr_full = first_half_timer;
        } */
#endif
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/09/01
<Update>	:
**************************************************************************/
static inline void PresetPwmOut(uint32_t index) { /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	uint32_t ccer_tmp;
	uint32_t ccmr_tmp;

	/* commutate */
	ccer_tmp = htim1.Instance->CCER;
	htim1.Instance->CCER = ((ccer_tmp & ~PWMOUT_MASK) | T_MDOUTptn[index]);  /* Port出力 */
	ccmr_tmp = htim1.Instance->CCMR1;
	htim1.Instance->CCMR1 = ((ccmr_tmp & ~OUTPUT_MODE_MASK12) | output_mode_table12[index]); /*PWM出力設定*/
	ccmr_tmp = htim1.Instance->CCMR2;
	htim1.Instance->CCMR2 = ((ccmr_tmp & ~OUTPUT_MODE_MASK3)  | output_mode_table3[index]); /*PWM出力設定*/


}


#endif /*#ifndef PWM_I_H*/
