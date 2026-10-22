/**************************************************************************
<Title>			:adc_i.h
<Description>	:AD変換関連のヘッダファイル（inline関数専用）
<Author/Date>	:Takeoka / 2020-05-20
<Update>		:
 **************************************************************************/
#ifndef ADC_I_H
#define ADC_I_H
#include "Sys_type.h"
#include "vipdc.h"
#include "PMD1GPar.h"
#include "dtc_i.h"
#include "stm32g0xx_hal.h"
#include "Com_def.h"

#define READSTART_NUM	((UI08)(NUM_DTCVT + NUM_DTCSHUNT + NUM_DTCVPN + NUM_DTCETC))		/* AD変換回数の合計 *//* Re(再)ADstart *//* @219swDOD[22/06/01] */
#define TRDADSTART1_NUM	1

enum eFreeRunAdcMode {kFreeRunSelTuw, kFreeRunSelTvw, kFreeRunSelTvu, kFreeRunSelTwu, kFreeRunSelTwv, kFreeRunSelTuv, kFreeRunSelEnd};

extern ADC_HandleTypeDef hadc1;
extern const uint32_t ad_seq_free_run_table[6];

static inline  UI16 GetAdStartTimingTrc(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI16 ad_start_timing_trc;	/* A/Dスタート時のTRC値を格納 */
	uint32_t timing_gap;
	uint32_t pwm_timer_cnt;
	uint32_t mode_timer_cnt;
	extern 	TIM_HandleTypeDef htim1;
	extern 	TIM_HandleTypeDef htim3;
	mode_timer_cnt = htim3.Instance->CNT;
	pwm_timer_cnt = htim1.Instance->CNT;
	pwm_timer_cnt += (pwm_timer_cnt > htim1.Instance->CCR4 ? 0 : htim1.Instance->ARR);
	timing_gap = pwm_timer_cnt - htim1.Instance->CCR4;
	ad_start_timing_trc = mode_timer_cnt - mTRD2MDTM(timing_gap);
	return ad_start_timing_trc;
}
 UI08 * const GetAdcSrcAddress(UI08 ad_port_num);

 void ClearAdcIntReq(void);

 void SetEtcAdResult(void);	/* ルネマイコン処理？1ﾏｲｺﾝ化で必要っぽい？★ */

 void ResetAdDtc(void);	/* ルネマイコン処理？STで不要？★ */

static inline UI08 bStopAD(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UI32 tmp_cr = hadc1.Instance->CR;
	if( (tmp_cr &  (ADC_CR_ADDIS | ADC_CR_ADEN)) != ADC_CR_ADEN) {
		return 0;
	}
	if( (hadc1.Instance->CR & (ADC_CR_ADSTP | ADC_CR_ADSTART)) == ADC_CR_ADSTART){
		hadc1.Instance->CR |= ADC_CR_ADSTP;
	}
#ifdef UNIT_TEST
	hadc1.Instance->CR &= ~ADC_CR_ADSTP;
	hadc1.Instance->CR &= ~ADC_CR_ADSTART;
#endif
	return 1;
}

#if SOFT_STOP_SW == ACTIVE
 void ResetFreeRunAdDtc(void);
#endif

#endif /* #ifndef ADC_I_H */

