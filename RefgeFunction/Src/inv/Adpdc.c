#include "Sys_type.h"

#include "vipdc.h"

#include "Com_def.h"
#include "PMD1GPar.h"
#include "Adpdc.h"
#include "angle_control.h"

#include "Pmd1G.h"
#include "Timea.h"
#include "Pmd_int.h"

#include "pwm.h"
#include "pwm_i.h"
#include "adc.h"
#include "adc_i.h"

#include "vi_phase_differential.h"
#include "dtc_i.h"

#include "RefSpeed.h"
#include "system_ram.h"
#include "PositionDetect.h"
#include "Dtyctrl.h"

#include "soft_stop.h"

#include "debug_inv.h"

#include "ModeTimer.h"

#include "InvCommonFunc.h"
#include "brake.h"

/*from angle_control.c start*/
sAngleDataTag angle;						/* 通電角に関する情報の構造体 */
/*from angle_control.c end*/

/*from PositionDetect.c start*/
UI08	ucPMD_PDjudge;						/*位置ﾎﾟｰﾄ判定値 格納変数 */
/*from PositionDetect.c end*/

extern TIM_HandleTypeDef htim3;

uint16_t ad_data_arr[ADC_SERIES_NUM]; //adc データの格納先


/* #define PWM_OFF_THRESHOLD (3) */ /* PWM OFF 検出での閾値 */
#define PWM_OFF_THRESHOLD (8) /* for test GMCC */
/* #define PWM_OFF_THRESHOLD (5) */ /* for test pana T143 @20rps */

/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
enum StateCheckSpike  chk_spike;					/* Spike検出												*/
													/* 	0:位置検出許可中(ｽﾊﾟｲｸ検出済み) 						*/
													/* 	1:ｽﾊﾟｲｸ検出許可待ち(ｽﾊﾟｲｸ未検出)ｽﾊﾟｲｸonまでにﾀｲﾑﾗｸﾞ発生する際への対応*/
													/* 	2:ｽﾊﾟｲｸ検出許可中(ｽﾊﾟｲｸ未検出) 							*/
UI08 is_skip_check_spike;							/* ｽﾊﾟｲｸ確認ｽｷｯﾌﾟﾌﾗｸﾞ *//* @263swDOD[23/07/26] */
/*** static ***/

enum StateZeroCrossMode {
    eNoDetect,
    eForcedLo,
    eForcedHi,
    eOverlapLo,
    eOverlapHi,
    eRunningLo,
    eRunningHi,
    eVectorLo,
    eVectorHi,
};

/********************************************/
/*		外部変数参照						*/		/* @037MzDOD[14/01/27] */
/********************************************/

/*******************************************
		プロトタイプ宣言
 *******************************************/

void INT_ADDTC(void);
static UI08 SwitchDetectZeroCrossMode(UI08 pdc_expect_hilo);         /* 出力によってIsZeroCross***とCorrectMcap***を選択 */
UI16 Calc_current_ave(void);
UI16 TimeNonlapStartSensorless(void);

UI08 ucPDC_cpADVPN[3];
UI08 adconv_cnt;
UI16 slope_volt_terminal;
UI16 dtc_enspkpdc;
UI08 ucMAIN_vltnow;				/*現在の電圧(AIN0A/D変換値) */
SI32 current_buf[D_MTBUFSZ]; 				/* 機械角ごとの電流平均値算出用バッファ */						/* @304swDOD[24/09/02] */
SI16 current_use[D_MTBUFSZ]; 				/* AD入力bit数と同じレンジ(12bit)の機械角ごとの電流平均値 */	/* @304swDOD[24/09/02] */
//const UI08 src_adstrt = 0x01;
const UI08 src_dtcen1 = READSTART;
const UI08 src_adicclr = 0x00;
const UI08 src_adcnt = 0x03;
const UI08 portout[8] = {0x40,0x00,0x40,0x00,0x40,0x00,0x40,0x00};


UI08 *psrc_adinsl;

const UI08 slope_volt_terminal_table[] = {	/* @010ywOOM[17/12/15] */
    60, 60, 60, 60, 60, 60, 60, 60, 60, 60,
    60, 54, 50, 46, 43, 40, 37, 35, 33, 32,
    30, 29, 27, 26, 25, 24, 23, 22, 21, 21,
    20, 19, 19, 18, 18, 17, 17, 16, 16, 15,
    15, 15, 14, 14, 14, 13, 13, 13, 12, 12,
    12, 12, 12, 11, 11, 11, 11, 11, 10, 10,
    10, 10, 10, 10, 9,  9,  9,  9,  9,  9,
    9,  8,  8,  8,  8,  8,  8,  8,  8,  8,
    7,  7,  7,  7,  7,  7,  7,  7,  7,  7,
    7,  7,  7,  6,  6,  6,  6,  6,  6,  6,
    6,  6,  6,  6,  6,  6,  6,  6,  6,  5,
    5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
    5,  5,  5,  5,  5,  5,  5,  5,
};

UI08 adcVt[2];															/* 前回と今回の端子電圧の変換結果を交互に格納 */
UI08 adcVpn[2];															/* 前回と今回のPN間電圧の変換結果を交互に格納 */
UI16 mcap_trc[2];														/* 前回と今回のMCAP*2(TRCと同じオーダー)を交互に格納 */
UI08 dtc_vpn;															/* 母線電圧 */
UI16 dtc_vpn12b;														/* 母線電圧 12bit *//* @318swROD[24/11/14] */
UI08 addtct_cnt;														/* スパイク検出後のA/D変換回数(スパイク検出時が1回目) */

UI16 mcapsub;
UI16 current_sub[2];
UI08 reverse_rotation_cnt;
UI08 detect_zero_cross_mode;

void InitDifferentialActivation(void){
    reverse_rotation_cnt = 0;
    detect_zero_cross_mode = 0;
	ResetFirstHalfTimer();
    mcapsub = D_FCDPRD;
    current_sub[0] = current_sub[1] = 0;
    slope_volt_terminal = 0;
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void InitDtc(void){
	ReinitDma();
}

/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
static inline  void SetDtcTime1st(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI08 dtctiming;
    dtctiming = 0;														/* 位置検出の1回目は0からスタート */
}

/**************************************************************************
<Title>			:
<Function>		:位置検出毎にvPMD_PdcPtnSetから呼ばれる。resetDTCがINLINEのため呼び出し関数を用意
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void ResetDTC2(UI08 ad_port_num, UI08 pdc_expect_hilo){
    detect_zero_cross_mode = SwitchDetectZeroCrossMode(pdc_expect_hilo); /* �^�]���[�h�ƃ[���N���X�̊��Ғl�͏�G�b�W�����G�b�W���̑g������I�� */
//    UpdateAdcSequence(ucPMD_mdidx); /* AD変換するポートを選択 */
	SetDtcTime1st();							 /* �_�u���T���v�����O�p�ɕϊ��^�C�~���O�𑁂����ɐݒ�A�_�u���T���v�����O�łȂ��ꍇ�͌��ʂȂ� */
}

/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:s：比較データ、*d:センター値格納用
<Return>		:A/D変換スタートからセンター値までの時間
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   char sel_mid(const UI16 s[], unsigned char *d){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    register UI16 a = *s, b = *(s + 2), c = *(s + 4); /*先頭アドレス、2個後のアドレス、更に2個後のアドレスのデータを平均する*/ /*AD検討 220512*/
    *d = (((UI16)a + (UI16)b + (UI16)c)>>4) / 3;
    return TIMERC_SMPL2;														/* 何回目のA/D変換かでMCAP値を調整する */
}
/**************************************************************************
<Title>			:sel_mid12b
<Function>		:12bitの母線電圧AD値を取得する。
<Description>	:3つのレジスタに格納された母線電圧AD値の平均を計算し、12bit値のまま格納する。
<Input>			:ADバッファアドレス、AD値格納先アドレス
<Return>		:ucPMD_FcLoflg
<Author>		:Takida	作成	@257swDOD[23/06/27]
<Date>			:2024/10/22
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
static inline   short sel_mid12b(const UI16 s[], unsigned short *d){	/* @318swROD[24/11/14] */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    register UI16 a = *s, b = *(s + 2), c = *(s + 4); /*先頭アドレス、2個後のアドレス、更に2個後のアドレスのデータを平均する*/
    *d = ((UI16)a + (UI16)b + (UI16)c) / 3;
    return TIMERC_SMPL2;														/* 何回目のA/D変換かでMCAP値を調整する */
}
static inline UI16 average3(const UI16 s[]){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    register UI16 a = *s, b = *(s + 2), c = *(s + 4); /*先頭アドレス、2個後のアドレス、更に2個後のアドレスのデータを平均する*/ /*AD検討 220512*/
    return (((UI16)a + (UI16)b + (UI16)c)) / 3;

}

/**************************************************************************
<Title>			:
<Function>		:
<Description>	:スパイク終了検出時の処理
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   void DetectedSpikeEnd(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	ClearTrcgrf();
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   UI08 IsSpikeEndLow(UI08 threshold_volt){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 terminal_volt_2nd = (ad_data_arr[ADSTNUM_VT+2] >> 4);/* 端子電圧とPN間電圧を交互に取得するためADSTNUM_VT+2 220512 */
    UI08 terminal_volt_3rd = (ad_data_arr[ADSTNUM_VT+4] >> 4);/* 端子電圧とPN間電圧を交互に取得するためADSTNUM_VT+4 220512 */
    /* 誘起電圧0クロスは下エッジを期待⇒スパイクは上エッジを期待 */
	if( terminal_volt_2nd < threshold_volt ){return FALSE;}
	if( terminal_volt_3rd < threshold_volt ){return FALSE;}	/* 終わり2回分≒9μsのA/Dが閾値を超えてなければスパイク継続 */

    if( terminal_volt_2nd < terminal_volt_3rd){             /* 誘起電圧0クロスは下エッジを期待。値は検出され難い大きいほうの電圧を採用 */
        adcVt[NOW_DATA] = terminal_volt_3rd;
    }
    else {
        adcVt[NOW_DATA] = terminal_volt_2nd;
    }
    return TRUE;											/* スパイクの終わりを設定 */
}

static inline   UI08 IsSpikeEndHigh(UI08 threshold_volt){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 terminal_volt_2nd = (ad_data_arr[ADSTNUM_VT+2] >> 4);/* 端子電圧とPN間電圧を交互に取得するためADSTNUM_VT+2 220512 */
    UI08 terminal_volt_3rd = (ad_data_arr[ADSTNUM_VT+4] >> 4);/* 端子電圧とPN間電圧を交互に取得するためADSTNUM_VT+4 220512 */

    /* 誘起電圧0クロスは上エッジを期待⇒スパイクは下エッジを期待 */
	if( terminal_volt_2nd > threshold_volt ){return FALSE;}
	if( terminal_volt_3rd > threshold_volt ){return FALSE;}	/* 終わり2回分≒9μsのA/Dが閾値を超えてなければスパイク継続 */

    if( terminal_volt_2nd > terminal_volt_3rd){             /* 誘起電圧0クロスは上エッジを期待。値は検出され難い大きいほうの電圧を採用 */
        adcVt[NOW_DATA] = terminal_volt_3rd;
    }
    else {
        adcVt[NOW_DATA] = terminal_volt_2nd;
    }
    return TRUE;											/* スパイクの終わりを設定 */
}

static inline   UI08 IsSpikeEnd(UI08 v, UI08 *vt){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 threshold_volt;
    switch(detect_zero_cross_mode){
    case eForcedLo:
    case eRunningLo:
        threshold_volt = (v >> THRESHOLD_SPIKE_END_SHIFT_NUM);
        break;
    case eForcedHi:
    case eRunningHi:
        threshold_volt = dtc_vpn - (v >> THRESHOLD_SPIKE_END_SHIFT_NUM);
        break;
    case eOverlapLo:
        threshold_volt = (v >> THRESHOLD_SPIKE_END_SHIFT_NUM);
        break;
    case eOverlapHi:
        threshold_volt = dtc_vpn - (v >> THRESHOLD_SPIKE_END_SHIFT_NUM);
        break;
    case eVectorLo:
        threshold_volt = v;
        break;
    case eVectorHi:
        threshold_volt = v;
        break;
	default:
		threshold_volt = v;
    }

    switch(detect_zero_cross_mode){
    case eNoDetect:
    default:
        return FALSE;

    case eForcedLo:
    case eRunningLo:
    case eOverlapLo:
    case eVectorLo:
        return IsSpikeEndLow(threshold_volt);

    case eForcedHi:
    case eRunningHi:
    case eOverlapHi:
    case eVectorHi:
        return IsSpikeEndHigh(threshold_volt);
    }
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   SI16 CorrectFirstHalfTimer(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
/* #define CORRECT_GAIN 	((SI16)(D_PWMONMIN >> 2))			取り合えずの値  ⇒ AD_SHUNT  */
#define CORRECT_GAIN	(132)								/* 取り合えずの値 */
#define CORRECT_MAX 	((SI16)( 0x7fff / CORRECT_GAIN))	/* ゲインをかけた時に超過しないように */
#define CORRECT_MIN 	((SI16)(-0x7fff / CORRECT_GAIN))	/* 同上 */
    SI16 correct_duty;

    current_sub[PAST_DATA] = current_sub[NOW_DATA];												/* 前回の差分を格納 */
    current_sub[NOW_DATA] = (SI16)D_CUR_THRESHOLD - (SI16)((UI16)(ad_data_arr[ADSTNUM_SHUNT] >> 4));	/* 今回の差分を格納 右辺第二項は暗黙の型変換で問題ないもののsigned intを利用のため明示 *//* @266swDOD[23/09/22] */

    correct_duty = (current_sub[NOW_DATA] + current_sub[NOW_DATA] - current_sub[PAST_DATA]);	/* 補正量算出(ﾁｭｰﾆﾝｸﾞ含めた演算式) 目標値に近い場合は、制御量を減らすという意図 */

    /* 上限チェック */
    if( correct_duty > CORRECT_MAX ){
        return CORRECT_MAX * CORRECT_GAIN;
    }
    /* 下限チェック */
    if( correct_duty < CORRECT_MIN ){
        return CORRECT_MIN * CORRECT_GAIN;
    }
    /* 通常 */
    return (correct_duty * CORRECT_GAIN);
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
void SuppressCurrent(void){
    SI16 correction_timer;

	if( ePMD_drvmode != D_forced ){return;}  /* 補正しない */
    if( adconv_cnt < 2 ){					/* 転流 1回目のAD値取得の時は前回転流時の補正値をそのまま採用する(転流直後はシャント電圧が下がってしまう(でも電流が下がっているわけではないよ) */
        NoCorrectNowFullPhase();
        return;
    }

    /* ベタON相の補正量計算[TRD] */
    correction_timer = CorrectFirstHalfTimer();

    /* ベタON相のタイマコンペアを計算 */
	CorrectNowFullPhase(correction_timer);
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   void UpdatePastData(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
     /* 前回のデータの更新 */
    mcap_trc[PAST_DATA] = mcap_trc[NOW_DATA];
    adcVpn[PAST_DATA] = adcVpn[NOW_DATA];
    adcVt[PAST_DATA] = adcVt[NOW_DATA];
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   UI08 DecideVoltPN(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    sel_mid(&ad_data_arr[ADSTNUM_VPN], &dtc_vpn);
    adcVpn[NOW_DATA] = dtc_vpn;
    return dtc_vpn;
}
/**************************************************************************
<Description>	:12bitPN間電圧を格納する
<Author/Date>	:Takida 2024/10/22
<Update>	:
**************************************************************************/
static inline   UI16 DecideVoltPN12b(void){	/* @318swROD[24/11/14] */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    sel_mid12b(&ad_data_arr[ADSTNUM_VPN], &dtc_vpn12b);
    return dtc_vpn12b;
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
UI08 SwitchDetectZeroCrossMode(UI08 pdc_expect_hilo){
    switch(ePMD_drvmode){
    case D_forced:
        return (eForcedLo + pdc_expect_hilo);

    case D_running:
        if( IsAngleLapOrGap() )
          return (eOverlapLo + pdc_expect_hilo);
        return (eRunningLo + pdc_expect_hilo);

    case D_vector:
        return (eVectorLo + pdc_expect_hilo);

    default:
        return eNoDetect;
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
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   UI08 IsZeroCrossDefault(UI08 dummy){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return FALSE;
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
#if DETECT_PWM_OFF == ACTIVE
static inline  UI08 IsZeroCrossForcedHiOff(UI08 half_volt_pn){ /* 制限：half_volt_pn <= 127 */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
	if( addtct_cnt <= 1 ){return FALSE;}				        /* ｽﾊﾟｲｸ検出直後の1回目のAD値は無視(ｽﾊﾟｲｸ電圧を誘起電圧と誤検出しない対策) */

//	volt_threshold = (half_volt_pn >> 4);
    volt_threshold = PWM_OFF_THRESHOLD;

//	OUTPUT_DA(A, volt_threshold);
	if( adcVt[NOW_DATA] < volt_threshold ){return FALSE;}
	if( adcVt[PAST_DATA] < volt_threshold ){return FALSE;}
    reverse_rotation_cnt = 0;								/* 逆転ｶｳﾝﾀｸﾘｱ						 */ /* @029MgOTT[17/04/11] */
    return TRUE;											/* 位置検出 確定					*/
}

static inline  UI08 IsZeroCrossForcedLoOff(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
    if( addtct_cnt <= 1 ){return FALSE;}                     /* ｽﾊﾟｲｸ検出直後の1回目のAD値は無視(ｽﾊﾟｲｸ電圧を誘起電圧と誤検出しない対策) */

//	volt_threshold = dtc_vpn - (half_volt_pn >> 4);
	volt_threshold = dtc_vpn - PWM_OFF_THRESHOLD;

//	OUTPUT_DA(A, volt_threshold);
	if( adcVt[NOW_DATA] > volt_threshold ){return FALSE;}
    if( adcVt[PAST_DATA] > volt_threshold ){return FALSE;}
    reverse_rotation_cnt = 0;								/* 逆転ｶｳﾝﾀｸﾘｱ						 */ /* @029MgOTT[17/04/11] */
    return TRUE;											/* 位置検出 確定					*/
}

static inline  UI08 IsZeroCrossRunningHiOff(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
	if( addtct_cnt <= 1 ){return FALSE;}				        /* ｽﾊﾟｲｸ検出直後の1回目のAD値は無視(ｽﾊﾟｲｸ電圧を誘起電圧と誤検出しない対策) */

#ifdef FIRST_HALF_SW
	volt_threshold =  dtc_vpn - PWM_OFF_THRESHOLD;    /* half_volt_pnはvpn/2のためMaxは127を想定 ⇒ 上限判定せず */
#else
	volt_threshold = PWM_OFF_THRESHOLD;
#endif //FIRST_HALF_SW

	if( adcVt[NOW_DATA] < volt_threshold )  return FALSE;
    return TRUE;											/* 位置検出 確定					*/
}

static inline  UI08 IsZeroCrossRunningLoOff(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
    if( addtct_cnt <= 1 ){return FALSE;}                     /* ｽﾊﾟｲｸ検出直後の1回目のAD値は無視(ｽﾊﾟｲｸ電圧を誘起電圧と誤検出しない対策) */

#ifdef FIRST_HALF_SW
	volt_threshold = PWM_OFF_THRESHOLD;    /* half_volt_pnはvpn/2のためMaxは127を想定 ⇒ 上限判定せず */
#else
	volt_threshold = dtc_vpn - PWM_OFF_THRESHOLD;
#endif //FIRST_HALF_SW


    if( adcVt[NOW_DATA] > volt_threshold )  return FALSE;
	return TRUE;											/* 位置検出 確定					*/
}
#endif

static inline   UI08 IsZeroCrossForcedHi(UI08 half_volt_pn){ /* 制限：half_volt_pn <= 127 */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;

  /*--- AD値 ｽｸﾘｰﾆﾝｸﾞ ---*/
    if( addtct_cnt <= 1 ){return FALSE;}				        /* ｽﾊﾟｲｸ検出直後の1回目のAD値は無視(ｽﾊﾟｲｸ電圧を誘起電圧と誤検出しない対策) */

  /*--- ｾﾞﾛｸﾛｽ閾値算出 ---*/
    volt_threshold = half_volt_pn + (half_volt_pn >> FORCED_THRESHOLD_SHIFT);    /* half_volt_pnはvpn/2のためMaxは127を想定 ⇒ 上限判定せず */

  /*--- ｾﾞﾛｸﾛｽ判定 ---*/									/* 強制転流時は誘起電圧検出不安定(速度低い・duty細くﾘﾝｷﾞﾝｸﾞ影響)なので2回連続閾値越えでｾﾞﾛｸﾛｽ確定とする */
	if( adcVt[NOW_DATA] < volt_threshold ){return FALSE;}
	if( adcVt[PAST_DATA] < volt_threshold ){return FALSE;}

	/*(状態) ｾﾞﾛｸﾛｽ確定 */

 /*--- ﾛｰﾀ-逆回転判定 ---*/
    if( adcVt[NOW_DATA] < adcVt[PAST_DATA] ){               /* 誘起電圧立上がり傾向なら 		*/
        reverse_rotation_cnt ++;                            /* 逆転ｶｳﾝﾀ ｲﾝｸﾘﾒﾝﾄ   				*/
        if( reverse_rotation_cnt >= REVERSE_JUDGE ){		/* 逆転ｶｳﾝﾀ 5回以上なら(1転流中) 	*/
			chk_spike = kStopReverse;	                    /* 逆転判定確定						*/
        }
        return FALSE;                                       /* 位置検出 未確定					*/
    }

	/*(状態) ｾﾞﾛｸﾛｽ確定 かつ 傾きも立上り方向 */
    reverse_rotation_cnt = 0;								/* 逆転ｶｳﾝﾀｸﾘｱ						 */ /* @029MgOTT[17/04/11] */

    return TRUE;											/* 位置検出 確定					*/
}
/**************************************************************************
 **************************************************************************/
static inline   UI08 IsZeroCrossForcedLo(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
  /*--- AD値 ｽｸﾘｰﾆﾝｸﾞ ---*/
    if( addtct_cnt <= 1 ){return FALSE;}                     /* ｽﾊﾟｲｸ検出直後の1回目のAD値は無視(ｽﾊﾟｲｸ電圧を誘起電圧と誤検出しない対策) */

  /*--- ｾﾞﾛｸﾛｽ閾値算出 ---*/
    volt_threshold = half_volt_pn - (half_volt_pn >> FORCED_THRESHOLD_SHIFT);    /* half_volt_pnはvpn/2のためMaxは127を想定 ⇒ 上限判定せず */

  /*--- ｾﾞﾛｸﾛｽ判定 ---*/                                    /* 強制転流時は誘起電圧検出不安定(速度低い・duty細くﾘﾝｷﾞﾝｸﾞ影響)なので2回連続閾値越えでｾﾞﾛｸﾛｽ確定とする */
	if( adcVt[NOW_DATA] > volt_threshold ){return FALSE;}
    if( adcVt[PAST_DATA] > volt_threshold ){return FALSE;}

	/*(状態) ｾﾞﾛｸﾛｽ確定 */

 /*--- ﾛｰﾀ-逆回転判定 ---*/
    if( adcVt[NOW_DATA] > adcVt[PAST_DATA] ){               /* 誘起電圧立上がり傾向なら 		*/
        reverse_rotation_cnt ++;                            /* 逆転ｶｳﾝﾀ ｲﾝｸﾘﾒﾝﾄ   				*/
        if( reverse_rotation_cnt >= REVERSE_JUDGE ){		/* 逆転ｶｳﾝﾀ 5回以上なら(1転流中) 	*/
			chk_spike = kStopReverse;	                    /* 逆転判定確定						*/
        }
        return FALSE;                                       /* 位置検出 未確定					*/
    }

    /*(状態) ｾﾞﾛｸﾛｽ確定 かつ 傾きも立上り方向 */
    reverse_rotation_cnt = 0;                               /* 逆転ｶｳﾝﾀｸﾘｱ						 */ /* @029MgOTT[17/04/11] */

    return TRUE;                                            /* 位置検出 確定					*/
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   UI08 IsZeroCrossRunningHi(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;

  /*--- AD値 ｽｸﾘｰﾆﾝｸﾞ ---*/
    if( (mcap_trc[NOW_DATA] < dtc_enspkpdc) && (addtct_cnt <= 1) ){return FALSE;}    /* ｽﾊﾟｲｸ電圧が狭い時は、ｽﾊﾟｲｸ直後のAD変換結果は無視 		 */
                                                                                    /* (つまり、ｽﾊﾟｲｸ電圧広い時は、ｽﾊﾟｲｸ直後1回目AD値も使用する) */	/* 遅れすぎて、すぐ0ｸﾛｽ判断しないといけない状態 */

    /*--- ｾﾞﾛｸﾛｽ閾値算出 ---*/

	if(uc100MS_pumpct < PUMPCT_END){												/* 給油回転数動作中 */	/* @024ywOTT[18/03/01] */
		volt_threshold = half_volt_pn + (half_volt_pn >> KYUYU_THRESHOLD_SHIFT);	/* リンギングノイズを拾いやすいため、閾値を母船電圧の９／１６に設定 */	/* @024ywOTT[18/03/01] */
	}else{																			/* @024ywOTT[18/03/01] */
        volt_threshold = half_volt_pn - ((UI16)half_volt_pn >> THRESHOLD_SHIFT_NUM);      /* 入力の時点でvolt_thresholdは1/2 さらに1/2で進み気味で位置検出をするよう設定 */
	}

  /*--- ｾﾞﾛｸﾛｽ判定 ---*/
    if( adcVt[NOW_DATA] < volt_threshold ){return FALSE;}                            /* 立上り閾値越えてなければ終了								 */
		/*SpikeEndの値が入ってる場合がある*/
	/* (状態) ｾﾞﾛｸﾛｽ確定 */
    return TRUE;                                                                    /* 位置検出 確定										     */
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   UI08 IsZeroCrossRunningLo(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;

  /*--- AD値 ｽｸﾘｰﾆﾝｸﾞ ---*/
    if( (mcap_trc[NOW_DATA] < dtc_enspkpdc) && (addtct_cnt <= 1) ){return FALSE;}	/* ｽﾊﾟｲｸ電圧が狭い時は、ｽﾊﾟｲｸ直後のAD変換結果は無視 		 */
                                                                                	/* (つまり、ｽﾊﾟｲｸ電圧広い時は、ｽﾊﾟｲｸ直後1回目AD値も使用する) */	/* 遅れすぎて、すぐ0ｸﾛｽ判断しないといけない状態 */

  /*--- ｾﾞﾛｸﾛｽ閾値算出 ---*/

	if(uc100MS_pumpct < PUMPCT_END){												/* 給油回転数動作中 */	/* @024ywOTT[18/03/01] */
		volt_threshold = half_volt_pn - (half_volt_pn >> KYUYU_THRESHOLD_SHIFT);	/* リンギングノイズを拾いやすいため、閾値を母船電圧の７／１６に設定 */	/* @024ywOTT[18/03/01] */
	}else{
        volt_threshold = half_volt_pn + ((UI16)half_volt_pn >> THRESHOLD_SHIFT_NUM);  	/* 入力の時点でvolt_thresholdは1/2 さらに1/2で進み気味で位置検出をするよう設定 */
	}

  /*--- ｾﾞﾛｸﾛｽ判定 ---*/
    if( adcVt[NOW_DATA] > volt_threshold ){return FALSE;}							/* 立上り閾値越えてなければ終了								 */
		/*SpikeEndの値が入ってる場合がある*/
	/* (状態) ｾﾞﾛｸﾛｽ確定 */
    return TRUE;                                     							    /* 位置検出 確定											 */
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
#if DETECT_PWM_OFF == ACTIVE
static inline  UI08 IsZeroCross(UI08 zero_cross_mode, UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	extern UI08 detect_pwm_mode;
    switch(zero_cross_mode + 9 * detect_pwm_mode){
    case eNoDetect:
        return IsZeroCrossDefault(half_volt_pn);

    case eForcedLo:
        return IsZeroCrossForcedLoOff(half_volt_pn);
    case eForcedHi:
        return IsZeroCrossForcedHiOff(half_volt_pn);

    case eOverlapLo:
    case eRunningLo:
		return IsZeroCrossRunningLoOff(half_volt_pn);
    case eOverlapHi:
    case eRunningHi:
		return IsZeroCrossRunningHiOff(half_volt_pn);

    case eVectorLo:
        return IsZeroCrossRunningLo(half_volt_pn);
    case eVectorHi:
        return IsZeroCrossRunningHi(half_volt_pn);

    case (eNoDetect + 9):
        return IsZeroCrossDefault(half_volt_pn);

    case (eForcedLo + 9):
        return IsZeroCrossForcedLo(half_volt_pn);
    case (eForcedHi + 9):
        return IsZeroCrossForcedHi(half_volt_pn);

    case (eOverlapLo + 9):
    case (eRunningLo + 9):
    case (eVectorLo + 9):
		return IsZeroCrossRunningLo(half_volt_pn);
    case (eOverlapHi + 9):
    case (eRunningHi + 9):
    case (eVectorHi + 9):
		return IsZeroCrossRunningHi(half_volt_pn);

    default:
        return FALSE;
    }

}
#else
static inline   UI08 IsZeroCross(UI08 zero_cross_mode, UI08 half_volt_pn){
    switch(zero_cross_mode){
    case eNoDetect:
        return IsZeroCrossDefault(half_volt_pn);

    case (eForcedLo):
        return IsZeroCrossForcedLo(half_volt_pn);
    case (eForcedHi):
        return IsZeroCrossForcedHi(half_volt_pn);

    case (eOverlapLo):
    case (eRunningLo):
    case (eVectorLo):
		return IsZeroCrossRunningLo(half_volt_pn);
    case (eOverlapHi):
    case (eRunningHi):
    case (eVectorHi):
		return IsZeroCrossRunningHi(half_volt_pn);

    default:
        return FALSE;
    }
}
#endif
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   UI16 CorrectMcapDefault(UI08 dummy){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return mcap_trc[NOW_DATA];
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   UI16 CorrectMcapCommon(UI16 differential_volt_threshold){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI16 correction_mcap;
    /* mcapの補正量の計算と補正結果を返す */
    correction_mcap = slope_volt_terminal * differential_volt_threshold;
    if( correction_mcap > mcapsub ){ /* 戻る時間がA/Dのキャプチャ時間を超過していないか一応確認 */
        return mcap_trc[PAST_DATA];   /* 戻る時間は最大でも前キャリアのキャプチャ時間まで */
    }
    return (mcap_trc[NOW_DATA] - correction_mcap); /* A/Dのキャプチャ時間から傾きに応じて時間を戻す */
}

/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   void CalcSlope(UI16 differential_volt_terminal){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    /* slope_volt_terminalの計算 */
	if( addtct_cnt < 3 ){return;}
	if( differential_volt_terminal <= 5 ){return;}

    slope_volt_terminal -= (slope_volt_terminal >> 4);
    slope_volt_terminal += ((mcapsub / differential_volt_terminal) >> 4); /* [注意]キャリアが7kくらいになるとmcapsub<<4がオーバーフロー */
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   UI16 CorrectMcapForcedHi(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
    UI08 differential_volt_terminal;
    UI08 differential_volt_threshold;

	/*--- 2値の誘起電圧ADよりｾﾞﾛｸﾛｽ位置を補正 ---*/						/* 処理時間に余裕あるので線形近似して精度確保 								  */
    mcapsub = mcap_trc[NOW_DATA] - mcap_trc[PAST_DATA];                 /* 位置検出後 少なくとも2回以上ad変換後のため、常に mcap_trc[0] > mcap_trc[1] */

    differential_volt_terminal = adcVt[NOW_DATA] - adcVt[PAST_DATA];
    CalcSlope((UI16)differential_volt_terminal);

    volt_threshold = 0;		        /* 入力の時点でhalf_volt_pnは1/2 さらに1/2で進み気味で位置検出をするよう設定 */
    differential_volt_threshold = adcVt[NOW_DATA] - volt_threshold;
    return CorrectMcapCommon((UI16)differential_volt_threshold);
}
/**************************************************************************
<Description>	:
<Author/Date>	:takeoka 2021/06/09
<Update>	:
**************************************************************************/
static inline   UI16 CorrectMcapForcedLo(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI08 volt_threshold;
    UI08 differential_volt_terminal;
    UI08 differential_volt_threshold;

	/*--- 2値の誘起電圧ADよりｾﾞﾛｸﾛｽ位置を補正 ---*/						/* 処理時間に余裕あるので線形近似して精度確保 								  */
    mcapsub = mcap_trc[NOW_DATA] - mcap_trc[PAST_DATA];                 /* 位置検出後 少なくとも2回以上ad変換後のため、常に mcap_trc[0] > mcap_trc[1] */
                                                                        /* IsZeroCrossForcedHiでadcon_cnt1以下はFALSEになるので本関数コールされないので、2以上の判定値不良。でもCorrectRunningは1以下でもコールされるので判定値いれてるよ */
    differential_volt_terminal = adcVt[PAST_DATA] - adcVt[NOW_DATA];
    CalcSlope(differential_volt_terminal);

    volt_threshold =  dtc_vpn;               /* 入力の時点でhalf_volt_pnは1/2 さらに1/2で進み気味で位置検出をするよう設定 */
    differential_volt_threshold = volt_threshold - adcVt[NOW_DATA];
    return CorrectMcapCommon(differential_volt_threshold);
}

/**************************************************************************
<Title>			:速度・誘起電圧AD → ｾﾞﾛｸﾛｽ補正値
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   void SelectSlopeTable(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#define END_SLOPE_TABLE_NUM (127)
    UI08 table_index = (UI08)(((UI16)ucSPD_rpmh * dtc_vpn) >> 8);

    if( table_index > END_SLOPE_TABLE_NUM ){
        table_index = END_SLOPE_TABLE_NUM;
    }
    slope_volt_terminal = (slope_volt_terminal_table[table_index] << 1);
}

static inline   UI16 CorrectMcapRunningHi(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI16 differential_volt_threshold;
    UI08 volt_threshold;

#ifdef FIRST_HALF_SW
	volt_threshold = dtc_vpn - (half_volt_pn >> 4);	/* リンギングノイズを拾いやすいため、閾値を母船電圧の９／１６に設定 */	/* @024ywOTT[18/03/01] */
#elif DETECT_PWM_OFF == ACTIVE
	if(IsDetectPwmOff()){
        volt_threshold = PWM_OFF_THRESHOLD; /* �v���� 0�̕����x��͏��Ȃ��B����ő��x���΂�� 2021/06/09 takeoka */
    } else {
		volt_threshold = half_volt_pn + ((UI16)half_volt_pn >> THRESHOLD_SHIFT_NUM);
    }
#else
        volt_threshold = half_volt_pn + ((UI16)half_volt_pn >> THRESHOLD_SHIFT_NUM);
#endif


 /*--- 2値の誘起電圧ADよりｾﾞﾛｸﾛｽ位置を補正 ---*/								/* ﾃｰﾌﾞﾙ固定値参照で処理時間の短縮。Running中は速度変動ないので精度求めない */
    SelectSlopeTable();                                                         /* 速度と電圧 → ﾃｰﾌﾞﾙより傾きを取得	 									*/

    if( adconv_cnt >= 2 )                                                       /* ?? Forcedにはなくてよい？ */
      mcapsub = mcap_trc[NOW_DATA] - mcap_trc[PAST_DATA];

    differential_volt_threshold = adcVt[NOW_DATA] - volt_threshold;
    return CorrectMcapCommon(differential_volt_threshold);
}

static inline   UI16 CorrectMcapRunningLo(UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI16 differential_volt_threshold;
    UI08 volt_threshold;
#ifdef FIRST_HALF_SW
	volt_threshold = (half_volt_pn >> 4);
#elif DETECT_PWM_OFF == ACTIVE
	if(IsDetectPwmOff()){
	    volt_threshold = dtc_vpn - PWM_OFF_THRESHOLD;/* �v���� dtc_vpn �̕����x��͏��Ȃ��B����ő��x���΂�� 2021/06/09 takeoka */
	} else {
        volt_threshold = half_volt_pn - ((UI16)half_volt_pn >> THRESHOLD_SHIFT_NUM);
	}
#else
    volt_threshold = half_volt_pn - ((UI16)half_volt_pn >> THRESHOLD_SHIFT_NUM);
#endif

	SelectSlopeTable();

    if( adconv_cnt >= 2 )
      mcapsub = mcap_trc[NOW_DATA] - mcap_trc[PAST_DATA];

    differential_volt_threshold = volt_threshold - adcVt[NOW_DATA];
    return CorrectMcapCommon(differential_volt_threshold);
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline   UI16 AdjustMcap(UI08 zero_cross_mode, UI08 half_volt_pn){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    switch(zero_cross_mode){
    case eNoDetect:
        return CorrectMcapDefault(half_volt_pn);
    case eForcedLo:
        return CorrectMcapForcedLo(half_volt_pn);
    case eForcedHi:
        return CorrectMcapForcedHi(half_volt_pn);
    case eOverlapLo:
    case eRunningLo:
    case eVectorLo:
       return CorrectMcapRunningLo(half_volt_pn);  /* overlap lo */
    case eOverlapHi:
    case eRunningHi:
    case eVectorHi:
        return CorrectMcapRunningHi(half_volt_pn);

    default:
        return 0;
    }
}

/******************************************************************
<Title>			:SetSkipAdSpikeIfNeed
<Function>		:ｽﾊﾟｲｸ判断結果が転流前後を判断する
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:2023/07/26 シャント電流検出によるうねり抑制制御追加 @263swDOD[23/07/26]
<Copyright>		:Panasonic Co. , Ltd.
******************************************************************/
void SetSkipAdSpikeIfNeed(void){				/* @263swDOD[23/07/26] */
    UI32 now_tm1_cnt = htim1.Instance->CNT;
    if(now_tm1_cnt > (D_PWMPERIODfrc - D_POS_DTECT_TM_TRD)){
        is_skip_check_spike = 1;
    }
}


/**************************************************************************
<Title>			:ModeWaitSpikeEdge
<Function>		:ｽﾊﾟｲｸ検出処理を行いｽﾊﾟｲｸの状態変数を返す
<Description>	:
<Input>			:
<Return>		:
<Author>		:竹岡
<Date>			:2023/07/26
<Update>		:
<Copyright>		:Panasonic Co. , Ltd.
 **************************************************************************/
enum StateCheckSpike ModeWaitSpikeEdge(UI08 half_volt_pn){				/* @263swDOD[23/07/26] */
    UI08 spike_end_state;
    enum StateCheckSpike tmp_chk_spike;

    tmp_chk_spike = chk_spike;
    spike_end_state = IsSpikeEnd(half_volt_pn, &adcVt[NOW_DATA]); /* 判定処理関数 第二引数には端子電圧を格納 */
    if( ucPMD_VctrMode == kDriveVector ) {
        if( is_skip_check_spike == 1 ){              /* 処理なし */
            is_skip_check_spike = 0;

        } else if( spike_end_state ) {     /* スパイク終わりなら */
            vZX_chkspk();                   /* 端子電圧の立下りを検出して、状態 *//* @008MzDOD[16/12/21] *//* 位置検出処理 *//* @008MzDOD[16/12/21] */
            tmp_chk_spike = kWaitZeroCross;     /* スパイク後の誘起電圧の有無やタイミングで通電角を増減させるために0クロスチェックに移行 */

        } else if( ucZX_ptn == DPTN_XON ) {
            vZX_chkspk();                   /* 端子電圧の立下りを検出して、状態 *//* @008MzDOD[16/12/21] *//* 位置検出処理 *//* @008MzDOD[16/12/21] */
            tmp_chk_spike = kOnlyAdConvert;

        } else {/* 処理なし */}
    } else if( spike_end_state ) {         /* スパイク終わりなら */
        DetectedSpikeEnd();                 /* スパイク検出処理 */
        tmp_chk_spike = kWaitZeroCross;         /* 次回位置検出時は誘起電圧位置検出を行う */

    } else {/* 処理なし */}
    return tmp_chk_spike;
}

#if 0
/**************************************************************************
<Title>			:INT_ADDTC
<Function>		:AD変換結果の取り込み、平均化、電流値算出、ｽﾊﾟｲｸ検出、位置検出、ブレーキ制御、次のAD変換設定
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:17/12/20 3.5V シャント抵抗による電流検知回路廃止に伴い、処理削除 吉田
				:2023/07/26 シャント電流検出によるうねり抑制制御追加 瀧田 @263swDOD[23/07/26]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void INT_ADDTC(void){
    UI08 half_volt_pn;					/* spike_end_stateはModeWaitSpikeEdge(UI08 half_volt_pn)内に移動 *//* @263swDOD[23/07/26] */
	SI16 adjusted_mcap_trc;
    SI16 large, small;

	mDisableInterrupt();				/* ﾏｲｺﾝ機能ﾚｼﾞｽﾀ操作前に多重割込み禁止する *//* @267swRRD[23/09/22] */
	DMA1->IFCR |= DMA_IFCR_CTCIF1; //0 割込み要求クリア
	hadc1.Instance->ISR = ADC_ISR_EOS; //flag クリア
	vPMD_PwmDisable();

    UpdatePastData();
#if SOFT_STOP_SW == ACTIVE /* SOFT_STOP_SW */
    if( chk_spike == kWaitFreeRunZeroCross ) {
    	large = average3(&ad_data_arr[ADSTNUM_VT]);
    	small = average3(&ad_data_arr[ADSTNUM_VPN]);
        SoftStopControl(large, small);
	} else {
#endif /* SOFT_STOP_SW */

		/* PN間電圧のAD結果を確定させて、1/2にする */
		DecideVoltPN();
		half_volt_pn = (dtc_vpn >> 1);
		DecideVoltPN12b(); /* コンプレッサ巻線保護制御用に12bitPN間電圧のAD結果を確定させる *//* @318swROD[24/11/14] */

		/*0  端子電圧の取得 */
		sel_mid(&ad_data_arr[ADSTNUM_VT], &adcVt[NOW_DATA]); /* const source[0-2], destination */
		mcap_trc[NOW_DATA] = GetAdStartTimingTrc();
		
		UpdateMaxCurrent((SI16)ad_data_arr[ADSTNUM_SHUNT]);				/* @263swDOD[23/07/26] */

		adconv_cnt++;
#if (NUM_DTCSHUNT > 0)	/* NUM_DTCSHUNT > 0 */		/* @266swDOD[23/09/22] */
		SuppressCurrent();/* *//* @016yxDOD[17/12/20] */
#endif	/* NUM_DTCSHUNT > 0 */
		/*spike 終わりを判定 */
		if( chk_spike == kWaitSpikeEdge ) { /* スパイク検出許可中なら */
		    chk_spike = ModeWaitSpikeEdge(half_volt_pn);				/* @263swDOD[23/07/26] */
	        addtct_cnt = 0;												/* @263swDOD[23/07/26] */
		}
		/* 位置検出を実施 */
		if( chk_spike == kWaitZeroCross ) { /* 位置検出許可中なら */
			addtct_cnt++; /* 位置検出判定回数のインクリメント スパイク終わり検出時に0クリア */
			if( IsZeroCross(detect_zero_cross_mode, half_volt_pn) ) { /* 判定値算出 */
				adjusted_mcap_trc = AdjustMcap(detect_zero_cross_mode, half_volt_pn); /* 補正処理 */
				renzokuhassei = 0; /* スパイクの終了を判定できなかった回数をクリア */
				if( ucPMD_VctrMode == kDriveVector ) { //位相差同期運転の通常時のみ判定を行う 移行中は時間も短かく処理が異なるため、無視
					if(ucZX_ptn == DPTN_UOFF_LAP){
						unPMD_MCAP = mTRC2MDTM(adjusted_mcap_trc); //INTTMR3でタイマ自体はクリアされるため、ここでは位置検出間隔のみ記録
						vZX_chkspk(); /* 端子電圧の立下りを検出して、状態 *//* @008MzDOD[16/12/21] *//* 位置検出処理 */														/* @008MzDOD[16/12/21] */
					}
                    chk_spike = kOnlyAdConvert;							/* @263swDOD[23/07/26] */
				} else {
					htim3.Instance->CNT -= adjusted_mcap_trc;
					ClearModeTimerIntRequest(TIM_IT_CC_ALL);
				/*	mStopPwmPD; *//* 先頭での処理と重複のためPWMでのDTC禁止処理を削除 *//* @263swDOD[23/07/26] */
					unPMD_MCAP = mTRC2MDTM(adjusted_mcap_trc); /* MCAPに格納[モードタイマベース] */													/* @008MzDOD[16/12/21] */
					INTPDC(); /* 位置検出処理 */																		/* @008MzDOD[16/12/21] */
				}
			}
		}
#if SOFT_STOP_SW == ACTIVE /* SOFT_STOP_SW */
	}
#endif /* SOFT_STOP_SW */
	switch (chk_spike) {
	default:
	case kWaitZeroCross:
	case kWaitSpikeEdge:
	case kStopReverse:
	case kOnlyAdConvert:
#if SOFT_STOP_SW == ACTIVE /* SOFT_STOP_SW */
	case kWaitFreeRunZeroCross:
#endif /* SOFT_STOP_SW */
		vPMD_PwmEnable();
		break;
	case kWaitCommutaion:
		break;
	}
	mEnableInterrupt();
}
#endif


/**************************************************************************
<Title>			:INT_ADDTC_BRAKE
<Function>		:AD変換結果の取り込み、平均化、電流値算出、ｽﾊﾟｲｸ検出、位置検出、ブレーキ制御、次のAD変換設定
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:17/12/20 3.5V シャント抵抗による電流検知回路廃止に伴い、処理削除 吉田
				:2023/07/26 シャント電流検出によるうねり抑制制御追加 瀧田 @263swDOD[23/07/26]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void INT_ADDTC_BRAKE(void){
    SI16 large, small;
    PWM_U_TMREG = ((D_PWMPERIODfrc >> 4) * 15);		 /* ブレーキ中のDuty */
	PWM_V_TMREG = ((D_PWMPERIODfrc >> 4) * 15);		 /* ブレーキ中のDuty */
	PWM_W_TMREG = ((D_PWMPERIODfrc >> 4) * 15);		 /* ブレーキ中のDuty */
    htim1.Instance->EGR |= TIM_EGR_COMG; //update pwm out

    large = average3(&ad_data_arr[ADSTNUM_VT]);
    small = average3(&ad_data_arr[ADSTNUM_VPN]);
    SoftStopControl(large, small);
    
	vPMD_PwmEnable();
    mEnableInterrupt();
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
#if 1
void chk_vltfluction(UI08 tmp_vltnow){
#define VLT_MTRSTOP		(5)												/* 瞬低復帰不可電圧 */
#define CNT_VLTDETECT	(20)											/* 電圧の最大値をリセットする周期 */
	static UI08 vltmax, vltmax_p;
	static UI08 vltmax_dtctprd;
	vltmax_dtctprd++;												/* 最大電圧検出判定範囲 */
	if(vltmax < tmp_vltnow){										/* 現在の電圧が最大電圧を上回る */
		vltmax = tmp_vltnow;										/* 最大電圧更新 */
	}

	/*vltmax_p = vltrecovery(vltmax,vltmax_p);*/

	if(vltmax_dtctprd >= CNT_VLTDETECT){							/* 最大電圧リフレッシュ周期？ */
		/* cless_undervlt_long(vltmax); */
		vltmax_dtctprd = 0;											/* 最大電圧リフレッシュ周期カウントをクリア */
		vltmax_p = vltmax;											/* 前回の周期の最大電圧を更新 */
		vltmax = VLT_MTRSTOP;										/* 今回の最大電圧を最小値にリフレッシュ */
	}
	/* cless_undervlt_short(tmp_vltnow); */
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
UI08 get_dtc_vpn(void){
    return dtc_vpn;														/* vMAIN_Getvltにも戻り値として返す */
}
/**************************************************************************
<Description>	:12bitPN間電圧を取得する
<Author/Date>	:Takida 2024/10/22
<Update>	:
**************************************************************************/
UI16 get_dtc_vpn12b(void){	/* @318swROD[24/11/14] */
    return dtc_vpn12b;
}
/**************************************************************************
<Title>			:_ADCONV0 		;vMAIN_Inttc1pros
<Function>		:Ａ／Ｄ変換処理（電圧読込用）
<Description>	:Ａ／Ｄ入力ポート[ANI0]をスキャン開始。
				:<Caution>	 	;電圧読み込み処理ﾙｰﾁﾝ”vMAIN_Getvlt()”の46ﾏｼﾝｻｸﾙ以上前に置くこと！！
<Input>			:
<Return>		:
<Author>		:-Outi	作成
<Date>			:2002-0128
<Update>		:Ver REF02A-0.00 2002-01-28
				:Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				:Ver REF045-3.00 2004-07-13 -Outi TOLS置換bug修正
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF090-0.00 2008-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]
				:Ver REF1010-0.01 2010-10-22 -Mura AD値取得処理整備 @008MwDOD[10/10/22]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void vMAIN_Adconv0(void) {									/*A/D変換処理 */
	/*注意！！ ﾁｬﾈﾙの指定はAD変換停止中に行うこと！！ */

}															/* */
UI08 CalcNewVolt(UI08 new_vlt){
	UI08 hi_limit;
	static unsigned8	ucMAIN_vltpast;				/*3.3ms前の電圧(AIN0A/D変換値) */
	/*unMAIN_bosen = ucMAIN_vltnow * (UI16)276;						ﾃﾞﾊﾞｯｶﾞ用ﾒﾓﾘに格納//変換結果⇒電圧値変換(<ex>100⇒27600≡276.00[V]) */
	ucMAIN_vltpast = ucMAIN_vltnow;							/*前回の電圧ﾃﾞｰﾀをramに格納 */
	if(ucMAIN_vltpast < D_VLOWLMT){
		ucMAIN_vltpast = new_vlt;								/* 初期化 */
	}
	/*上下限丸め処理>>>開始<<<                              // */
	if(new_vlt <= (ucMAIN_vltpast - 2)){					/*)ucbuf≦前回の電圧－２ なら */ /*＃ 午後 09:11 REF052-0.00 */
		new_vlt = ucMAIN_vltpast - 2;						/*))ucbuf = 前回の電圧 - 2(5.52v) */
	}else if(new_vlt < ucMAIN_vltpast){												/*)前回の電圧+1すれば＝ucbuf なら */	/* 午後 12:00 REF045-4.00 */
		new_vlt = ucMAIN_vltpast - 1;						/*))ucbuf = 前回の電圧 - １(2.76v) */
	}													/*) */	/* 午後 12:00 REF045-4.00 */
//	}else if(ad_vlt > ucMAIN_vltpast){						/*今回の電圧 > 前回の電圧 なら */
	else if(new_vlt >= (ucMAIN_vltpast + 2)){					/*)ucbuf≧前回の電圧＋２ なら */ /*＃ 午後 09:11 REF052-0.00 */
		new_vlt = ucMAIN_vltpast + 2;						/*))ucbuf = 前回の電圧 + 2(5.52v) */
	}else if(new_vlt > ucMAIN_vltpast){												/*)前回の電圧-1すれば＝ucbuf なら */	/* 午前 11:59 REF045-4.00 */
		new_vlt = ucMAIN_vltpast + 1;						/*))ucbuf = 前回の電圧 + １(2.76v) */
	}													/*) */	/* 午前 11:59 REF045-4.00 */
	else{													/*今回の電圧 = 前回の電圧 なら */	/* 午前 11:58 REF045-4.00 */
		new_vlt = ucMAIN_vltpast;								/*今回の電圧履歴⇒ucbuf格納 */
	}														/*) */	/* 午前 11:58 REF045-4.00 */
/*上下限丸め処理>>>終了<<<                                  // */
														/* */
/*上下限ﾘﾐｯﾀｰ処理>>>開始<<<    海外向けには電圧要注意       // */
	if((D_VHILMT2 > D_VHILMT) && (ucTYPE_global == 1)){
		hi_limit = D_VHILMT2;
	} else {
		hi_limit = D_VHILMT;
	}
	if(new_vlt > hi_limit){
		new_vlt = hi_limit;							/*             {(562*0.99+4.02*1.01)/(4.02*1.01)} */
	}else if( new_vlt < D_VLOWLMT){							/*下限ﾘﾐｯﾀｰ 69.49[v]={(23+0.5)/255}*{5*1.05}* */	/* 午前 11:57 REF045-4.00 */
		new_vlt = D_VLOWLMT;							/*             {(562*1.01+4.02*0.99)/(4.02*0.99)} */
	}													/* */	/* 午前 11:57 REF045-4.00 */
/*上下限ﾘﾐｯﾀｰ処理>>>終了<<<                                    */
	return new_vlt;/* */

}
#endif
/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void set_dtc_enspkpdc(UI16 s){
    dtc_enspkpdc = s;													/* dtc_enspkpdcへのデータセット用アクセス関数 */
}

void ReinitDma(void){
	uint32_t tmp_ccr;

	hdma_adc1.Instance->CCR &= ~DMA_CCR_EN; /*DMAチャネルを一度停止*/

	hdma_adc1.Instance->CPAR = (__IO uint32_t)&(hadc1.Instance->DR); /*ペリフェラルの転送元アドレス 転送元（Peripheral Address）をADCデータレジスタに設定 */  /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	hdma_adc1.Instance->CMAR = (__IO uint32_t)ad_data_arr;			  /*メモリの転送先アドレス 転送先（Memory Address）をad_data_arrに設定*/

    /*ad_data_arr = PWM_Handle_M1.DmaBuffCCR;*/ /* DMA転送先アドレスを更新 */

	hdma_adc1.Instance->CNDTR = ADC_SERIES_NUM;		  /* 転送回数  1サイクルの転送数を設定（配列要素数分）*/
	tmp_ccr = hdma_adc1.Instance->CCR;
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_MEM2MEM_Msk) /*| DMA_CCR_MEM2MEM_Msk*/); 					/* 14: メモリ間転送なし */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_PL_Msk)      | (DMA_CCR_PL_1 | DMA_CCR_PL_0)); 				/* 13:12 優先順位 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_MSIZE_Msk)   | (/*DMA_CCR_MSIZE_1 |*/ DMA_CCR_MSIZE_0)); 	/* 11:10  メモリのデータ転送サイズ　16bit:01 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_PSIZE_Msk)   | (/*DMA_CCR_PSIZE_1 |*/ DMA_CCR_PSIZE_0)); 	/* 9: 8  ペリフェラルの データ転送サイズ　16bit:01 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_MINC_Msk)    | DMA_CCR_MINC);								/* 7: メモリの転送先アドレスをインクリメントする */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_PINC_Msk)    /*| DMA_CCR_PINC*/);							/* 6 : ペリフェラルの転送元アドレスをインクリメントしない */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_CIRC_Msk)    | DMA_CCR_CIRC);								/* 5:  サーキュラーモード利用 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_DIR_Msk)     /*| DMA_CCR_DIR*/); 							/* 4: データ転送方向 p->m:0 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_TEIE_Msk)    /*| DMA_CCR_TEIE*/); 							/* 3:  転送エラー割り込み有効化 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_HTIE_Msk)    /*| DMA_CCR_HTIE*/); 							/* 2:1/2転送完了割り込み有効化 */
	tmp_ccr = ((tmp_ccr & ~DMA_CCR_TCIE_Msk)	| DMA_CCR_TCIE); 								/* 1:転送完了割り込み有効化 */
	hdma_adc1.Instance->CCR = tmp_ccr; /* 0反映 */

	hdma_adc1.Instance->CCR |= DMA_CCR_EN; /* DMA再開 */
}

/* シャント電流検出オフセット処理 */
/* 0電流電圧=offset の取得は、モータ停止中は繰り返し実施 */
/* 取得したAD値を所定回数積算し、取得回数で除算することで平均を算出し、offsetとする */

#define NUM_CNT_CUR_OFS 	(250)	/* ｵﾌｾｯﾄ算出用に取得するﾃﾞｰﾀ数（この個数のデータを平均する */				/* @304swDOD[24/09/02] */
#define AD_CENTER       	(2048)	/* 2の12乗÷２、12bitのADCのため0～4095のセンター値 */						/* @304swDOD[24/09/02] */
#define AD_CUR_SHIFT    	(4)		/* 戻り値のﾋﾞｯﾄ数に合わせるためのﾋﾞｯﾄｼﾌﾄ数、12bitから8bitに変換するので4 */	/* @304swDOD[24/09/02] */
#define CURRENT_LPF_SHIFT	(6)																					/* @304swDOD[24/09/02] */
#define OUTPUT_CHANGE_TIME_3R3MS 		(int)(2 * 60 * 300)														/* @304swDOD[24/09/02] */

enum eModeCurrentOffset {					/* @304swDOD[24/09/02] */
    kCurOfsMdInit = 0,		/* 0:初期化中 */
    kCurOfsMdSum,			/* 1:取得値積算中 */
    kCurOfsMdCalc,			/* 2:積算完了＆平均算出中 */
    kCurOfsMdReset,			/* 3:ﾘｾｯﾄ中(積算値＆積算ｶｳﾝﾄをﾘｾｯﾄ) */
    kCurOfsMdEnd			/* 4:ｵﾌｾｯﾄ算出終了状態（本定義不使用。。。） */
};

typedef struct {							/* @304swDOD[24/09/02] */
    UI08 mode;
    UI08 cnt;
    UI32 sum_current0_ad;
    UI32 offset;
    UI32 offset12;
}     TStructCurrentOffset;
TStructCurrentOffset cur_ofs;

/**************************************************************************
<Title>			:StoreCurrentOffset
<Function>		:ゼロ電流時の電圧値(ｵﾌｾｯﾄ)の積算処理。
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* 積算処理 */
void StoreCurrentOffset(SI16 ad_data){			/* @304swDOD[24/09/02] */
    cur_ofs.sum_current0_ad += ad_data;
}

/**************************************************************************
<Title>			:CalcCurrentOffset
<Function>		:ゼロ電流時の電圧値(ｵﾌｾｯﾄ)を計算処理。結果を8bit値で返す。
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
 /* オフセットを計算。戻り値は8bit。戻り値とは別に12bitのオフセットも保存 */
UI08 CalcCurrentOffset(void){					/* @304swDOD[24/09/02] */
    if(cur_ofs.sum_current0_ad == 0){
        return 1;
    }
    cur_ofs.offset12 = (cur_ofs.sum_current0_ad / NUM_CNT_CUR_OFS);
    return (cur_ofs.offset12 >> AD_CUR_SHIFT);
}
/**************************************************************************
<Title>			:ResetCurrentOffset
<Function>		:ゼロ電流時の電圧値(ｵﾌｾｯﾄ)を計算の繰返し用の初期化処理。
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* 繰り返し実施用に積算とカウンタをクリア */
void ResetCurrentOffset(void){					/* @304swDOD[24/09/02] */
    cur_ofs.sum_current0_ad = 0;
    cur_ofs.cnt = 0;
}

/**************************************************************************
<Title>			:InitCurrentOffset
<Function>		:ゼロ電流時の電圧値(ｵﾌｾｯﾄ)を計算のための初期化処理。ｵﾌｾｯﾄ値も初期化する。
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* 1回だけ実施。オフセットも含めた初期化 */
void InitCurrentOffset(void){					/* @304swDOD[24/09/02] */
    ResetCurrentOffset();
    cur_ofs.offset = 0;
    cur_ofs.offset12 = 0;
}

/**************************************************************************
<Title>			:StateMachineCurrentOffset
<Function>		:ゼロ電流時の電圧値(ｵﾌｾｯﾄ)を計算するために必要な状態情報を返す
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* 0電流計算用処理の状態遷移 */
UI08 StateMachineCurrentOffset(UI08 mode){		/* @304swDOD[24/09/02] */
    switch(mode){
        case kCurOfsMdInit:
            mode = kCurOfsMdSum;
            break;
        case kCurOfsMdSum:
            if(cur_ofs.cnt >= NUM_CNT_CUR_OFS){
                mode = kCurOfsMdCalc;
            } else {
                mode = kCurOfsMdSum;
            }
            break;
        case kCurOfsMdCalc:
                mode = kCurOfsMdReset;
            break;
        case kCurOfsMdReset:
            mode = kCurOfsMdSum;
            break;
        default:
            mode = kCurOfsMdInit;
            break;
    }
    return (UI08)mode;
}
/**************************************************************************
<Title>			:CurrentOffset
<Function>		:ゼロ電流時の電圧値(ｵﾌｾｯﾄ)を計算して計算結果を返す
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* 0電流の電圧値を計算する */
UI08 CurrentOffset(void){						/* @304swDOD[24/09/02] */

    switch(cur_ofs.mode){
        case kCurOfsMdInit:
            InitCurrentOffset();
            break;
        case kCurOfsMdSum:
            StoreCurrentOffset((SI16)ad_data_arr[ADSTNUM_SHUNT]);
            cur_ofs.cnt++;
            break;
        case kCurOfsMdCalc:
            cur_ofs.offset = CalcCurrentOffset();
            break;
        case kCurOfsMdReset:
            ResetCurrentOffset();
            break;
        default:
            break;
    }

    cur_ofs.mode = StateMachineCurrentOffset(cur_ofs.mode);	/* 計算処理後に状態遷移させる */

    return cur_ofs.offset;
}

/**************************************************************************
<Title>			:IsFinishCurrentOffset
<Function>		:オフセットが取得できているかどうかのﾌﾗｸﾞを返す
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* オフセットが０かどうかで取得できているかを判定。そのため、オフセットは起動時に0にする必要あり */
UI08 IsFinishCurrentOffset(void){				/* @304swDOD[24/09/02] */
    return (cur_ofs.offset > 0);		/* kCurOfsMdEndを利用してで判定してはどうか??by瀧田 */
}
/**************************************************************************
<Title>			:GetCurOffset12
<Function>		:12bitのオフセット値を返す
<Author>		:Takeoka
<Date>			:2024.04.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
SI16 GetCurOffset12(void){						/* @304swDOD[24/09/02] */
    return (SI16)(cur_ofs.offset12);
}

/**************************************************************************
<Title>			:StoreCurrentBuf
<Function>		:電流検出結果を機械角に対応した場所に格納する。
				:将来的に負荷変動やトルク変動の検出に利用できる見込みで
				:本処理により機械角に対応した電流値を保持できるようにしている。
<Author>		:Takeoka,Takida
<Date>			:2024.07.24 (機種実装2024.09.02) @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
/* 電流検出結果を機械角に対応した場所に格納する。利用時にはシフト必要
 *   e.g. buf[0] = 512, hold_cur = 2008, offset = 2000
 *        利用したいのは「 hold_cur - offset」(=8)の平均
 *        バッファには 8<<CURRENT_LPF_SHIFT が平均として格納される
 *        buf[0] = 512 + (2008 - 2000) - (512 >> CURRENT_LPF_SHIFT)
 *               = 512 + 8 - 8
 *               = 512 (= (2008-2000)<<CURRENT_LPF_SHIFT) */
void StoreCurrentBuf(SI16 hold_cur){											/* @304swDOD[24/09/02] */
    current_buf[ucPMDi_mtbufpt] +=
    		( ((SI32)hold_cur - cur_ofs.offset12) - (current_buf[ucPMDi_mtbufpt] >> CURRENT_LPF_SHIFT) );
	current_use[ucPMDi_mtbufpt] = current_buf[ucPMDi_mtbufpt] >> CURRENT_LPF_SHIFT;	/* AD入力bit数と同じレンジ(12bit)に変換 */
}
/**************************************************************************
<Title>			:CleareCurrentBuf
<Function>		:電流検出結果をｸﾘｱする。
<Author>		:Takida
<Date>			:2024.09.02 @304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
void ClearCurrentBuf(void){														/* @304swDOD[24/09/02] */
    UI08 i;
	for(i = 0; i < D_MTBUFSZ; i++){										/* 1回転の転流数でループ */
		current_buf[i] = 0;											/* 停止後は電流流れていないのでクリア */
		current_use[i] = 0;											/* 停止後は電流流れていないのでクリア */
	}
}
/**************************************************************************
<Title>			:Calc_current_ave
<Function>		:コンプレッサ巻線電流平均値計算
<Description>	:機械周期毎の電流ピーク値の平均値を算出する
				:ｺﾝﾌﾟﾚｯｻ巻線の温度は電流ピーク値の平均値の2乗と相関があるため
				:本処理にて計算した値を保護判定に用いる。
<Author>		:Takida
<Date>			:2024.09.02		@304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
UI16 Calc_current_ave(void){							/* 電流平均値計算 */
    UI08 i;												/* 機械周期毎の電流ピーク値の平均値計算 用自動変数 */
	UI16 current_sum = 0;									/* 電流平均値算出過程での合計値(18個の合計) */
	UI16 current_ave = 0;									/* 電流平均値(18個の平均) */
	
	for(i = 0; i < D_MTBUFSZ; i++){
		current_sum = current_sum + current_use[i];		/* 合計計算 */
	}
	current_ave = current_sum / D_MTBUFSZ;				/* 平均計算 */

	return current_ave;									/* 平均値を返す */
}

/*PositionDetect.c*/

/**************************************************************************
<Title>			:vPMD_PdcPtnSet 		;INTTMR
<Function>		:信号検期待値設定
<Description>	:信号検出取込port信号レベルを３回連続chk⇒期待値と合致⇒信号確定処理(INTPDC)呼出
<Input>			:ucPMDi_MtrRckChck,ucPMD_NowMDOUTpd,ucPMD_NowMDOUTpd
<Return>		:ucPMD_PDptnU,ucPMD_PDptnV,ucPMD_PDptnW,ucPMD_PDpotMask,ucPMD_PDintAdPt,ucPMD_PDjudge,ucPMD_NowMDOUTpd
<Author>		:-Outi	作成
<Date>			:2008-0411
<Update>		:Ver REF081-0.00 2008-04-11
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vPMD_PdcPtnSet(void){												/*信号検期待値設定*/
 	UI08 ucADDTCmdidx;                  								/*D_runningとD_vector区別用mdidx*/			/* @008MzDOD[16/12/21] */
/*↓---------------------矩形波VCT対応----------------------------------------*/	/* @008MzDOD[16/12/21] */
//	if(ePMD_drvmode != D_vector){														/*センサレス中なら*/
 	if(ucPMD_VctrMode != kDriveVector){
 		ucADDTCmdidx = ucPMD_mdidx;												/*標準mdidx採用(既存同様)*/
	}else if((2 <= ucPMD_mdidx)&&(ucPMD_mdidx <= 4)){           					/*矩vct中でmdidx＜3の時(U相↑検出～U+相ON中)*/
        ucADDTCmdidx = 2;														/*矩vct中mdidx=0固定*/
	}else{																		/*矩vct中でmdidx≧3の時(U相↓検出～U-相ON中)*/
		ucADDTCmdidx = 5;														/*矩vct中mdidx=3固定*/
	}									 										/**/
/*↑---------------------矩形波VCT対応----------------------------------------*/	/* @008MzDOD[16/12/21] */

	/*ucPMD_PDjudge = judge_tbl[ucADDTCmdidx];*/		//a 位置検出の向きを決定			/* D_runningとD_vectorで配列先を変更*/			/* @008MzDOD[16/12/21] */

	UpdateAdcSequence(ucADDTCmdidx); /* AD変換するポートを選択 */
	ResetDTC2(ucPDC_portsel, ucPMD_PDjudge);
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
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline  UI16 TimeLimitSpikeEndSensorless(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    if( renzokuhassei > 0 )
      return unPMDi_MCAPdumy - (unPMDi_MCAPdumy >> 3);

    if( IsAngleLap() )
      return unPMDi_MCAPdumy;
    return unPMDi_MCAPdumy + (unPMDi_MCAPdumy >> 2);
}


/**************************************************************************
<Title>			:vPMDi_Setcomtptn 		;INTPDC,INTTMR3
<Function>		:120ﾟ転流処理
<Description>	:	[120ﾟ]転流ﾊﾟﾀﾝ＆転流ﾀｲﾐﾝｸﾞ･ｻﾝﾌﾟﾘﾝｸﾞ開始ﾀｲﾐﾝｸﾞ･TimeOut時間のｾｯﾄ
				:		Local 関数  転流用ベースパタン・データ＆転流タイミングのセット.    00/12/14
<Input>			:ucPMD_boostmd,ucPMD_ldang,unSPD_rpmcrnt,ePMD_drvmode
<Return>		:unPMD_MDOUT, unPMD_CMP1, unPMD_CMP2, unPMD_CMP3
<Author>		:-Outi	作成
<Date>			:2002-0216
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver REF036-0.07 2002-11-11 -hide unPMDi_MCAPdumyの格納(ﾐﾗｰRAM新設)
				:Ver REF036-0.10 2003-01-07 -hide 新矩形波駆動用運転ﾓｰﾄﾞD_vector,unr_VctrMode新設
				:Ver REF04C-0.00 2004-09-07 -Outi ﾗｯﾌﾟ転流仕様見直し
				:Ver REF052-0.00 2005-05-19 -Outi DR指摘対応
				:Ver REF052-1.00 2005-06-13 -Outi ﾊﾞｸﾞ修正
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF090-0.04 2009-06-11 -Outi 起動時回路異常検出削除 @016axOTM[09/06/11]
				:17/12/15 オーバーフロー扱い処理にならないよう修正 吉田
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vPMDi_Setcomtptn(void) {						/*転流ﾊﾟﾀﾝ(通常)ｾｯﾄ処理 */
    UI16 commutate_period;

    /* 次のﾓｰﾄﾞのMDOUTパタンを取得 */
    PresetNormalMdout(ucPMD_mdidx);
    UpdateMdout();
    PWM_wait();                                         /* PWMタイマ設定 */
    vPMD_PdcPtnSet();                                   /* 位置検出処理設定 */

    /* 速度更新 */
    commutate_period = unPMD_MCAP;						/* ﾃﾝﾎﾟﾗﾘ(16bit)←60ﾟe(MCAP) */
    if( IsFailPeriodTooShort(commutate_period) ){		/* 転流間隔が早すぎてロック保護以下 */
    	ErrMotorLock();									/* ロック保護フラグセット ＆ 停止処理 */
    	return;
    }
    CommutatePeriodLowPassFilter(commutate_period);
    UpdateCommutatePeriodRotation(commutate_period);

    /* 各タイマの設定 */
    if( ucPMD_pdct > 0 ){
        unPMD_CMP1 = TimeNonlapStartSensorless();
        unPMD_CMP3 = 0x7fff;                      			/* TRCGRA=0xffffでは、GRA一致でTRCリセットしてもオーバーフロー扱いとなるため、TRCGRA=0xfffeとなるようCMP3を0xffff⇒0x7fffに変更 CMP3(INTTMR3割込周期)←３回転時60ﾟ期間 */	/* @013ywOOM[17/12/15] */
        cspk_CMP2_2nd = TimeLimitSpikeEndSensorless();
        set_dtc_enspkpdc(mMDTM2TRC(unPMDi_MCAPdumy - (unPMDi_MCAPdumy >> 2)));
    } else {
        unPMD_CMP1 = 100;								    /* 2012-05-28 竹岡 */
        unPMD_CMP3 = D_FCDPRD;
        cspk_CMP2_2nd = 0xffff;
        set_dtc_enspkpdc(0xffff);
    }

	htim3.Instance->ARR = mMDTM2TRC(unPMD_CMP3);                    /* TRC ｾﾞﾈﾗﾙﾚｼﾞｽﾀA */
	htim3.Instance->CCR1 = 0xffff;									/* TRC ｾﾞﾈﾗﾙﾚｼﾞｽﾀB */
    mDisableLapTimerInt();
    UpdateTrcgrcAndPwmOutIfNeeded(unPMD_CMP1);

    /* ad変換処理の回数リセット */
    chk_spike = kWaitCommutaion;
}

/**************************************************************************/
inline  UI16 TimeNonlapStartSensorless(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (Deg60RatioAngle2ModeTime(angle.nonlap_start, unPMDi_MCAPdumy) >> 1); /* nonlap_startが2倍なので、計算結果を1/2 */
}
/**************************************************************************
<Title>			:vPMDi_Setcomtlap 		;INTPDC
<Function>		:広角転流処理
<Description>	:	[ﾗｯﾌﾟ]転流ﾊﾟﾀﾝ＆転流ﾀｲﾐﾝｸﾞ･ｻﾝﾌﾟﾘﾝｸﾞ開始ﾀｲﾐﾝｸﾞ･TimeOut時間のｾｯﾄ
				:		Local 関数  転流用ﾍﾞｰｽﾊﾟﾀﾝ･ﾃﾞｰﾀ＆転流ﾀｲﾐﾝｸﾞのｾｯﾄ.    02/03/02
<Input>			:ucOVLP_ovlapang,ucPMD_ldang,hCurMecRps
<Return>		:unPMD_MDOUT,unPMD_CMP1,unPMD_CMP2,unPMD_CMP3,unPMDi_MCAPdmybuf[],unPMDi_MCAPdumy,unPMDi_MCAPDmyRead,unPMDi_mtbuf[]
<Author>		:-Outi	作成
<Date>			:2002-0302
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver Pre036-0.00 2002-06-01	-Outi 新矩形波駆動０次試作ｺｰﾃﾞｨﾝｸﾞ
				:Ver REF036-0.00 2002-06-27 -hide 新矩形波駆動(VCT)移行回転数取得仕様(SSL⇒VCT移行仕様)
				:Ver REF036-0.02 2002-08-23 -hide ﾗｯﾌﾟ通電時進角最適化設計
				:Ver REF036-0.04 2002-11-05 -hide 新矩形波仮想位置検知用ﾀｲﾏ変更TMR3←TMR2
				:Ver REF036-0.05 2002-11-06 -hide 仮想位置検知初期設定
				:Ver REF036-0.07 2002-11-11 -hide unPMDi_MCAPdumyの格納(ﾐﾗｰRAM新設)
				:Ver REF036-0.08 2002-11-14 -hide 処理時間短縮のためLap8条件追加
				:Ver REF036-0.09 2002-11-20 -hide ロック検知追加
				:Ver REF036-0.10 2003-01-07 -hide 新矩形波駆動用運転ﾓｰﾄﾞD_vector,unr_VctrMode新設
				:Ver REF036-0.11 2003-01-10 -hide 不等号ミス修正
				:Ver REF036-0.13 2003-01-20 -hide ﾛｯｸ検知ﾚﾍﾞﾙ変更
				:Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				:Ver REF045-2.02 2004-06-12 -Outi ovlapang"9"参照不具合修正
				:Ver REF04C-0.00 2004-09-07 -Outi ﾗｯﾌﾟ転流仕様見直し
				:Ver REF04C-0.01 2005-01-23 -Outi VCT駆動中の通電角調整仕様の変更
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF090-0.03 2009-05-17 -Outi 回路異常検出仕様設計内部dr結果反映(BUG) @011ahRRD[09/05/17]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
static inline  void vPMDi_Setcomtlap(void) {							/*転流ﾊﾟﾀﾝ(ﾗｯﾌﾟ)ｾｯﾄ処理 */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    UI16 commutate_period;

    UpdateMdout();
    PwmWaitLap();
    /* ITTMR1(ラップ用)のタイミング設定 */
    unPMD_CMP1_lap = TimeOverlapStartSensorless();
    mEnableLapTimerInt();
    UpdateTrcgrbAndPwmOutIfNeeded(unPMD_CMP1_lap);

    /* 速度の更新 */
    commutate_period = unPMD_MCAP;							/* ﾃﾝﾎﾟﾗﾘ(16bit)←ModeTime(60ﾟe) */
    if( IsFailPeriodTooShort(commutate_period) ){		/* 転流間隔が早すぎてロック保護以下 */
    	ErrMotorLock();									/* ロック保護フラグセット ＆ 停止処理 */
    	return;
    }
    CommutatePeriodLowPassFilter(commutate_period);  /* 転流用60degの計算 */
    UpdateCommutatePeriodRotation(commutate_period);        /* 1回転の平均速度計算用 */

    /* 各タイマの設定 */
    unPMD_CMP1 = TimeNonlapStartSensorless();               /* INTTMR1のタイミング設定 */
    unPMD_CMP3 = 0xffff;									/* CMP3のリフレッシュ処理 */
    cspk_CMP2_2nd = TimeLimitSpikeEndSensorless();
    set_dtc_enspkpdc(mMDTM2TRC(unPMDi_MCAPdumy - (unPMDi_MCAPdumy >> 2)));
    ClearTrcgra();

    /* ad変換処理の回数リセット */
    chk_spike = kWaitCommutaion;											/* @008MzDOD[16/12/21] */
//    adconv_cnt = 0;											/* @008MzDOD[16/12/21] */

}

/**************************************************************************
<Title>			:INTPDC 		;ﾍﾞｸﾀ割込呼出
<Function>		:位置検出処理
<Description>	:	位置検出中→転流&ﾛｯｸCHK判定処理ﾓｼﾞｭｰﾙCALL／強制転流(Forced)中→位置検ﾓｰﾄﾞ(Running)移行／else→No_Operation
				:		備考     : 	ﾚｼﾞｽﾀﾊﾞﾝｸ１ 多重割込禁止
				:					 							  _______
				:		_INT0 ≫ _INT1 ≫ _INTEMG  ≫ _INTTMR3≫ |_INTPDC｜ ≫ _INTTMR1  ≫ _INTTMR2 ≫ _INTTC1
				:					 							  ￣￣￣￣
<Input>			:ucOVLP_ovlapang,ucPMD_boostmd,ePMD_drvmode,ucPMDi_MtrRckChck.4
<Return>		:unPMD_MDOUT,unPMD_CMP1,unPMDi_MCAPdmybuf[],unPMDi_mtbuf[]
<Author>		:-Outi	作成
<Date>			:2002-0216
<Update>		:Ver REF02A-0.00 2002-06-03
				:Ver REF036-0.01 2002-07-02 -hide 新矩形波駆動用ｶｳﾝﾀucr_vectorct機能増設(VCT⇒SSL移行)
				:Ver REF036-0.09 2002-11-20 -hide ロック検知追加
				:Ver REF036-0.10 2003-01-07 -hide 新矩形波駆動用運転ﾓｰﾄﾞD_vector,unr_VctrMode新設
				:Ver REF036-0.13 2003-01-20 -hide ﾛｯｸ検知ﾚﾍﾞﾙ変更
				:Ver REF045-1.00 2004-06-02 -Outi SPK検知仕様の追加
				:Ver REF045-1.01 2004-06-03 -Outi ﾛｯｸ検知仕様の変更とｳﾈﾘ検知仕様の追加
				:Ver REF045-1.05 2004-06-06 -Outi SPK偏差＞7.5ﾟ仕様に変更
				:Ver REF045-1.11 2004-06-09 -Outi SPK検知関連ﾒﾓﾘ極力INTPDCﾉﾐで処理
				:Ver REF045-1.12 2004-06-09 -Outi SPKrfrshctのﾘﾌﾚｯｼｭ時は"0ｸﾘｱ"でなく"D_SPKRFRSHTM代入"仕様に変更
				:Ver REF045-2.00 2004-06-12 -Outi SPK≧4*SPKminなら補正判定する仕様を追加
				:Ver REF045-2.01 2004-06-12 -Outi 回転数ﾀﾞｳﾝ仕様vSPD_Setrpmﾀﾞﾐｰ追加に伴う変更
				:Ver REF045-2.03 2004-06-17 -Outi SPK幅偏差によるｳﾈﾘ検知を加速中にも許可する仕様を追加
				:Ver REF045-2.04 2004-06-19 -Outi 変数"spdtrgt"(vSPD_Setrpm)をT_TrgtSpdTbl[ucSPD_rpmh]参照ﾒﾓﾘとして使用
				:Ver REF04C-0.00 2004-09-07 -Outi ﾗｯﾌﾟ転流仕様見直し
				:Ver REF052-0.00 2005-05-19 -Outi DR指摘対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.04 2008-06-05 -Outi 信号検出処理修正	@000ahOTM[08/06/05](発覚08/05/21)
				:Ver REF090-0.01 2009-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]
				:Ver REF090-0.04 2009-06-11 -Outi 起動時回路異常検出削除 @016axOTM[09/06/11]
				:Ver REF1000-0.00 2010-05-25 -Mura 停止時振動対策インプリ  @006MwHOM[10/05/25]
				:Ver REF1000-0.01 2010-06-11 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
				:Ver REF1000-0.02 2010-06-22 -Mura 停止オフセット受取りタイミング修正  @012MzORT[10/06/22]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void INTPDC(void) {														/*位置検出割込処理 */
	UI16 tmp_trque_duty;
#if 0
	HAL_GPIO_TogglePin(debug_SPIN1_GPIO_Port, debug_SPIN1_Pin);			/* debug output SPIN1 => motor position detection flag */
#endif

    if( angle.lap_start == 0 ){                                         /* 即転流条件 */
//        if( IsAngleGap() ){
//            unPMD_MDOUT = T_MDOUTgap[ucPMD_mdidx];					/*次のﾓｰﾄﾞのMDOUTパタンを取得 */
//        }
//        else{
//            unPMD_MDOUT = T_MDOUTlap[ucPMD_mdidx];					/*次のﾓｰﾄﾞのMDOUTパタンを取得 */
//        }
        UpdateMdout();
    	PwmWaitLap();
        PWM_out();													/* PWM出力処理 */
    }/*＃ 午後 04:17 REF04C-0.00 */
    if( IsAngleLapOrGap() ){
        vPMDi_Setcomtlap();										/*[ﾗｯﾌﾟﾊﾟﾀﾝ転流設定] */
#if TRQ_CTL == ACTIVE
        vPMDc_UpdateDuty(unPMDc_dutyn);							/* トルク制御DUTY更新 */	/* @012MzDOD[17/02/13] */
#endif /* TRQ_CTL */
        return;													/*INTPDCを抜ける */
    }
    switch(ePMD_drvmode){											/*運転状態witch */
    case D_stop:													/*<Stop>    なら */
    case D_standby:													/*<Standby> なら */
    case D_position:												/*<Position>なら */
        unPMD_MCAP = 0;												/* Dummy read ⇒MCAPを空にするお呪い */
        break;														/*  */
    case D_forced:													/*<Forced>  なら  01/01/14 */
//    	if(ucPMD_pdct >= D_FCDPDCT + 10){ //処理簡略化のため ucPMD_pdct の閾値D_FCDPDCTが255に近い値では速度と極数によって正常動作でここの条件成立の可能性あり 245以下に抑えれば理論上6極80r/sでも問題なし
//    		vPMD_Brake();
//    	}
    	//割込み時間短縮のため ここでは保護をかけない 移行しなかった場合 3.3ms側で保護を掛ける
    	if(ucPMD_pdct < D_FCDPDCT){
    		ucPMD_pdct ++;							/*down count -> up count 位置検出回数ｶｳﾝﾀ←ﾃﾞｸﾘﾒﾝﾄ */
    	}
        //no break
    case D_running:													/*<Running>なら  01/01/14 */
    case D_vector:													/* Vector -> Running 移行前に入る可能性あるが、基本的にLAPを残して遷移するため入らない */
    	vPMDi_Setcomtptn();									/*転流ﾊﾟﾀﾝｾｯﾄ処理 */
    	if(uc100MS_pumpct < PUMPCT_END ){					//給油改善前まで実施
    		SelectStopPosition();
    	}
#if TRQ_CTL == ACTIVE
    	tmp_trque_duty = TorqueControlCommutate(unPMD_MCAP, unPMDc_dutyn);
		vPMDc_UpdateDuty(tmp_trque_duty);											/* PWM関連レジスタ設定処理 */
#endif /* TRQ_CTL */
        break;														/* */
    }
}
/*PositionDetect.c*/

/*from adc.c start*/
/**************************************************************************
<Function>		:
<Author>		:
<Date>			:
<Update>		:
 **************************************************************************/
void UpdateAdcSequence(uint32_t index){
//	while(hadc1.Instance->CR & ADC_CR_ADSTART){
//		hadc1.Instance->CR |= ADC_CR_ADSTP;  //ccmr chselr など設定変更のため、AD変換を一旦ストップ
//	}
    /*
	hadc1.Instance->ISR = ADC_ISR_CCRDY; //flag クリア
	hadc1.Instance->CHSELR = ad_seq_table[index];   // AD変換テーブル切り替え用
    */
}
/*from adc.c end*/
