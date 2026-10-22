#ifndef INV_COMMON_FUNC_H
#define INV_COMMON_FUNC_H

/**************************************************************************
<FileName>		:InvCommonFunc.h
<Function>		:�ʎY�\�t�g���ʊ֐�
<Program>		:
<Author>		:
<Date>			:
<Update>		:2002-0128
<Copyright>		:Panasonic Co. , Ltd.
**************************************************************************/


/****************************************************************************/
/*                                                                          */
/*      macro definition                                                    */
/*                                                                          */
/****************************************************************************/

#define    mDisableInterrupt()   __disable_irq()
#define    mEnableInterrupt()  	 __enable_irq()

#define ADSTNUM_SHUNT (0)                                               /* DTC�ł�PN�ԓd��A/D�ϊ����ʂ̊i�[�z��ԍ��̐擪 *//* @267swRRD[23/09/22] */
#define ADC_SERIES_NUM (8)



/****************************************************************************/
/*                                                                          */
/*      Type definition                                                    */
/*                                                                          */
/****************************************************************************/
/*v2LV_Ctrl2lv() */
#ifdef ONECHIP		/*ﾒｲﾝ-INV 1ﾏｲｺﾝ*/											/* @012MzDOD[17/02/19] */
#define  D_MINHZRYON  10					/* ﾘﾚｰON中の最小回転数  10rps */
#elif !defined( ACTIVATE_PAM )
#define  D_MINHZRYON  10					/* ﾘﾚｰON中の最小回転数  30rps */
#else	//(ndef ONECHIP) && (ndef DETECT_PWM_OFF)			/*INV専用ﾏｲｺﾝ */
#define  D_MINHZRYON  33					/* ﾘﾚｰON中の最小回転数  33rps */ /*HTA日本向けで、閾値を30rps→33rpsに変更*/
#define	 D_MINHZRYONBUFFER 2				/* リレーONは、リレーオンする回転数と2rpsのbufferを考慮 */
#endif  //ONECHIP

											/*  仕様上は”93秒±5%”と記されているが、実際の値は16(×6=96秒)としている。  */
											/*この誤差は６秒ｶｳﾝﾀを使用したために発生したもので、ｼｽﾃﾑ担当者にも了解済み。  */
#define  D_RSNNRPM	0						/*~03-0605 -Outi 気柱共鳴回避ﾀﾞｳﾝ回転数(0～10rps) */ /* 午後 07:16 REF052-1.** */
#define  D_RSNMINRPM  40					/*~03-0605 -Outi 気柱共鳴回避許可回転数(最小) */
#define  D_RSNMAXRPM  47					/*~03-0605 -Outi 気柱共鳴回避許可回転数(最小) */
#define  D_VLTLMT0	80						/* 最大回転数ﾘﾐｯﾀ動作ﾚﾍﾞﾙ　220Vdc */

/*５強制θ＝150転流制御(D_vector) */
#define  D_ChngFrcdrpm 60					/* ~03-0419 -Outi Forced駆動の最高回転数判定基準回転数(60rps) */
#define  D_CHNGFRCD	(1000000/D_ChngFrcdrpm/3/ D_POLE)	/*Forced駆動の最高回転数判定基準周期 */
#ifdef CHANGE1HZ
#define  D_HILOADMAXSPD 99					/* 高負荷時の最高回転数 */	/* 午後 10時28分 */
#else
#define  D_HILOADMAXSPD 80					/* 高負荷時の最高回転数 */	/* 午後 10時28分 */
#endif
#define  D_LOLOADMAXSPD 99					/* 低負荷時の最高回転数 */


#define D_TIME_FAIL_SAFE_MONITOR_100MS 10 //倍電圧切り替え後過電流を監視する時間 単位[100ms] e.g. 10 -> 10*100ms = 1s
#define D_TIME_WAIT_ON2OFF_100MS (31)	  /*100msカウンタ用の判定閾値. 3.1sec*/
#define D_TIME_WAIT_STABLE_ON_100MS (10)  /*100msカウンタ用の判定閾値. 1.0sec*/

/*PMD1GPar.hより*/
#define  D_PUMPAMPPRD 3						/*10ms (2002/12/27-hide) */
#define  D_PUMPAMPINC  5					/*Duty:  0.5%　給油改善制御時Dutyｱｯﾌﾟ率 5[‰/10ms] *//* TKD91E20PBF用設定 *//* @144swDOD[21/05/27] */
#define  D_PUMPAMPINC2 5					/*Duty:  0.5%　給油改善制御時Dutyｱｯﾌﾟ率 5[‰/10ms] */ /*~05-0615 -Outi 起動時回転加速度UP変更*/ /* 午前 09:56 REF052-1.xx */ /* @023awOTT[05/06/15] */
#define  D_PUMPRPM  33						/* ~03-0327 -Outi 給油改善制御中回転数  33回転（通常時）REF03Da(2U技術試作から変更) */
#define  D_PUMPMES  27						/* 圧縮機停止時振動抑制制御 停止基準位置検出用起動回転数 */ /* @008MwHOM[10/06/11] */
#define  D_BOOSTSPD	(1000000/D_PUMPRPM/3/ D_POLE)	/*~02-1231 -Outi給油改善制御中回転周期（通常時）  */
#define  D_PUMPINI	27						/* ~03-0630 -Outi 給油改善制御中回転数  27回転（ｲﾆｼｬﾙ時）_setrpm()AtPAM_MODE.sにて使用 */
#define  D_BOOSTINI	(1000000/D_PUMPINI/3/ D_POLE)	/*~02-1231 -Outi 給油改善制御中回転周期（ｲﾆｼｬﾙ時） */
#define  D_RUNPRD  1						/*通常運転中  Dutyﾁｪｯｸ周期  3.3ms ~02-0213 -Outi */
#define  D_ECORPMMAX  25					/*省エネ運転中最大回転数 ~03-0630 -Outi ｲﾆｼｬﾙ起動回転数変更 */
#define PUMPCT_END	(UI08)(150)				/* 給油改善制御時間*/	/* @012MzDOD[17/02/13] */

/* SwitchPamDutyMode */
#define BASE_TIME_MS 2						/* 基本時間単位　2ms */
#define D_2LV_ON_WAIT_TIME	(8/BASE_TIME_MS)	/* 停止時にオンからオフへと切替えるのに待機している時間　310ms */
#define D_2LV_BOOSTED_TIME	(500/BASE_TIME_MS)	/* 2LVブースト状態の時間　500ms */
#define D_2LV_DROPPED_TIME	(500/BASE_TIME_MS)	/* 2LVオフ状態への移行時間　500ms */

enum ePamModeTag {
	kWaitOn2Off, 		//停止時にオンからオフへと切替えるのに待機している状態　モータの回生の影響があるため、停止するまでを待つ時間、オンで待つ状態
	kStopOff, 			//停止中でリレーオフ状態
	kStableOn, 		//圧縮機起動前にリレーをオンして、電圧が安定するのを待っている状態
	kForcedOn, 			//速度によらず、リレーをオンしている状態 フェールセーフや給油改善中
	kNormalRunningOn,	//リレーがオンしている状態 kRunningOffへ遷移OK
	kRunningOff			//低速で運転中でリレーがオフしている状態
};

enum eFailSafeModeTag {
	kUnmonitorFailSafe, //全波整流でフェイルセーフの監視を行っていない
	kMonitorFailSafe,   //フェイルセーフに入る必要があるか監視中
	kOverTimeMonitor,   //フェイルセーフの監視終了
	kInFailSafe         //フェイルセーフ中
};

enum eRelayStateTag{
	kRelayOFF = STATE_OFF,
	kRelayON = STATE_ON
};

enum ePamDutyModeTag {
	kWaitBoost,
	kBoosting,
	kStableBoost,
	kBoosted,
	kDropping,
	kDropped
};

struct sPamControl {
	enum ePamModeTag 	  ctrl_mode;
	enum eFailSafeModeTag fail_safe_mode;
	enum eRelayStateTag   relay_state;
	enum ePamDutyModeTag  duty_mode;
};

extern struct sPamControl pam;

extern unsigned8	ucTYPE_position_duty;		/*位置決め方式及び起動Duty	*/
extern unsigned8	ucTYPE_global;				/*海外機種判別確定	*/
extern UI08 ucTYPE_carrier_freq;	/* @250swDOD[23/05/18] */
extern unsigned8	ucTYPE_undvlt_int;			/*直流電圧低下保護仕様	*/

extern enum ePamModeTag SwitchPamMode(enum ePamModeTag mode);
extern void v2LV_Drvry(UI08 state);

extern unsigned8	ucSPD_sfrqfx;				/*指令Hz(確定値)<dec> */

extern uint16_t u16s_vbus;

extern unsigned8	ucPMDi_mtbufpt;
extern unsigned16	hCurMecRps;
extern unsigned16  hRampMecRps;

extern enum ePamModeTag SwitchPamMode(enum ePamModeTag mode);
extern void ClearOrStopKdwatictIfNeeded(enum ePamModeTag mode);
extern void ClearOrStopRyoffctIfNeeded(enum ePamModeTag mode);
extern void ClearOrStopKdwatictIfNeeded(enum ePamModeTag mode);
extern void vSPD_Getrpmh(void);
extern UI08 NeedsPrepareStopCtrl(UI08 direction_spd);
extern void ClearOrStop2lfsctIfNeeded(enum eFailSafeModeTag mode);
extern UI08 GetStandardSpeed(void);
extern UI08 IsVoltBoosted(void);
extern UI08 IsSpeedResonance(void);
extern void PrepareStopCtrl(void);
extern unsigned8 ucSPD_sfrqfx;				/*指令Hz(確定値)<dec> */
extern void vHOG_Shtvlt_judge(void);
extern void v2LV_2lflsafe(enum ePamModeTag mode);
extern UI08 SetSpeedLimit(UI08 ucbuf);
extern void vSPD_H50sfrqfx(void);
extern void PumpCountStopIfNeeded(void);
extern UI16 TorqueControlCommutate(UI16 usMdTime, UI16 tmp_duty);

/**************************************************************************
<FileName>		:PMD1GPar.h
**************************************************************************/
/*基本パラメータ */
#define  D_fc  16							/*発振子周波数 16MHz */
#define  D_f32 32							/*クロック分周32分周 */
#define  D_f8  8							/*クロック分周 8分周 */
#define  D_f4  4							/*クロック分周 4分周 */
#define  D_f2  2							/*クロック分周 2分周 */
#define  D_f1  1							/*クロック分周 1分周(分周ﾅｼ) */
#define  D_fTRC		(1)						/*trcgrd設定値を処理遅延(trcgrd割込&trd1割込内位置検許可設定)考慮した補正値 27us */
/*位置検出機能関連 */

/*from Torque.h*/
#define TRQ_SUMRANGE	(8)												/* 機械位置間隔を積算する個数 */
/*from Torque.h*/

#endif /* INV_COMMON_FUNC_H */


