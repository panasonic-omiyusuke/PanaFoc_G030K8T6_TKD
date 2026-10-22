/**************************************************************************
<FileName>		:OUTPUT_I.c
<Function>		:出力処理(16進⇒BCD変換処理、送信ﾃﾞｰﾀｾｯﾄ処理)
<Program>		:ucTRNS_Nrm2bcd(),vTRNS_Outputi()
<Author>		:大内山 智則 -Outi
<Date>			:2002-0128
<Update>		:Ver REF02A-0.00 2002-05-11
				;Ver REF045-1.00 2004-06-04 -Outi HTRｲﾝﾊﾞｰﾀ関連ﾃﾞｰﾀ送信仕様の追加
				;Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				;Ver REF052-1.00 2005-06-26 -Outi ｳﾈﾘ状態信号送信仕様追加
				;Ver REF061-0.00 2006-04-14 -Outi 2ﾚﾍﾞﾙﾘﾚｰ廃止対応(2ﾚﾍﾞﾙ保護制御廃止)
				;Ver REF061-0.01 2006-06-01 -Tomo 2ﾚﾍﾞﾙ保護判定削除
				:Ver REF070-0.00 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF090-0.00 2008-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]004axXXX[09/05/08]
				;Ver REF1000-0.01 2010-04-23 -Mura PamHogo対応	@001MwDOD[10/04/22]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
 **************************************************************************/


/********************************************
		インクルードファイル
 ********************************************/
#include "Sys_type.h"
#include "Output_i.h"
#include "vipdc.h"
#include "PMD1GPar.h"						/*inv関連ﾊﾟﾗﾒｰﾀ */

#include "Trans_Inv.h"
#include "Pmd1G.h"
#include "Speedcnt.h"
#include "RefSpeed.h"
#include "angle_control.h"
#include "Timea.h"
#include "main.h"
#include "InvCommonFunc.h"
#include "mc_configuration_registers.h"
#include "motor_process.h"
#include "motor_error.h"
#include "pwm.h"
#include "system_ram.h"
#include "Dtyctrl.h"
#include "bus_voltage_sensor.h"
#include "brake.h"
#include "mc_api.h"

/*******************************************
		マクロ定義
 *******************************************/
#define HOGO_WIRECHKNUM		(3)				/* ｺﾝﾌﾟﾚｯｻ巻線保護通信データのチェック回数 *//* @305swDOD[24/09/02] */
/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
UI08 sfrq_d;
unsigned16 unTRNS_iq_decia;    /* q軸電流[A]*10 通信送信用(上限判定はTrans_Inv.cで実施) */

/*** static ***/
const UCHAR ucg_wireptntbl[HOGO_WIRECHKNUM] = { /* ｺﾝﾌﾟﾚｯｻ巻線保護通信データのチェック用 *//* @305swDOD[24/09/02] */
	0x0,				/* 初期チェック値(0であることを最初にチェックする。) */
	0x5,				/* 1回目チェック値 */
	0xA					/* 2回目チェック値 */
};
/********************************************/
/*		外部変数参照						*/		/* @037MzDOD[14/01/27] */
/********************************************/

/*******************************************
		プロトタイプ宣言
 *******************************************/
unsigned8 ucTRNS_PWMfreq();
UI08 ucTRNS_carriernow(unsigned16 ull_carriernow);
unsigned8 ucTRNS_vltnow(unsigned16 ull_vltnow);
unsigned8 GetRoundedMecSpeed(unsigned16 SPD_current_rps);
unsigned8 GetRoundedHalfMotorPower(unsigned16 fl_pwr_avg);
unsigned8 ucTRNS_Nrm2bcd(unsigned8 nrm);
static unsigned8 ucTRNS_BrakeVoltTop2Dec(SI16 sl_prev_value);

unsigned8 bCurMecSpeedRps;

/**************************************************************************
<Title>			:ucTRNS_BrakeVoltTop2Dec
<Function>		:端子電圧をDEC化
<Description>	:負値は0へ丸め、10で割った値を送信用DECへ変換する。
<Input>			:sl_prev_value(端子電圧計測値)
<Return>		:DEC(0～159)
<Author>		:Sakamoto
<Date>			:2026-07-29
<Update>		:
<Copyright> 	:Panasonic Co.,Ltd.
**************************************************************************/
static unsigned8 ucTRNS_BrakeVoltTop2Dec(SI16 sl_prev_value)
{
	UI16 ul_prev_abs;
	unsigned8 uc_prev_dec_tx;

	if (sl_prev_value < 0)
	{
		ul_prev_abs = 0;
	}
	else
	{
		ul_prev_abs = (UI16)sl_prev_value;
	}

	ul_prev_abs /= 10U;

	if (ul_prev_abs > 159U)
	{
		ul_prev_abs = 159U;
	}

	uc_prev_dec_tx = (unsigned8)ul_prev_abs;

	return uc_prev_dec_tx;
}


/**************************************************************************
<Title>			:ucTRNS_Nrm2bcd 		;vTRNS_Outputi
<Function>		:16進 ⇒BCD変換
<Description>	:実Hzをﾏｲｺﾝ内部の16進表示から通信用のBCD表示に変換。
<Input>			:hCurMecRps 実回転数（小数点第１位まで）
<Return>		:ucr_frq 実回転数（小数点以下四捨五入）
<Author>		:-Outi	作成
<Date>			:2002-0219
<Update>		:Ver REF02A-0.00 2002-02-19
				;Ver REF045-1.00 2004-06-04 -Outi HTRｲﾝﾊﾞｰﾀ関連ﾃﾞｰﾀ送信仕様の追加
				;Ver REF052-1.00 2005-06-26 -Outi ｳﾈﾘ状態信号送信仕様追加
				;Ver REF1000-0.01 2010-04-23 -Mura PamHogo対応	@001MwDOD[10/04/22]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
unsigned8 ucTRNS_Nrm2bcd(unsigned8 nrm)	/*16進 ⇒BCD変換(引数：16進) */
{
	unsigned8    bcd;								/*戻り値BCD */
	unsigned8    syou;								/*ﾃﾝﾎﾟﾗﾘ2(8bit)準備 */
	unsigned8    amari;								/*ﾃﾝﾎﾟﾗﾘ3(8bit)準備 */

	syou  = nrm / 10;								/* syou = 実回転数 / 10 */
	amari = nrm % 10;								/* amari = 実回転数 / 10 */
	bcd = syou * 16 + amari;						/*ﾃﾝﾎﾟﾗﾘ←BCD */

	return bcd	;									/*BCD戻し */

}
/**************************************************************************
<Title>			:uc_wirehgstjdg
<Function>		:コンプ巻線保護ﾌﾗｸﾞ送信ﾃﾞｰﾀ設定
<Description>	:ﾒｲﾝﾏｲｺﾝとシーケンシャルに通信を判定するようにしてｺﾝﾌﾟﾚｯｻ巻線
				:保護ﾌﾗｸﾞを送信する。
<Input>			:通信受信データ
<Return>		:ｺﾝﾌﾟﾚｯｻ巻線保護ﾌﾗｸﾞ送信データ
<INPUT(EXT)>	:-
<OUTPUT(EXT)>	:-
<Author>		:Takida
<Date>			:24/09/02  @305swDOD[24/09/02]
<Update>		:
<Copyright>		:Panasonic Co.,Ltd.
**************************************************************************/
static UCHAR uc_wirehg_tx(UCHAR ucl_wirehgflg){
	static UCHAR ucl_chkmod = CLR;								/* チェック回数 */
	static UCHAR ucl_flg_tx = CLR;								/* ﾒｲﾝﾏｲｺﾝへの巻線保護フラグ送信データ */

	if( ucl_chkmod < HOGO_WIRECHKNUM ){							/* チェック回数が規定数を満たしていない ? */
		if(ucl_wirehgflg == ucg_wireptntbl[ucl_chkmod]){		/* 受信の巻線保護制御フラグがチェック値と一致 ? */
			ucl_chkmod++;										/* チェック回数をカウントアップ */
			ucl_flg_tx = ucg_wireptntbl[ucl_chkmod];			/* 送信データに次のチェック値を設定 */
		}
	}
	if( ucl_chkmod >= HOGO_WIRECHKNUM ){						/* チェック回数が規定数を満たした ? */
		ucl_chkmod = CLR;										/* チェック回数をｸﾘｱ */
	}

	return (ucl_flg_tx);
}
/**************************************************************************
<Title>			:vTRNS_Outputi 		;vMAIN_Inttc1pros
<Function>		:送信データセット処理
<Description>	:主局マイコンへの送信データのセットを行う。
<Input>			:hCurMecRps[実Hz],unPMDc_dutyn[Duty値],ucHOG_movF[ICP状態ﾃﾞｰﾀ],ucr_ovlap[lap角]
<Return>		:ucTRNS_ifrq_b[実Hz],ucTRNS_duty[Duty値],ucTRNS_ista[ICP状態ﾃﾞｰﾀ],ucTRNS_lap[lap角]
<Author>		:-Outi	作成
<Date>			:2002-0219
<Update>		:Ver REF02A-0.00 2002-02-19
				;Ver.REF02A-1.00 2002-04-04 追加 ucTRNS_lap演算・格納処理
				;Ver REF045-4.00 2004-08-05 -Outi 静的解析結果対応(if/else文｛｝追加)
				;Ver REF052-1.00 2005-06-26 -Outi ｳﾈﾘ状態信号送信仕様追加
				;Ver REF061-0.00 2006-04-14 -Outi 2ﾚﾍﾞﾙﾘﾚｰ廃止対応(2ﾚﾍﾞﾙ保護制御廃止)
				;Ver REF061-0.01 2006-06-01 -Tomo 2ﾚﾍﾞﾙ保護判定削除
				:Ver REF070-0.00 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF090-0.00 2008-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]004axXXX[09/05/08]
				;Ver REF1000-0.01 2010-04-23 -Mura PamHogo対応	@001MwDOD[10/04/22]
				:2023/07/26 シャント電流検出によるうねり抑制制御追加に伴いCT廃止のためH53処理ｺﾒﾝﾄｱｳﾄ 瀧田 @263swDOD[23/07/26]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Outputi(void){							/*送信ﾃﾞｰﾀｾｯﾄ処理 */
	unsigned8  rpmcom = 0;								/*実回転(8bit)準備 */
	unsigned8  ucbuf = 0;								/*ﾃﾝﾎﾟﾗﾘ2(8bit)準備 */
	unsigned16 unbuf = 0;								/*ﾃﾝﾎﾟﾗﾘ3(16bit)準備 */
	qd_f_t Iqd_now;
	float_t fl_iq_decia;
	/* */
	ucTRNS_ista = 0;								/*一度、状態データをクリアする */
	ucTRNS_ista2 = 0;								/*一度、状態データをクリアする *//* @305swDOD[24/09/02] */
	/*駆動周波数---------------------------- */
	if ( ucTRNS_sfrq_b == 0x00 ){					/*運転停止／起動ｼｰｹﾝｽ･ﾛｯｸ中 00-0529 -AK */
		ucTRNS_ifrq_b = 0x00;						/*回転数は0r/sec */
		/*ＩＣＰﾕﾆｯﾄ動作状況-------------------- */
	}else{											/*ucr_UNTEN >= 01 ICPﾕﾆｯﾄ動作中 */
		ucTRNS_ista |= 0x10;						/*icpﾕﾆｯﾄ動作中 ucTRNS_ista.4 */
		
		/*------------------------------------------------------------
		* 現在の平均機械速度を取得し、四捨五入で整数化
		*------------------------------------------------------------*/
		ucbuf = GetRoundedMecSpeed(hCurMecRps);		/*ﾃﾝﾎﾟﾗﾘ←現行Hz少数１位以下四捨五入 */
		bCurMecSpeedRps = ucbuf;					/* 現行Hzをrpsに変換してグローバル変数に格納 */
		if ( u8g_SPD_set == 0 ){						/*回転数指令 00-0529 -ak */
			ucTRNS_ifrq_b = 0;						/*実Hz←０ */
		}else{										/* */
			ucTRNS_ifrq_b = ucTRNS_Nrm2bcd(bCurMecSpeedRps);	/*実回転数格納(bcd)  */
		}											/* */
		rpmcom = vSPD_Bcd2nrm(ucTRNS_sfrq_b);		/*ﾃﾝﾎﾟﾗﾘ1←指令Hz(BCD)10進変換値 */
		unbuf = (unsigned16)(u8g_SPD_set * 10);		/*ﾃﾝﾎﾟﾗﾘ3←目標[rps×10] */
		if ( rpmcom == ucbuf ){						/*ucTRNS_sfrq_b 指令Hzと駆動Hzが等しい 00-0529 -ak */
			if ( ucTRNS_ifrq_b != ucTRNS_sfrq_b){	/*ucTRNS_ifrq_b,ucTRNS_sfrq_b 実Hz／指令Hz(BCD)が異なる 00-0529 -ak */
				/*ﾓｰﾄﾞが安定していても,指令が変更されたとき */
				ucTRNS_ista |= 0x20;				/*コマンド実行中 set1 ucTRNS_ista.5 */
			}else if(hCurMecRps <= (unbuf+10)){	/*実回転数≦駆動Hz＋１ */
				if(hCurMecRps >= (unbuf-10)){	/*実回転数≧駆動Hz－１ */
					ucTRNS_ista &= 0xdf;			/*コマンド実行終了 clr1 ucTRNS_ista.5 */
				}									/* */
			}										/* */
		}else{										/*指令回転数と目標回転数が異なる */
			ucTRNS_ista |= 0x20;					/*コマンド実行中 set1 ucTRNS_ista.5 */
		}											/* */
	}												/* */

	/*** 送信ﾃﾞｰﾀ処理：DUTY ***/
	ucbuf = (unsigned8) (unPMDc_dutyn / 10);		/*ﾃﾝﾎﾟﾗﾘ3←Duty[%](=Duty[‰]/10) */
	if ( ucbuf >= 100 ){							/* 上限ﾘﾐｯﾄ */
		ucbuf = 100;								/* ﾃﾝﾎﾟﾗﾘ3←100[%]丸め込み */
	}												/* */
	ucTRNS_duty = ucbuf;							/*Duty(送信用)←ﾃﾝﾎﾟﾗﾘ3 */
	
	/*** 送信ﾃﾞｰﾀ処理：平均電力 ***/
	ucTRNS_pwr  = GetRoundedHalfMotorPower((unsigned16)u16g_MOTOR_pwr_avg);	/*平均電力[W]を送信用に丸め込み */

	/*** 送信ﾃﾞｰﾀ処理：q軸電流(Iq) [A]*10, DEC ***/
	Iqd_now = MC_GetIqdMotor1_F();
	fl_iq_decia = Iqd_now.q * 10.0F;
	if (fl_iq_decia < 0.0F)
	{
		unTRNS_iq_decia = 0U;
	}
	else
	{
		unTRNS_iq_decia = (unsigned16)fl_iq_decia;
	}
	
	/*** 送信ﾃﾞｰﾀ処理：端子電圧 ***/
	ucTRNS_prev_large_dec = ucTRNS_BrakeVoltTop2Dec(prev_large);	/* 端子電圧A(上位2桁BCD) */
	ucTRNS_prev_small_dec = ucTRNS_BrakeVoltTop2Dec(prev_small);	/* 端子電圧B(上位2桁BCD) */

#if 0 /* 今後の検討で必要になる可能性ありのためｺﾒﾝﾄとして本モジュールを残す */
	if(ucPMD_VctrMode == kDriveVector){						/*VCTﾓｰﾄﾞ≡４(矩形波VCT運転中)なら */
		ucTRNS_lap = ucOVLP_UNElap;					/*送信ﾃﾞｰﾀlap←ｳﾈﾘ発生時lap角 */
		ucTRNS_lap |= 0x10;							/*運転状態信号←SET(矩形波VCT中) */
		if(unPMD_CMP3 > T_TrgtSpdTbl[ucSPD_rpmh]){	/*現在60ﾟe周期＞目標60ﾟe周期(加速中)なら */
			ucTRNS_lap |= 0x20;						/*加速中信号←SET(加速中) */
		}else{										/*現在60ﾟe周期≦目標60ﾟe周期(減速or一定速中)なら */
			ucTRNS_lap &= 0xdf;						/*加速中信号←CLR(減速or一定速中) */
		}											/* */
/*		if(ucMAIN_vltnow < 0x7f){			*/		/*母線電圧＜127(350V) */
/*			ucTRNS_duty = ucMAIN_vltnow;	*/		/*duty信号ﾃﾞｰﾀ←母線電圧 */
/*		}else{								*/		/*母線電圧≧127(350V) */
/*			ucTRNS_duty = 0x7f;				*/		/*母線電圧←127(350V) */
/*		}									*/		/* */
	}else{											/*VCTﾓｰﾄﾞ≠４(ｾﾝｻﾚｽ運転中)なら */
		ucTRNS_lap = (angle.overlap >> 6);				/*ﾗｯﾌﾟ角(送信用)←ﾗｯﾌﾟ角(制御用) */
		ucOVLP_UNElap = (angle.overlap >> 6);			/*ｳﾈﾘ発生時lap角←ﾗｯﾌﾟ角(制御用) */
	}/*＃ 午後 09:17 REF052-1.00 */
#endif

/*** 送信ﾃﾞｰﾀ処理：通電角 ***/
	ucg_TRNS_angle = 0x00;

/*** 送信ﾃﾞｰﾀ処理：異常ﾌﾗｸﾞ2 ***/
/*	if(ucZX_CTjudgeCnt2 < D_H53TIME)		 */		/* CT回路異常確定用ｶｳﾝﾀ2＜H53確定時間 */				/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	{										 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		if(ucZX_CTerrorFlgs & 0x02){}		 */		/* 異常FLGAｾｯﾄ状態<成り行き運転状態>⇒未処理 */		/* @004axOOM[09/05/08] */		/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		else								 */		/* CT回路異常判断FLG2(異常FLGA)ｸﾘｱ状態<回路正常状態> *//* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		{									 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*			ucZX_CTjudgeCnt2 = 0;			 */		/* CT回路異常確定用ｶｳﾝﾀ2(異常FLGA確定用ｶｳﾝﾀ)←ｸﾘｱ */	/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		}									 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	}else{}									 */		/* CT回路異常確定用ｶｳﾝﾀ2≧H53確定時間⇒未処理 */		/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*											 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	if(ucZX_CTjudgeCnt1 >= D_H53NUM)		 */		/* CT回路異常確定用ｶｳﾝﾀ1≧H53確定回数 */				/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	{										 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		ucTRNS_ista2 |= 0x40;				 */		/* H53-FLG←ｾｯﾄ */										/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	}else if(ucZX_CTjudgeCnt2 >= D_H53TIME)	 */		/* CT回路異常確定用ｶｳﾝﾀ2≧H53確定時間 */				/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	{										 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		ucTRNS_ista2 |= 0x40;				 */		/* H53-FLG←ｾｯﾄ */										/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	}else									 */		/* CT回路異常確定用ｶｳﾝﾀ1･2＜H53確定値(条件不成立) */	/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	{										 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*		ucTRNS_ista2 &= 0xbf;				 */		/* H53-FLG←ｸﾘｱ */										/* @004axOOM[09/05/08] */	/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */
/*	}										 */																							/* CT不使用のためｺﾒﾝﾄｱｳﾄ*//* @263swDOD[23/07/26] */


/*** 送信ﾃﾞｰﾀ処理：異常ﾌﾗｸﾞ ***/
	if ( ucHOG_sttF < 2 ){							/*01-05-22 -outi */
		ucTRNS_ista &= 0xbf;						/* icpﾕﾆｯﾄ正常 clr1 ucTRNS_ista.6 */
	}else{											/*保護状態 */
		ucTRNS_ista |= 0x40;						/* icpﾕﾆｯﾄ異常 set1 ucTRNS_ista.6 */
		ucTRNS_ista &= 0xdf;						/* ｺﾏﾝﾄﾞ実行終了 clr1 ucTRNS_ista.5 00-0529 -ak */
		ucTRNS_ista &= 0xef;						/* icpﾕﾆｯﾄ停止中 clr1 ucTRNS_ista.4 00-0529 -ak  */
		if((ucHOG_sttF & 0x10) != 0){				/*  icp保護(過電流) F_ihogst  */	/* 午後 03:49 REF045-4.00 */
			ucTRNS_ista |= 0x01;					/* set1 ucTRNS_ista.0 */
		}											/* 午後 03:49 REF045-4.00 */
		if((ucHOG_sttF & 0x04) != 0){				/*  過電圧保護 F_svhost  */ /*＠~07-0422 -Outi 機種切り替え対応 */
			if( ucTYPE_global == 1 ){				/*[海外機種]なら */
				ucTRNS_ista |= 0x02;				/* set1 ucTRNS_ista.2 */
			}else{									/*[国内機種]なら */
				if(ucTYPE_2level == 1){	/*(2Level制御ON)∧(通信正常)*/		/* @001MwDOD[10/04/22]_STA */
					ucTRNS_ista |= 0x02;			/*2ﾚﾍﾞﾙ電圧保護用*/
				}else{								/*(2Level制御OFF)∧(通信正常)*/
				/*何もしない*/
				}																					/* @001MwDOD[10/04/22]_END */
			}										/* */
		}											/*＃ 午後 08:42 REF070-0.00 */
		if((ucHOG_sttF & 0x08) != 0){				/*  直流電圧低下保護 F_svhost  */	/* 午後 03:48 REF045-4.00 */
			ucTRNS_ista |= 0x04;					/* set1 ucTRNS_ista.2 */
		}											/* 午後 03:48 REF045-4.00 */
		if((ucHOG_sttF & 0x20) != 0){				/*  ﾛｯｸ保護 F_rchost  */	/* 午後 03:47 REF045-4.00 */
			ucTRNS_ista |= 0x08;					/* set1 ucTRNS_ista.3 */
		}											/* 午後 03:48 REF045-4.00 */
		if((ucHOG_sttF & 0x02) != 0){				/* ｺﾝﾌﾟﾚｯｻ巻線保護 */														/* @305swDOD[24/09/02] */
			ucTRNS_ista2 |= uc_wirehg_tx(ucTRNS_wirehgflg & 0x0F);	/* ﾒｲﾝﾏｲｺﾝから受信したﾃﾞｰﾀ下位4bitに応じて送信ﾃﾞｰﾀを決定 */	/* @305swDOD[24/09/02] */
		}
	}

	/*--------------送信ﾃﾞｰﾀ処理：ｷｬﾘｱ周波数-------------*/		/* @104yxDOD[18/10/03] */
	ucg_TRNS_PWMfreq = ucTRNS_carriernow(PWM_FREQUENCY);	/*キャリア周波数設定*/

	/*--------------母線電圧設定------------------------*/
   ucMAIN_vltnow = ucTRNS_vltnow(u16s_vbus);   /*母線電圧設定*/
}

/**************************************************************************
<Title>			:ucTRNS_carriernow
<Function>		:キャリア周波数を8bitのDEC値で返す
<Description>	:
<Input>			:
<Return>		:
<Author>		:Sakamoto
<Date>			:2026-07-01
<Update>		:
<Copyright> 	:Panasonic Co.,Ltd.
**************************************************************************/
UI08 ucTRNS_carriernow(unsigned16 ull_carriernow){
	unsigned8 uc_carriernow_nrm;

	uc_carriernow_nrm = (unsigned8)(ull_carriernow / 100U);				/*上位2桁(10の位以上)を抽出*/
	if (uc_carriernow_nrm > 159){
		uc_carriernow_nrm = 159U;									/*2桁通信データ上限へ丸め*/
	}
	return uc_carriernow_nrm;
}

/**************************************************************************
<Title>			:ucTRNS_vltnow
<Function>		:母線電圧の上位2桁を抽出し、2桁BCDへ変換する
<Description>	:
<Input>			:
<Return>		:ull_vltnow(母線電圧[AD値,7bit])
<Author>		:Sakamoto
<Date>			:2026-07-01
<Update>		:
<Copyright> 	:Panasonic Co.,Ltd.
**************************************************************************/
UI08 ucTRNS_vltnow(unsigned16 ull_vltnow){
	unsigned8 uc_vltnow_nrm;

	uc_vltnow_nrm = (unsigned8)(ull_vltnow / 10U);				/*上位2桁(10の位以上)を抽出*/
	if (uc_vltnow_nrm > 99U){
		uc_vltnow_nrm = 99U;									/*2桁BCD上限へ丸め*/
	}
	return ucTRNS_Nrm2bcd(uc_vltnow_nrm);
}

/**************************************************************************
<Title>			:GetRoundedMecSpeed
<Function>		:現在の回転数unSPD_rpmcrntをrpsに変換し、四捨五入して8bitの整数値に変換
<Description>	:
<Input>			:hCurMecRps(実回転数, 16bit)
<Return>		:ucTRNS_rps(実回転数rps, 8bit)→ucTRNS_ifrq_b(実回転数rps, 8bit)へ格納予定
<Author>		:Sakamoto
<Date>			:2026-06-15
<Update>		:
<Copyright> 	:Panasonic Co. 	,Ltd.
**************************************************************************/
unsigned8 GetRoundedMecSpeed(unsigned16 hCurMecRps_val){
	unsigned8 ucTRNS_rps;		/*実回転数rps(8bit)*/
	ucTRNS_rps = (unsigned16)((hCurMecRps_val + RPS_X10_ROUND_OFFSET) / RPS_X10_DIVISOR);	/*四捨五入して8bitの整数値に変換*/
	return (unsigned8)ucTRNS_rps;
}

/**************************************************************************
<Title>			:GetRoundedHalfMotorPower
<Function>		:モータの電力の平均値を半分にして、上限値を超えないように丸める
<Description>	:
<Input>			:
<Return>		:
<Author>		:Sakamoto
<Date>			:2026-07-29
<Update>		:
<Copyright> 	:Panasonic Co. 	,Ltd.
**************************************************************************/
unsigned8 GetRoundedHalfMotorPower(unsigned16 fl_pwr_avg){
	unsigned16 ucTRNS_halfpower;		/*モータ出力[W]*/

	ucTRNS_halfpower = fl_pwr_avg >> 1; /*モータ出力÷2 */
	if(ucTRNS_halfpower > 159){
		ucTRNS_halfpower = 159U;	/*通信データ上限へ丸め*/
	}
	return (unsigned8)ucTRNS_halfpower;
}
