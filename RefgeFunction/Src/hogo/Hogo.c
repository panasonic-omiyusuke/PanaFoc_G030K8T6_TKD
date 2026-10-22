/**************************************************************************
<FileName>		:HOGO.c
<Function>		:ｲﾝﾊﾞｰﾀ保護制御
<Program>		:vHOG_Hogo(), vHOG_Shtvlt(), vHOG_Pamhogo(), vHOG_Ipmhg(), vHOG_Rockhg(), vHOG_5minwt()
<Author>		:大内山 智則 -Outi
<Date>			:2002-0510
<Update>		:Ver REF02A-0.00 2002-05-24
				:Ver REF036-0.00 2002-12-18 -hide ﾛｯｸ保護判定時転流周期取得仕様に変更（ｶﾌﾟｾﾙ化）
				:Ver REF036-1.00 2002-12-27 -hide 基本ﾀｲﾏ2ms⇒3.3ms変更
				:Ver REF03D-0.00 2002-05-17 -Outi 適切なｱﾙｺﾞﾘｽﾞﾑに変更
				:Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF052-0.00 2005-05-10 -Outi DR指摘対応
				:Ver REF058-0.00 2005-07-05 -Outi サンケンＩＰＭ対応
				:Ver REF061-0.00 2006-04-13 -Outi 2ﾚﾍﾞﾙﾘﾚｰ廃止対応(2ﾚﾍﾞﾙ保護制御廃止)
				:Ver REF061-0.01 2006-04-27 -Outi IPM論理判定処理追加
				:Ver REF070-0.00 2007-01-09 -Outi Ver.4試作基板対応
				:Ver REF070-0.01 2007-04-16 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.03 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-05-19 -Outi 機種切替＆Fo入力端子合理化
				:Ver REF1000-0.01 2010-01-20 -Mura 2Level対応	@000MwDOD[10/01/20]
				:Ver REF1000-0.02 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/05]
				:Ver REF1000-0.03 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/22]
				:Ver REF1000-0.04 2010-05-10 -Mura 2Level保護制御対応     @003MxDOD[10/05/10]
				:Ver REF1010-0.01 2010-10-27 -Mura P12検知(過電圧保護判定)処理対応 @011MwDOD[10/10/27]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
**************************************************************************/


/********************************************
		インクルードファイル
 ********************************************/
#include "Sys_type.h"
#include "Hogo.h"								/* */
#include "PMD1GPar.h"							/*PMD系パラメータファイル */
#include "vipdc.h"
#include "Adpdc.h"

#include "Timea.h"
#include "main.h"
#include "Dtyctrl.h"
#include "Trans_Inv.h"
#include "Pmd1G.h"
#include "Pmd_int.h"

#include "Torque.h"
#include "Speedcnt.h"

#include "system_ram.h"
#include "RefSpeed.h"
#include "PositionDetect.h"

#include "pwm.h"	/* for vPMD_Brake */
#include "adc.h"	/* for vPMD_Brake */

#include "stm32g0xx_it.h"

#include "InvCommonFunc.h"
#include "motor_error.h"
#include "motor_process.h"
#include "Hogo.h"


/*******************************************
		マクロ定義
 *******************************************/
/*-------------------------------------------------------------------------------------------------- */
/*vHOG_Shtvlt() */
/*vHOG_Pamhogo() */
/*vHOG_Rockhg() */							/* ﾛｯｸ保護ﾚﾍﾞﾙﾊﾟﾗﾒｰﾀﾌｧｲﾙ変更可能仕様に改良 */
#define  D_DUTYS  60						/*Duty:  6  %　lock保護判定Duty（基準） */
#define  D_rckhgminrpm	  5					/* ﾛｯｸ保護検出用最低回転数        5 rps */
#define  D_RCKHGMAXSPD	(1000000/D_rckhgminrpm/3/ D_POLE) /*   5rps相当転流周期 */
#define  D_rckhgmaxrpm	120					/* ﾛｯｸ保護検出用最低回転数      120 rps */
#define  D_RCKHGMINSPD	(1000000/D_rckhgmaxrpm/3/ D_POLE) /* 120rps相当転流周期 */

#define MASK_HOGOFLG (0xFE)						/* 保護ﾌﾗｸﾞ(ucHOG_movF,ucHOG_sttF)マスク用定義 *//* 1111 1110 保護割当てに応じて変更のこと *//* @311syRRD[24/09/04] */
#define MASK_HOGOFLGCOM (0x3E)					/* 通信データに使用する保護ﾌﾗｸﾞ(ucTRNS_istaやucTRNS_ista2に反映する)マスク用定義 *//* 0011 1110 保護割当てに応じて変更のこと *//* @311syRRD[24/09/04] */

/* #define CURRENT_PROTECTWIRE (840) */			/* 840(12bit,AD入力からｵﾌｾｯﾄを減算)←1.50A相当 */	/* @304swDOD[24/09/02] */
#define CURRENT_PROTECTWIRE (560)				/* 560(12bit,AD入力からｵﾌｾｯﾄを減算)←1.00A相当 *//* EY2設試I時点の仮値2024.09.02 *//* @304swDOD[24/09/02] */
#define VxI_PROTECTWIRE (437*1616)				/* 電流0.780A相当は437、AC電圧100V相当は1616 */	/* @318swROD[24/11/14] */
#define TIME_PROTECTWIRE (10)					/* 10*6sec=1min */									/* @304swDOD[24/09/02] */
#define TIME_CALC_CRNT_AVERAGE (20)				/* 20*3.3msec=66.66ms *//* 最低回転数15r/s想定 */	/* @304swDOD[24/09/02] */

/********************************************/
/*		変数宣言							*/		/* @037MzDOD[14/01/27] */
/********************************************/
/*** global ***/
unsigned8	ucHOG_movF;					/*保護動作格納ﾒﾓﾘ */
		/*   ---- ---1 : unused.  */
		/*   ---- --1- : ｺﾝﾌﾟﾚｯｻ巻線保護.  */
		/*   ---- -1-- : ２レベル異常保護.  */
		/*   ---- 1--- : 直流電圧低下保護.  */
		/*   ---1 ---- : IPM保護 EMG error. */
		/*   --1- ---- : ﾛｯｸ保護. */
		/*   -1-- ---- : 強制転流失敗error. */
		/*   1--- ---- : 位置検出間隔(60ﾟe) TimeOut error.  */
unsigned8	ucHOG_sttF;					/*保護状態格納ﾒﾓﾘ */
		/*   ---- ---1 : unused.  */
		/*   ---- --1- : ｺﾝﾌﾟﾚｯｻ巻線保護.  */
		/*   ---- -1-- : ２レベル異常保護.  */
		/*   ---- 1--- : 直流電圧低下保護.  */
		/*   ---1 ---- : IPM保護 EMG error. */
		/*   --1- ---- : ﾛｯｸ保護. */
		/*   -1-- ---- : 強制転流失敗error. 				※通信データ(ucTRNS_istaやucTRNS_ista2)に反映しない */
		/*   1--- ---- : 位置検出間隔(60ﾟe) TimeOut error. 	※通信データ(ucTRNS_istaやucTRNS_ista2)に反映しない */
enum eShortVltType uc2LV_scantyvlt_judge;					/*直流電圧低下検知手段切替ﾌﾗｸﾞ*/	/*@001MwDOD[10/04/05]*/
unsigned8	uc2LV_scantyvlt_skip;					/*直流電圧低下保護Skipﾌﾗｸﾞ*/ 		/*@001MwDOD[10/04/22]*/
unsigned8	ucMINtm_HOGOct;				/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間の計測ｶｳﾝﾀ */	/* @021ywDOD[18/01/25] */
unsigned8	ucg_HOGO_lockct;			/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀﾛｯｸ保護(FO信号含む)継続回数ｶｳﾝﾀ */	/* @101ywOTT[18/09/14] */
unsigned8	ucHOG_sttF_old;				/* 以上連続回数カウント用、前回保護状態 */				/* @101ywOTT[18/09/14] */
unsigned16	current_wire;				/* ｺﾝﾌﾟ巻線保護判断用電流値 *//* @304swDOD[24/09/02] */
unsigned8	ucHOG_ave_timect;			/* 電流平均計算間隔用カウント 3.3ms*TIME_CLALC_CRNT_AVERAGE *//* @304swDOD[24/09/02] */
UI32	ulHOG_VxI;						/* PN間電圧とコンプ巻線電流の乗算値 *//* @318swROD[24/11/14] */

/*from Speedcnt.c start*/
#define  D_DLNHGTM		250		/*共振保護制御動作時間（250）	*/
#define  D_DLNRLSTM		40		/*共振保護制御解除時間（40）	*/
/*from Speedcnt.c end*/
//from midware

enum ePamProtectModeTag pam_protect_mode;

unsigned8	ucHOG_1stON;				/* 2ﾚﾍﾞﾙﾘﾚｰ異常(過電圧) 初回強制判定用ﾌﾗｸﾞ */	/* @022ywOTT[18/02/16] */


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/*** static ***/
/*******************************************
		プロトタイプ宣言
 *******************************************/
void vHOG_Shtvlt_judge(void);				/*直流電圧低下検知手段切替処理 */		/* @001MwDOD[10/04/05]*/

//extern	unsigned16	unPMD_GetMCAPdumy();
//extern	void  vHOG_Shtvlt_judge(void);						/* @059MwOTH[11/01/26] */

UI08 SetSpeedLimit(UI08 ucbuf);
void v2LV_Drvry(UI08 state);
void v2LV_2lflsafe(enum ePamModeTag mode);
void vHOG_Shtvlt_judge(void);
void vSPD_Getrpmh(void);
void PrepareStopCtrl(void);
UI08 NeedsPrepareStopCtrl(UI08 direction_spd);
void vHOG_5minwt(void);
void CheckDline(void);


#ifndef LOCKTST														/* 通常(LOCK保護ｱﾘ) */
void ErrMotorLock(void)  {
	vPMD_Brake();
	ucHOG_movF |= 0x20;
}
void ErrTimeOut(void){
	vPMD_Brake();
	ucHOG_movF |= 0x80;
}
void ErrShortVlt(void){
	vPMD_Brake();
	ucHOG_movF |= 0x08;
}
#else
void ErrMotorLock(void){};
void ErrTimeOut(void){};
void ErrShortVlt(void){};
#endif

void EnableShtVltBreak(void){
	UI32 tmp_af1;
	tmp_af1 = htim1.Instance->AF1;
	if( (tmp_af1 & TIM1_AF1_BKCMP1E) == 0 ){		/* COMP1(不足電圧)でのBRK無効設定なら */
		tmp_af1 = (tmp_af1 | TIM1_AF1_BKCMP1E);		/* 2LvリレーオンのタイミングでCOMP1(不足電圧割込)有効設定 */
		htim1.Instance->AF1 = tmp_af1;				/* ﾚｼﾞｽﾀに設定を反映 */
	}
}
void DisableShtVltBreak(void){
	UI32 tmp_af1;
	tmp_af1 = htim1.Instance->AF1;
	if( (tmp_af1 & TIM1_AF1_BKCMP1E) ){				/* COMP1(不足電圧割込)が有効なら */
		htim1.Instance->AF1 = (tmp_af1 & ~TIM1_AF1_BKCMP1E);/* 2LvリレーオンのタイミングでCOMP1(不足電圧割込)無効設定にする */
	}
}
void DisarmPwmTimerBreak(void){
	UI32 tmp_bdtr;
	tmp_bdtr = htim1.Instance->BDTR;
	tmp_bdtr |= (TIM_BDTR_BK2DSRM | TIM_BDTR_BKDSRM);	/* BRK,BRK2割込みを解除 */
	htim1.Instance->BDTR = tmp_bdtr;
}

/**************************************************************************
<Title>			:vHOG_Shtvlt_judge 		;vHOG_Hogo
<Function>		:直流電圧低下検知手段切替処理
<Description>	:(背景) 2ﾚﾍﾞﾙ制御追加に伴い、ﾒｲﾝ指令(直流電圧低下保護検知仕様切替)・運転状態の
				:		組合せによって外部割込み仕様・AD検知仕様を切替える必要性がでてきたため、
				:		本関数を新規作成し、1元的に不足電圧検知仕様を管理する。
				:(内容) 入出力の組合せを以下に記載
				:		直流電圧低下保護がAD検知仕様のとき ⇒ 全運転状態でAD検知仕様
				:		直流電圧低下保護が外部割込み仕様で2ﾚﾍﾞﾙ制御無のとき ⇒ 全運転で外部割込み検知仕様
				:		直流電圧低下保護が外部割込み仕様で2ﾚﾍﾞﾙ制御有りで
				:			(停止中)
				:			   全電圧のとき ⇒ AD検知仕様
				:			   倍電圧で指令回転数有のとき ⇒ 不足電圧保護処理Skip (※1)
				:			   倍電圧で指令回転数無のとき ⇒ 外部割込み仕様
				:			(停止以外)
				:			   ﾘﾚｰOFFのとき ⇒ AD検知仕様
				:			   昇圧中のとき ⇒ AD検知仕様
				:			   ﾘﾚｰONのとき  ⇒ 外部割込み仕様
				:
				:			(※1) 不足電圧保護Skipする理由
									・停止時指令回転数有のﾀｲﾐﾝｸﾞでCTｵﾌｾｯﾄ計測でAD7を使用するためAD検知仕様使用不可。
									・更に上記ﾀｲﾐﾝｸﾞで2LevelﾘﾚｰもONされるが、ﾘﾚｰON直後は昇圧中で倍電圧ﾚﾍﾞﾙに到達してないため外部割込み仕様使用不可。
				:					・よって停止時＋指令回転数有＋ﾘﾚｰON時は不足電圧保護処理をSkipする仕様とする。
<Input>			:ucTYPE_undvlt_int(不足電圧保護仕様切替ﾌﾗｸﾞ)
				:ucTYPE_2level(2Level切替制御ﾌﾗｸﾞ)
				:ucTYPE_uart(通信機種判別確定)
				:uc2LV_enry(ﾘﾚｰ動作状態関連ﾌﾗｸﾞ)
<Return>		:uc2LV_scantyvlt_judge（直流電圧低下検知手段切替ﾌﾗｸﾞ）
<Author>		:mura作成
<Date>			:2010/04/05
<Update>		:Ver REF1000-0.02 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/05]
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vHOG_Shtvlt_judge(void){
	if(ucTYPE_2level == 0){	//2レベルリレーが無い == Vpnは200V系
		if(ucTYPE_undvlt_int == 0){// 外部割込み仕様でない ∧ ucTYPE_global == 1 (前提) or 倍電圧整流
			uc2LV_scantyvlt_judge = kShortVltAdHi;
		} else {
			uc2LV_scantyvlt_judge = kShortVltInt;
		}
	} else { // ad仕様 && ucTYPE_2level == 1 && ucTYPE_global == 0(前提)
		if( !IsVoltBoosted() ){ 									// 昇圧が完了していない == リレーがオンしていないまたは昇圧中
			uc2LV_scantyvlt_judge = kShortVltAdLo; 					// takeoka 昇圧中とリレーオフ中は電圧が低いので外部割込みし様であってもADで低電圧の判定を利用
//OUTPUT_DA(A,0x40);
		} else if(ucTYPE_undvlt_int == 0){ 							// Takeoka AD仕様 ∧ リレーがオンしている ∧ 昇圧中でない
			uc2LV_scantyvlt_judge = kShortVltAdHi;
//OUTPUT_DA(A,0x80);
		} else if((ePMD_drvmode == D_stop) && IsSpeedDrive()){ 	// takeoka 停止から起動しようとしている状態
			uc2LV_scantyvlt_judge = kShortVltSkip;					// takeoka 電圧が十分に上がっていないため、保護判定を行わないようにする
//OUTPUT_DA(A,0xc0);
		} else {													// takeoka 倍電圧整流 ∧外部割込み∧十分に電圧が上がっている
			uc2LV_scantyvlt_judge = kShortVltInt;
//OUTPUT_DA(A,0xff);
		}
	}
}

/**************************************************************************
<Title>			:chk_spdlimit
<Function>		:制限速度の決定
<Description>	:速度制限の優先度に従い最終的な駆動回転数(ucSPD_rpmh)を決定する
<Input>			:ucTYPE_uart(仮),指令速度<dec>(ucbuf)
<Return>		:制限速度(ucbuf)
<Author>		:竹岡 義典
<Date>			:2010-06-15
<Update>		:
 **************************************************************************/
UI08 SetSpeedLimit(UI08 ucbuf){
	ucbuf = SpdLimitDline(ucbuf);									/* Dline共振回転数制限処理 */
	return ucbuf;														/* 決定した回転数制限値を戻す */
}



/**************************************************************************
<Description>	:保護フラグクリア処理
<Author/Date>	:竹岡 義典
<Update>		:
 **************************************************************************/
void ClearErrorFlags(void){
	ucHOG_movF = 0;							/*保護動作ﾌﾗｸﾞ←ｸﾘｱ */
	ucHOG_sttF = 0;							/*保護状態ﾌﾗｸﾞ←ｸﾘｱ */
}

/**************************************************************************
<Title>			:spdlimit_Dline
<Function>		:Dline共振用制限速度の決定
<Description>	:指令速度とDline共振保護の状態により、制限速度を決定
<Input>			:関数呼び出し時点での速度指令(uctmp_sfrqfx)←他関数により決定された制限速度を含む
<Return>		:制限速度(uctmp_sfrqfx)
<Author>		:竹岡 義典
<Date>			:2010-06-15
<Update>		:
 **************************************************************************/
UI08 SpdLimitDline(UI08 uctmp_sfrqfx){									/* D_R3_0、D_R3_1を使用 モジュールをPam_mode.cに移動すれば別定義は解消 */
	if(ucTYPE_Dline == 0){						/*  Dﾗｲﾝ共振保護制御仕様 無 */
		return uctmp_sfrqfx;
	}

	if(IsSpeedResonance()							                /* 共振保護動作状態で        */
			&& (uctmp_sfrqfx > SPD_DEFAULT)){						/* 指令回転数 > limit_speed なら */
		uctmp_sfrqfx = SPD_DEFAULT;							        /* 駆動回転数←D_R3_1    */
	} else {														/* 共振保護非動作状態∨指令回転数≦D_R3_1なら */
        uctmp_sfrqfx = uctmp_sfrqfx;                                /* 駆動回転数←指令回転数 */     
	}
	return uctmp_sfrqfx;
}

/**************************************************************************
<Title>			:vHOG_Wirehg
<Function>		:ｺﾝﾌﾟ巻線保護
<Description>	:電流値が閾値を超える⇒動作ﾌﾗｸﾞSET
<Author>		:Takida
<Date>			:2024.09.02		@304swDOD[24/09/02]
<Copyright> 	:Panasonic Co. , Ltd.
 **************************************************************************/
void vHOG_Wirehg(void){						/*ｺﾝﾌﾟ巻線保護 *//* @304swDOD[24/09/02] */

	ucHOG_ave_timect++;									/* TC1(3.3ms)毎にｶｳﾝﾀｲﾝｸﾘ */
	if(ucHOG_ave_timect >= TIME_CALC_CRNT_AVERAGE){		/* 電流平均値算出間隔設定時間を経過したら */
		current_wire = Calc_current_ave();				/* ｺﾝﾌﾟ電流平均値を格納 */
		ucHOG_ave_timect = 0;							/* ｶｳﾝﾀｸﾘｱ */
		ulHOG_VxI = (UI32)( get_dtc_vpn12b() * current_wire );	/* PN間電圧×コンプ電流 *//* @318swROD[24/11/14] */
	}else{
		/* 何もしない */
	}
	
	if(ulHOG_VxI >= VxI_PROTECTWIRE){	/* @318swROD[24/11/14] */
		/* ｺﾝﾌﾟ巻線保護ﾀｲﾏｶｳﾝﾀ uc6Stmr_wirehgct を進める */
	}else{
		uc6Stmr_wirehgct = 0;							/* ｺﾝﾌﾟ巻線保護ﾀｲﾏｶｳﾝﾀCLR */
	}
	
	if((ucHOG_movF & 0x02) != 0){							/* ｺﾝﾌﾟ巻線保護状態で */
		if(uc6Stmr_hgrlsct >= D_HOGO_CLR_TIME){				/* 保護動作ﾌﾗｸﾞｸﾘｱ用(6s毎)ｶｳﾝﾀ≧5分なら */
			if(ulHOG_VxI >= VxI_PROTECTWIRE){				/* 保護閾値を超えているなら *//* @318swROD[24/11/14] */
				ucHOG_movF |= 0x02;							/* ｺﾝﾌﾟ巻線保護動作ﾌﾗｸﾞ(ucHOG_movF.1)←SET */
			}else{											/* 保護閾値未満なら */
				ucHOG_movF &= 0xfd;							/* ｺﾝﾌﾟ巻線保護ﾌﾗｸﾞ(ucHOG_movF.1)←CLR */
			}
		}
	}else{													/* ｺﾝﾌﾟ巻線保護状態でない */
		if(uc6Stmr_wirehgct >= TIME_PROTECTWIRE){			/* 閾値超えが設定時間以上継続で*/
			ucHOG_movF |= 0x02;								/* ｺﾝﾌﾟ巻線保護動作ﾌﾗｸﾞSET */
		}
	}
}

/**************************************************************************
<Title>			:vHOG_5minwt 		;vHOG_Hogo
<Function>		:保護解除5分待機処理
<Description>	:	ucHOG_movF≡0⇒保護動作開始から5分経過していれば保護状態ﾌﾗｸﾞCLR／else⇒ﾌﾗｸﾞ転写【動作→状態】
				:	ucHOG_movF≠0⇒保護動作ﾌﾗｸﾞを保護状態ﾌﾗｸﾞに転写（ucHOG_sttF=ucHOG_movF）
				:	5分待機後の状態ﾌﾗｸﾞCLR時には通信異常保護ﾌﾗｸﾞは保持したまま
				:	ucHOG_movF	---- ---1 : unused.
				:				---- --1- : ｺﾝﾌﾟﾚｯｻ巻線保護.
				:				---- -1-- : ２レベル異常保護.
				:				---- 1--- : 直流電圧低下保護.
				:				---1 ---- : IPM保護 EMG error.
				:				--1- ---- : ﾛｯｸ保護.
				:				-1-- ---- : 強制転流失敗error.
				:				1--- ---- : 位置検出間隔(60ﾟe) TimeOut error.
				:	ucHOG_sttF	---- ---1 : 通信異常保護.
				:				---- --1- : ｺﾝﾌﾟﾚｯｻ巻線保護.
				:				---- -1-- : ２レベル異常保護.
				:				---- 1--- : 直流電圧低下保護.
				:				---1 ---- : IPM保護 EMG error.
				:				--1- ---- : ﾛｯｸ保護.
:				:				-1-- ---- : 強制転流失敗error. 									※通信データ(ucTRNS_istaやucTRNS_ista2)に反映しない
				:				1--- ---- : 位置検出間隔(60ﾟe) TimeOut error. 	※通信データ(ucTRNS_istaやucTRNS_ista2)に反映しない
<Input>			:ucHOG_movF
<Return>		:ucHOG_sttF
<Author>		:-Outi	作成
<Date>			:2002-0419
<Update>		:Ver REF02A-0.00 2002-05-24
				:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void  vHOG_5minwt(void){						/* ５分起動待ち  */
												/* */
	if ( ucHOG_movF != 0 ){						/*保護中であれば */
		ucHOG_sttF |= ucHOG_movF;				/*保護情報を残す */
	}											/* */

	if( (ucHOG_sttF & MASK_HOGOFLG) != 0){		/*保護状態であれば *//* ｺﾝﾌﾟﾚｯｻ巻線保護フラグも含むように変更 *//* @304swDOD[24/09/02] *//* ﾚﾋﾞｭｰ指摘対応*//* @311syRRD[24/09/04] */
		if(uc6Stmr_hgrlsct >= D_HOGO_CLR_TIME){	/*6s×50 = 5min *//* 保護復帰時間不具合修正*//* @235sgORH[23/01/13] */
			uc6Stmr_hgrlsct = 0;				/*保護解除ｶｳﾝﾀ←ｸﾘｱ */
			ucHOG_movF &= 0xdf;					/*ﾛｯｸ保護動作ﾌﾗｸﾞ←ｸﾘｱ */
			if(ucHOG_movF == 0){				/*ucHOG_movFが0であれば */
				ucHOG_sttF &= ~MASK_HOGOFLG;			/* 保護状態 ｸﾘｱ *//* @311syRRD[24/09/04] */
				uc6Stmr_hgrlsct = 0;			/*保護解除ｶｳﾝﾀ←ｸﾘｱ */
			}else{								/*もう一度繰り返す */
				uc6Stmr_hgrlsct = D_HOGO_CLR_TIME;			/*保護解除ｶｳﾝﾀ←50(5分) */
				ucHOG_movF &= MASK_HOGOFLGCOM;	/*00111110b *//* @311syRRD[24/09/04] */
			}									/* */
		}else{
		}/* */
	}else{										/*保護状態でなければ */	/* 午後 12:10 REF045-4.00 */
		uc6Stmr_hgrlsct = 0;					/*保護解除ｶｳﾝﾀ←ｸﾘｱ */
	}
    
    if(ucHOG_sttF == 0){						/* 保護状態でなければ */					/* @021ywDOD[18/01/25] */
		ucMINtm_HOGOct = 0;						/* ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間ｶｳﾝﾀｸﾘｱ */	/* @021ywDOD[18/01/25] */
	}else{										/* 保護状態ならば */						/* @021ywDOD[18/01/25] */
		/* ｲﾝﾊﾞｰﾀ保護継続ｶｳﾝﾀｸﾘｱしない */													/* @021ywDOD[18/01/25] */
	}/* */	/* 午後 12:09 REF045-4.00 */
}	

/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:竹岡
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
UI08 vSPD_DlineJudge(UI16 rpmcrnt){
	UI16 limit_low;
	UI16 limit_high;

	if(ucTYPE_Dline_point == 1){	/*Dﾗｲﾝ共振ポイント：45～49r/s */ /*＠~07-0422 -Outi機種切り替え対応 */
		limit_low  = D_DLNHGLOLMT_1;
		limit_high = D_DLNHGHILMT_1;	/*Dline回避制御要否判定処理 */	/* @004axOOM[09/05/08] */
	} else {								/*Dﾗｲﾝ共振ポイント：44～47r/s */
		limit_low  = D_DLNHGLOLMT_0;
		limit_high = D_DLNHGHILMT_0;	/*Dline回避制御要否判定処理 */	/* @004axOOM[09/05/08] */
	}
	return ((rpmcrnt >= limit_low)				/* ※DLNHGLOLMT≦実回転数≦DLNHGHILMT[rps]*/					/* @004axOOM[09/05/08] */
		  &&(rpmcrnt <= limit_high));			/* ※DLNHGLOLMT≦実回転数≦DLNHGHILMT[rps]*/					/* @004axOOM[09/05/08] */
}


/**************************************************************************
<Title>			:
<Function>		:
<Description>	:
<Input>			:
<Return>		:
<Author>		:竹岡
<Date>			:
<Update>		:
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void CheckDline(void){
	if(ucTYPE_Dline == 0){						/* Dﾗｲﾝ共振保護制御仕様 無 */
        ucSPD_rpmfgmd &= ~0x40;					/* 共振保護ﾌﾗｸﾞ←ｸﾘｱ         */
        uc10MSt_dlnhgct = 0;					/* 条件※継続時間ｶｳﾝﾀ←ｸﾘｱ   */
        ucMINtm_dlnrlsct = 0;					/* 保護制御継続時間ｶｳﾝﾀ←ｸﾘｱ */
        return;
    }

	/* Dﾗｲﾝ共振保護制御仕様 有 */
	if(!IsSpeedResonance()){		/* 共振保護フラグ非動作中（未動作/解除中） */
		if(vSPD_DlineJudge(hCurMecRps)){				/* ※DLNHGLOLMT≦実回転数≦DLNHGHILMT[rps]*/					/* @004axOOM[09/05/08] */
			if(uc10MSt_dlnhgct >= D_DLNHGTM){			/* 条件※継続時間≧共振保護制御開始判定時間(D_DLNHGTM)なら */	/* @004axOOM[09/05/08] */
				ucSPD_rpmfgmd |= 0x40;					/* 共振保護ﾌﾗｸﾞ←ｾｯﾄ */											/* @004axOOM[09/05/08] */
				uc10MSt_dlnhgct = 0;					/* 条件※継続時間ｶｳﾝﾀ←ｸﾘｱ */									/* @004axOOM[09/05/08] */
			}else{										/* 条件※継続時間不成立なら*/									/* @004axOOM[09/05/08] */
				ucSPD_rpmfgmd &= ~0x40;					/* 共振保護ﾌﾗｸﾞ←ｸﾘｱ(ﾘﾌﾚｯｼｭ) */									/* @004axOOM[09/05/08] */
			}
		}else{											/* 条件※でなければ             */								/* @004axOOM[09/05/08] */
			ucSPD_rpmfgmd &= ~0x40;						/* 共振保護ﾌﾗｸﾞ←ｸﾘｱ(ﾘﾌﾚｯｼｭ) */									/* @004axOOM[09/05/08] */
			/* -------  CT廃止対応  -------	*/
			/*if(ucZX_CTerrorFlgs & 0x04)	*/			/* CT回路異常判断FLG1(異常FLGB)ｾｯﾄ状態<回路異常状態> */		/* @004axOOM[09/05/08] */	/* @008MzDOD[16/12/21] */
			/*{								*/																										/* @008MzDOD[16/12/21] */
			/*	ucSPD_rpmfgmd |= 0x40;		*/			/* 共振保護ﾌﾗｸﾞ←ｾｯﾄ */											/* @004axOOM[09/05/08] */	/* @008MzDOD[16/12/21] */
			/*	ucZX_CTerrorFlgs = 0x03;	*/			/* CT回路異常判断FLG2(異常FLGA)←ｾｯﾄ<成り行き運転> */			/* @004axOOM[09/05/08] */	/* @008MzDOD[16/12/21] */
			/*}else{}						*/			/* CT回路異常判断FLG1(異常FLGB)ｸﾘｱ状態<回路正常状態> */		/* @004axOOM[09/05/08] */	/* @008MzDOD[16/12/21] */
			uc10MSt_dlnhgct = 0;						/* 継続時間ｶｳﾝﾀ←ｸﾘｱ(ﾘﾌﾚｯｼｭ) */									/* @004axOOM[09/05/08] */
		}
		ucMINtm_dlnrlsct = 0;				/* 保護制御継続時間ｶｳﾝﾀ←ｸﾘｱ */
	}else{									/* 保護制御動作中で             */
        if(ucTRNS_sfrq_dec <= SPD_DEFAULT){	/* メインからの指令回転数が、42rps以下であれば保護フラグをクリア */
            ucSPD_rpmfgmd &= ~0x40;			/* 共振保護ﾌﾗｸﾞ←ｸﾘｱ         */
            ucMINtm_dlnrlsct = 0;			/* 保護制御継続時間ｶｳﾝﾀ←ｸﾘｱ */
        }else if(ucMINtm_dlnrlsct >= D_DLNRLSTM){	/* 保護制御時間≧共振保護制御終了時間(D_DLNRLSTM)なら    */
			ucSPD_rpmfgmd &= ~0x40;			/* 共振保護ﾌﾗｸﾞ←ｸﾘｱ         */
			ucMINtm_dlnrlsct = 0;			/* 保護制御継続時間ｶｳﾝﾀ←ｸﾘｱ */
		}else{
			ucSPD_rpmfgmd |= 0x40;			/* 共振保護ﾌﾗｸﾞ←ｾｯﾄ(ﾘﾌﾚｯｼｭ) */
		}
		uc10MSt_dlnhgct = 0;				/* 条件※継続時間ｶｳﾝﾀ←ｸﾘｱ   */
	}
}
