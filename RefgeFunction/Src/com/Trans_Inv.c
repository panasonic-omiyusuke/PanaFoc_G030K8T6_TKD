/**************************************************************************
<FileName>		:Hw_trns.c
<Function>		:通信処理
<Program>		:vTRNS_Hw_trns, vTRNS_Initmd, vTRNS_Sendmd, vTRNS_Rcvemd
<Author>		:甲田 篤志 -AK
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF02A-0.01 2002-04-09 連続送信監視ﾀｲﾏ追加
				:Ver REF02A-1.00 2002-04-18 変数ｸﾞﾛｰﾊﾞﾙ変更
				:Ver REF036-1.00 2002-12-27 -hide 基本ﾀｲﾏ2ms⇒3.3ms変更
				:Ver REF046-0.00 2004-03-02 -Outi 47F=>41Uﾏｲｺﾝ置換
				:Ver REF067-0.00 2006-07-10 -Outi H50マイコン不具合補強
				:Ver REF067-0.01 2006-07-12 -Tomo 東芝推奨H50対策仕様追加
				:Ver REF070-0.01 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-25 -Tomo ソフト設計ＤＲ指摘対応
				:Ver REF070-0.03 2007-05-07 -Outi 機種切り替え対応
				:Ver REF070-0.04 2007-05-07 -Outi P63とﾒｲﾝﾏｲｺﾝ通信仕様不一致時のfailsafe仕様追加
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF081-0.01 2008-06-18 -Outi ﾙﾈｻｽ指摘0617対応@001agORM[08/06/18](発覚08/06/12)
				:Ver REF1000-0.01 2010-05-11 -Mura INVﾏｲｺﾝｺｰﾄﾞ表示対応 @004MxDOD[10/05/11]
				:Ver REF1000-0.02 2010-06-11 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
				:Ver REF1000-0.03 2010-06-16 -Mura 停止時振動対策 メイン通信処理対応  @011MwOTT[10/06/16]
<Copyright>		:Matsushita Refrigerator Co. , Ltd.
 **************************************************************************/
//#pragma SECTION program dtf_program		/* 本命令以降の関数をDataFlash領域へ配置 */	/* @011MzDOD[17/02/08] */

/********************************************
		インクルードファイル
 ********************************************/
#include "vipdc.h"
#include "Trans_Inv.h"
#include "Timea.h"
#include "Dtyctrl.h"

#include "stm32g0xx_hal.h"		/* HALドライバ使用  */
#include "stm32g0xx_hal_uart.h"	/* HALドライバ使用  */
#include "main.h"				/* デバッグポート出力で使用 */
#include "motor_process.h"
#include "brake.h"
/*******************************************
		マクロ定義
 *******************************************/

/*******************************************
		変数宣言
 *******************************************/
/*定数の宣言 */
const unsigned char NAK  = 0xFF;				/*NAK ｺﾏﾝﾄﾞ */
const unsigned char ACK1 = 0xF1;				/*ACK1ｺﾏﾝﾄﾞ */
const unsigned char ACK2 = 0xF2;				/*ACK2ｺﾏﾝﾄﾞ */
const unsigned char ACK4 = 0xF4;				/*ACK4ｺﾏﾝﾄﾞ */
const unsigned char ACK5 = 0xF5;				/*ACK5ｺﾏﾝﾄﾞ */

/*変数の宣言 */
enum TrmdTag eTRNS_trmd;						/*通信ﾓｰﾄﾞ */
enum SemdTag eTRNS_semd;						/*送信ﾓｰﾄﾞ */
enum RcmdTag eTRNS_rcmd;						/*受信ﾓｰﾄﾞ */

/* unsigned char ucTRNS_seba[10];*/				/*送信ﾊﾞｯﾌｧ */			/* @004MxDOD[10/05/11]_STA */
#if TRQ_CTL == ACTIVE
unsigned char ucTRNS_seba[cTDBAFAC];											/* 2010-07-15 竹岡 */
#else /* TRQ_CTL */
unsigned char ucTRNS_seba[11];											/* @004MxDOD[10/05/11]_END */
#endif /* TRQ_CTL */
unsigned char ucTRNS_secn;						/*送信ﾊﾞｲﾄ数ｶｳﾝﾀ */
unsigned char ucTRNS_seen;						/*送信済ﾊﾞｲﾄ数ｶｳﾝﾀ */
unsigned char ucTRNS_sefg;						/*送信ﾌﾗｸﾞ */

/* unsigned char ucTRNS_rcba[30];*/					/*受信ﾊﾞｯﾌｧ */			/* @008MwHOM[10/06/11]_STA */
unsigned char ucTRNS_rcba[cRDBAFAC];					/*受信ﾊﾞｯﾌｧ */			/* @008MwHOM[10/06/11]_END */
unsigned char ucTRNS_rccn;						/*受信ｶｳﾝﾀ */
unsigned char ucTRNS_rcfg;						/*受信ﾌﾗｸﾞ */
unsigned char ucTRNS_rcer;						/*受信ｴﾗｰﾌﾗｸﾞ */

/*extern UI08 ucTRNS_crctduty;	*/				/* @007MwDOD[11/05/10] */		/*@011MwRRD[11/05/17]*/
/*extern UI08 ucTRNS_basepos;	*/				/* @007MwDOD[11/05/10] */		/*@011MwRRD[11/05/17]*/
/* UI08 ucTRNS_crctduty;		*/				 								/*@011MwRRD[11/05/17]*/	/*@012MwRRD[11/05/18]*/
UI16 unTRNS_crctduty;							 														/*@012MwRRD[11/05/18]*/ /*@014MwRRD[11/06/01]*/
UI08 ucTRNS_basepos;							 								/*@011MwRRD[11/05/17]*/
UI08 ucTRNS_wirehgflg;							/* ｺﾝﾌﾟﾚｯｻ巻線保護ﾌﾗｸﾞ *//* @305swDOD[24/09/02] */

unsigned char	ucg_TR_stbyct;			/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀ(ｽﾀﾝﾊﾞｲ受信後のACK1送信ｶｳﾝﾀ) */	/* @086sxDOD[20/04/10] */

unsigned8	ucTRNS_trer;				/*通信異常関連ﾒﾓﾘ */
unsigned8	ucTRNS_ifrq_b;				/*実回転数 At通信系ﾓｼﾞｭｰﾙ[BCD] */
unsigned8	ucTRNS_sfrq_b;				/*指令Hz   At通信系ﾓｼﾞｭｰﾙ[BCD] */
unsigned8	ucTRNS_duty;				/*Duty[%]  At通信系ﾓｼﾞｭｰﾙ[Nrm] */
unsigned8	ucTRNS_pwr;				/*平均電力[W]  At通信系ﾓｼﾞｭｰﾙ[Nrm] */
unsigned8	ucTRNS_prev_large_dec;	/*端子電圧A(上位2桁BCD)送信用 */
unsigned8	ucTRNS_prev_small_dec;	/*端子電圧B(上位2桁BCD)送信用 */
unsigned8	ucTRNS_ista;				/*異常状態ﾃﾞｰﾀ  */
		/*   ---- ---1 ;過電流保護状態	0:正常 1:過電流保護状態 */
		/*   ---- --1- ;２ﾚﾍﾞﾙ保護状態	0:正常 1:２ﾚﾍﾞﾙ保護状態 */
		/*   ---- -1-- ;直電圧低下保護状態	0:正常 1:直流電圧低下保護状態 */
		/*   ---- 1--- ;ロック保護状態	0:正常 1:ロック保護状態 */
		/*   ---1 ---- ;ﾕﾆｯﾄ動作状態	0:ﾕﾆｯﾄ動作中 1:ﾕﾆｯﾄ動作停止中 */
		/*   --1- ---- ;ｺﾏﾝﾄﾞ実行状態	0:ｺﾏﾝﾄﾞ実行中 1:ｺﾏﾝﾄﾞ実行終了 */
		/*   -1-- ---- ;ﾕﾆｯﾄ正/異常状態	0:ﾕﾆｯﾄ正常 1:ﾕﾆｯﾄ異常 */
		/*   1--- ---- ;unused */
unsigned8	ucTRNS_ista2;				/*異常状態ﾃﾞｰﾀ2  *//* @104yxDOD[18/10/03] */
		/*   ---- ---1 ;ｺﾝﾌﾟﾚｯｻ巻線保護状態(下位4bitで構成) *//* @305swDOD[24/09/02] */
		/*   ---- --1- ;ｺﾝﾌﾟﾚｯｻ巻線保護状態(下位4bitで構成) *//* @305swDOD[24/09/02] */
		/*   ---- -1-- ;ｺﾝﾌﾟﾚｯｻ巻線保護状態(下位4bitで構成) *//* @305swDOD[24/09/02] */
		/*   ---- 1--- ;ｺﾝﾌﾟﾚｯｻ巻線保護状態(下位4bitで構成) *//* @305swDOD[24/09/02] */
		/*   ---1 ---- ;unused */
		/*   --1- ---- ;unused */
		/*   -1-- ---- ;unused *//* CT不使用のためCT回路異常判断なし*//* @263swDOD[23/07/26] */
		/*   1--- ---- ;unused */

unsigned8	ucTRNS_sfrg;				/*通信受信ﾃﾞｰﾀ */
		/*   ---- ---1 ;TEST3動作状態	0:通常状態 1:TEST3 */
		/*   ---- --1- ;騒音測定ﾓｰﾄﾞ	0:通常状態 1:騒音測定ﾓｰﾄﾞ中 */
		/*   ---- -1-- ;unused */
		/*   ---- 1--- ;ｺﾝﾌﾟ保護状態	0:通常状態 1:ｺﾝﾌﾟ保護制御中 */
		/*   ---1 ---- ;unused */
		/*   --1- ---- ;停止基準位置検出用起動回転数切替 0：33rps 1：27rps */
		/*   -1-- ---- ;unused */
		/*   1--- ---- ;unused */
unsigned8	ucTRNS_type;				/*通信受信ﾃﾞｰﾀ(機種判別) */
		/*   ---- ---1 ;ｵｰﾊﾞｰﾗｯﾌﾟ及び矩形波ﾍﾞｸﾄﾙ有り?	0：無し 1:有り	*/
		/*   ---- --1- ;位置決め方式及び起動Duty 		0：VWX(2.1)→WX(3.6)相式、起動6.00% 1:VWX(2.8)→WX(4.8)相式、起動8.00%	*/
		/*   ---- -1-- ;ｷｬﾘｱ周波数選択温度上昇対策 		0：10kHz 1:コンプ保護時3kHz	*/
		/*   ---- 1--- ;Dﾗｲﾝ共振保護制御 				0：無し 1:有り	*/
		/*   ---1 ---- ;Dﾗｲﾝ共振ﾎﾟｲﾝﾄ 					0：44～47r/s(H50:42r/s) 1:46～49r/s(H50:44r/s)	*/
		/*   --1- ---- ;直流電圧低下保護仕様 			0：外部割込み仕様 1:AD検知仕様	*/
		/*   -1-- ---- ;最大回転数ﾘﾐｯﾀ 					0：所定 1:最大回転数58r/s	*/
		/*   1--- ---- ;unused 							0： 1:	*/
unsigned8	ucTRNS_stopctrl;	/*転流停止制御用データ*/ /* @008MwHOM[10/06/11] */
		/*   ---- 1111 ;転流停止ｵﾌｾｯﾄ ｎ＝0-17(0～360ﾟ･･･ 20ﾟ/ｎ) 18以上で圧縮機停止時振動抑制制御OFF */
		/*   -111 0000 ;転流停止基準位置連続一致回数 0:3回 1:5回 2:7回 3：9回  */

unsigned8	ucTRNS_lap;					/*ﾗｯﾌﾟ角   At通信系ﾓｼﾞｭｰﾙ[BCD] */
		/*   ---- 1111 ;ｵｰﾊﾞｰﾗｯﾌﾟ量	ｎ＝0-15(0～60ﾟ･･･ 3.75ﾟ/ｎ) */
		/*   ---1 ---- ;矩形波VCT動作	0:VCT非動作 1:VCT動作中 */
		/*   --1- ---- ;unused */
		/*   -1-- ---- ;unused */
		/*   1--- ---- ;unused */

unsigned8	ucg_TRNS_angle;			/* 送信用通電角 */		/* @104yxDOD[18/10/03] */
unsigned8	ucg_TRNS_PWMfreq;		/* 送信用ｷｬﾘｱ周波数 */	/* @104yxDOD[18/10/03] */

unsigned8	ucg_TR_ista;			/* INV状態*/
unsigned8	ucg_TR_ista2;			/* INV状態2*/

#ifndef ONECHIP	/*INV専用ﾏｲｺﾝ*/		/* 国内・海外ｿﾌﾄ共通化のためｺﾝﾊﾟｲﾙｴﾗｰ対応*/	/* @002MxOOM[17/02/21] */
unsigned8	ucg_CMP_sfrq;			/* ｺﾝﾌﾟ回転数*/
#endif
unsigned16	ucg_TR_rpmcrnt;		/* @044MgORM[15/09/08] */

unsigned8	ucTRNS_DEFHrate;  			/* ﾃﾞﾌﾛｽﾄﾋｰﾀ 	通電率 	*/
unsigned8	ucTRNS_PCEVAHrate;  		/* ﾌﾚﾝﾁﾋｰﾀ 		通電率	*/

/*** static ***/
//static UI08 testram;

/*******************************************
		プロトタイプ宣言
 *******************************************/
/*関数のプロトタイプ宣言 */
void vTRNS_Hw_trns(void);						/*ｴﾝﾄﾘﾎﾟｲﾝﾄ */

void vTRNS_Initmd(void);						/*ｲﾆｼｬﾙ処理 */
void vTRNS_Sendmd(void);						/*送信処理 */
void vTRNS_Rcvemd(void);						/*受信処理 */

void vTRNS_Sendmd0(void);						/*NAK設定 */
void vTRNS_Sendmd1(void);						/*ACK1設定 */
void vTRNS_Sendmd2(void);						/*ACK2設定 */
void vTRNS_Sendmd3(void);						/*ﾃﾞｰﾀ設定 */
void vTRNS_Sendmd4(void);						/*ACK4設定 */
void vTRNS_Sendmd5(void);						/*ACK5設定 */
void vTRNS_Sendmd6(void);						/*送信実行 */

void vTRNS_Rcvemd0(void);						/*受信待機 */
void vTRNS_Rcvemd1(void);						/*ｺﾏﾝﾄﾞ受信 */
void vTRNS_Rcvemd2(void);						/*ﾃﾞｰﾀ受信 */
void vTRNS_Rcvemd3(void);						/*受信ｴﾗｰ */

void ucTRNS_clear_recieve_error_flg(void);		/*受信エラーフラグクリア*/
unsigned char ucTRNS_get_recieve_error_flg(void);		/*受信エラー判断 */
unsigned char ucTRNS_Fcc(const unsigned char *pBuf);	/*FCC計算 */
unsigned char ucTRNS_Fcc_in(const unsigned char *pBuf);	/*FCC読込み */
unsigned char ucTRNS_Rcdefin(void);						/*ﾃﾞｰﾀ確定処理 */
void vTRNS_Setsenddat(void);							/*送信ﾃﾞｰﾀ格納 */
void vTRNS_Setrcvedat(void);							/*受信ﾃﾞｰﾀ格納 */

void vTRNS_Main2Inv_dat(void);														/* @038MwDOD[10/12/13]_STA */
void vTRNS_Inv2Main_dat(void);														/* @038MwDOD[10/12/13]_STA */
/**************************************************************************
<Title>			:vTRNS_Hw_trns 		;main
<Function>		:通信処理(ﾒｲﾝﾙｰﾁﾝ)
<Description>	:通信処理のエントリポイント
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Hw_trns(void)
{
	/*通信ﾓｰﾄﾞの種類に応じて処理を分岐する */
	switch (eTRNS_trmd)
	{
	case cInit:
		vTRNS_Initmd();			/*ｲﾆｼｬﾙﾓｰﾄﾞ */
		break;

	case cSend:
		vTRNS_Sendmd();			/*送信ﾓｰﾄﾞ */
		break;

	case cRcve:
		vTRNS_Rcvemd();			/*受信ﾓｰﾄﾞ */
		break;

	default:
		eTRNS_trmd = cInit;		/*通信ﾓｰﾄﾞ→[ｲﾆｼｬﾙﾓｰﾄﾞ] */
	}

	/*送信動作が連続した場合ｲﾆｼｬﾗｲｽﾞ動作を行う */
	if (eTRNS_trmd == cSend)		/*送信ﾓｰﾄﾞ? */
	{								/*02-0416 -AK {}抜け修正 */
		/* 【注意！！】	下行68(680msec)という数字は送信ﾃﾞｰﾀｷｬﾗｸﾀ数に依存｡	 */
		/* 				ｷｬﾗｸﾀ数が変更になる場合は下行の判定時間要注意！！	 */
		if (uc10MSt_sdobsvct > 144)	/*送信ﾓｰﾄﾞ連続171msec×4=680msec以上ﾁｪｯｸ *//* データ数13ｷｬﾗ(5→7→18)増加の差分より、10bit/2400bps*13キャラ＋11.25ms*12≒189msを加算 */
		{							/*02-0410 -AK 340→68修正 */
			eTRNS_trmd = cInit;		/*通信ﾓｰﾄﾞｲﾆｼｬﾗｲｽﾞ(受信待機ﾓｰﾄﾞ) */
			uc10MSt_sdobsvct = 0;	/*連続送信監視ﾀｲﾏｸﾘｱ */
		}
	}
	else							/*送信ﾓｰﾄﾞでないならば連続送信監視ﾀｲﾏｸﾘｱ */
	  uc10MSt_sdobsvct = 0;		/*連続送信監視ﾀｲﾏｸﾘｱ */


	return;
}

/**************************************************************************
<Title>			:vTRNS_Initmd 		;vTRNS_Hw_trns
<Function>		:イニシャルモード処理（待機モード）
<Description>	:通信設定を初期化する
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF081-0.01 2008-06-18 -Outi ﾙﾈｻｽ指摘0617対応@001agORM[08/06/18](発覚08/06/12)
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Initmd()
{
	/*変数の宣言 */
	unsigned char X_lpct;							/*ﾙｰﾌﾟｶｳﾝﾀ */

	for (X_lpct = 0; X_lpct < cTDBAFAC; X_lpct++)	/*送信ﾊﾞｯﾌｧのｸﾘｱ */
	  ucTRNS_seba[X_lpct] = 0x00;

	ucTRNS_secn = 0;								/*送信ﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */
	ucTRNS_seen = 0;								/*送信済みﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */

	for (X_lpct = 0; X_lpct < cRDBAFAC; X_lpct++)	/*受信ﾊﾞｯﾌｧのｸﾘｱ */
	  ucTRNS_rcba[X_lpct] = 0x11;

	ucTRNS_rccn = 0;								/*受信ﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */

	ucTRNS_rcer = 0x00;								/*受信ｴﾗｰﾌﾗｸﾞのｸﾘｱ */


	eTRNS_trmd = cRcve;								/*通信ﾓｰﾄﾞ→2[受信ﾓｰﾄﾞ] */
	eTRNS_rcmd = cRcveExec;							/*受信待機→受信ﾓｰﾄﾞ:0 */

	
	/* ------------------ STM32G0 ------------------> */
#if 1
	
	//huart1.Init.WordLength = UART_WORDLENGTH_9B;		/* 受信・送信ともにWORDLENGTH_9B *//*設定済みのため不要*/
	huart1.Init.BaudRate = 2400;						/* 受信・送信ともにBaudRate=2400bps *//*設定済みのため不要*/
	huart1.Init.StopBits = UART_STOPBITS_1;				/* 受信時は1bit */
	huart1.Instance->CR1 |= USART_CR1_UE;				/* USART 有効 */
	huart1.Instance->CR1 &= ~USART_CR1_TE;				/* Transmitter disable*/
	huart1.Instance->CR1 &= ~USART_CR1_RE;				/* Receiver disable*/
	ucTRNS_clear_recieve_error_flg();					/* recieve error clear */

	//huart1.Init.Parity = UART_PARITY_EVEN;			/*設定済みのため不要*/
	
	//CLEAR_BIT(huart1.Instance->CR1, USART_CR1_FIFOEN);	/*!< FIFO mode enable *//*設定済みのため不要*/
	//huart1.Init.Mode = UART_MODE_TX_RX;				/*設定済みのため不要*/
	//huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;		/*設定済みのため不要*/
	//huart1.Init.OverSampling = UART_OVERSAMPLING_16;	/*設定済みのため不要*/
	//huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;	/*設定済みのため不要*/
	//huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;	/*設定済みのため不要*/
	//huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;	/*設定済みのため不要*/
	huart1.Instance->CR1 |= USART_CR1_RE;				/* Receiver enable*/
	//SET_BIT(huart1.Instance->RQR, UART_RXDATA_FLUSH_REQUEST);		/*RXDATA_FLUSH_REQUEST*//*★試し*/
	
#endif
	/* <------------------ STM32G0 ------------------ */
#if 0	/* ------------------ Renesas ------------------> */
	ucCPU_U1C1 = cRcveUART;							/*ﾃﾞｰﾀ受信設定 *//* 送信バッファ空、受信データあり、受信禁止、送信データ無し、送信禁止 *//* ※受信禁止 設定をすると受信エラーフラグがクリアされる */
	ucCPU_U1MR = cRcvUART2;							/* UART2 transmit/receive mode register */		/* パリティ設定、ストップビット設定、送信データ無し、送信禁止 */
	ucCPU_U1C0 = cRcvUART3;							/* UART2 transmit/receive control register0 */	/* 連続受信モード許可、送信バッファ空、受信データあり、受信禁止、送信データあり、送信許可 */
	ucCPU_U1BRG = 0x33;								/* UART2 bit rate generator */ 					/* ビットレート・分周 (51) 設定 */
/*	ucCPU_PINSR1 = 0x70;		*/					/* Port function select register1(UART1)*/		/* @003MzDOD[16/09/13] */
	ucCPU_U1SR = 0x0A;																				/* 端子割り当て */
	bCPU_RE_U1C1 = 1;								/* reception enabled */							/* 受信許可 ←設定完了後に許可する */
	ucCPU_S1TIC = 0x00;								/* UART2 transmit interrupt control register*/	/* 割込みレベル設定 レベル0 最低レベル  他の割込みが優先 */
	ucCPU_S1RIC = 0x00;								/* UART2 receive interrupt control register	*/	/* 割込みレベル設定 レベル0 最低レベル  他の割込みが優先 */
#endif	/* <------------------ Renesas ------------------ */
	
	return;
}


/**************************************************************************
<Title>			:vTRNS_Sendmd 		;vTRNS_Hw_trns
<Function>		:送信モード処理
<Description>	:送信モードに応じて送信設定処理を行い、送信モードを更新する
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd()
{
	/*送信ﾓｰﾄﾞの種類に応じて処理を分岐する */
	switch (eTRNS_semd)						/*送信ﾓｰﾄﾞ判定 */
	{
	case cSendNak:
		vTRNS_Sendmd0();				/*NAKの送信設定 */
		eTRNS_semd = cSendWait;			/*送信処理→送信ﾓｰﾄﾞ:6 */
		break;

	case cSendAck1:
		vTRNS_Sendmd1();				/*ACK1の送信設定 */
		eTRNS_semd = cSendWait;			/*送信処理→送信ﾓｰﾄﾞ:6 */
		break;

	case cSendAck2:
		vTRNS_Sendmd2();				/*ACK2の送信設定 */
		eTRNS_semd = cSendWait;			/*送信処理→送信ﾓｰﾄﾞ:6 */
		break;

	case cSendData:
		vTRNS_Sendmd3();				/*送信ﾃﾞｰﾀの送信設定 */
		eTRNS_semd = cSendExec;			/*送信処理→送信ﾓｰﾄﾞ:7 */
		break;

	case cSendAck4:
		vTRNS_Sendmd4();				/*ACK4の送信設定 */
		eTRNS_semd = cSendWait;			/*送信処理→送信ﾓｰﾄﾞ:6 */
		break;

	case cSendAck5:
		vTRNS_Sendmd5();				/*ACK5の送信設定 */
		eTRNS_semd = cSendWait;			/*送信処理→送信ﾓｰﾄﾞ:6 */
		break;

	case cSendWait:
		if (ucTC1tm_tmc > 10)			/*tc1=2msの為ｶｳﾝﾄｱｯﾌﾟ値変更 *//* HY2(3.3ms)では6→HTA(2.0ms)では10 2026.07.30*//* 2.0ms*10=20ms ※実際は不等号のため2.0ms*(10+1)=22ms */
		  eTRNS_semd = cSendExec;		/*送信処理→送信ﾓｰﾄﾞ:7 */
		break;

	case cSendExec:
		if (ucTC1tm_tmc > 5)			/*TC1=2msの為ｶｳﾝﾄｱｯﾌﾟ値変更 *//* HY2(3.3ms)では3→HTA(2.0ms)では5 2026.07.30*//* 2.0ms*5=10ms ※実際は不等号のため2.0ms*(5+1)=12ms */
		{
			ucTC1tm_tmc = 0;			/*通信ﾀｲﾏのｸﾘｱ */
			vTRNS_Sendmd6();			/*送信実行 */
		}
		break;

	default:
		eTRNS_trmd = cInit;				/*通信ﾓｰﾄﾞ→0[ｲﾆｼｬﾙﾓｰﾄﾞ] */
		break;
	}

	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd0
<Function>		:ＮＡＫの送信設定［送信モード：０］
<Description>	:送信バッファにＮＡＫをセットし、ＮＡＫ送信フラグを設定する
				:    送信フラグ構成
				:            7654 3210bit
				:     X_Flag 00** ****
				:              || |||+- ACK1送信設定
				:              || ||+-- ACK2送信設定
				:              || |+--- ﾃﾞｰﾀ送信設定
				:              || +---- ACK4送信設定
				:              |+------ ACK5送信設定
				:              +------- NAK送信設定
				:Caution>	現仕様ではＮＡＫは使用しない
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd0()
{
	ucTRNS_seba[0] = NAK;	/*ｺﾏﾝﾄﾞの設定[NAK:FF]→送信ﾊﾞｯﾌｧへ */

	ucTRNS_secn = 1;		/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:1] */

	ucTRNS_sefg = 0x20;		/*送信ﾌﾗｸﾞの設定:20H[NAK送信] */


	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd1
<Function>		:ＡＣＫ１の送信設定［送信モード：１］
<Description>	:送信バッファにＡＣＫ１をセットし、ＡＣＫ１送信フラグを設定する
				:    送信フラグ構成
				:            7654 3210bit
				:     X_Flag 00** ****
				;              || |||+- ACK1送信設定
				:              || ||+-- ACK2送信設定
				:              || |+--- ﾃﾞｰﾀ送信設定
				:              || +---- ACK4送信設定
				:              |+------ ACK5送信設定
				:              +------- NAK送信設定
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd1()
{
	ucTRNS_seba[0] = ACK1;	/*ｺﾏﾝﾄﾞの設定[ACK1:F1]→送信ﾊﾞｯﾌｧへ */

	ucTRNS_secn = 1;		/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:1] */

	ucTRNS_sefg = 0x01;		/*送信ﾌﾗｸﾞの設定:01H[ACK1送信] */


	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd2
<Function>		:ＡＣＫ２の送信設定［送信モード：２］
<Description>	:送信バッファにＡＣＫ２をセットし、ＡＣＫ２送信フラグを設定する
				:    送信フラグ構成
				:            7654 3210bit
				:     X_Flag 00** ****
				:              || |||+- ACK1送信設定
				:              || ||+-- ACK2送信設定
				:              || |+--- ﾃﾞｰﾀ送信設定
				:              || +---- ACK4送信設定
				:              |+------ ACK5送信設定
				:              +------- NAK送信設定
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd2()
{
	ucTRNS_seba[0] = ACK2;	/*ｺﾏﾝﾄﾞの設定[ACK2:F2]→送信ﾊﾞｯﾌｧへ */

	ucTRNS_secn = 1;		/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:1] */

	ucTRNS_sefg = 0x02;		/*送信ﾌﾗｸﾞの設定:02H[ACK2送信] */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd3
<Function>		:状態データの送信設定［送信モード：３］
<Description>	:送信バッファに送信データをセットし、データ送信フラグを設定する
				:    送信フラグ構成
				:            7654 3210bit
				:     X_Flag 00** ****
				:              || |||+- ACK1送信設定
				:              || ||+-- ACK2送信設定
				:              || |+--- ﾃﾞｰﾀ送信設定
				:              || +---- ACK4送信設定
				:              |+------ ACK5送信設定
				:              +------- NAK送信設定
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF070-0.01 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-25 -Tomo ソフト設計ＤＲ指摘対応
				:Ver REF070-0.03 2007-05-07 -Outi 機種切り替え対応
				:Ver REF1000-0.01 2010-05-11 -Mura INVﾏｲｺﾝｺｰﾄﾞ表示対応 @004MxDOD[10/05/11]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd3()
{
	/*変数の宣言 */
	unsigned char X_fcc;					/*ﾌﾚｰﾑﾁｪｯｸｺｰﾄﾞ */


	/*送信ﾊﾞｯﾌｧにﾃﾞｰﾀを格納する	 */
		/*		ucTRNS_seba[0] = 0x07;	*/			/*ﾊﾞｲﾄｶｳﾝﾀの設定 */			/* @004MxDOD[10/05/11]_STA */
#if TRQ_CTL == ACTIVE
		ucTRNS_seba[0] = cTDBAFAC - 3;
#else /* TRQ_CTL */
		ucTRNS_seba[0] = 0x08;				/*ﾊﾞｲﾄｶｳﾝﾀの設定 */			/* @004MxDOD[10/05/11]_END */
#endif /* TRQ_CTL */
	ucTRNS_seba[1] = 0x02;					/*ﾃﾞｰﾀｺﾏﾝﾄﾞの設定 */

	vTRNS_Setsenddat();						/*送信ﾃﾞｰﾀの格納 */

	X_fcc = ucTRNS_Fcc(ucTRNS_seba);		/*FCCの計算 */

		/*		ucTRNS_seba[8] = ((X_fcc >> 4) & 0x0F);	*/	/*FCC上位4ﾋﾞｯﾄ格納 02-0403 -AK */			/* @004MxDOD[10/05/11]_STA */
		/*		ucTRNS_seba[9] = (X_fcc & 0x0F);		*/	/*FCC下位4ﾋﾞｯﾄ格納 */
		/*		ucTRNS_secn = 10;						*/	/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:10] */
#if TRQ_CTL == ACTIVE
		ucTRNS_seba[cTDBAFAC-2] = ((X_fcc >> 4) & 0x0F);
		ucTRNS_seba[cTDBAFAC-1] = (X_fcc & 0x0F);
		ucTRNS_secn = cTDBAFAC;
#else /* TRQ_CTL */
		ucTRNS_seba[9] = ((X_fcc >> 4) & 0x0F);	/*FCC上位4ﾋﾞｯﾄ格納*/
		ucTRNS_seba[10] = (X_fcc & 0x0F);		/*FCC下位4ﾋﾞｯﾄ格納 */
		ucTRNS_secn = 11;						/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:11] */				/* @004MxDOD[10/05/11]_END */
#endif /* TRQ_CTL */

	ucTRNS_sefg = 0x04;						/*送信ﾌﾗｸﾞの設定:04H[運転状態ﾃﾞｰﾀ送信］ */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd4
<Function>		:ＡＣＫ４の送信設定［送信モード：４］
<Description>	:送信バッファにＡＣＫ４をセットし、ＡＣＫ４送信フラグを設定する
				:    送信フラグ構成
				:            7654 3210bit
				:     X_Flag 00** ****
				:              || |||+- ACK1送信設定
				:              || ||+-- ACK2送信設定
				:              || |+--- ﾃﾞｰﾀ送信設定
				:              || +---- ACK4送信設定
				:              |+------ ACK5送信設定
				:              +------- NAK送信設定
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd4()
{
	ucTRNS_seba[0] = ACK4;	/*ｺﾏﾝﾄﾞの設定[ACK4:F4]→送信ﾊﾞｯﾌｧへ */

	ucTRNS_secn = 1;		/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:1] */

	ucTRNS_sefg = 0x08;		/*送信ﾌﾗｸﾞの設定:08H[ACK4送信] */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd5
<Function>		:ＡＣＫ５の送信設定［送信モード：５］
<Description>	:送信バッファにＡＣＫ５をセットし、ＡＣＫ５送信フラグを設定する
				:    送信フラグ構成
				:            7654 3210bit
				:     X_Flag 00** ****
				:              || |||+- ACK1送信設定
				:              || ||+-- ACK2送信設定
				:              || |+--- ﾃﾞｰﾀ送信設定
				:              || +---- ACK4送信設定
				:              |+------ ACK5送信設定
				:              +------- NAK送信設定
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd5()
{
	ucTRNS_seba[0] = ACK5;	/*ｺﾏﾝﾄﾞの設定[ACK4:F4]→送信ﾊﾞｯﾌｧへ */

	ucTRNS_secn = 1;		/*送信ﾊﾞｲﾄ数の設定[ﾊﾞｲﾄ数:1] */

	ucTRNS_sefg = 0x10;		/*送信ﾌﾗｸﾞの設定:08H[ACK4送信] */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Sendmd6
<Function>		:送信実行
<Description>	:・送信データが存在する場合は、送信バッファの内容を送信する。
				: ・送信完了した場合は、コマンド受信待ち状態へ移行する。
				:   但し、ＡＣＫ２送信後は状態データの送信設定を行う。
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Sendmd6()
{
	/*変数の宣言 */
	unsigned char X_lpct;								/*ﾙｰﾌﾟｶｳﾝﾀ */

	if (ucTRNS_seen < ucTRNS_secn)						/*送信中? */
	{
		
#if 0	/* renesas *//* 送信許可設定  */
		ucCPU_U1MR = cSendUART;							/* UART2 transmit/receive mode register */	/* @003MzDOD[16/09/13] */
		bCPU_TE_U1C1 = 1;								/* transmission enabled */					/* @003MzDOD[16/09/13] */
		bCPU_RE_U1C1 = 0;								/* reception disabled */					/* @003MzDOD[16/09/13] */
		bCPU_STPS_U1MR = 1;								/* 2 stop bits */							/* 送受信の許可/不許可設定後にストップビット長さ変更 */
#endif	/* renesas */

		//if (ucCPU_UARTSR & cTBEP)						/*ﾊﾞｯﾌｧｴﾝﾌﾟﾃｨ? */
		/* ------------------ STM32G0 ------------------> */
		if ( READ_BIT(huart1.Instance->ISR, USART_ISR_TXE_TXFNF) != 0U )	/*送信ﾊﾞｯﾌｧｴﾝﾌﾟﾃｨ? *//* STM32 */
		{
			//huart1.Init.WordLength = UART_WORDLENGTH_9B;		/* 受信・送信ともにWORDLENGTH_9B *//*設定済みのため不要*/
			huart1.Init.BaudRate = 2400;						/* 受信・送信ともにBaudRate=2400bps */
			huart1.Init.StopBits = UART_STOPBITS_2;				/* 送信時は2bit */
			huart1.Instance->CR1 |= USART_CR1_UE;				/*UART enable*/
			huart1.Instance->CR1 &= ~USART_CR1_RE;				/*Receiver disable*/
			huart1.Instance->CR1 |= USART_CR1_TE;				/*Transmitter Enable*/
			huart1.Instance->TDR = ucTRNS_seba[ucTRNS_seen++];	/*Transmitter data set*/

			ucTC1tm_tmc = 0;							/*通信ﾀｲﾏｶｳﾝﾀ 初期化 */
		}
		/* <------------------ STM32G0 ------------------ */
	}
	else												/*送信完了 */
	{
		for (X_lpct = 0; X_lpct < cTDBAFAC; X_lpct++)	/*送信ﾊﾞｯﾌｧのｸﾘｱ */
		  ucTRNS_seba[X_lpct] = 0x00;

		ucTRNS_secn = 0;								/*送信ﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */
		ucTRNS_seen = 0;								/*送信済みﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */

		//if(bCPU_TXEPT_U1C0==1)							/* no data in transmission(transmission completed) */	/* @003MzDOD[16/09/13] */
		if( (huart1.Instance->ISR & USART_ISR_TC) != 0U )	/* transmission completed *//* STM32 */
		{
			if (ucTRNS_sefg == 0x02)					/*ACK2の送信後? */
			{
				eTRNS_trmd = cSend;						/*通信ﾓｰﾄﾞを送信ﾓｰﾄﾞに設定 */
				eTRNS_semd = cSendData;					/*送信ﾓｰﾄﾞをﾃﾞｰﾀ送信に設定 */
			}
			else										/*ACK2以外の送信後? */
			{
				if( (ucTRNS_sefg == 0x01) && (ucTRNS_rcfg == 0x01) ){	/*ACK1送信後かつｽﾀﾝﾊﾞｲ受信後? */	/* @086sxDOD[20/04/10] */
					if( ucg_TR_stbyct < UCHAR_MAX ){				/*最大値ﾁｪｯｸ	 */						/* @086sxDOD[20/04/10] */
						ucg_TR_stbyct ++;							/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀをｲﾝｸﾘ */			/* @086sxDOD[20/04/10] */
					}else{											
						ucg_TR_stbyct = UCHAR_MAX;					/*最大値で固定*/						/* @086sxDOD[20/04/10] */
					}
				}else{												/*[ｽﾀﾝﾊﾞｲ受信後かつACK1送信後]以外? */	/* @086sxDOD[20/04/10] */
					ucg_TR_stbyct = 0;								/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀをｸﾘｱ  */			/* @086sxDOD[20/04/10] */
					ucMINtm_stbyct = 0;								/*ｽﾀﾝﾊﾞｲ連続異常計測ﾀｲﾏｶｳﾝﾀをｸﾘｱ */		/* @088swRRD[20/04/22] */
				}
				eTRNS_trmd = cRcve;						/*通信ﾓｰﾄﾞを受信ﾓｰﾄﾞに設定 */
				eTRNS_rcmd = cRcveExec;					/*受信ﾓｰﾄﾞを受信待機に設定 */
		/* ------------------ STM32G0 ------------------> */
				huart1.Instance->CR1 &= ~USART_CR1_TE;			/*Transmitter disable*/
				huart1.Instance->CR1 |= USART_CR1_RE;			/*Receiver Enable*/
		/* <------------------ STM32G0 ------------------ */

#if 0	/* ------------------ Renesas ------------------> *//* 受信許可設定  */
				bCPU_TE_U1C1	=	0;					/* transmission disabled */	/* @003MzDOD[16/09/13] */
				bCPU_RE_U1C1	=	1;					/* reception enabled */		/* @003MzDOD[16/09/13] */
				bCPU_STPS_U1MR	=	0;					/* 1 stop bit */			/* @003MzDOD[16/09/13] */
#endif	/* <------------------ Renesas ------------------ */

			}
		}
	}

	return;
}

/**************************************************************************
<Title>			:vTRNS_Rcvemd 		;vTRNS_Hw_trns
<Function>		:受信モード処理
<Description>	:受信モードに応じて受信処理の分岐を行う。
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Rcvemd()
{

	/*受信ﾓｰﾄﾞの種類に応じて処理を分岐する */
	switch (eTRNS_rcmd)				/*受信ﾓｰﾄﾞ */
	{
	case cRcveExec:
		vTRNS_Rcvemd0();		/*受信中又は待機中 */
		break;

	case cRcveCmd:
		vTRNS_Rcvemd1();		/*ｺﾏﾝﾄﾞの受信 */
		break;

	case cRcveData:
		vTRNS_Rcvemd2();		/*制御ﾃﾞｰﾀの受信 */
		break;

	case cRcveErr:
		vTRNS_Rcvemd3();		/*受信ｴﾗｰ処理 */
		break;

	default:
		eTRNS_trmd = cInit;		/*通信ﾓｰﾄﾞ→0[ｲﾆｼｬﾙﾓｰﾄﾞ］ */
	}

	return;
}

/**************************************************************************
<Title>			:vTRNS_Rcvemd0
<Function>		:受信中［受信モード：０］
<Description>	:受信データの受信バッファへの格納とコマンド／データの種別の
				: 判定を行い、受信モードを設定する．
				: 受信タイマを監視し、受信データ待ち状態で待機する
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF067-0.00 2006-07-10 -Outi H50マイコン不具合補強
				:Ver REF067-0.01 2006-07-12 -Tomo 東芝推奨H50対策仕様追加
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Rcvemd0()
{
	UI16	unBuf;											/*16bitﾃﾝﾎﾟﾗﾘ */
	
	ucTRNS_trer = 0x01;										/*通信ｴﾗｰﾌﾗｸﾞのｾｯﾄ */	/* @002MxOOM[17/02/21] *//* @227sgOTT[22/08/29] */

	/* ------------------ STM32G0 ------------------> */
	if ( (huart1.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0U )	/*受信ﾊﾞｯﾌｧﾌﾙ? */
	{
		//huart1.Init.WordLength = UART_WORDLENGTH_9B;		/* 受信・送信ともにWORDLENGTH_9B *//*設定済みのため不要*/
		huart1.Init.BaudRate = 2400;						/* 受信・送信ともにBaudRate=2400bps */
		huart1.Init.StopBits = UART_STOPBITS_1;			/* 受信時は1bit */
		huart1.Instance->CR1 |= USART_CR1_UE;			/*UART enable*/
		huart1.Instance->CR1 &= ~USART_CR1_TE;			/*Transmitter disable*/
		huart1.Instance->CR1 |= USART_CR1_RE;			/*Receiver enable*/
		//unBuf = huart1.pRxBuffPtr;							/*受信ﾊﾞｯﾌｧdata読込 */
		unBuf = huart1.Instance->RDR;					/*受信ﾊﾞｯﾌｧdata読込 *//* データのみ */
		
		if (ucTC1tm_tmc > 20)								/*TC1=2msのためｶｳﾝﾄｱｯﾌﾟ値変更 2ms×20＝40ms *//* HY2(3.3ms)では12→HTA(2.0ms)では20 2026.07.30*/
		  ucTRNS_rccn = 0;								/*受信ﾊﾞｲﾄ数ｸﾘｱ */

		ucTC1tm_tmc = 0;									/*通信ﾀｲﾏｶｳﾝﾀ 初期化 */

		ucTRNS_rcer = ucTRNS_get_recieve_error_flg();		/*受信ｴﾗｰﾌﾗｸﾞの設定 */
		if (ucTRNS_rcer){									/*受信ｴﾗｰﾌﾗｸﾞの設定 */
		  eTRNS_rcmd = cRcveErr;							/*受信ｴﾗｰ→受信ﾓｰﾄﾞ:3 */
		}
		
		if ((UI08)(unBuf & 0x00ff) >= 0xF0){				/* 一線式通信回路への対応(これがないと操作のACKを拾ってしまいエラーになる) */
			return;
		}	
		ucTRNS_rcba[ucTRNS_rccn] = (UI08)(unBuf & 0x00ff);	/*ﾃﾞｰﾀ受信 */
		
	/* <------------------ STM32G0 ------------------ */
		if (ucTRNS_rccn == 0)								/*受信ﾀｲﾐﾝｸﾞがずれた場合ｺﾏﾝﾄﾞ/ﾃﾞｰﾀ誤認識するためﾃﾞｰﾀ判定処理 */
		{
			if (ucTRNS_rcba[ucTRNS_rccn] < 0xA0)			/*ﾃﾞｰﾀがやってきた 02-0410 -AK 0x0A→0xA0修正 */
			{
				if (((ucTRNS_rcfg & 0x0F) != 0x03) && ((ucTRNS_rcfg & 0x0F) != 0x09))	/*出力指令でないならば */
				{
					eTRNS_rcmd = cRcveErr;					/*受信ｴﾗｰ */
				}
			}
		}

		if (eTRNS_rcmd == cRcveExec)						/*受信ﾓｰﾄﾞ:0? [受信中又は待機中ﾓｰﾄﾞ] */
		{
			if (ucTRNS_rcba[ucTRNS_rccn] > 0xA3)			/*ｺﾏﾝﾄﾞ異常 02-0424 -AK */
			{
				ucTRNS_rccn = 0;							/*受信ﾊﾞｲﾄ数ｸﾘｱ */
			}

			ucTRNS_rccn++;									/*02-0424 -AK 受信ｶｳﾝﾀ ｲﾝｸﾘﾒﾝﾄ位置変更 */

			if (ucTRNS_rccn >= 2)							/*2ﾊﾞｲﾄ以上受信? */
			{
				if (ucTRNS_rcba[0] >= 0xA0)					/*ｺﾏﾝﾄﾞの受信 (先頭ﾃﾞｰﾀ >= A0H) */
				{
					eTRNS_rcmd = cRcveCmd;					/*受信完了→受信ﾓｰﾄﾞ:1 */
					ucTRNS_trer = 0x00;						/*通信ｴﾗｰﾌﾗｸﾞのｸﾘｱ */				/* @002MxOOM[17/02/21] */
				}
				else										/*ﾃﾞｰﾀの受信 (先頭ﾃﾞｰﾀ <= AFH) */
				{
					if ((ucTRNS_rcba[0] + 2) >= ucTRNS_rccn)/*BCCﾃﾞｰﾀ+2 > 受信ﾊﾞｲﾄ数･ｶｳﾝﾀ? */
					{
						eTRNS_rcmd = cRcveExec;				/*受信中→受信ﾓｰﾄﾞ:0 */
					}
					else									/*ﾃﾞｰﾀ受信完了 */
					{
						eTRNS_rcmd = cRcveData;				/*受信完了→受信ﾓｰﾄﾞ:2 */
						ucTRNS_trer = 0x00;					/*通信ｴﾗｰﾌﾗｸﾞのｸﾘｱ */			/* @002MxOOM[17/02/21] */
					}
				}
			}

			if (ucTRNS_rccn > cRDBAFAC)						/*受信ﾊﾞｯﾌｧのｵｰﾊﾞﾌﾛｰ?(受信ﾊﾞｲﾄ数･ｶｳﾝﾀ > 30) */
			  eTRNS_rcmd = cRcveErr;						/*ｵｰﾊﾞﾌﾛｰ→受信ﾓｰﾄﾞ:3 */
		}
	}
	else{													/*受信ﾊﾞｯﾌｧﾌﾙでないなら */
/* エラークリア処理場所変更		ucTRNS_clear_recieve_error_flg();	*/				/* recieve error clear */
	}/*＃ 午前 10:59 REF067-0.00 */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Rcvemd1
<Function>		:コマンドの受信［受信モード：１］
<Description>	:受信コマンドに応じてＡＣＫ／ＮＡＫの送信設定を行う
				:	受信フラグ構成
				:                7654 3210bit
				:         X_Flag **0* ****
				:                || | |||+- ｽﾀﾝﾊﾞｲ受信(初回受信後は常に1)
				:                || | ||+-- 出力指令
				:                || | |+--- 入力要求
				:                || | +---- 出力再指令
				:                || +------ ﾃﾞｰﾀ正常受信済
				:                |+-------- ｺﾏﾝﾄﾞｴﾗｰ
				:                +--------- 未ｽﾀﾝﾊﾞｲ時ｺﾏﾝﾄﾞ受信
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Rcvemd1()
{
	/*変数の宣言 */
	unsigned char X_lpct;								/*ﾙｰﾌﾟｶｳﾝﾀ */

	//if (ucTRNS_rcba[0] == (ucTRNS_rcba[1]&0xFF))		/*ｺﾏﾝﾄﾞ構成の確認 */
	if (ucTRNS_rcba[0] == (ucTRNS_rcba[1]^0xFF))		/*ｺﾏﾝﾄﾞ構成の確認 */
	{
		/*ｺﾏﾝﾄﾞの種類に応じて受信ﾌﾗｸﾞを設定する */
		switch (ucTRNS_rcba[0])							/*ｺﾏﾝﾄﾞ種別の判定 */
		{
		case 0xA0:									/*ｽﾀﾝﾊﾞｲｺﾏﾝﾄﾞ */
			ucTRNS_rcfg = 0x01;						/*ｽﾀﾝﾊﾞｲ受信[受信ﾌﾗｸﾞ:01h] */
			break;

		case 0xA1:									/*出力指令ｺﾏﾝﾄﾞ */
			if (ucTRNS_rcfg &= 0x01)				/*ｽﾀﾝﾊﾞｲ受信済? */
			  ucTRNS_rcfg |= 0x02;				/*出力指令受信[受信ﾌﾗｸﾞ:03h] */
			else{
				ucTRNS_rcfg = 0x80;					/*ACK4返信[受信ﾌﾗｸﾞ:80h]  */
			}
			break;

		case 0xA2:									/*入力要求ｺﾏﾝﾄﾞ */
			if (ucTRNS_rcfg &= 0x01)				/*ｽﾀﾝﾊﾞｲ受信済? */
			  ucTRNS_rcfg |= 0x04;				/*入力要求受信[受信ﾌﾗｸﾞ:05h] */
			else{
				ucTRNS_rcfg = 0x80;					/*ACK4返信[受信ﾌﾗｸﾞ:80h] */
			}
			break;

		case 0xA3:									/*出力再指令ｺﾏﾝﾄﾞ */
			if (ucTRNS_rcfg & 0x01)					/*ｽﾀﾝﾊﾞｲ受信済? */
			{
				ucTRNS_rcfg &= 0x11;				/*出力再指令受信 */
				ucTRNS_rcfg |= 0x08;				/*[受信ﾌﾗｸﾞ:09h(ﾃﾞｰﾀ未)/19h(ﾃﾞｰﾀ済)] */
			}
			else{									/*ｽﾀﾝﾊﾞｲ未受信? */
				ucTRNS_rcfg = 0x80;					/*ACK4返信[受信ﾌﾗｸﾞ:80h] */
			}
			break;

		default:									/*他局ｺﾏﾝﾄﾞ */
			ucTRNS_rcfg &= 0x01;
			ucTRNS_rcfg |= 0x40;					/*[受信ﾌﾗｸﾞ:40h(ｽﾀﾝﾊﾞｲ未)/41h(ｽﾀﾝﾊﾞｲ済)] */
		}
	}
	else												/*ｺﾏﾝﾄﾞ反転二連送ｴﾗｰ */
	{
		ucTRNS_rcfg &= 0x01;
		ucTRNS_rcfg |= 0x40;							/*[受信ﾌﾗｸﾞ:40h(ｽﾀﾝﾊﾞｲ未)/41h(ｽﾀﾝﾊﾞｲ済)] */
	}

	/*受信ﾌﾗｸﾞの種類に応じてﾓｰﾄﾞの設定を行う */
	switch (ucTRNS_rcfg)
	{
	case 0x01:										/*ｽﾀﾝﾊﾞｲ受信 */
		eTRNS_trmd = cSend;							/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
		eTRNS_semd = cSendAck1;						/*ACK1の送信→送信ﾓｰﾄﾞ:1 */
		break;

	case 0x03:										/*出力指令受信[ｽﾀﾝﾊﾞｲ OK] */
		eTRNS_trmd = cSend;							/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
		eTRNS_semd = cSendAck1;						/*ACK1の送信→送信ﾓｰﾄﾞ:1 */
		break;

	case 0x05:										/*入力要求受信[ｽﾀﾝﾊﾞｲ OK] */
		eTRNS_trmd = cSend;							/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
		eTRNS_semd = cSendAck2;						/*ACK2の送信→送信ﾓｰﾄﾞ:2 */
		break;

	case 0x19:										/*出力再指令受信[ｽﾀﾝﾊﾞｲ OK] */
		/*ﾃﾞｰﾀ正規受信 */
		eTRNS_trmd = cSend;							/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
		eTRNS_semd = cSendAck5;						/*ACK5の送信→送信ﾓｰﾄﾞ:5 */
		break;

	case 0x09:										/*出力再指令受信[ｽﾀﾝﾊﾞｲ OK] */
		/*ﾃﾞｰﾀ未受信 */
		eTRNS_trmd = cSend;							/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
		eTRNS_semd = cSendAck1;						/*ACK1の送信→送信ﾓｰﾄﾞ:1 */
		break;

	case 0x80:										/*ｽﾀﾝﾊﾞｲ未受信時にｺﾏﾝﾄﾞ受信 */
		eTRNS_trmd = cSend;							/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
		eTRNS_semd = cSendAck4;						/*ACK4の送信→送信ﾓｰﾄﾞ:4 */
		break;

	default:
		eTRNS_trmd = cInit;							/*通信ﾓｰﾄﾞ→0[ｲﾆｼｬﾙﾓｰﾄﾞ] */
	}

	for (X_lpct = 0; X_lpct < cRDBAFAC; X_lpct++)		/*受信ﾊﾞｯﾌｧのｸﾘｱ */
		ucTRNS_rcba[X_lpct] = 0x00;

	ucTRNS_rccn = 0;									/*受信ﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Rcvemd2
<Function>		:データの受信［受信モード：２］
<Description>	: ・受信データ内容の正当性をチェックし、データ領域に格納して
				:   ＡＣＫ１の送信設定を行う。
				: ・不正データ受信の場合は、コマンド受信待ち状態へ移行する。
				:   但し、スタンバイコマンド未受信の場合はＡＣＫ４の送信設定
				:   を行う。
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Rcvemd2()
{
	/*変数の宣言 */
	unsigned char X_lpct;							/*ﾙｰﾌﾟｶｳﾝﾀ */

	if (ucTRNS_rcba[1] == 0x00)							/*ﾃﾞｰﾀｺﾏﾝﾄﾞのﾁｪｯｸ[=00H?] */
	{
		if ((ucTRNS_rcfg & 0x08) || (ucTRNS_rcfg & 0x02))		/*出力指令受信後又は出力再指令受信後？ */
		{
			if (ucTRNS_Fcc(ucTRNS_rcba) == ucTRNS_Fcc_in(ucTRNS_rcba))/*FCCがOK→ｲﾝﾊﾞｰﾀ制御ﾃﾞｰﾀの設定 */
			{
				if (ucTRNS_Rcdefin() == 0x00)			/*ｲﾝﾊﾞｰﾀ制御ﾃﾞｰﾀがOK? */
				{
					vTRNS_Setrcvedat();					/*受信ﾃﾞｰﾀの格納 */

					ucTRNS_rcfg |= 0x10;				/*ﾃﾞｰﾀ正常受信→#13H,#19H */

					eTRNS_trmd = cSend;					/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
					eTRNS_semd = cSendAck1;				/*ACK1の送信→送信ﾓｰﾄﾞ:1 */
				}
				else									/*ﾃﾞｰﾀが論理的に異常→ｺﾏﾝﾄﾞ待ち */
				{
					eTRNS_trmd = cRcve;					/*通信ﾓｰﾄﾞ→2[受信ﾓｰﾄﾞ] */
					eTRNS_rcmd = cRcveExec;				/*受信待機→受信ﾓｰﾄﾞ:0 */
				}
			}
			else										/*FCCがNG→ｺﾏﾝﾄﾞ待ち */
			{
				eTRNS_trmd = cRcve;						/*通信ﾓｰﾄﾞ→2[受信ﾓｰﾄﾞ] */
				eTRNS_rcmd = cRcveExec;					/*受信待機→受信ﾓｰﾄﾞ:0 */
			}
		}
		else											/*出力指令後でも出力再指令後でもない */
		{
			if (ucTRNS_rcfg & 0x01)						/*ｽﾀﾝﾊﾞｲ OK? */
			{
				eTRNS_trmd = cRcve;						/*通信ﾓｰﾄﾞ→2[受信ﾓｰﾄﾞ] */
				eTRNS_rcmd = cRcveExec;					/*受信待機→受信ﾓｰﾄﾞ:0 */
			}
			else										/*ｽﾀﾝﾊﾞｲ未処理 */
			{
				eTRNS_trmd = cSend;						/*通信ﾓｰﾄﾞ→1[送信ﾓｰﾄﾞ] */
				eTRNS_semd = cSendAck4;					/*ACK4の送信→送信ﾓｰﾄﾞ:4 */
			}
		}
	}
	else
	{
		eTRNS_trmd = cRcve;								/*通信ﾓｰﾄﾞ→2[受信ﾓｰﾄﾞ] 02-0403 -AK */
		eTRNS_rcmd = cRcveExec;							/*受信待機→受信ﾓｰﾄﾞ:0 */
	}

	for (X_lpct = 0; X_lpct < cRDBAFAC; X_lpct++)		/*受信ﾊﾞｯﾌｧのｸﾘｱ */
	  ucTRNS_rcba[X_lpct] = 0x00;

	ucTRNS_rccn = 0;									/*受信ﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */

	return;
}

/**************************************************************************
<Title>			:vTRNS_Rcvemd3
<Function>		:受信エラー処理［受信モード：３］
<Description>	:通信モード設定をイニシャルする
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Rcvemd3()
{
	ucTRNS_rccn = 0;					/*受信ﾊﾞｲﾄ数ｶｳﾝﾀのｸﾘｱ */

	eTRNS_trmd = cInit;					/*通信ﾓｰﾄﾞ→0[ｲﾆｼｬﾙﾓｰﾄﾞ] */

	return;
}

	
/**************************************************************************
<Title>			:vTRNS_Rcvemd1
<Function>		:コマンドの受信［受信モード：１］
<Description>	:受信コマンドに応じてＡＣＫ／ＮＡＫの送信設定を行う
				:	受信フラグ構成
				:                7654 3210bit
				:         X_Flag **0* ****
				:                || | |||+- ｽﾀﾝﾊﾞｲ受信(初回受信後は常に1)
				:                || | ||+-- 出力指令
				:                || | |+--- 入力要求
				:                || | +---- 出力再指令
				:                || +------ ﾃﾞｰﾀ正常受信済
				:                |+-------- ｺﾏﾝﾄﾞｴﾗｰ
				:                +--------- 未ｽﾀﾝﾊﾞｲ時ｺﾏﾝﾄﾞ受信
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
	/**************************************************************************
<Function>		:受信エラーを判断する
<Author>		:Takida
<Date>			:2021.11.29
<Update>		:
 **************************************************************************/
unsigned char ucTRNS_get_recieve_error_flg(void)
{
	unsigned char rcver_flg = 0x00;
	
	if( (huart1.Instance->ISR & UART_FLAG_PE) != 0U ){	/* parity error ? */
		rcver_flg |= 0x01;										/* set error flag *//*00000001*/
	}
	if( (huart1.Instance->ISR & UART_FLAG_FE) != 0U ){	/* framing error ? */
		rcver_flg |= 0x02;										/* set error flag *//*00000010*/
	}
	if( (huart1.Instance->ISR & UART_FLAG_NE) != 0U ){	/* noise detection error ? */
		rcver_flg |= 0x04;										/* set error flag *//*00000100*/
	}
	if( (huart1.Instance->ISR & UART_FLAG_ORE) != 0U ){	/* overrun error ? */
		rcver_flg |= 0x08;										/* set error flag *//*00001000*/
	}
	return rcver_flg;											/* return receive error flag */
}
/**************************************************************************
<Function>		:受信エラーフラグをクリアする
<Author>		:Takida
<Date>			:2021.11.30
<Update>		:
 **************************************************************************/
void ucTRNS_clear_recieve_error_flg(void)
{

	huart1.Instance->ICR |= USART_ICR_PECF;		/* parity error clear */
	huart1.Instance->ICR |= USART_ICR_FECF;		/* framing error error clear */
	huart1.Instance->ICR |= USART_ICR_NECF;		/* noise detected error clear */
	huart1.Instance->ICR |= USART_ICR_ORECF;	/* overrun error clear */
	
	return;
}
/**************************************************************************
<Title>			:unsigned char ucTRNS_Fcc(const unsigned char *pBuf)
<Function>		:FCCの算出
<Description>	:送受信バッファデータのＦＣＣを計算する
<Input>			:*pBuf ﾊﾞｯﾌｧ先頭ｱﾄﾞﾚｽ
<Return>		:X_fcc ＦＣＣ
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
unsigned char ucTRNS_Fcc(const unsigned char *pBuf)
{
	/*変数の宣言 */
	unsigned char X_fcc = 0x00;		/*ﾌﾚｰﾑﾁｪｯｸｺｰﾄﾞ */
	unsigned char X_bc = *pBuf;		/*ﾊﾞｲﾄｶｳﾝﾀ */
	unsigned char X_lpct;			/*ﾙｰﾌﾟｶｳﾝﾀ */


	for (X_lpct = 0; X_lpct <= X_bc; X_lpct++)
	  X_fcc ^= *pBuf++;			/*FCC算出 */

	return X_fcc;
}

/**************************************************************************
<Title>			:ucTRNS_Fcc_in
<Function>		:受信データＦＣＣ値の読込み
<Description>	:受信バッファのＦＣＣを取り出す
<Input>			:*pBuf ﾊﾞｯﾌｧ先頭ｱﾄﾞﾚｽ
<Return>		:X_fcc ＦＣＣ
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
unsigned char ucTRNS_Fcc_in(const unsigned char *pBuf)
{
	return pBuf[*pBuf+1] * 16 + pBuf[*pBuf+2];
}

/**************************************************************************
<Title>			:unsigned char ucTRNS_Rcdefin
<Function>		:データ確定処理
<Description>	:受信データの正当性をチェックする
<Input>			:
<Return>		:X_rdfg 受信ﾃﾞｰﾀﾌﾗｸﾞ	00h	正常
				:       				FFh	異常
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
unsigned char ucTRNS_Rcdefin()
{
	/*変数の宣言 */
	unsigned char X_rdfg = 0x00;				/*受信ﾃﾞｰﾀﾌﾗｸﾞ ｸﾘｱ */
	unsigned char X_lpct;						/*ﾙｰﾌﾟｶｳﾝﾀ */


	if (ucTRNS_rcba[1] == 0x00)					/*ｲﾝﾊﾞｰﾀ制御ﾃﾞｰﾀ? */
	{
		for (X_lpct = 2; X_lpct <= ucTRNS_rcba[0]; X_lpct++)
		{
			if (ucTRNS_rcba[X_lpct] > 0x9F)		/*ﾃﾞｰﾀ上限 ﾁｪｯｸ */
			  X_rdfg = 0xFF;					/* ﾃﾞｰﾀ値異常 */
		}
	}
	else
	{
		X_rdfg = 0xFF;							/* ﾃﾞｰﾀ値異常 */
	}

	return X_rdfg;
}

/**************************************************************************
<Title>			:vTRNS_Setsenddat
<Function>		:送信データの格納
<Description>	:送信データを取得し送信バッファに格納する
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF070-0.01 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-25 -Tomo ソフト設計ＤＲ指摘対応
				:Ver REF1000-0.01 2010-05-11 -Mura INVﾏｲｺﾝｺｰﾄﾞ表示対応 @004MxDOD[10/05/11]
				:Ver REF1100-0.00 2011-04-30 -Mura ソフト設計内部DR指摘対応 @002MwDOD[11/04/30]
				:Ver REF1100-0.01 2011-06-16 -Mura 評価用マクロ(NOTCHTEST/STARTUP_TEST)対応 @019MwOTM[11/06/16]
				:18/10/03 ﾜｲﾔﾚｽLAN_ﾛｸﾞ取得対応(送信ﾃﾞｰﾀ18ｷｬﾗ化) 吉田
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Setsenddat()
{
//	extern UI08 ucPMD_ldang;																	/* 使ってない、削除する 20210520*/
//	extern UI16 un100MSt_trqct,unTRQ_pwmonmin;													/* 使ってない、削除する 20210520*/
	//	extern  unsigned char ucTRQ_crctarry[18];	/*@014MwRRD[11/06/01]*/						/* 使ってない、削除する 20210520*/
//	extern  unsigned16 unTRQ_crctarry[18];		/*@014MwRRD[11/06/01]*/							/* 使ってない、削除する 20210520*/
//	extern UI08 ucTRQ_maxposN, ucTRQ_fixpos,ucTRQ_maxposHidx,ucTRQ_maxposH[4]/*, tmpcnt*/;		/* 使ってない、削除する 20210520*/
#if defined(STARTUP_TEST)
#elif defined(NOTCHTEST)
#define OUTDATA_NUM 7
#define PERIOD 1
	static UI08 viewindx = 0;
	UI08 tmp;
	ucTRNS_seba[ 2] = ucTRNS_ista;						 	/* @019MwOTM[11/06/16]_STA */
	/*  */
	ucTRNS_seba[ 3] = mdidxbuf[0] / 100;
	ucTRNS_seba[ 4] = mdidxbuf[0] % 100;
	ucTRNS_seba[ 5] = mdidxbuf[1] / 100;
	ucTRNS_seba[ 6] = mdidxbuf[1] % 100;
	ucTRNS_seba[ 7] = mdidxbuf[2] / 100;
	ucTRNS_seba[ 8] = mdidxbuf[2] % 100;
	ucTRNS_seba[ 9] = mdidxbuf[3] / 100;
	ucTRNS_seba[10] = mdidxbuf[3] % 100;
	ucTRNS_seba[11] = mdidxbuf[4] / 100;
	ucTRNS_seba[12] = mdidxbuf[4] % 100;
	ucTRNS_seba[13] = mdidxbuf[5] / 100;
	ucTRNS_seba[14] = mdidxbuf[5] % 100;

	viewindx = (viewindx >= (PERIOD * OUTDATA_NUM -1) ? 0 : viewindx + 1);
	ucTRNS_seba[15] = tmp = viewindx / PERIOD;
	if(tmp < 3){														/* 起動時 */
		ucTRNS_seba[16] = err_log[0][tmp*2    ];					/* 2回目 0:通常 1:一致   2:位置ズレ */
		ucTRNS_seba[17] = err_log[0][tmp*2 + 1];					/* 3回目 0:通常 1:多数決 2:不一致 */
	}
	else if(tmp < 6){													/* トルク制御中 */
		ucTRNS_seba[16] = err_log[1][tmp*2 - 6];					/* 2回目 3:通常 4:一致   5:位置ズレ */
		ucTRNS_seba[17] = err_log[1][tmp*2 - 5];					/* 3回目 3:通常 4:多数決 5:不一致 */
	}
	else if(tmp < 7){
		ucTRNS_seba[16] = err_log[0][6];							/* 6:起動不良 */
		ucTRNS_seba[17] = err_log[1][6];							/* 6:トルク制御中位置ズレ>±4 */
	}
	if(ucTRNS_seba[16] > 0x9f){
		ucTRNS_seba[16] = 0x9f;
	}
	if(ucTRNS_seba[17] > 0x9f){
		ucTRNS_seba[17] = 0x9f;
	}
#else	/* 通常時 */

	/*--- (1 BYTE)	INV保護状態１ ---*/
	if (ucTRNS_ista < 0xA0){					/* 通信ﾃﾞｰﾀ値上限判定 */
	  ucTRNS_seba[2] = ucTRNS_ista;				/* 異常状態ﾃﾞｰﾀ1を送信ﾊﾞｯﾌｧに格納 */
	}else{
	  ucTRNS_seba[2] = 0x9F;					/* 上限値を送信ﾊﾞｯﾌｧに格納 */
	}

	/*--- (2 BYTE)	INV保護状態２ ---*/
	if(ucTRNS_ista2 < 0xA0){					/* 通信ﾃﾞｰﾀ値上限判定 */
		ucTRNS_seba[3] = ucTRNS_ista2;  		/* 異常状態ﾃﾞｰﾀ2を送信ﾊﾞｯﾌｧに格納 */
	}else{
		ucTRNS_seba[3] = 0x9F;					/* 上限値を送信ﾊﾞｯﾌｧに格納 */
	}

	/*--- (3 BYTE)	空き ---*/
	ucTRNS_seba[4] = 0;							/* 空き */

	/*--- (4 BYTE)	指令回転数 ---*/
	if (ucTRNS_sfrq_b < 0xA0)		/*A0h未満の場合					 */
	  ucTRNS_seba[5] = ucTRNS_sfrq_b;/* 指令回転数を送信ﾊﾞｯﾌｧに格納 */
	else							/*A0h以上の場合 */
	  ucTRNS_seba[5] = 0x9F;		/* 上限値を送信ﾊﾞｯﾌｧに格納 */

	/*--- (5 BYTE)	実回転数 ---*/
	if (ucTRNS_ifrq_b < 0xA0)		/*A0h未満の場合					 */
	  ucTRNS_seba[6] = ucTRNS_ifrq_b;/* 実回転数を送信ﾊﾞｯﾌｧに格納 */
	else							/*A0h以上の場合 */
	  ucTRNS_seba[6] = 0x9F;		/* 上限値を送信ﾊﾞｯﾌｧに格納 */

	/*--- (6 BYTE)	INV仕様識別ｺｰﾄﾞ ---*/
	if (ucTRNS_type < 0xA0)		/*A0h未満の場合					 */
	  ucTRNS_seba[7] = ucTRNS_type;	/* INV仕様識別ｺｰﾄﾞを送信ﾊﾞｯﾌｧに格納 */					/* @003MxDOD[12/04/14] */
//	  ucTRNS_seba[7] = 0;		/* bit0：「ﾄﾙｸ制御禁止ﾌﾗｸﾞ」ﾒｲﾝには常時ﾄﾙｸ制御許可で送信 */		/* @003MxDOD[12/04/14] */
	/* ucTRNS_seba[7] = ucTRQ_crctarry;機種判別ﾃﾞｰﾀを送信ﾊﾞｯﾌｧに格納 */
	else						/*A0h以上の場合 */
	  ucTRNS_seba[7] = 0x9F;	/* 上限値を送信ﾊﾞｯﾌｧに格納 */

	/*--- (7 BYTE)	予約 Duty[%] ---*/
	ucTRNS_seba[8] = 0;			/* 正弦波でのDuty計算方法を要検討 */

	/*--- (8 BYTE)	Iq(q軸電流) [A]*10, DEC ---*/
	if(unTRNS_iq_decia < 0xA0U){
		ucTRNS_seba[9] = (unsigned char)unTRNS_iq_decia;
	}else{
		ucTRNS_seba[9] = 0x9F;
	}


	/*--- (9 BYTE)	平均電力 ---*/
	if(ucTRNS_pwr < 0xA0){					/* 通信ﾃﾞｰﾀ値上限判定 */
		ucTRNS_seba[10] = ucTRNS_pwr;			/* 平均電力を送信ﾊﾞｯﾌｧに格納 */
	}else{
		ucTRNS_seba[10] = 0x9F;					/* 上限値を送信ﾊﾞｯﾌｧに格納 */
	}

	/*--- (10 BYTE)	ｷｬﾘｱ周波数 ---*/
	if(ucg_TRNS_PWMfreq < 0xA0){					/* 通信ﾃﾞｰﾀ値上限判定 */
		ucTRNS_seba[11] = ucg_TRNS_PWMfreq;			/* ｷｬﾘｱ周波数／100[Hz]を送信ﾊﾞｯﾌｧに格納 */
	}else{
		ucTRNS_seba[11] = 0x9F;						/* 上限値を送信ﾊﾞｯﾌｧに格納 */
	}

	/*--- (11 BYTE)	母線電圧 ---*/
	if(ucMAIN_vltnow < 0xA0){						/* 通信ﾃﾞｰﾀ値上限判定 */
		ucTRNS_seba[12] = ucMAIN_vltnow;			/* 母線電圧を送信ﾊﾞｯﾌｧに格納 */
	}else{
		ucTRNS_seba[12] = 0x9F;						/* 上限値を送信ﾊﾞｯﾌｧに格納 */
	}

	/*--- (12 BYTE)	INV運転状態 ---*/
	ucTRNS_seba[13] = ( ((UI08)(ePMD_drvmode & 0x0F)) | (IsFullWave() ? 0 : 0x30) );	/* INV運転モードと2レベルリレー状態を送信ﾊﾞｯﾌｧに格納 */

	/*--- (13 BYTE)	端子電圧A(ブレーキ制御中のみ) ---*/
	ucTRNS_seba[14] = ucTRNS_prev_large_dec;

	/*--- (14 BYTE)	端子電圧B(ブレーキ制御中のみ) ---*/
	ucTRNS_seba[15] = ucTRNS_prev_small_dec;

	/*--- (15～16 BYTE)	空き ---*/
	ucTRNS_seba[16] = 0;							/* 空き */
	ucTRNS_seba[17] = 0;							/* 空き */

	/*--- (17,18 BYTE)	ﾏｲｺﾝｺｰﾄﾞ ---*/
	ucTRNS_seba[18] = (UI08)((MICONCODE & 0xFF00) >> 8);/* ﾏｲｺﾝｺｰﾄﾞ上位1byteを送信ﾊﾞｯﾌｧに格納 */
	ucTRNS_seba[19] = (UI08)(MICONCODE & 0x00FF);		/* ﾏｲｺﾝｺｰﾄﾞ下位1byteを送信ﾊﾞｯﾌｧに格納 */

#endif

	return;
}

/**************************************************************************
<Title>			:vTRNS_Setrcvedat
<Function>		:受信データの格納
<Description>	:受信データをバッファから取得する
<Input>			:
<Return>		:
<Author>		:-AK	作成
<Date>			:2002-0320
<Update>		:Ver REF02A-0.00 2002-03-20
				:Ver REF070-0.01 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-25 -Tomo ソフト設計ＤＲ指摘対応
				:Ver REF070-0.04 2007-05-07 -Outi P63とﾒｲﾝﾏｲｺﾝ通信仕様不一致時のfailsafe仕様追加
				:Ver REF1000-0.01 2010-06-11 -Mura 停止時振動対策 メイン通信処理追加  @008MwHOM[10/06/11]
				:Ver REF1000-0.02 2010-06-16 -Mura 停止時振動対策 メイン通信処理対応  @011MwOTT[10/06/16]
				:Ver REF1000-0.02 2011-06-15 -Mura 工程検査中トルク制御OFF対応		  @018MxOTM[11/06/15]
				:17/06/29 デバッグ用処理削除 吉田
				:17/12/20 デバッグ用処理削除(トルク制御) 吉田
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Setrcvedat()
{
#if (defined(ICETEST) || defined(STARTUP_TEST))	/* ICE用 */	/* @012MzDOD[17/02/18] */
//#if 1 //for test
	/* ﾒｲﾝﾏｲｺﾝからの受信ﾃﾞｰﾀを無視 */
#else	/*通常*/
	/*--- (1 BYTE)	指令回転数 ---*/
	ucTRNS_sfrq_b = ucTRNS_rcba[2];		/* 指令回転数 */	/* @007ygDOD[17/06/29] */
	/* ucTRNS_sfrq_b = 0x25;		指令回転数 */
	u8g_TRNSset_spd_flg = 1;    /*メインからの回転数指令がセットされたらフラグをSET*/
	ucMINtm_H50ct = 0;          /*H50異常断続運転でなくなるためカウンタをクリア*/
	/*--- (2 BYTE)	ｲﾝﾊﾞｰﾀﾓｰﾄﾞ選択 ---*/
	ucTRNS_sfrg = ucTRNS_rcba[3];		/*受信状態ﾃﾞｰﾀ */
	/*--- (3 BYTE)	INV仕様識別ｺｰﾄﾞ ---*/
	if (ucTRNS_rcba[0] >= 0x04){		/*BCの確認(ﾃﾞｰﾀ数=3) */ /*＠~07-0507 -Outi P63とﾒｲﾝﾏｲｺﾝ通信仕様不一致時のfailsafe仕様追加 */ /* @009awOTT[07/04/27] */
		ucTRNS_type = ucTRNS_rcba[4];	/*INV仕様識別ｺｰﾄﾞ */
	}else{								/*BCの確認(ﾃﾞｰﾀ数≠3byte) */
		ucTRNS_sfrg |= 0x10;			/*通信データ未確定状態セット*/
	}

	#if 0
		ucTRNS_sfrg &= ~(0x10);/*通信を無視*/ 
		ucTRNS_type = 0x09;/*INV仕様識別コード*/
	#endif


	/*--- (4 BYTE)	停止時振動対応制御ﾃﾞｰﾀ ---*/
	ucTRNS_stopctrl = ucTRNS_rcba[5];					/* 転流停止制御用データ 	*/

	/*--- (5 BYTE)	ﾄﾙｸ制御 最大DUTY補正量 ---*/
	if((ucTRNS_sfrg & 0x40) != 0){						/* 工程検査中 				*/
		unTRNS_crctduty = 0;							/* Duty補正量0クリア 		*/
	}else{
		if(ucTRNS_rcba[6] <= 40){						/* DUTY補正設定上限40%		*/
			unTRNS_crctduty = (ucTRNS_rcba[6] * 10);	/* 最大DUTY補正値			*/
		}else{											/* DUTY補正40%より大なら	*/
			unTRNS_crctduty = 100;						/* DUTY補正15%に固定		*/
		}
	}

	/*--- (6 BYTE)	ﾄﾙｸ制御 DUTY補正印加開始位置 ---*/
	if( (ucTRNS_rcba[7] >= 2) && (ucTRNS_rcba[7] <= 15) ){		/* トルク制御許可後のucTRQ_fixposの値 設定範囲は[2～((D_MTBUFSZ-1)-2)] */ 	  /* @007MwDOD[11/05/10] */
		ucTRNS_basepos  = ucTRNS_rcba[7];						/* DUTY補正印加開始位置 */ 												  /* @007MwDOD[11/05/10] */ /*@011MwRRD[11/05/17]*/
	}else{														/* 設定値「0」or「1」or「16」or「17」のとき */ 							  /* @007MwDOD[11/05/10] */
		ucTRNS_basepos  = 8;									/* 基準位置 「7」に設定*/ 												  /* @007MwDOD[11/05/10] */ /*@011MwRRD[11/05/17]*/
	}

	/*--- (7 BYTE)	空き ---*/
	ucTRNS_wirehgflg = ucTRNS_rcba[8];							/* ｺﾝﾌﾟﾚｯｻ巻線保護ﾌﾗｸﾞ(ﾒｲﾝﾏｲｺﾝ-INVﾏｲｺﾝのｼｰｹﾝｼｬﾙ通信用) 下位4bitを使用 *//* @305swDOD[24/09/02] */

	/*--- (8 BYTE)	空き ---*/
		/* 空き */												/* */ /* @246swDOD[23/03/31] */

	/*--- (9 BYTE)	空き ---*/
		/* 空き */												/* */ /* @246swDOD[23/03/31] */

	/*--- (10 BYTE) 空き ---*/
		/* 空き */												/* */ /* @246swDOD[23/03/31] */

	/*--- (11 BYTE) 空き ---*/
		/* 空き */												/* */ /* @246swDOD[23/03/31] */

	/*--- (12 BYTE) 空き---*/
		/* 空き */												/* */ /* @246swDOD[23/03/31] */

	/*--- (13 BYTE) 空き ---*/
		/* 空き */												/* */ /* @246swDOD[23/03/31] */

	/*--- (14 BYTE) 空き ---*/
		/* 空き */

	/*--- (15 BYTE)	デフロストヒータ通電率 ---*/
		ucTRNS_DEFHrate  = ucTRNS_rcba[16];

	/*--- (16 BYTE)	PCエバヒータ通電率 ---*/
		ucTRNS_PCEVAHrate  = ucTRNS_rcba[17];

	/*--- (17 BYTE) 空き ---*/
		/* 空き */	
	/*--- (18 BYTE) 空き ---*/
		/* 空き */	
	/*--- (19 BYTE) 空き ---*/
		/* 空き */	
	/*--- (20 BYTE) 空き ---*/
		/* 空き */	
	/*--- (21 BYTE) 空き ---*/
		/* 空き */	
	/*--- (22 BYTE) 空き ---*/
		/* 空き */	
#endif

	return;
}


#if 0
/**************************************************************************
<Title>			:vTRNS_Main2Inv_dat
<Function>		:メイン送信データ取得
<Description>	:メイン⇒インバータへの通信データを取得する
<Input>			:
<Return>		:
<Author>		:村上 作成
<Date>			:2010-1213
<Update>		:Ver REF1010-0.01 2010-12-13 -Mura  @038MwDOD[10/12/13]
				:BL7Jr-BB260  2016/3/25  Mura  SVA51C圧縮機対応  @000MzDOD[16/03/25]
				:2023/10/10 通電角制御ﾌﾗｸﾞ割当て直し対応 瀧田 @272sxDOD[23/10/10]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void vTRNS_Main2Inv_dat(void)														/* @038MwDOD[10/12/13]_STA */
{

#if (defined(ICETEST) || defined(STARTUP_TEST) || defined(UNIT_TEST))	  								/* @002MxOOM[17/02/21] */
	/* ﾒｲﾝﾏｲｺﾝからの受信ﾃﾞｰﾀを無視 */
#else
	ucTRNS_sfrq_b =	ucg_CMP_sfrq;	/* 指令回転数 */

/*	ucTRNS_sfrg   = 0x00;	*/		/* 受信状態ﾃﾞｰﾀ */
	ucTRNS_sfrg &= 0xd1 ;			/* b5=0	起動回転数切替フラグ（33rps起動）*/	/* @070MwDOD[11/03/23] *//* ucTRNS_sfrgの.b0に割当て直し *//* @272sxDOD[23/10/10] */
									/* b3=0	温度上昇対策|ｷｬﾘｱ低速ﾓｰﾄﾞFLG	 */
									/* b2=0	パイプ共振モードFLG				 */
									/* b1=0	騒音測定モードFLG				 */

	ucTRNS_type   = 0x06;			/* 機種判別ﾃﾞｰﾀ */							/* @008MzDOD[13/04/3] */
		/*	00000110 */
		/*	|||||||+----0：0  bit0：ｵｰﾊﾞｰﾗｯﾌﾟ及び矩形波ﾍﾞｸﾄﾙ	 ●0：無し 		  1：有り				*/
		/*	||||||+-----1：1  bit1：国内/海外切替				   0：国内		●1：海外				*/
		/*	|||||+------2：1  bit2：下相駆動信号極性			   0：Lo-Active	●1：Hi-Active(IPM)		*/
		/*	||||+-------3：0  																			*/
		/*	|||+--------4：0  																			*/
		/*	||+---------5：0  bit5：直流電圧低下保護仕様		 ●0：AD検知仕様  1：外部割込み仕様		*/
		/*	|+----------6：0  bit6：2レベル制御	 				 ●0：無 		  1：有					*/
		/*	+-----------7：0	-		 																*/

//	if(ucg_IN_motor != 0){					/* 高巻数モータなら		*/	/* @000MzDOD[16/03/25] */
//		ucTRNS_type |= 0x02;				/* 高巻数用duty選択 	*/	/* @000MzDOD[16/03/25] */
//	}else{									/* 低巻数モータなら		*/	/* @000MzDOD[16/03/25] */
//		ucTRNS_type &= 0xfd;				/* 低巻数用duty選択 	*/	/* @000MzDOD[16/03/25] */
//	}																	/* @000MzDOD[16/03/25] */
//	ucTRNS_stopctrl = 18;					/* 転流停止制御用データ */	/*「18」=停止時振動制御OFF*/
#endif
	return;
}																				/* @038MwDOD[10/12/13]_END */

/**************************************************************************
<Title>			:vTRNS_Inv2Main_dat
<Function>		:インバータ送信データ取得
<Description>	:インバータ⇒メインへの通信データを取得する
<Input>			:
<Return>		:
<Author>		:村上 作成
<Date>			:2010-1213
<Update>		:Ver REF1010-0.01 2010-12-13 -Mura  @038MwDOD[10/12/13]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
**************************************************************************/
void vTRNS_Inv2Main_dat(void)														/* @038MwDOD[10/12/13]_STA */
{
	ucg_TR_ista = ucTRNS_ista;		/*INV状態*/
	ucg_TR_ista2 = (ucTRNS_lap|ucTRNS_ista2);		/*ﾗｯﾌﾟ角*/
}								/* @038MwDOD[10/12/13]_END */
#endif
