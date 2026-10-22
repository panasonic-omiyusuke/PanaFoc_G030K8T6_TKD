
#ifndef VIPDC_H
#define VIPDC_H

#include "Sys_type.h"

#define RENESAS_TO_ST32

/*------------------------------*/
/*		切替え用定義			*/
/*------------------------------*/
/*#define ACTIVE   (1)*/							/* 制御有効設定用定義*/
/*#define INACTIVE (0)*/							/* 制御無効設定用定義*/
#define LO_ACTIVE (0)							/* Lo-active用定義					*//* @145swDOD[21/05/28] */
#define HI_ACTIVE (1)							/* Hi-active用定義					*//* @145swDOD[21/05/28] */


/*------------------------------*/
/*		INV ﾏｲｺﾝｺｰﾄﾞ設定		*/	/*★★★ リリース前要確認 ★★★*/
/*------------------------------*/
/* ※注意※ 通信に使っているため、上位下位それぞれのbyteが0xA0を超えないよう注意 */
#define MICONCODE	0x0A27	 					/* INVﾏｲｺﾝCODE (P板検査用)			*/

/*------------------------------*/
/*  オプションバイト設定確認用  */	/*★★★ リリース前要確認 ★★★*/
/*------------------------------*/
/* STM32G071C8T6 */
#define		mREADOPBYTE(address)	(*((uint32_t *) address))	/* 下位32bitの設定値確認のため32bit定義 */	/* @208swDOD[22/04/11] */

#define		OPBYTEADDRS1	0x1FFF7800				/* オプションバイトアドレス1 */
#define		OPBYTEADDRS2	0x1FFF7808				/* オプションバイトアドレス2 */
#define		OPBYTEADDRS3	0x1FFF7810				/* オプションバイトアドレス3 */
#define		OPBYTEADDRS4	0x1FFF7818				/* オプションバイトアドレス4 */
#define		OPBYTEADDRS5	0x1FFF7820				/* オプションバイトアドレス5 */
#define		OPBYTEADDRS6	0x1FFF7828				/* オプションバイトアドレス6 */
#define		OPBYTEADDRS7	0x1FFF7848				/* オプションバイトアドレス7 */ /* WRP2A_STARTは、STMG030K8T6ではDualBank非搭載のため設定値確認不要 */
#define		OPBYTEADDRS8	0x1FFF7850				/* オプションバイトアドレス8 */ /* WRP2A_ENDは、STMG030K8T6ではDualBank非搭載のため設定値確認不要 */

/*#define		OPBYTECHK1		0xDFFF81AA	*/			/* オプションバイト比較値：設試まで=F7FFE1AA */
#define		OPBYTECHK1		0xDFFF81BB				/* オプションバイト比較値：量試以降=0xF7FF01BB */					
#define		OPBYTECHK2		0x00000000				/* オプションバイト比較値 */										
#define		OPBYTECHK3		0x80000000				/* オプションバイト比較値 */										
#define		OPBYTECHK4		0x0000001F				/* オプションバイト比較値 */
#define		OPBYTECHK5		0x0000001F				/* オプションバイト比較値 */
#define		OPBYTECHK6		0x00000000				/* オプションバイト比較値 */
#define		OPBYTECHK7		0x00000000				/* オプションバイト比較値 */
#define		OPBYTECHK8		0x00000000				/* オプションバイト比較値 */

#define     OPBYTECHK1_MASK	0x074F60FF				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/									
/*#define     OPBYTECHK2_MASK	0x00000000*/				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											
/*#define     OPBYTECHK3_MASK	0x00000000*/				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											
#define     OPBYTECHK4_MASK	0x007F007F				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											
#define     OPBYTECHK5_MASK	0x007F007F				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											
/*#define     OPBYTECHK6_MASK	0x00000000*/				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											
/*#define     OPBYTECHK7_MASK	0x0000007F*/				/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											
/*#define     OPBYTECHK8_MASK	0x0000007F*/	/* オプションバイト比較値マスク *//*マイコン変更の際には、予約Bit位置を確認し決定のこと*/											

/*------------------------------*/
/*		INV 処理選択設定		*/	/*★★★ リリース前要確認 ★★★*/
/*------------------------------*/
/*------- 基本 option ------*/
#define H50_DANZOKU			ACTIVE				/* H50 断続運転仕様実装 			*/
#define UPPER_ARM			LO_ACTIVE			/* 駆動素子 上ｱｰﾑactive設定			*//* @008lwORM[24/06/21] */
#define LOWER_ARM  			LO_ACTIVE			/* 駆動素子 下ｱｰﾑactive設定			*//* @145swDOD[21/05/28] */
#define FO_POLALITY			LO_ACTIVE			/*上下アームと同様にFO信号もActive設定の変更のみで対応できるよう変更*//* @008lwORM[24/06/21] */
/*------- INV制御(部品) option ------*/
#define RLY_2LEVEL			ACTIVE				/* 2レベルリレー搭載				*//* ACTIVE:2Lvﾘﾚｰﾎﾟｰﾄに2Lvﾘﾚｰ実装or部品接続なし、INACTIVE:2Lvﾘﾚｰﾎﾟｰﾄに他部品実装 */
/*------- Main制御ソフト option ------*/
/*#define DATA_FLASH		*/					/* ﾃﾞｰﾀﾌﾗｯｼｭ記憶処理				*//* メイン制御処理でﾃﾞｰﾀ記憶必要な場合に有効化する */
/*#define ONECHIP			*/					/* 1chip通信処理切替				*/
/*------- INV制御ソフト option ------*/
#define TRQ_CTL				ACTIVE				/* トルク制御 						*//*updatedutyタイミングが変わる、割込み中の圧迫が増える*//*基本ACTIVEがよい*//*万が一処理時間長い場合はINACTIVEにして対応可能にする*/
#define DETECT_PWM_OFF		INACTIVE			/* PWMオフ時位置検出				*/
//#define 										/* 停止時振動抑制制御 ACTIVE:ブレーキ制御あり、INACTIVE：ブレーキ制御なし（惰性制御） *//*メインから即０rps指令これば実質無効*/
#define SOFT_STOP_SW		ACTIVE			/* 停止時振動抑制制御・ブレーキ制御  ACTIVE:ブレーキ制御あり、INACTIVE：ブレーキ制御なし（惰性制御） */
//#define JIAXI_SEQ_STOP		INACTIVE			/* Jiaxipera社製コンプ停止シーケンス */
//#define DIF_PLSR_START		INACTIVE			/* 差圧起動 *//* メインでの切替えあるかを要確認のこと★ */


/*------- 評価 option ------*/
/*#define STARTUP_TEST		*/					/* ﾄﾙｸ制御 起動テスト用 				*/
/*#define NOTCHTEST			*/					/* ﾄﾙｸ制御 ﾉｯﾁでの機械工程位置確認用 	*/
/*#define TESTCODE			*/					/* 竹岡制御有無							*/
/*#define LABVIEW			*/
/*-------- ICE --------*/
/*	#define ICETEST			*/					/* (定義有)ICE用 		/ (定義無)通常 */
/*	#define LOCKTST			*/					/* (定義有)LOCK保護ﾅｼ 	/ (定義無)通常 */

#if 0		/* DA出力時に使用 */
#define DA_OUT
static inline void OutputDacData(unsigned short data){
	extern DAC_HandleTypeDef hdac1;
	hdac1.Instance->DHR12R1 = data;
}
#endif

/*---- Debug Port ----*//* 定義を有効にしてから使用する */
#if 0 /* Debug用定義 */
#define SELECT_PIN(A) (uint32_t)debug_SPIN##A##_Pin
#define SELECT_PORT(A) debug_SPIN##A##_GPIO_Port
#define SET_PIN(A) SELECT_PORT(A)->BSRR = SELECT_PIN(A)
#define CLR_PIN(A) SELECT_PORT(A)->BRR = SELECT_PIN(A)
#define TGL_PIN(A) HAL_GPIO_TogglePin(SELECT_PORT(A), SELECT_PIN(A))
#endif /* Debug用定義 */
/*---- Debug Port ----*//* 下記をcopyして使用する */
#if 0  /* Debug用記述 */
HAL_GPIO_TogglePin(debug_SPIN1_GPIO_Port, debug_SPIN1_Pin);			/* SPIN1反転 */
HAL_GPIO_WritePin(debug_SPIN1_GPIO_Port, debug_SPIN1_Pin, SET);		/* SPIN1セット */
HAL_GPIO_WritePin(debug_SPIN1_GPIO_Port, debug_SPIN1_Pin, RESET);	/* SPIN1クリア */

HAL_GPIO_TogglePin(debug_SPIN2_GPIO_Port, debug_SPIN2_Pin);			/* SPIN2反転 */
HAL_GPIO_WritePin(debug_SPIN2_GPIO_Port, debug_SPIN2_Pin, SET);		/* SPIN2セット */
HAL_GPIO_WritePin(debug_SPIN2_GPIO_Port, debug_SPIN2_Pin, RESET);	/* SPIN2クリア */

HAL_GPIO_TogglePin(debug_SPIN3_GPIO_Port, debug_SPIN3_Pin);			/* SPIN3反転 */
HAL_GPIO_WritePin(debug_SPIN3_GPIO_Port, debug_SPIN3_Pin, SET);		/* SPIN3セット */
HAL_GPIO_WritePin(debug_SPIN3_GPIO_Port, debug_SPIN3_Pin, RESET);	/* SPIN3クリア */

HAL_GPIO_TogglePin(FREQ_PULSE_OUT_GPIO_Port, FREQ_PULSE_OUT_Pin);			/* SPIN4反転 */
HAL_GPIO_WritePin(FREQ_PULSE_OUT_GPIO_Port, FREQ_PULSE_OUT_Pin, SET);		/* SPIN4セット */
HAL_GPIO_WritePin(FREQ_PULSE_OUT_GPIO_Port, FREQ_PULSE_OUT_Pin, RESET);	/* SPIN4クリア */
#endif  /* Debug用記述 */

/*------------------------------*/
/*        マクロ定義			*/
/*------------------------------*/



/* CT回路異常検出 */ //要検討：タイマを整理した際に移動★
#define D_H53NUM	(10)												/*異常FLG@によるH53確定回数10回*/
#define D_H53TIME	(180)												/*異常FLGAによるH53確定時間180分*/



#define OFFTIME (10)		/* 1は設定禁止 */ /* @236swDOD[23/01/17] */
							/* クロックが32MHz以上は値の変更必要。10×新しいクロック周波数[MHz]/32 */
							/* クロックが16MHzの倍数であれば #define OFFTIME　4*D_fTRD/16 + 1 の定義でよいがそれ以外の場合にNGのため要検討2022.12.14 */
#if (OFFTIME == 1)
#define OFFTIME (2)
#endif

#define USER 0	/* 0:OUCHI/1:TAKEOKA */
#if USER
#define TAKEOKA
#else
/* #define OUTI */
#endif


#define ACTIVATE_PAM               //通電角制御を利用する場合は、定義





#define MDTM_INT_ALL_ENABLE (TIM_DIER_CC4IE | TIM_DIER_CC3IE | TIM_DIER_CC2IE | TIM_DIER_CC1IE | TIM_DIER_UIE)
#define MDTM_INT_LAP_ENABLE (TIM_DIER_CC1IE)
#define MDTM_INT_DUMMY_PDC_ENABLE (TIM_DIER_CC4IE)
#define PWM_U_TMREG (htim1.Instance->CCR1)
#define PWM_V_TMREG (htim1.Instance->CCR3)
#define PWM_W_TMREG (htim1.Instance->CCR2)
#define MDTM_LAP_TMREG (htim3.Instance->CCR1)
#define MDTM_120_TMREG (htim3.Instance->CCR2)
#define MDTM_ADCEN_TMREG (htim3.Instance->CCR3)
#define MDTM_FAILSAFE_TMREG (htim3.Instance->CCR4)
#define MDTM_RESET_TMREG (htim3.Instance->ARR)

#define MDOUT_STOP_INDEX (6)
#define MDOUT_STANDBY_INDEX (7)

#define TIM_IT_CC_ALL (TIM_IT_UPDATE |TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4)
#define IsCompareMatch(TIM_IT_CCx_Or_TIM_IT_UPDATE) (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_CCx_Or_TIM_IT_UPDATE))
#define ClearModeTimerIntRequest(TIM_IT_CCx_Or_TIM_IT_UPDATE) (__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CCx_Or_TIM_IT_UPDATE))

#define asm(a)


#endif	/* #ifndef VIPDC_H */




