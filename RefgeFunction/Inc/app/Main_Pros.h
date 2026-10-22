#ifndef MAIN_PROS_H
#define MAIN_PROS_H

#include "Sys_type_main.h"
#include "PMD1GPar.h"
#include "Ram.h"

/**************************************************************************
<File Name>		:Main_Pros
<Function>		:メインルーチン処理
<Program>		:
<Author>		:村上 和宏
<Date>			:14/03/19
<Update>		:
<Copyright>		:Panasonic Refrigeration Co.,Ltd.
**************************************************************************/
/********************************************/
/*		      マクロ定義					*/
/********************************************/
#define		CNT_20MS	6					/*20ms*//* @073HxDOD[11/05/11] */
#define		H_CYCLE		10					/* ﾋｰﾀ通電ｻｲｸﾙ 10秒	*/	/* @005MzDOD[16/10/04] */
#define		H_CYCLEW	0x0A00				/* ﾋｰﾀ通電ｻｲｸﾙ 10秒	*/	/* @005MzDOD[16/10/04] */

/* 通信異常時(H50) ﾋｰﾀ通電率設定 */
#define 	H50_RATE_DEFH		0			/* H50時ﾃﾞﾌﾛｽﾄﾋｰﾀ	   0% */	/* @018MzDOD[17/03/04] */
#define 	H50_RATE_PCEVAH	0			/* H50時ﾌﾚﾝﾁﾋｰﾀ		 0% */	/* @018MzDOD[17/03/04] */
/*ヒータ追加の場合は、HY2ソフトから流用のこと(タグ:HY2EY2_inv_SOFT_l241217-2)*/

/* ﾌｪｰﾙｾｰﾌ ﾃﾞﾌﾛｽﾄﾋｰﾀ最大時間 */
#define  	DEFH_CYCMAX  0x4600				/*ﾃﾞﾌﾛｽﾄﾋｰﾀON MAXｻｲｸﾙ 70分 */	/* @003lxORM[24/06/12] */

/********************************************/
/*		外部変数参照						*/		/* @037MzDOD[14/01/27] */
/********************************************/
extern UCHAR	ucg_IN_add[ETC_ADNUM];	/* ANﾃﾞｰﾀ 格納*/	/* @002MxOOM[17/02/21] */  
extern st_2btm	stg_TIMEH[TMHCT];	/* 時間ﾀｲﾏ			*/
extern st_2btm	stg_TIMEM[TMMCT];	/* 分ﾀｲﾏ			*/
extern st_2btm	stg_TIMES[TMSCT];	/* 秒ﾀｲﾏ			*/

extern UCHAR	ucg_MAIN_ptest;		/* 検査制御ﾌﾗｸﾞ		*/
extern volatile UCHAR ucg_MAIN_40up;		/* 40msﾀｲﾑUP		*/
extern volatile UCHAR	ucg_MAIN_20up;		/* 20msﾀｲﾑUP		*/
extern UCHAR	ucMAIN_wdrfg;		/* ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄﾌﾗｸﾞ*/

extern UCHAR	ucg_E2P_inmd;		/* E2PROMｲﾆｼｬﾙﾓｰﾄﾞ	*/
extern UCHAR	ucg_E2P_e2req;		/* E2PROM R/W要求	*/
extern UCHAR	ucg_E2P_e2md;		/* E2PROMﾓｰﾄﾞ		*/

extern UCHAR	ucg_INTf_testpa;		/*ﾎﾟｰﾄ入力確定ﾃﾞｰﾀ*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INTf_testpb;		/*ﾎﾟｰﾄ入力確定ﾃﾞｰﾀ*/	/* @099zDOD[14/05/12] */
extern UCHAR	ucg_INTf_test2;		/*  TEST2端子入力	*/	/* @099zDOD[14/05/12] */

extern UCHAR	ucg_IN_kisyu;			/* 機種切替入力確定ﾃﾞｰﾀ */ 	/* @100zDOD[14/05/12] */
extern UCHAR	ucg_INf_kisyu;		/* 機種切替入力確定ﾃﾞｰﾀ */ 		/* @100zDOD[14/05/12] */

extern union byte_def TRBCR_addr;
extern union byte_def WDTR_addr;

extern UCHAR	ucg_INT_PULSE;			/* 発振制度確認ﾊﾟﾙｽ出力ﾌﾗｸﾞ  */

/*さくじょはじめ*/
/* #ifdef RENESAS_TO_ST32 */
/* extern TIM_HandleTypeDef htim1;			 @225swDOD[22/06/17] */
/* extern TIM_HandleTypeDef htim15;	 3.3ms ブレーキ制御用*/
/* #else  RENESAS_TO_ST32 */
/* #endif  RENESAS_TO_ST32 */
/*さくじょおわり*/

#endif /* MAIN_PROS_H */
