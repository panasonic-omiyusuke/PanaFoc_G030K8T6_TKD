/**************************************************************************
<File Name>		:INTTM51.h
<Function>		:ＩＮＴＴＭ５１割込み処理(3.3ms割込み処理)
<Program>		:
<Author>		:村上  和宏
<Date>			:14/03/13
<Update>		:
<Copyright>		:Panasonic Co.,Ltd.
**************************************************************************/

/********************************************/
/*		      マクロ定義					*/
/********************************************/
#define		AD_ST			0				/*A/D変換処理電源投入時*/
#define		AD_INI			1				/*A/D変換処理ｲﾆｼｬﾙ*/
#define		AD_NORMAL		2				/*A/D変換処理通常*/

#define		ADMD_INI		0				/*A/Dﾓｰﾄﾞｲﾆｼｬﾙ*/
/*#define	ADMD_MAX	mARRAY_NUM(T_AN)*/	/*A/DﾓｰﾄﾞMAX*/		/* @023MzDOD[13/10/16] */
#define		ADMD_MAX	ETC_ADNUM			/*A/DﾓｰﾄﾞMAX*/		/* @023MzDOD[13/10/16] */

#define		ADG_P0				7			/*P0ｸﾞﾙｰﾌﾟ(AN0～AN7)*/	/* @014IwDOD[10/11/01] */
#define		INPUT_SETDT			6			/* 入力確定用 10ms * 6回*/	/* @099zDOD[14/05/12] */

 #define	NUM_ADUPDATE		10			/* AD値更新連続回数(通常)	*/	/* @107MzDOD[14/05/27] */ /* @117MzDOD[14/06/09] */ /* @120MzDOD[14/06/13] */
 #define	NUM_ADUPDATE_FCT	1			/* AD値更新連続回数(P板検査)*/	/* @120MzDOD[14/06/13] */

/********************************************/
/*		外部変数参照						*/		/* @037MzDOD[14/01/27] */
/********************************************/
extern st_2btm	stg_TIMEM[TMMCT];	/* 分ﾀｲﾏ			*/
extern UCHAR	ucg_IN_add[ETC_ADNUM];	/* ANﾃﾞｰﾀ 格納*/	/* @002MxOOM[17/02/21] */  /* ファイルの読み込みがレイヤーをまたぐ */

extern UCHAR	ucg_MAIN_20up;		/* 20msﾀｲﾑUP*/
extern UCHAR	ucg_MAIN_40up;		/* 40msﾀｲﾑUP*/
extern UCHAR	ucg_MAIN_ptest;		/* 検査制御ﾌﾗｸﾞ*/

extern union byte_def P0_addr;
extern union byte_def P1_addr;
extern union byte_def P2_addr;
extern union byte_def P3_addr;
extern union byte_def P4_addr;
extern union byte_def P5_addr;
extern union byte_def P6_addr;

extern UCHAR ucg_INT_DEFH;	 		/* ﾃﾞﾌﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_PCEVAH;		/* ﾌﾚﾝﾁｰﾀｰ 		出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_PIPEH;	 		/* 給水ﾊﾟｲﾌﾟﾋｰﾀｰ出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_VCH;	 		/* VCﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_VCTOPH;		/* VC天面ﾋｰﾀｰ 	出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_PFH;	 		/* PFﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_SCH;	 		/* SCﾋｰﾀｰ 		出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_TANKH;			/* 給水ﾀﾝｸﾋｰﾀｰ 	出力ﾌﾗｸﾞ*/
extern UCHAR ucg_INT_DRNH;			/* ﾄﾞﾚﾝﾁｭｰﾌﾞﾋｰﾀ	出力ﾌﾗｸﾞ*//* @246swDOD[23/03/31] */
extern UCHAR ucg_INT_PULSE;			/* 発振制度確認ﾊﾟﾙｽ出力ﾌﾗｸﾞ  */

/*******************************************
		プロトタイプ宣言
 *******************************************/

int vMAIN_IsOptionByteCheck(void);
