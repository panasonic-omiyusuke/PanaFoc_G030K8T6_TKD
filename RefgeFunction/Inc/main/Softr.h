/**************************************************************************
<File Name>		:SOFTR.h
<Function>		:SOFTR.c用ヘッダファイル
<Program>		:
<Author>		:大内山 智則 -Outi
<Date>			:2002-0219
<Update>		:Ver REF045-4.00 2004-08-04 -Outi 静的解析結果対応(if/else文｛｝追加)
				:Ver REF061-0.00 2006-04-27 -Outi IPM論理判定処理追加
				:Ver REF070-0.01 2007-04-09 -Tomo 機種切り替え対応
				:Ver REF070-0.02 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-04-22 -Outi 機種切り替え対応
				:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
				:Ver REF090-0.00 2009-05-11 -Outi IEC対応 @006axOOM[09/05/11]
				:Ver REF1000-0.01 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/22]
				:Ver REF1010-0.01 2010-11-01 -Mura 三菱IPM基板用IO対応 @013MwDOD[10/11/01]
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/


/********************************************
		インクルードファイル
 ********************************************/
#include	"Sys_type.h"



/*******************************************
		マクロ定義
 *******************************************/

/*******************************************
		変数宣言
 *******************************************/
extern  unsigned8	ucMAIN_chkint;			/*割込ﾁｪｯｸｶｳﾝﾀ@MAIN.c *//* 多重定義ｴﾗｰ対応 *//* @259swDOD[23/06/30] */
/*extern  unsigned8	ucSPD_sfrqtp_b;			指令Hz(格納値)<BCD> */
/*extern  unsigned8	ucTRNS_ifrq_b;			実回転数 @通信系ﾓｼﾞｭｰﾙ */

/*extern  unsigned8	uc6Stmr_wdr15ct;		ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄ発生用15分ｶｳﾝﾀ@TIMEA.c */
extern  unsigned8	ucMAIN_wdrfg;			/*ｳｫｯﾁﾄﾞｯｸﾞﾘｾｯﾄ発生用ﾌﾗｸﾞ *//* 多重定義ｴﾗｰ対応 *//* @259swDOD[23/06/30] */

/*extern  unsigned8	ucHOG_movF;				保護動作ﾌﾗｸﾞ */

/*extern  unsigned8	ucTYPE_uart;			通信機種判別確定 */
/*extern  unsigned8	ucTYPE_global;			海外機種判別確定 */
/*extern  unsigned8	ucSPD_rpmfgmd;			回転数制御用ﾌﾗｸﾞ＆ﾓｰﾄﾞ設定RAM @006axOOM[09/05/11] */
/*extern  unsigned8	uc100MS_pumpct;			給油改善制御ｶｳﾝﾀ(150,16秒) @006axOOM[09/05/11] */
/* enum	StateTag{	D_stop,	D_standby,	D_position,	D_forced,	D_running,	D_vector};	 */							/* @006axOOM[09/05/11] */
/*extern  ePMD_drvmode;						State of Pmd1G @006axOOM[09/05/11] */
/*extern  unsigned8	ucMINtm_H50ct;			通信異常時強制off/on制御用ｶｳﾝﾀ（通信異常確定後にｶｳﾝﾄUpするｶｳﾝﾀ）@006axOOM[09/05/11] */
/*extern  unsigned8	ucMINtm_HOGOct;			ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀ保護継続時間ｶｳﾝﾀ @021ywDOD[18/01/25] */
/*extern	unsigned8	ucg_HOGO_lockct;		ﾏｲｺﾝﾘｾｯﾄ用：ｲﾝﾊﾞｰﾀﾛｯｸ保護継続回数ｶｳﾝﾀ @101ywOTT[18/09/14] */
/*extern	unsigned8	uc6Stmr_hgrlsct;		保護状態ﾌﾗｸﾞ解除用ｶｳﾝﾀ(50,５分) @101ywOTT[18/09/14] */
/*extern  unsigned8	uc10MSt_intct;			main確定(10ms毎)ｶｳﾝﾀ@TIMEA.c @006axOOM[09/05/11] */
/*extern	unsigned8	uc2LV_scantyvlt_skip;	直流電圧低下保護Skipﾌﾗｸﾞ @001MwDOD[10/04/22] */

/*extern unsigned8	ucg_IN_kisyu;				 機種切替入力確定ﾃﾞｰﾀ @100zDOD[14/05/12] */

/*SFR*/
/*
extern union byte_def PM0_addr;
extern union byte_def PM1_addr;
extern union byte_def CM0_addr;
extern union byte_def CM1_addr;
extern union byte_def MSTCR_addr;
extern union byte_def CM3_addr;
extern union byte_def PRCR_addr;
extern union byte_def RSTFR_addr;
extern union byte_def OCD_addr;
extern union byte_def WDTR_addr;
extern union byte_def WDTS_addr;
extern union byte_def WDTC_addr;
extern union byte_def AIER0_addr;
extern union byte_def AIER1_addr;
extern union byte_def CSPR_addr;
extern union byte_def TRASR_addr;
extern union byte_def TRBRCSR_addr;
extern union byte_def TRCPSR0_addr;
extern union byte_def TRCPSR1_addr;
extern union byte_def TRDPSR0_addr;
extern union byte_def TRDPSR1_addr;
extern union byte_def TIMSR_addr;
extern union byte_def FRA0_addr;
extern union byte_def FRA1_addr;
extern union byte_def FRA2_addr;
extern union byte_def OCVREFCR_addr;
extern union byte_def FRA3_addr;
extern union byte_def FRA6_addr;
extern union byte_def FRA7_addr;
extern union byte_def CMPA_addr;
extern union byte_def VCAC_addr;
extern union byte_def VCA1_addr;
extern union byte_def VCA2_addr;
extern union byte_def VW1C_addr;
extern union byte_def VW2C_addr;
extern union byte_def FMRDYIC_addr;
extern union byte_def INT4IC_addr;
extern union byte_def VW0C_addr;
extern union byte_def ADMOD_addr;
extern union byte_def ADINSEL_addr;
extern union byte_def ADCON0_addr;
extern union byte_def ADCON1_addr;
extern union byte_def P0_addr;
extern union byte_def P1_addr;
extern union byte_def PD0_addr;
extern union byte_def PD1_addr;
extern union byte_def P2_addr;
extern union byte_def P3_addr;
extern union byte_def P4_addr;
extern union byte_def P5_addr;
extern union byte_def P6_addr;
extern union byte_def PD2_addr;
extern union byte_def PD3_addr;
extern union byte_def PD4_addr;
extern union byte_def PD5_addr;
extern union byte_def PD6_addr;
extern union byte_def P1DRR_addr;
extern union byte_def P2DRR_addr;
extern union byte_def P1DRR_addr;
extern union byte_def P2DRR_addr;
extern union byte_def DRR0_addr;
extern union byte_def DRR1_addr;
extern union byte_def PUR1_addr;
extern union byte_def INTEN_addr;
extern union byte_def INTEN1_addr;
extern union byte_def INTF_addr;
extern union byte_def INTF1_addr;
extern union byte_def KIEN_addr;
extern union byte_def PUR0_addr;
extern union byte_def PUR1_addr;
extern union byte_def TRACR_addr;
extern union byte_def TRAIOC_addr;
extern union byte_def TRAMR_addr;
extern union byte_def TRAPRE_addr;
extern union byte_def TRA_addr;
extern union byte_def LINCR2_addr;
extern union byte_def LINCR_addr;
extern union byte_def LINST_addr;
extern union byte_def TRBCR_addr;
extern union byte_def TRBOCR_addr;
extern union byte_def TRBIOC_addr;
extern union byte_def TRBMR_addr;
extern union byte_def TRBPRE_addr;
extern union byte_def TRBSC_addr;
extern union byte_def TRBPR_addr;
extern union byte_def TRCMR_addr;
extern union byte_def TRCCR1_addr;
extern union byte_def TRCIER_addr;
extern union byte_def TRCSR_addr;
extern union byte_def TRCIOR0_addr;
extern union byte_def TRCIOR1_addr;
extern union byte_def TRCCR2_addr;
extern union byte_def TRCDF_addr;
extern union byte_def TRCOER_addr;
extern union byte_def TRCADCR_addr;
extern union byte_def TRDECR_addr;
extern union byte_def TRDADCR_addr;
extern union byte_def TRDSTR_addr;
extern union byte_def TRDMR_addr;
extern union byte_def TRDPMR_addr;
extern union byte_def TRDFCR_addr;
extern union byte_def TRDOER1_addr;
extern union byte_def TRDOER2_addr;
extern union byte_def TRDOCR_addr;
extern union byte_def TRDDF0_addr;
extern union byte_def TRDDF1_addr;
extern union byte_def TRDCR0_addr;
extern union byte_def TRDIORA0_addr;
extern union byte_def TRDIORC0_addr;
extern union byte_def TRDSR0_addr;
extern union byte_def TRDIER0_addr;
extern union byte_def TRDPOCR0_addr;
extern union byte_def TRDCR1_addr;
extern union byte_def TRDIORA1_addr;
extern union byte_def TRDIORC1_addr;
extern union byte_def TRDSR1_addr;
extern union byte_def TRDIER1_addr;
extern union byte_def TRDPOCR1_addr;
extern union byte_def U0BRG_addr;
extern union byte_def U1BRG_addr;
extern union byte_def FMR2_addr;
extern union byte_def FMR1_addr;
extern union byte_def FMR0_addr;
extern union byte_def TRASR_addr;
extern union byte_def TRBRCSR_addr;
extern union byte_def TRCPSR0_addr;
extern union byte_def TRCPSR1_addr;
extern union byte_def TRDPSR0_addr;
extern union byte_def TRDPSR1_addr;
extern union byte_def TIMSR_addr;
extern union byte_def U0SR_addr;
extern union byte_def U1SR_addr;
extern union byte_def U2SR0_addr;
extern union byte_def U2SR1_addr;
extern union byte_def SSUIICSR_addr;
extern union byte_def INTSR_addr;
extern union byte_def PINSR_addr;
extern union byte_def SSBR_addr;
extern union byte_def VLT0_addr;
extern union byte_def VLT1_addr;
extern union byte_def INTCMP_addr;
extern union byte_def DTCTL_addr;
extern union byte_def DTCEN0_addr;
extern union byte_def DTCEN1_addr;
extern union byte_def DTCEN2_addr;
extern union byte_def DTCEN3_addr;
extern union byte_def DTCEN4_addr;
extern union byte_def DTCEN5_addr;
extern union byte_def DTCEN6_addr;
*/

/*******************************************
		プロトタイプ宣言
 *******************************************/
void	vMAIN_Softr(void);				/* ｿﾌﾄﾘｾｯﾄ処理呼び出し */

