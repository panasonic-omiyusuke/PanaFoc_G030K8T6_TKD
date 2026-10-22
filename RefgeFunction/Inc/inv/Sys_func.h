/**************************************************************************
#ifdef DOC
File Name		：Sys_func.h
Function		：
Description		：関数外部宣言処理
Program			：
Author			：小柳　英二郎
Date			：05/04/25
Update			：05/05/10 REF051-1.01 FCFANOUT追加
				：05/05/27 REF051-1.02 BZRST1,BZRST2追加
				：10/11/25 BW465 BY601のlibfanc.cを移植 村上 @019MwDOD[10/11/22]
				：10/11/25 BW465 FCファン制御要求仕様対応 村上 @021MwDOD[10/11/22]
				：11/ 1/24 BW465  村上	 @058MwOTT[11/01/20] 
Copyright		：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/

void vTIME_hour(void);				/*070425*/
void vTIME_min(void);				/*070425*/
void vTIME_sec(void);				/*070425*/

void vTEST_pcbchk(void);			/*070426*//*追加*/
void vTEST_IGBTchk(void);														/* IGBT検査制御 */	/* @112MzDOD[14/06/03] */
void vTEST_IPM_OutputOFF(void);		/* IPM検査用出力停止処理 *//* @262swDOD[23/07/05] */
void vTEST_IPM_GPIO_Init(void);		/* IPM検査用GPIO設定処理 *//* @262swDOD[23/07/05] */

void vTEST_IGBTon(GPIO_TypeDef *PORT_CHARGE,uint16_t PIN_CHARGE,GPIO_TypeDef *PORT_UPPER,uint16_t PIN_UPPER,GPIO_TypeDef *PORT_LOWER,uint16_t PIN_LOWER);		/* IGBT出力処理 */	/* @262swDOD[23/07/05] */

void vTEST_HEATERon(UCHAR sel);		/* ﾋｰﾀ出力処理 */
void vTEST_HEATERoff(void);			/* ﾋｰﾀ停止処理 */
void vTEST_HEATERchk(void);			/* ﾋｰﾀ切替処理 */

void vMAIN_MainPros (void);			/* @000MzDOD[10/08/26] */
void vMAIN_inidtset (void);			/*メインデータ初期設定*/ /* @008IxOOM[10/10/06] */			/* @059MwOTH[11/01/26] */
void vMAIN_MainInttm51(void);		/*メイン仮想3.3ms割込み */	/* @008MwDOD[10/10/22] */		/* @059MwOTH[11/01/26] */

#if 0 /*HTAでは、P版検査をODMで行うため本モジュールは不要*/
void vINT_potinput(void);			/* 入力処理 */		/* @099zDOD[14/05/12] */
void vINT_potoutput(void);			/* 出力処理 */		/* @099zDOD[14/05/12] */
#endif
void vINT_potinset(UCHAR *adrw,UCHAR *adrx,UCHAR *adry,UCHAR *adrz,UCHAR ucl_cndt);			/* @099zDOD[14/05/12] */

void vTMSH_cnt(st_4btm *time);															/* @000IzDOD[10/09/13]_04 */
void vTIME_cnt(st_2btm *time,UCHAR ucl_timesct,UCHAR ucl_cntdt,UCHAR ucl_lomax );		/* @000IzDOD[10/09/13]_04 */
UCHAR ucTIME_mcntmstmup(void);															/* @000IzDOD[10/09/13]_04 */
UCHAR ucTIME_hcntmtmup(void);															/* @000IzDOD[10/09/13]_04 */

void vMAIN_INT_intc10(void);										/* @073HxDOD[11/05/11] */
void vMAIN_TR_trans(void);											/* @073HxDOD[11/05/11] */

void vMAIN_Heater(void);							/* @005MzDOD[16/10/04] */
USHORT unFUNC_heatontimeset( USHORT ,USHORT);		/* @005MzDOD[16/10/04] */
USHORT unFUNC_max_un(USHORT, USHORT );				/* @005MzDOD[16/10/04] */
