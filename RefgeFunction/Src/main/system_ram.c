/*
 * system_ram.c
 *
 *  Created on: 2020/10/20
 *      Author: 2993017
 */


#include "Sys_type.h"
#include "system_ram.h"
#include "angle_control.h"

UI08 ucTYPE_uart;
UI08 ucTYPE_2level;						/* 2Level制御有無 */

eStateTag   ePMD_drvmode;			 	/* State of Pmd1G */
unsigned8	ucTYPE_global;				/* 海外機種判別フラグ */
unsigned8	ucg_TIM_2ms;				/* 2msタイマカウンタ */
unsigned8	ucTYPE_Dline;				/* Dライン共振保護制御 */
unsigned8	ucTYPE_Dline_point;			/* Dライン共振ポイント */
unsigned8	ucTC1tm_spdctrlct;			/* 目標回転数更新用カウンタ */
unsigned8	ucTC1tm_VctrCtr;			/* ベクトル制御回転数カウンタ */
unsigned16	unPMDc_dutyn;				/* Duty(PWM変調Amplitude) */
unsigned16	hCurMecRps;					/* 現在の回転数Hz(/16bit) */
unsigned16  hRampMecRps;				/* ランプ指令回転数Hz(拡張rps/16bit) */
unsigned16	usAGC_cdang;				/* 通電角: conduction_angle運転時の変更 */ /* 1536 ~ 2560 */
unsigned8	ucTYPE_position_duty;		/* 位置決め分解能および起動Duty */

extern sAngleDataTag angle;				/* 通電角に関する構造体 */
extern unsigned8	ucHOG_sttF;			/* 保護状態フラグ */
extern unsigned8	ucMINtm_HOGOct;		/* インバータ保護継続時間カウンタ */	/* @021ywDOD[18/01/25] */
extern unsigned8	ucSPD_rpmfgmd;		/* 回転数制御用フラグ&モード設定RAM */
