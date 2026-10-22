/**************************************************************************
#ifdef DOC
File Name	：sys_type.h
Description	：記述標準化・簡略化定義ファイル
Remark		：履歴	03/02/20 作成
			  履歴	03/04/16 修正(MEC依頼内容)
Date		：03/02/20
Copyright	：Matsushita Refrigeration Co.,Ltd.
#endif
**************************************************************************/
#ifndef SYSTYPEM_H
#define SYSTYPEM_H
/*#include "sfr_r835c.h"			 @028MzDOD[13/10/31] */
#include "vipdc.h"				/* @028MzDOD[13/10/31] */
/*#include "Adpdc.h"*/


/****************************************************************/
/*					ポートマクロ定義							*/
/****************************************************************/

/*-----AD-----*/
#define	ucg_AD_atc		ucg_IN_add[ADNUM_ATC]	/* AD値格納 ATC  A/D */		/* @002MxOOM[17/02/21] */
#define	ucg_AD_fcv		ucg_IN_add[ADNUM_VOL]	/* AD値格納 FCVR A/D */		/* @002MxOOM[17/02/21] */
#define	ucg_AD_dfc		ucg_IN_add[ADNUM_DEF]	/* AD値格納 DFF  A/D */		/* @002MxOOM[17/02/21] */
#define	ucg_AD_pcc		ucg_IN_add[ADNUM_PCC]	/* AD値格納 PCC  A/D */		/* @002MxOOM[17/02/21] */

/*----入力-----*/
#define		PORT_PULSE		p2_0		/* 発振周波数パルス			*/
#define		PORT_PCB1		p6_6		/* P板検査A入力				*/	/* @010MzDOD[17/02/04] */
#define		PORT_PCB2		p6_7		/* P板検査B入力				*/	/* @010MzDOD[17/02/04] */
#define		PORT_KISYU		p4_4		/* FOポート入力(P板検査用)	*/	/* @013MzDOD[17/02/24] */
#define		PORT_FO			p4_5		/* FOポート入力(P板検査用)	*/
#define		PORT_RX			p6_4		/* 通信Rx(P板検査用)		*/

/*----出力ト----*/
#define		PORT_PFH		p0_0 		/* PFﾋｰﾀｰ 					*/	/* @010MzDOD[17/02/04] */
#define		PORT_VCH		p0_1 		/* VCﾋｰﾀｰ 					*/	/* @010MzDOD[17/02/04] */
#define		PORT_TANKH		p0_2 		/* 給水ﾀﾝｸﾋｰﾀｰ 				*/	/* @010MzDOD[17/02/04] */
#define		PORT_PIPEH		p0_3 		/* 給水ﾊﾟｲﾌﾟﾋｰﾀｰ 			*/	/* @010MzDOD[17/02/04] */
#define		PORT_VCTOPH		p0_4 		/* VC天面ﾋｰﾀｰ 				*/	/* @010MzDOD[17/02/04] */
#define		PORT_DEFH		p0_5 		/* ﾃﾞﾌﾋｰﾀｰ 					*/	/* @010MzDOD[17/02/04] */
#define		PORT_FRENCHH	p6_1 		/* ﾌﾚﾝﾁｰﾀｰ 					*/	/* @010MzDOD[17/02/04] */
#define		PORT_SCH		p6_2 		/* SCﾋｰﾀｰ 					*/	/* @010MzDOD[17/02/04] */

#ifdef RENESAS_TO_ST32
#else
#define		PORT_2LEVEL		p4_3		/* 2ﾚﾍﾞﾙﾘﾚｰ出力				*/	/* @010MzDOD[17/02/04] */
#endif
#define		PORT_TX			p6_3		/* 通信Tx(P板検査用)		*/

#define		PORT_CODE0		p3_0		/* ﾏｲｺﾝｺｰﾄﾞ出力(bit0)		*/	/* @010MzDOD[17/02/04] */
#define		PORT_CODE1		p3_1		/* ﾏｲｺﾝｺｰﾄﾞ出力(bit1)		*/	/* @010MzDOD[17/02/04] */
#define		PORT_CODE2		p3_5		/* ﾏｲｺﾝｺｰﾄﾞ出力(bit2)		*/	/* @010MzDOD[17/02/04] */
#define		PORT_CODE3		p3_7		/* ﾏｲｺﾝｺｰﾄﾞ出力(bit3)		*/	/* @010MzDOD[17/02/04] */


/* ---FCT検査 IGBG出力--- */
#define		PORT_U			0x04		/* P22						 */	/* @010MzDOD[17/02/04] */
#define		PORT_V			0x02		/* P21						 */	
#define		PORT_W			0x08		/* P23						 */	/* @010MzDOD[17/02/04] */
#define		PORT_X			0x20		/* P25						 */	/* @010MzDOD[17/02/04] */
#define		PORT_Y			0x40		/* P26						 */	/* @010MzDOD[17/02/04] */
#define		PORT_Z			0x80 		/* P27						 */	/* @010MzDOD[17/02/04] */

/****************************************************************/
/*					汎用マクロ定義								*/
/****************************************************************/

typedef unsigned char	UCHAR;	/* 符号無し 8ビット定義 */
typedef unsigned short	USHORT;	/* 符号無し16ビット定義 */
typedef unsigned long	ULONG;	/* 符号無し32ビット定義 */
typedef signed char		CHAR;	/* 符号あり 8ビット定義 */
typedef signed short	SHORT;	/* 符号あり16ビット定義 */
typedef signed long		LONG;	/* 符号あり32ビット定義 */

typedef	struct {			/* 汎用ビットフィールド定義 */
	UCHAR	p0		: 1;	/* 0bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p1		: 1;	/* 1bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p2		: 1;	/* 2bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p3		: 1;	/* 3bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p4		: 1;	/* 4bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p5		: 1;	/* 5bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p6		: 1;	/* 6bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p7		: 1;	/* 7bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
} MB_BTF;

typedef	volatile struct {	/* 最適化抑制ビットフィールド定義 		*/
	UCHAR	p0		: 1;	/* 割り込み処理との排他を目的として		*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p1		: 1;	/* VOLATILE宣言を付けたビットフィールド */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p2		: 1;	/* を準備する。これにより、ANDやOR命令 	*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p3		: 1;	/* によるビット演算が行われなくなり、 	*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p4		: 1;	/* BSETやBCLR命令が生成される 			*/ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p5		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p6		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p7		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
} RB_BTF;

typedef	struct {			/* 汎用ビットフィールド定義 */
	UCHAR	p0		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p1		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p2		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p3		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p4		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p5		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p6		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p7		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p8		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p9		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p10		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p11		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p12		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p13		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p14		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	p15		: 1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
} MBW_BTF;

typedef struct {
	UCHAR	Lo		: 8; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	Hi		: 8; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
} MW_BTF;

typedef union {
	USHORT	word;			/* 2byte */
	UCHAR	byte[2];		/* 1byte + 1byte */
} US_BTF;

typedef struct {			/* 汎用ﾋﾞｯﾄﾌｨｰﾙﾄﾞ定義 ﾆﾌﾞﾙ単位(4bit) */
	UCHAR	L		: 4;	/* L:4bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	H		: 4;	/* H:4bit */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
} MN_BTF;

typedef union {			/* 汎用ビットフィールド定義 */
	UCHAR	bitall;
/*	MB_BTF	bit;*/
} UB_BTF;

typedef union {				/* 汎用2ﾊﾞｲﾄﾀｲﾏ定義 (2byte) */	 /* @000IzDOD[10/09/13]_04 */
	USHORT word;
	struct{
		UCHAR lo;
		UCHAR hi;
	} byte;
} st_2btm;

typedef union {				/* 汎用4ﾊﾞｲﾄﾀｲﾏ定義 (4byte) */	/* @000IzDOD[10/09/13]_04 */
	ULONG dword;
	struct{
		USHORT lo;
		USHORT hi;
	} word;
	struct{
		UCHAR ms;
		UCHAR sec;
		UCHAR min;
		UCHAR hour;
	} byte;
} st_4btm;

typedef struct {			/* 汎用ﾋﾞｯﾄﾌｨｰﾙﾄﾞ定義 (2bit) */
	UCHAR	L0		: 2; /*　コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	L1		: 2; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	H0		: 2; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	UCHAR	H1		: 2; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
} M2_BTF;

#define	CLR		0			/* フラグ状態用(ＯＦＦ) */
#define	SET		!CLR		/* フラグ状態用(ＯＮ) */

#define SEL_P0G		0x3F	/* AD変換 ﾎﾟｰﾄｸﾞﾙｰﾌﾟ0選択(b7-6 "00") */	/* @002MzDOD[13/03/22] */
#define SEL_P1G		0x40	/* AD変換 ﾎﾟｰﾄｸﾞﾙｰﾌﾟ1選択(b7-6 "01") */	/* @002MzDOD[13/03/22] */



#endif /* SYSTYPEM_H */
