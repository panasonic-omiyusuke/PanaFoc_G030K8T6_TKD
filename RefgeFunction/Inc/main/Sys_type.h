/**************************************************************************
<File Name>		:sys_type.h
<Function>		:記述標準化・簡略化定義ファイル
<Program>		:
<Author>		;大内山 智則 -Outi
<Date>			:07/01/10
<Update>		:Ver REF081-0.00 2008-04-22 -Outi ﾙﾈｻｽMCU置換対応
<Copyright>		:Matsushita Refrigeration Co.,Ltd.
**************************************************************************/
#ifndef SYS_TYPE_H
#define SYS_TYPE_H
#include "stm32g0xx_hal.h"
#include "vipdc.h"
typedef  uint8_t  unsigned8;	/* 符号無し 8ビット定義 */
typedef  uint16_t   unsigned16;	/* 符号無し16ビット定義 */
#ifdef UNIT_TEST
typedef	signed		int		SI32;
typedef	unsigned	int		UI32;
#else
typedef	int32_t		SI32;
typedef	uint32_t	UI32;
#endif
typedef	int16_t		SI16;
typedef	uint16_t	UI16;
typedef	int8_t		SI08;
typedef	uint8_t		UI08;

/*""VAL DEF""*/
union	SDL	{                           /* signed 32bit RAM */
	SI32	lng;                                    /* 32bit access */
	struct	{
		UI16	lo;                                 /* 16bit access L */
		SI16	hi;                                 /* 16bit access H */
	}wrd;
};
union	UDL	{                           /* unsigned 32bit RAM */
	UI32	lng;                                    /* 32bit access */
	struct	{
		UI16	lo;                                 /* 16bit access L */
		UI16	hi;                                 /* 16bit access H */
	}wrd;
};
union	UDS	{                           /* unsigned 16bit RAM */
    UI16    wrd;
	struct	{
		UI08	lo;                                 /* 16bit access L */
		UI08	hi;                                 /* 16bit access H */
	}byt;
    struct  {
        UI08    b0:1;                                 /* 8bit access LL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b1:1;                                 /* 8bit access LH */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b2:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b3:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b4:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b5:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b6:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b7:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b8:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b9:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b10:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b11:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b12:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b13:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b14:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b15:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    }bit;
};
typedef     union   SDL     SD32;
typedef     union   UDL     UD32;
typedef     union   UDS     UD16;

union	UDC	{                           /* signed 16bit RAM */
    UI08    byt;
    struct  {
        UI08    b0:1;                                 /* 8bit access LL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b1:1;                                 /* 8bit access LH */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b2:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b3:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b4:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b5:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b6:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
        UI08    b7:1;                                 /* 8bit access HL */ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    }bit;
};
typedef     union   UDC     UD08;

typedef enum StateTag{							/* @007MzDOD[16/12/21] */ /* 差圧起動 */
	D_stop,
	D_standby,
	D_position,
	D_forced,
	D_running,
	D_vector
} eStateTag;

/********************************************************
*  declare SFR bit										*
********************************************************/
struct bit_def {
	char  b0:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b1:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b2:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b3:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b4:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b5:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b6:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	char  b7:1; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
};
union byte_def{
  struct bit_def bit;
  unsigned char	byte;
};

enum SpeedControlModeTag {
    kSpeedMaintenanceMode,
    kSpeedDecelerationMode,
    kSpeedAccelerationMode,
};
enum SpeedDifferentialModeTag {
    kSpeedDifferentialMoreThan2Rps,
    kSpeedDifferentialSome0,
    kSpeedDifferentialMoreThan1Rps,
    kSpeedDifferentialLittle4,
    kSpeedDifferentialLittle3,
    kSpeedDifferentialLittle2,
    kSpeedDifferentialLittle1,
    kSpeedDifferentialLittle0,
};
struct tSpeedDataTag {
	enum SpeedControlModeTag control_mode;
	enum SpeedControlModeTag pre_control_mode;
	enum SpeedDifferentialModeTag differential_mode;
};

enum eCarrierModeTag {
	kCarrierInit,
	kCarrierLowSpeed,
	kCarrierHighSpeed,
	kCarrierMax,
};


#define FALSE 					(0)
#define TRUE 					(1)

#define		UCHAR_MAX		0xFF				/* UCHAR型MAX *//* 指摘対応-200420 */


#define STATE_ON  				(1)
#define STATE_OFF 				(0)

#ifdef UNIT_TEST
#define INLINE  inline static
#define asm(a)
#define STATIC
#else
#define INLINE  static inline
#define STATIC static
#endif


#endif /* SYS_TYPE_H */
