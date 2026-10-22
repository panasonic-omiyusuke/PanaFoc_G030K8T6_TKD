#ifndef ANGLECONTROL_H
#define ANGLECONTROL_H


/********************************************/
/*		マクロ定義							*/
/********************************************/

#define DEGREE_60_RESOLUTION_BIT (10) /* 通電角60度の分解能 10なら60/2^10[deg] */
/* vi phase diff ctrl */
#define PANGL_CNT_LEAD			(500)									/* 広角までの連続進み回数 */
#define PANGL_CNT_DELAY			(500)									/* 侠角までの連続遅れ回数 */
#define PANGL_CNT_DELAY1PULSE	(1)										/* 侠角までの遅れ1パルス発生時の連続回数 */

enum AngleModeTag {				/* 通電角の状態定義 */
    kGapAngleUnderMinMode,			/* gap＆下限未満 */
    kGapAngleMinMode,				/* gap＆下限 */
    kGapAngleMode,					/* gap */
    kBasicAngleMinMode,				/* basic(120deg)＆下限 */
    kBasicAngleMode,				/* basic */
    kLapAngleMinMode,				/* lap＆下限 */
    kLapAngleMode,					/* lap */
    kLapAngleMaxMode,				/* lap＆上限 */
    kLapAngleOverMaxMode,			/* lap＆上限より大きい */
};

typedef struct AngleDataTag {
    UI08 mode;
    UI16 overlap;
    UI16 lap_start;
    UI16 nonlap_start;
} sAngleDataTag;

/********************************************/
/*		外部変数参照						*/
/********************************************/
extern sAngleDataTag angle;					/* 後述のINLINE関数で使用 */
extern unsigned8	ucTYPE_overlap;

/* 以下、要検討 */
/* 通信で出力用に global化  staticにしたい */
extern UI16 usAGC_cdang;				/* 通電角度：conduction_angleから名称変更 *//* 1536 ~ 2560 */

/********************************************/
/*		公開関数    						*/
/********************************************/
void AngleControl(UI08 is_duty_full);
void InitAngleControl(void);



/********************************************/
/*		変数宣言							*/
/********************************************/
extern unsigned8	ucTYPE_overlap;



/********************************************/
/*		関数宣言							*/
/********************************************/
/* static inline �֐���static inline �֐��Ƃ��đ���c�t�@�C��������Ăяo�����߂ɂ́Ah�t�@�C���ɋL�q���Ȃ���΂Ȃ�Ȃ��H */
/* static inline �֐���static inline �֐��Ƃ��đ���c�t�@�C��������Ăяo�����߂ɂ́Ah�t�@�C���ɋL�q���Ȃ���΂Ȃ�Ȃ��H */
static inline  UI16 Deg60RatioAngle2ModeTime(UI32 ratio_angle, UI16 degree60_time){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (UI16)((ratio_angle * degree60_time) >> DEGREE_60_RESOLUTION_BIT);
}

static inline  UI08 IsAngleMin(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
	if( angle.mode == kBasicAngleMinMode ){
		return TRUE;
	}
	if( angle.mode <= kGapAngleMinMode ){
		return TRUE;   /* kGapAngleMinMode or kGapAngleUnderMinMode */
	}
	return FALSE;
}

static inline UI08 IsAngleBasic(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (angle.overlap == 0);
}

static inline UI08 IsAngleLapOrGap(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return !IsAngleBasic();
}

static inline UI08 IsAngleGap(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (angle.mode <= kGapAngleMode);
}

static inline UI08 IsAngleLapMax(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (angle.mode == kLapAngleMaxMode);
}
static inline UI08 IsAngleLap(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (angle.mode >= kLapAngleMinMode);
}
static inline UI08 IsAngleMinLap(void){ /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    return (angle.mode == kLapAngleMinMode);
}






#endif  /* #ifndef ANGLECONTROL_H */
