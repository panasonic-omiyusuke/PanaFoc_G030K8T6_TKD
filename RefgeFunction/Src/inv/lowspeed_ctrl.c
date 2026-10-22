/**************************************************************************
<FileName>		:lowspeed_ctrl.c
<Function>		:低速制御中処理
<Program>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:Ver1.00
<Copyright>		:Panasonic Co. , Ltd.
 **************************************************************************/

/********************************************
		インクルードファイル
 ********************************************/
#include "mc_type.h"
#include "lowspeed_ctrl.h"
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "main.h"
/*******************************************
		マクロ定義
 *******************************************/

/*******************************************
		変数宣言
 *******************************************/
uint8_t ucg_LOW_ElecAngleSector = 0;    /* 電気角を60度毎に6分割したときのセクタ番号を格納する*/
uint16_t ung_LOW_MecAveSpeed = 0;       /* 機械角周期単位で平均化されたモータ推定速度を格納する */
uint8_t ucg_LOW_HogoFlg = 0;            /* 各ビットに対応する保護状態を格納する:Hi→保護状態、Lo→非保護状態 */
		/*   ---- ---1 : 速度うねり保護.  */
		/*   ---- --1- : 異常速度検知保護.  */
		/*   ---- -1-- : unused.  */
		/*   ---- 1--- : unused.  */
		/*   ---1 ---- : unused. */
		/*   --1- ---- : unused. */
		/*   -1-- ---- : unused. */
		/*   1--- ---- : unused.  */
uint8_t ucg_LOW_HogoSttfFlg = 0;        /* 保護フラグに格納された保護情報を保持する:Hi→保護状態、Lo→非保護状態 */
		/*   ---- ---1 : 速度うねり保護.  */
		/*   ---- --1- : 異常速度検知保護.  */
		/*   ---- -1-- : unused.  */
		/*   ---- 1--- : unused.  */
		/*   ---1 ---- : unused. */
		/*   --1- ---- : unused. */
		/*   -1-- ---- : unused. */
		/*   1--- ---- : unused.  */
uint8_t ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_OUT_OF_RANGE;  /* 低速制御中の制御モードを格納する */
extern uint16_t hRampMecRps;
extern uint8_t u8g_SPD_cmd;
extern uint8_t u8g_KYUYUflg;
uint8_t ucg_TDCflg = 0;  /* 上死点フラグ */
uint8_t FirstcallAtEst_flg; /* 推定処理初回呼び出しフラグ */
/*******************************************
		プロトタイプ宣言
 *******************************************/

/**************************************************************************
<Title>			:vLowSpeed_JudgeSpeedUneri
<Function>		:速度うねり判定処理
<Description>	:機械角平均速度が目標速度に対して±1.0rpsより大きい場合、保護フラグをHiにする
<Input>			:
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_JudgeSpeedUneri(void){
    uint16_t unl_MecAveSpeedError;  /* 機械角周期平均速度と速度指令の絶対値差分を格納する */

    /* 機械角平均速度と速度指令の絶対値差分を計算 */
    if((u8g_SPD_cmd * 10) >= ung_LOW_MecAveSpeed){
        unl_MecAveSpeedError = u8g_SPD_cmd * 10 - ung_LOW_MecAveSpeed;
    }else{
        unl_MecAveSpeedError = ung_LOW_MecAveSpeed - u8g_SPD_cmd * 10;
    }

    if(unl_MecAveSpeedError > SPEED_UNERI_THRESHOLD){   /* 速度差の絶対値が速度うねり判定閾値より大きければ */
        ucg_LOW_HogoFlg |= 0x01;                        /* 速度うねり発生　→　0bitをHiに設定し、速度うねり保護情報を格納する */
    }else{                                              /* 速度差の絶対値が速度うねり判定閾値以下であれば */
        ucg_LOW_HogoFlg &= ~0x01;                       /* 速度うねり未発生　→　0bitをLoに設定し、速度うねり保護情報をクリア */
    }
}

/**************************************************************************
<Title>			:vLowSpeed_JudgeAbnormalSpeed
<Function>		:異常速度判定処理
<Description>	:推定速度が目標速度に対して±5.5rpsより大きい場合、保護フラグをHiにする
<Input>			:Estspeed_q0(固定少数点位置:0bit目)
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_JudgeAbnormalSpeed(int16_t Estspeed_q0){
    uint16_t unl_SpeedError;    /* 推定速度と速度指令の絶対値差分を格納する */

    /* 推定速度と速度指令の絶対値差分を計算 */
    if((u8g_SPD_cmd * 10) >= Estspeed_q0){
        unl_SpeedError = u8g_SPD_cmd * 10 - Estspeed_q0;
    }else{
        unl_SpeedError = Estspeed_q0 - u8g_SPD_cmd * 10;
    }

    if(unl_SpeedError > SPEED_ABNORMAL_THRESHOLD){      /* 速度差の絶対値が異常速度判定閾値より大きければ */
        ucg_LOW_HogoFlg |= 0x02;                        /* 異常速度発生　→　1bitをHiに設定し、異常速度保護情報を格納する */
    }else{                                              /* 速度差の絶対値が異常速度判定閾値以下であれば */
        ucg_LOW_HogoFlg &= ~0x02;                       /* 異常速度未発生　→　1bitをLoに設定し、異常速度保護情報をクリア */
    }
}

/**************************************************************************
<Title>			:vLowSpeed_HogoReleaseWait
<Function>		:保護解除待機処理
<Description>	:保護解除5分待機処理　※引数が1の場合は初期化処理を行う
<Input>			:Mode
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_HogoReleaseWait(uint8_t Mode){
    static uint32_t ull_5minCnt = 0;    /* 保護保持フラグ解除用　5分待機カウントを格納する */

    /* 初期化モードの場合　→　保護関連変数の初期化処理 */
    if(Mode == INIT_MODE){
        ucg_LOW_HogoFlg = 0;
        ucg_LOW_HogoSttfFlg = 0;
        ull_5minCnt = 0;
        return;
    }

    /* 通常（非初期化）モードの場合　→　保護解除5分待機処理 */

    /* 保護発生履歴を保持し、異常消滅後も5分間は省エネ制御を禁止する */
    if(ucg_LOW_HogoFlg != 0){
        ucg_LOW_HogoSttfFlg |= ucg_LOW_HogoFlg;
    }
    /* 保護解除待ち処理 */
    if(ucg_LOW_HogoSttfFlg != 0){               /* 保護発生から5分経過するまで省エネ制御を禁止する */
        if(ull_5minCnt >= HOGO_CLR_CNT_TH_1MS){ /* 保護解除条件の判定を開始する */
            if(ucg_LOW_HogoFlg == 0){           /* 現在保護が未発生 */
                ucg_LOW_HogoSttfFlg = 0;        /* 保持された保護情報をクリア */
                ull_5minCnt = 0;                /* 5分待機カウントをクリア */
            }else{
                /* 現在も保護発生中　→　保護フラグLoになるまで待機 */
            }
        }else{                                  /* 保護解除禁止期間の経過時間を計測する */
            ull_5minCnt ++;
        }
    }else{                                      /* 保護解除待機中ではないためタイマをリセットする */
        ull_5minCnt = 0;
    }
}

/**************************************************************************
<Title>			:vLowSpeed_Task_1ms
<Function>		:低速制御中　1ms処理
<Description>	:低速制御中の1ms処理を行う関数を呼び出す
<Input>			:Estspeed_q0(固定少数点位置:0bit目), hTargetSpeed
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_Task_1ms(int16_t Estspeed_q0, int16_t hTargetSpeed){
    static uint16_t unl_5secCnt = 0;    /* 速度指令到達後の5秒待機カウントを格納する */

    /* 制御状態に応じた処理 */
    switch(ucg_LOW_LowSpdCtrlState){
        /****M0 低速制御対象外：低速制御対象外のため省エネ制御を禁止する****/
        case LOWSPD_STATE_OUT_OF_RANGE:
            /* no break */
        /****M1 加減速中：速度変化中のため保護判定および省エネ判定は行わない****/
        case LOWSPD_STATE_ACCEL_DECEL:
            unl_5secCnt = 0;
            vLowSpeed_HogoReleaseWait(INIT_MODE);
            break;
        /****M2 速度安定待ち：速度指令到達後の安定化時間を待つ****/
        case LOWSPD_STATE_WAIT_STABLE:
            unl_5secCnt++;
            vLowSpeed_HogoReleaseWait(INIT_MODE);
            if(unl_5secCnt >= SPD_STABLE_WAIT_CNT_TH_1MS){      /* 速度指令到達後に十分な安定化時間が経過したため省エネ制御許可判定モードに遷移 */
                unl_5secCnt = SPD_STABLE_WAIT_CNT_TH_1MS;
            }
            break;
        /****M3 省エネ制御許可判定：保護状態を監視しながら省エネ制御を許可する***/
        case LOWSPD_STATE_ECO_ENABLE:
            /* no break */
        /****M4 保護保持・解除待ち：保護発生後5分経過するまで省エネ制御を禁止する****/
        case LOWSPD_STATE_PROTECT_HOLD:
            vLowSpeed_JudgeSpeedUneri();
            vLowSpeed_JudgeAbnormalSpeed(Estspeed_q0);
            vLowSpeed_HogoReleaseWait(NORMAL_MODE);             /* 保護解除まで最小5分待機する */
            break;
        /****状態破損時のフェールセーフ処理****/
        default:
            unl_5secCnt = 0;
            vLowSpeed_HogoReleaseWait(INIT_MODE);
            break;
    }

    /* 制御状態遷移 */
    if(hTargetSpeed > LOWSPD_CONTROL_START_SPEED){                  /* 目標速度が低速制御対象範囲外の場合は対象外モードへ遷移 */
        ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_OUT_OF_RANGE;
    }else if(u8g_SPD_cmd != hRampMecRps){                           /* 低速制御範囲かつ速度変化中は加減速モードへ遷移 */
        ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_ACCEL_DECEL;
    }else{
        switch(ucg_LOW_LowSpdCtrlState){
            /****M0 低速制御対象外：低速制御対象外のため省エネ制御を禁止する****/
            case LOWSPD_STATE_OUT_OF_RANGE:
                if(hTargetSpeed <= LOWSPD_CONTROL_START_SPEED){     /* 目標速度が低速制御対象範囲内になった場合は加減速モードへ遷移 */
                    ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_ACCEL_DECEL;
                }
                break;
            /****M1 加減速中：速度変化中のため保護判定および省エネ判定は行わない****/
            case LOWSPD_STATE_ACCEL_DECEL:
                if(u8g_SPD_cmd == hRampMecRps){                     /* 速度指令への追従が完了した場合は速度安定待ちモードへ遷移 */
                    ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_WAIT_STABLE;
                }
                break;
            /****M2 速度安定待ち：速度指令到達後の安定化時間を待つ****/
            case LOWSPD_STATE_WAIT_STABLE:
                if(unl_5secCnt >= SPD_STABLE_WAIT_CNT_TH_1MS){      /* 速度指令到達後に十分な安定化時間が経過したため省エネ制御許可判定モードに遷移 */
                    ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_ECO_ENABLE;
                }
                break;
            /****M3 省エネ制御許可判定：保護状態を監視しながら省エネ制御を許可する***/
            case LOWSPD_STATE_ECO_ENABLE:
                if(ucg_LOW_HogoSttfFlg != 0){                       /* 保護発生履歴があるため省エネ制御を禁止する */
                    ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_PROTECT_HOLD;
                }
                break;
            /****M4 保護保持・解除待ち：保護発生後5分経過するまで省エネ制御を禁止する****/
            case LOWSPD_STATE_PROTECT_HOLD:
                if(ucg_LOW_HogoSttfFlg == 0){                       /* 保護解除条件が成立したため省エネ制御許可判定モードへ復帰する */
                    ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_ECO_ENABLE;
                }
                break;
            /****状態破損時のフェールセーフ処理****/
            default:
                ucg_LOW_LowSpdCtrlState = LOWSPD_STATE_OUT_OF_RANGE;/* 状態異常発生時は安全側として低速制御対象外へ遷移する */
                break;
        }
    }
}

/**************************************************************************
<Title>			:vLowSpeed_ElecAngleToSector
<Function>		:角度セクタ判定処理
<Description>	:電気角を60度毎に6分割し、セクタ番号を算出する
<Input>			:elecAngleQ15_pm180(固定少数点位置:15bit目) ※電気角：-180°～+180°
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_ElecAngleToSector(int16_t elecAngleQ15_pm180)
{
  uint16_t unl_ElecAngleQ15_360;   /* 固定少数点位置が15bit目で定義された電気角を0～65535の1周表現に変換した値 */

    /*
   * 角度範囲変換
   *   -180° (-32767) → 1
   *     0° (     0) → 32768
   *  +180° ( 32767) → 65535
   */
    unl_ElecAngleQ15_360 = (uint16_t)(elecAngleQ15_pm180 + ELEC_ANGLE_HALF_U16);

    /* セクタ算出 */
    ucg_LOW_ElecAngleSector = (uint8_t)(unl_ElecAngleQ15_360 / SECTOR_WIDTH_U16);   /* SECTOR_WIDTH_U16 = 360° / セクタ数（u16換算） */

    /* 上限クリップ */
    if (ucg_LOW_ElecAngleSector > SECTOR_MAX){
        ucg_LOW_ElecAngleSector = SECTOR_MAX;
    }
}

/**************************************************************************
<Title>			:vLowSpeed_CalcAvgMechSpeed
<Function>		:平均機械角速度計算処理
<Description>	:機械角を18分割し、各セクタ間のカウント数を配列に格納し、平均機械角速度を計算する
<Input>			:
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_CalcAvgMechSpeed(void){
    static uint8_t ucl_ElecAngleSectorDl = 0;               /* 前回の電気角セクタ番号を格納する */
    static uint8_t ucl_bufcnt = 0;                          /* 配列バッファカウンタを格納する */
    static uint8_t ucl_period60cnt = 0;                     /* 現在の電気角セクタの継続時間カウンタ */
    static uint8_t ucl_period60cnt_buf[MEC_PERIOD_SECTOR_NUM] = {0};   /* 過去の電気角セクタの継続時間カウント数を格納する */
    static uint16_t unl_MecPeriodCnt = 0;                   /* 機械角1周期分の継続時間カウント数を格納する */
    uint8_t i;  /* 配列加算用ループ変数 */

    if(ucl_period60cnt >= 255){
        ucl_period60cnt = 255; /* カウンタ上限クリップ */
    }else{
        ucl_period60cnt ++; /* セクタ切替までカウンタ加算 */
    }

    if(ucg_LOW_ElecAngleSector != ucl_ElecAngleSectorDl){   /* セクタ切替が発生した場合 */
        /* 配列バッファカウンタを進める */
        ucl_bufcnt ++;
        if(ucl_bufcnt >= MEC_PERIOD_SECTOR_NUM){
            ucl_bufcnt = 0;
        }

        /* 電気角60度間のカウント数を配列に格納 */
        ucl_period60cnt_buf[ucl_bufcnt] = ucl_period60cnt;
        ucl_period60cnt = 0;

        /* 機械角周期分のカウント数を更新 */
        unl_MecPeriodCnt = 0;
        for(i = 0 ; i < MEC_PERIOD_SECTOR_NUM ; i++){
            unl_MecPeriodCnt += ucl_period60cnt_buf[i];
        }

        /* 機械角速度計算　小数点第一位まで計算 */
        if(unl_MecPeriodCnt > 0){
            ung_LOW_MecAveSpeed = 10 * CURRENT_LOOP_HZ / unl_MecPeriodCnt;
        }
    }
    ucl_ElecAngleSectorDl = ucg_LOW_ElecAngleSector;    /* 前回のセクタ番号を保存 */
}

/**************************************************************************
<Title>			:vLowSpeed_EstTDCpoint
<Function>		:上死点位置推定処理
<Description>	:機械角周期内で最低速度時の機械角位置を推定し、その情報から上死点位置を推定する
<Input>			:hMeasuredSpeed_q0(固定少数点位置:0bit目)
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_EstTDCpoint(int16_t hMeasuredSpeed_q0){
    static uint8_t ucl_ElecAngleSectorDl2 = 0;               /* 前回の電気角セクタ番号を格納する */
    static uint8_t ucl_bufcnt2 = 0;                          /* 配列バッファカウンタを格納する */
    static int32_t ll_EstValueMechDl_q4 = 0;
    int16_t hMeasuredSpeed_q4;
    int32_t acc;
    static int32_t ll_EstSpeed60deg_buf[MEC_PERIOD_SECTOR_NUM] = {0};
    int32_t MinSpeed;
    static uint8_t MinSpeeedbufCnt = 0;
    static uint8_t EstFlg = 0;
    static uint8_t TDCbufcnt = 0;
    static uint8_t ucl_TDCareaflg = 0;
    static uint8_t ucl_TDCareaflg_dl = 0;

    /* 給油運転中　➡　各セクタの切替時の平均推定速度を計算する */
    if(u8g_KYUYUflg == 1){
        EstFlg = 0;
        if(ucg_LOW_ElecAngleSector != ucl_ElecAngleSectorDl2){   /* セクタ切替が発生した場合 */
            /* 配列バッファカウンタを進める */
            ucl_bufcnt2 ++;
            if(FirstcallAtEst_flg == 1){
                FirstcallAtEst_flg = 0;
                ucl_bufcnt2 = ucg_LOW_ElecAngleSector;
            }
            if(ucl_bufcnt2 >= MEC_PERIOD_SECTOR_NUM){
                ucl_bufcnt2 = 0;
            }

            /* 推定速度のIIRフィルタ処理 */
            hMeasuredSpeed_q4 = hMeasuredSpeed_q0 << 4;   /* Q0 → Q4 */
            ll_EstValueMechDl_q4 = ll_EstSpeed60deg_buf[ucl_bufcnt2];   /* 前回値を保存 */
            acc = (int32_t)(IIR_ONE_Q15 - IIR_ALPHA_Q15) * ll_EstValueMechDl_q4;
            acc += (int32_t)IIR_ALPHA_Q15 * hMeasuredSpeed_q4;
            ll_EstSpeed60deg_buf[ucl_bufcnt2] = (int32_t)(acc / 32768); /* Q19 → Q4 */
        }

    /* 給油運転終了後　➡　最小速度時の機械角（18分割値）を求め、上死点位置を推定する */
    }else{
        FirstcallAtEst_flg = 1;
        /* 最小速度時の機械角（18分割値）を求める（給油運転後、初回呼び出し時のみ） */
        if((EstFlg == 0)&&(u8g_SPD_cmd != 0)){
            MinSpeed = ll_EstSpeed60deg_buf[0];
            MinSpeeedbufCnt = 0;
            for(int i = 0; i < MEC_PERIOD_SECTOR_NUM; i++){
                if(ll_EstSpeed60deg_buf[i] < MinSpeed){
                    MinSpeed = ll_EstSpeed60deg_buf[i];
                    MinSpeeedbufCnt = i;
                }
            }
            EstFlg = 1;
        /* 不要な変数は初期化（最小速度時の機械角位置が推定完了している場合） */
        }else{
            ll_EstValueMechDl_q4 = 0;
            for(int i = 0; i < MEC_PERIOD_SECTOR_NUM; i++){
                ll_EstSpeed60deg_buf[i] = 0;
            }
        }

        /* 上死点位置の推定処理（最小速度時の機械角位置が推定完了している場合） */
        if(EstFlg == 1){
            /* セクタ単位で上死点エリアを判定 */
            if(ucg_LOW_ElecAngleSector != ucl_ElecAngleSectorDl2){   /* セクタ切替が発生した場合 */
                ucl_bufcnt2 ++;
                if(ucl_bufcnt2 >= MEC_PERIOD_SECTOR_NUM){
                    ucl_bufcnt2 = 0;
                }
                TDCbufcnt = MinSpeeedbufCnt + TDC_CORRECT_BUFCNT;
                if(TDCbufcnt >= MEC_PERIOD_SECTOR_NUM){
                    TDCbufcnt -= MEC_PERIOD_SECTOR_NUM;
                }
                if(TDCbufcnt == ucl_bufcnt2){
                    ucl_TDCareaflg = 1;
                }else{
                    ucl_TDCareaflg = 0;
                }
            }
            /* 上死点エリアに移行したタイミングのみ上死点検知フラグをHiにする */
            if((ucl_TDCareaflg == 1)&&(ucl_TDCareaflg_dl == 0)){
                ucg_TDCflg = 1;
            }else{
                ucg_TDCflg = 0;
            }
        }
    }
    ucl_TDCareaflg_dl = ucl_TDCareaflg;
    ucl_ElecAngleSectorDl2 = ucg_LOW_ElecAngleSector;    /* 前回のセクタ番号を保存 */
}
/**************************************************************************
<Title>			:vLowSpeed_Task_200us
<Function>		:低速制御中　200μs処理
<Description>	:低速制御中の200μs処理を行う関数を呼び出す
<Input>			:hElAngle_q15(固定少数点位置:15bit目), hMeasuredSpeed_q0(固定少数点位置:0bit目)
<Return>		:
<Author>		:Omi
<Date>			:2026.7.17
<Update>		:
<Copyright> 	:Panasonic Co. , Ltd.
**************************************************************************/
void vLowSpeed_Task_200us(int16_t hElAngle_q15, int16_t hMeasuredSpeed_q0){
    vLowSpeed_ElecAngleToSector(hElAngle_q15);   /* 角度セクタ判定処理 */
    vLowSpeed_CalcAvgMechSpeed();                /* 平均機械角速度計算処理 */
    vLowSpeed_EstTDCpoint(hMeasuredSpeed_q0);   /* 上死点位置推定処理 */
}

/****END OF FILE****/
