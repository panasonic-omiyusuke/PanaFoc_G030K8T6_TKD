/******************************************************************
<File Name> ：motor_error.c
<Function> 	：马达异常相关函数
<Program> 	：
<Author> 	：董明鉴
<Date> 		：2024/10/10
<Update> 	：
<Copyright> ：Panasonic Co. , Ltd.
******************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "motor_error.h"
#include "mc_tasks.h"
#include "led.h"
#include "usart_control.h"
#include "motor_process.h"
#include "Adpdc.h"   /* chk_spike, kWaitFreeRunZeroCross を使う場合 */
#include "brake.h"
#include "Hogo.h"
#include "InvCommonFunc.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t u16l_pwr_rcv_cnt  = 0;
static uint16_t u16l_pwr_blce_cnt = 0;
uint16_t u16s_vbus = 0x00;
volatile uint16_t u16g_MOTOR_pwr_avg = 0u; /*通信参照用の平均電力[W]*/
static float fs_currA = 0.0;
static float fs_currB = 0.0;
static float fs_currC = 0.0;
uint8_t u8g_hog_5minstop_cnt = 0;          /*保護停止5min回数カウント(ﾘｾｯﾄ処理用)*/
uint16_t debug_u16t_vbus_min = 0;          /*デバッグ用：不足電圧保護閾値*/
uint16_t debug_u16t_vbus_min_unlock = 0;   /*デバッグ用：不足電圧保護解除閾値*/
uint16_t debug_u16t_vbus_max = 0;          /*デバッグ用：過電圧保護閾値*/
uint16_t debug_u16t_vbus_max_unlock = 0;   /*デバッグ用：過電圧保護解除閾値*/
uint16_t debug_u16l_ov_drop_delay_cnt = 0; /*デバッグ用：kDropped移行後のOV閾値遅延カウンタ*/

/* Private function prototypes -----------------------------------------------*/
static void Err_Jdg_Ipm(uint16_t* u16p_errflg);
static void Err_Jdg_OverVolt(uint16_t* u16p_errflg);
static void Err_Jdg_UnderVolt(uint16_t* u16p_errflg);
static void Err_Jdg_OutCtrl(uint16_t* u16p_errflg);
static void Err_Jdg_OverCur(uint16_t* u16p_errflg);
static void Err_Jdg_PhaseLoss(uint16_t* u16p_errflg);
static void Err_Jdg_OverPwr(uint16_t* u16p_errflg);
static void Err_Jdg_OffsetErr(uint16_t* u16p_errflg);

static void Motor_Curr_get(void);
static void Motor_Err_RemainCnt(void);

MOTOR_ERR_JDG st_motor_err = 
{
    /* 异常标志定义 */
    .u16_errflg = &u16g_MOTOR_errflg,
    /* 函数定义 */
    {
        &Err_Jdg_Ipm,
        &Err_Jdg_OverVolt,
        &Err_Jdg_UnderVolt,
        &Err_Jdg_OutCtrl,
        &Err_Jdg_OverCur,
        &Err_Jdg_PhaseLoss,
        &Err_Jdg_OverPwr,
        &Err_Jdg_OffsetErr
    }
};

/******************************************************************
<Title> 		：Motor_Err_Chk(void)
<Function> 		：马达异常检查
<Description> 	：10ms主周期执行，检查异常标志，异常5min等待计时判定
<Input> 		：none
<Return> 		：True:有异常/异常持续中    False:无异常/异常持续时间已过
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：2025/3/6 董明鉴 追加OLP恢复判定
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
uint8_t Motor_Err_Chk(void)
{
    uint8_t retval = FALSE;
    /*---------有无异常检查--------------*/
    if((MCI_GetCurrentFaults(&Mci[M1]) != MC_NO_FAULTS))
    {
        retval = TRUE;
    }
    /*----------异常持续检查-----------*/
    Motor_Err_RemainCnt();      /*异常持续时间计数*/
    if(u8g_ERR_remain == TRUE)
    {
        retval = TRUE;
    }
    return retval;
}

/******************************************************************
<Title> 		：Motor_Err_Jdg(void)
<Function> 		：马达异常检测
<Description> 	：1ms中断中执行，各异常判定，异常标志置位
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
void Motor_Err_Jdg(void)
{
    uint8_t i = 0;

    /*--------------母线电压获取------------------------*/
    u16s_vbus = VBS_GetAvBusVoltage_V(&(BusVoltageSensor_M1._Super));   /*读取母线电压平均值 以V为单位*/

    /*--------------电流值获取--------------------------*/
    Motor_Curr_get();

    /*--------------逐一检查异常------------------------*/
    for ( i = 0; i < ERR_NUM; i++)
    {
        st_motor_err.errjdg[i](st_motor_err.u16_errflg);
    }

    /*--------------st_motor_err.errjdgをucHOGに反映----------------*/
    SET_MotorErr_ucHOG_movF(); /*根据异常标志设置Hogo.c中的异常状态位*/

    /*--------------异常检出 关闭PWM--------------------*/
    if(MCI_GetFaultState(&Mci[M1]) != (uint32_t)MC_NO_FAULTS)
    {
        if(u8g_RUN_state != STATE_ERRSTOP)
        {
            u8g_ERR_remain = TRUE;
        }
        PWMC_SwitchOffPWM(pwmcHandle[M1]);              /*关闭PWM输出*/

        FOC_Clear(M1);
    }
}

/******************************************************************
<Title> 		：Err_Jdg_Ipm(uint16_t* u16p_errflg)
<Function> 		：IPM异常（硬件过流）判定
<Description> 	：检查Fault端口信号
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_Ipm(uint16_t* u16p_errflg)
{
    /*检测到Fault信号*/
    if(TRUE == pwmcHandle[M1]->driverProtectionFlag)
    {
        pwmcHandle[M1]->driverProtectionFlag = FALSE;
        (*u16p_errflg) |= F_ERR_IPMERR;
        u8g_LED_err = LED_ERR_HWCU;
        MCI_FaultProcessing(&Mci[M1],MC_DP_FAULT,~MC_DP_FAULT);
    }
}

/******************************************************************
<Title> 		：Err_Jdg_OverVolt(uint16_t* u16p_errflg)
<Function> 		：过电压检测
<Description> 	：过电压检测及恢复判定
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：2025/1/21     董明鉴      检测由中频处理转到载波中断中
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_OverVolt(uint16_t* u16p_errflg)
{
    static uint8_t u8l_Vbus_over_cnt = 0x00;            /*异常计数*/
    static uint8_t u8l_Vbus_over_reccnt = 0x00;         /*恢复计数*/
    static uint16_t u16l_ov_drop_delay_cnt = 0u;        /*kDropped移行後のOV閾値遅延*/
    uint16_t u16t_vbus_max = 0x00;
    uint16_t u16t_vbus_max_unlock = 0x00;

    if (ucTYPE_2level == 1)
    {
        /* kDroppedまたはkDropping以外では遅延カウンタをリセット */
        if (pam.duty_mode != kDropped)
        {
            u16l_ov_drop_delay_cnt = 0u;
        }

        switch (pam.duty_mode)
        {
            case kDropping:
            case kWaitBoost:
            case kBoosting:
            case kStableBoost:
            case kBoosted:
                u16t_vbus_max = OVERVOLT_THRESHOLD_2LVON;
                u16t_vbus_max_unlock = OVERVOLT_UNLOCK_THRESHOLD_2LVON;
                break;

            case kDropped:
                /* 実リレーOFF遅れ吸収: 一定時間は2LVON閾値を維持(kDroppedから5000ms) */
                if (u16l_ov_drop_delay_cnt < OVERVOLT_2LVOFF_APPLY_DELAY_MS)
                {
                    u16l_ov_drop_delay_cnt++;
                    u16t_vbus_max = OVERVOLT_THRESHOLD_2LVON;
                    u16t_vbus_max_unlock = OVERVOLT_UNLOCK_THRESHOLD_2LVON;
                }
                else
                {
                    u16t_vbus_max = OVERVOLT_THRESHOLD_2LVOFF;
                    u16t_vbus_max_unlock = OVERVOLT_UNLOCK_THRESHOLD_2LVOFF;
                }
                break;

            default:
                u16t_vbus_max = OVERVOLT_THRESHOLD_2LVON;
                u16t_vbus_max_unlock = OVERVOLT_UNLOCK_THRESHOLD_2LVON;
                break;
                    
        }
    }
    else
    {
        u16t_vbus_max = OVERVOLT_THRESHOLD_2LVON;
        u16t_vbus_max_unlock = OVERVOLT_UNLOCK_THRESHOLD_2LVON;
    }

    debug_u16t_vbus_max = u16t_vbus_max; /*デバッグ用*/
    debug_u16t_vbus_max_unlock = u16t_vbus_max_unlock;

    /*---------过电压异常判定--------------*/
    if(u16s_vbus > u16t_vbus_max)                            /*电压>420V*/
    {
        if(u8l_Vbus_over_cnt >= OVERVOLT_NUM)           /*连续5ms*/
        {
            u8l_Vbus_over_reccnt = OVERVOLT_NUM;        /*恢复时间设置为5ms*/
            (*u16p_errflg) |= F_ERR_OVERVOLT;           /*异常标志置位*/
            u8g_LED_err = LED_ERR_OV;
            MCI_FaultProcessing(&Mci[M1],MC_OVER_VOLT,~MC_OVER_VOLT);
        }
        else
        {
            u8l_Vbus_over_cnt++;
        }
    }
    /*---------过电压恢复判定--------------*/
    else if (u16s_vbus > u16t_vbus_max_unlock){               /*保護閾値<=電圧<保護解除閾値<*/
        /* do nothing */
    }
    else                                                /*电压<420V*/
    {
        if(u8l_Vbus_over_reccnt > 0)                    /*连续5ms，异常恢复*/
        {
            u8l_Vbus_over_reccnt--;
        }
        else
        {
            u8l_Vbus_over_cnt = 0x00;
            MCI_FaultProcessing(&Mci[M1],MC_NO_ERROR,~MC_NO_ERROR);
        }
    }
}

/******************************************************************
<Title> 		：Err_Jdg_UnderVolt(uint16_t* u16p_errflg)
<Function> 		：低电压检测
<Description> 	：低电压检测及恢复判定
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_UnderVolt(uint16_t* u16p_errflg)
{
    static uint8_t u8l_Vbus_under_cnt = 0x00;            /*异常计数*/
    static uint8_t u8l_Vbus_under_reccnt = 0x00;         /*恢复计数*/
    uint16_t u16t_vbus_min = 0x00;
    uint16_t u16t_vbus_min_unlock = 0x00;

    if (ucTYPE_2level==1){
        switch(pam.duty_mode)
        {
            case kDropping: /*降圧中*/
            case kWaitBoost: /*昇圧準備待ち*/
            case kBoosting: /*昇圧中*/
            case kDropped: /*降圧完了*/
            case kStableBoost: /*昇圧安定*/
                u16t_vbus_min = UNDERVOLT_THRESHOLD_2LVOFF; /*昇圧安定*/
                u16t_vbus_min_unlock = UNDERVOLT_UNLOCK_THRESHOLD_2LVOFF;/*昇圧安定*/
                break;
            case kBoosted: /*昇圧完了*/
                u16t_vbus_min = UNDERVOLT_THRESHOLD_2LVON; /*昇圧完了*/
                u16t_vbus_min_unlock = UNDERVOLT_UNLOCK_THRESHOLD_2LVON;/*昇圧完了*/
                break;
            default:
                u16t_vbus_min = UNDERVOLT_THRESHOLD_2LVOFF;
                u16t_vbus_min_unlock = UNDERVOLT_UNLOCK_THRESHOLD_2LVOFF;
                break;
        }
    }else{
        u16t_vbus_min = UNDERVOLT_THRESHOLD_2LVOFF;
        u16t_vbus_min_unlock = UNDERVOLT_UNLOCK_THRESHOLD_2LVOFF;
    }

    debug_u16t_vbus_min = u16t_vbus_min; /*デバッグ用*/
    debug_u16t_vbus_min_unlock = u16t_vbus_min_unlock; /*デバッグ用*/

    /*---------低电压异常判定--------------*/
    if(u16s_vbus < u16t_vbus_min)                        /*電圧<保護閾値*/
    {
        if(u8l_Vbus_under_cnt >= UNDERVOLT_NUM){         /*连续5ms*/
           u8l_Vbus_under_reccnt = UNDERVOLT_NUM;        /*恢复时间设置为5ms*/
           (*u16p_errflg) |= F_ERR_UNDERVOLT;            /*异常标志置位*/
           u8g_LED_err = LED_ERR_UV;                      /*设置LED显示低电压异常*/
            MCI_FaultProcessing(&Mci[M1],MC_UNDER_VOLT,~MC_UNDER_VOLT);
        }
        else
        {
            u8l_Vbus_under_cnt++;
        }
    } else if (u16s_vbus < u16t_vbus_min_unlock){               /*保護閾値<=電圧<保護解除閾値<*/
        /* do nothing */
    }
    /*---------低电压恢复判定--------------*/
    else                                                /*电压>180V*/
    {
        if(u8l_Vbus_under_reccnt > 0)                    /*连续5ms，异常恢复*/
        {
            u8l_Vbus_under_reccnt--;
        }
        else
        {
            u8l_Vbus_under_cnt = 0x00;
            MCI_FaultProcessing(&Mci[M1],MC_NO_ERROR,~MC_NO_ERROR);
        }
    }
}

/******************************************************************
<Title> 		：Err_Jdg_OutCtrl(uint16_t* u16p_errflg)
<Function> 		：脱调检测
<Description> 	：推定转速连续2s小于阈值，判定为脱调
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_OutCtrl(uint16_t* u16p_errflg)
{
    /*脱调检测*/
    static uint8_t u8s_LockErr_bemfcnt = 0;         /*反电动势异常计数*/
    static uint8_t u8s_LockErr_spdcnt = 0;          /*速度计算异常计数*/
    int16_t hMecSpeedUnit = STO_VOLT_PLL_M1._Super.hAvrMecSpeedUnit;
    int16_t hBemf_d = STO_VOLT_PLL_M1.hBemf_d_LPF; /*LPF(Ed) α = 0.2*/
    int16_t hBemf_q = STO_VOLT_PLL_M1.hBemf_q_LPF; /*LPF(Eq) α = 0.2*/
    int32_t wBemf_rate = (int32_t)0;

    if(hBemf_d < 0)         /*Ed abs*/   
    {
        hBemf_d = -hBemf_d;
    }
    if(hBemf_q  < 1)        /*make sure Eq > 0*/
    {
        hBemf_q = (int16_t)1;
    }
    wBemf_rate = (int32_t)((hBemf_d << 8) / hBemf_q);         /* Ed/Eq */

    /*仅在启动和运转中检测*/
    if((SWITCH_OVER == MC_GetSTMStateMotor1()))
    {
        if(wBemf_rate > LOCK_BEMF_MAX)            /* Ed/Eq > 0.6*/ /*(コメント対応必要)LOCK_BEMF_MAX が Ed/Eq最大? 0.6となる設計の考えをコメントに記載すること */
        {
            u8s_LockErr_bemfcnt++;
            if(u8s_LockErr_bemfcnt > LOCK_CNT_START)   /*启动时连续150ms*/
            {
                u8s_LockErr_bemfcnt = (uint8_t)0;
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);    /*异常触发*/
                (*u16p_errflg) |= F_ERR_OUTCTRL;        /*脱调异常标志置位*/
                u8g_LED_err = LED_ERR_LOCK;
            }
        }
        else
        {
            u8s_LockErr_bemfcnt = 0;
        }
        if(hMecSpeedUnit < LOCK_SPDUNIT_MIN)                 /*观测速度小于15rps*/
        {
            u8s_LockErr_spdcnt++;
            if(u8s_LockErr_spdcnt > LOCK_CNT_START)                /*启动时连续150ms*/
            {
                u8s_LockErr_spdcnt = (uint8_t)0;
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);    /*异常触发*/
                (*u16p_errflg) |= F_ERR_OUTCTRL;        /*脱调异常标志置位*/
                u8g_LED_err = LED_ERR_LOCK;
            }
        }
        else
        {
            u8s_LockErr_spdcnt = 0;
        }
    }
    else if((RUN == MC_GetSTMStateMotor1()))
    {
        /*33ms 触发*/
        if(wBemf_rate > LOCK_BEMF_MAX)            /* Ed/Eq > 0.6 (256*0.6 = 154)*/
        {
            u8s_LockErr_bemfcnt++;
            if(u8s_LockErr_bemfcnt > LOCK_CNT_RUN)   /*运转中连续30ms*/
            {
                u8s_LockErr_bemfcnt = (uint8_t)0;
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);    /*异常触发*/
                (*u16p_errflg) |= F_ERR_OUTCTRL;        /*脱调异常标志置位*/
                u8g_LED_err = LED_ERR_LOCK;
            }
        }
        else
        {
            u8s_LockErr_bemfcnt = 0;
        }
        if(hMecSpeedUnit < LOCK_SPDUNIT_MIN)         /*观测速度小于10rps*/
        {
            u8s_LockErr_spdcnt++;
            if(u8s_LockErr_spdcnt > LOCK_CNT_RUN)     /*运转中连续30ms*/
            {
                u8s_LockErr_spdcnt = (uint8_t)0;
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);    /*异常触发*/
                (*u16p_errflg) |= F_ERR_OUTCTRL;        /*脱调异常标志置位*/
                u8g_LED_err = LED_ERR_LOCK;
            }
        }
        else
        {
            u8s_LockErr_spdcnt = 0;
        }
    }
    /*其他状态不检测*/
    else
    {
        MCI_FaultProcessing(&Mci[M1], MC_NO_ERROR, ~MC_NO_ERROR);
        u8s_LockErr_bemfcnt = (uint8_t)0;
        u8s_LockErr_spdcnt = (uint8_t)0;
    }
    
}

/******************************************************************
<Title> 		：Err_Jdg_OverCur(uint16_t* u16p_errflg)
<Function> 		：过电流
<Description> 	：电流连续30ms大于阈值判定为过电流
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_OverCur(uint16_t* u16p_errflg)
{
    static uint8_t u8l_cur_over_cnt = 0x00;
    static uint8_t u8l_cur_rec_cnt = 0x00;

    /*---------------软件过电流异常判定------------------------------------*/
    if((fs_currA >= CURR_MAX) || (fs_currB >= CURR_MAX) || (fs_currC >= CURR_MAX))
    {
        if(u8l_cur_over_cnt >= OVERCUR_NUM)            /*连续30ms过电流*/
        {
            u8l_cur_rec_cnt = OVERCUR_NUM;             /*恢复时间设定为30ms*/
            MCI_FaultProcessing(&Mci[M1],MC_SW_OVERCURR,~MC_SW_OVERCURR);
            (*u16p_errflg) |= F_ERR_OVERCUR;            /*软件过流标志*/
            u8g_LED_err = LED_ERR_SWCU;
        }
        else
        {
            u8l_cur_over_cnt++;
        }
    }
    /*---------------软件过电流恢复判定------------------------------------*/
    else
    {
        if(u8l_cur_rec_cnt > 0)                         /*恢复时间未到*/
        {
            u8l_cur_rec_cnt--;
        }
        else
        {
            u8l_cur_over_cnt = 0;
            MCI_FaultProcessing(&Mci[M1],MC_NO_ERROR,~MC_NO_ERROR);
        }
    }
}

/******************************************************************
<Title> 		：Err_Jdg_PhaseLoss(uint16_t* u16p_errflg)
<Function> 		：缺相保护
<Description> 	：启动和运转时连续1s检测到某相电流小于阈值
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_PhaseLoss(uint16_t* u16p_errflg)
{
    static uint16_t u16l_PHa_cnt = 0x0000u;
    static uint16_t u16l_PHb_cnt = 0x0000u;
    static uint16_t u16l_PHc_cnt = 0x0000u;
    uint16_t uAux = FALSE;
    uint16_t u16l_loss_num = 0u;
    float fl_cur_sum = 0.04;

    /*fl_cur_sum = (fs_currA + fs_currB + fs_currC) / 6;*/              /*取三相电流绝对值的平均值的一半为判定阈值*/

    /* ブレーキ中(フリーランゼロクロス待ち)は欠相判定しない */
    if (u8g_brake_state == BRAKE_STOPPING)
    {
        u16l_PHa_cnt = 0u;
        u16l_PHb_cnt = 0u;
        u16l_PHc_cnt = 0u;
        return;
    }
    /*---------------------------检测时间设定------------------------------------------*/
    /*启动中*/
    if((MC_GetSTMStateMotor1() == START) || (MC_GetSTMStateMotor1() == SWITCH_OVER))            
    {
        u16l_loss_num = LOSS_CNT_START;     /*检测时间1s*/
    }
    /*运转中*/
    else if(MC_GetSTMStateMotor1() == RUN)
    {
        u16l_loss_num = LOSS_CNT_RUN;       /*检测时间9ms*/
    }
    /*其他*/
    else
    {
        return; /*不进行缺相检测*/
    }
    /*---------------------------电流判定---------------------------------------------*/
    if((fs_currA < fl_cur_sum)) /*A相电流小于阈值*/
    {
        if(u16l_PHa_cnt < u16l_loss_num)
        {
            u16l_PHa_cnt++;
        }
        else
        {
            uAux = TRUE;
        }
    }
    else
    {
        u16l_PHa_cnt = 0;
    }

    if((fs_currB < fl_cur_sum)) /*B相电流小于阈值*/
    {
        if(u16l_PHb_cnt < u16l_loss_num)
        {
            u16l_PHb_cnt++;
        }
        else
        {
            uAux = TRUE;
        }
    }
    else
    {
        u16l_PHb_cnt = 0;
    }

    if((fs_currC < fl_cur_sum)) /*C相电流小于阈值*/
    {
        if(u16l_PHc_cnt < u16l_loss_num)
        {
            u16l_PHc_cnt++;
        }
        else
        {
            uAux = TRUE;
        }
    }
    else
    {
        u16l_PHc_cnt = 0;
    }

    if(TRUE == uAux)
    {
        (*u16p_errflg) |= F_ERR_PHASELOSS;
        MCI_FaultProcessing(&Mci[M1],MC_PHASE_LOSS,~MC_PHASE_LOSS);
        u8g_LED_err = LED_ERR_LP;
        u16l_PHa_cnt = 0x0000u;
        u16l_PHb_cnt = 0x0000u;
        u16l_PHc_cnt = 0x0000u;
    }
}

/******************************************************************
<Title> 		：Err_Jdg_OverPwr(uint16_t* u16p_errflg)
<Function> 		：过负荷
<Description> 	：连续2s检测到功率大于阈值判定为过负荷
<Input> 		：
<Return> 		：
<Author> 		：董明鉴
<Date> 			：2024/10/10
<Update> 		：2025/3/4  董明鉴 追加OLP保护式样
                : 2026/1/22 周浩鹏 追加有无OLP保护器开关，OLP合理化机种需加载OLP_None宏定义
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_OverPwr(uint16_t* u16p_errflg)
{
    uint16_t fl_pwr_avg;
    float fl_pwr_avg_raw;
    static uint16_t u16l_pwr_limit_cnt = 0;
    static uint16_t u16l_pwr_rcv_cnt = 0;
    static uint32_t u32l_pwr_shut_cnt = 0;          /*停机计时器*/

    fl_pwr_avg_raw = MC_GetAveragePowerMotor1_F(); /*读取功率平均值，单位W，数据类型float*/
    if(fl_pwr_avg_raw < MOTOR_PWR_AVG_MIN_F)
    {
        u16g_MOTOR_pwr_avg = MOTOR_PWR_AVG_MIN_U16;
    }
    else if(fl_pwr_avg_raw > MOTOR_PWR_AVG_MAX_F)
    {
        u16g_MOTOR_pwr_avg = MOTOR_PWR_AVG_MAX_U16;
    }
    else
    {
        u16g_MOTOR_pwr_avg = (uint16_t)fl_pwr_avg_raw;
    }
    fl_pwr_avg = u16g_MOTOR_pwr_avg; /* volatileからローカルへコピーして判定に使用 */
#ifdef OLP_None
    /*-----模式切换，计时清零-----*/
   {   extern uint8_t ucTRNS_sfrg;
       u8g_FCcmd = ucTRNS_sfrg & 0x40;
    }
    if(u8g_FCflg != u8g_FCcmd)              
    {
        u32l_pwr_shut_cnt = 0;
        u16l_pwr_limit_cnt = 0;
        u16l_pwr_rcv_cnt = 0;
        u16l_pwr_blce_cnt = 0;
        u8g_FCflg = u8g_FCcmd;
    }
    /*-----工程检查模式------*/
    if((u8g_FCflg & 0x40U) != 0U)
    {
#endif
        if(fl_pwr_avg > POWER_LIMIT)                             /*220W 制限*/
        {
            if(u16l_pwr_limit_cnt >= POWER_JDG_CNT)
            {
                u8g_SPD_limit = SPD_LIMIT_STATE_ACTIVE;          /*开启速度限定*/
                u8g_pwr_rcv_flg = TRUE;                          /*过功率恢复标志位置位 */
                u16l_pwr_limit_cnt = 0;
            }
            else
            {
                u16l_pwr_limit_cnt++;
            }
            if(fl_pwr_avg >  POWER_SHUT)                         /*250W 停机*/
            {
                /*---停机计时---*/
                if(u32l_pwr_shut_cnt >= OLPNUM_FC_SHUT)          /*连续检测*/
                {
                    u8g_OLPflg = TRUE;                           /*OLP 标志*/
                    MCI_FaultProcessing(&Mci[M1],MC_OVER_PWR,0);
                    (*u16p_errflg) |= F_ERR_OVERPWR;
                    u8g_LED_err = LED_ERR_OP;
                }
                else
                {
                    u32l_pwr_shut_cnt++;
                }
            }
            else
            {
                u32l_pwr_shut_cnt = 0;
            }
            u16l_pwr_blce_cnt = 0;
            u16l_pwr_rcv_cnt = 0;
        }
        else if(fl_pwr_avg < POWER_RCV)
        {   
            if(u8g_pwr_rcv_flg == TRUE)
            {
                if(u16l_pwr_rcv_cnt >= POWER_JDG_CNT)
                {
                    u8g_SPD_limit = SPD_LIMIT_STATE_RECOVERY;    /*制限恢复*/
                    u16l_pwr_rcv_cnt = 0;
                }
                else
                {
                    u16l_pwr_rcv_cnt ++;
                }
            }
            else
            {
                u8g_SPD_limit = SPD_LIMIT_STATE_OFF;
                u16l_pwr_rcv_cnt = 0;
            }
            u16l_pwr_blce_cnt = 0;
            u16l_pwr_limit_cnt = 0;
        }
        else
        {
            if(u8g_SPD_limit == SPD_LIMIT_STATE_RECOVERY)
            {
                if(u16l_pwr_blce_cnt >= POWER_JDG_CNT)
                {
                    u8g_SPD_limit = SPD_LIMIT_STATE_BALANCE;      /*转速平衡*/
                    u16l_pwr_blce_cnt = 0;
                }
                else
                {
                    u16l_pwr_blce_cnt++;
                }

            }
            u16l_pwr_limit_cnt = 0;
            u16l_pwr_rcv_cnt = 0;
        }
#ifdef OLP_None
    }
    /*正常模式*/
    else
    {
        if(fl_pwr_avg > POWER_SHUT_NOOLP)                            /*功率大于220W 停机*/
        {
            if(u32l_pwr_shut_cnt >= OLPNUM_NOR_SHUT)            /*连续300s检测*/
            {
                u8g_OLPflg = TRUE;                              /*OLP 标志*/
                MCI_FaultProcessing(&Mci[M1],MC_OVER_PWR,0);
                (*u16p_errflg) |= F_ERR_OVERPWR;
                u8g_LED_err = LED_ERR_OP;
            }
            else
            {
                u32l_pwr_shut_cnt++;
            }
        }
        else
        {
            u32l_pwr_shut_cnt = 0;
        }

    }
#endif
    
}

/******************************************************************
<Title>         ：Err_Jdg_OffsetErr(uint16_t* u16p_errflg)
<Function>      ：相电流零点漂移
<Description>   ：压缩机停机时检测，连续10ms检测到电流值不在允许范围内，
                ：判定为零点漂移
<Input>         ：
<Return>        ：
<Author>        ：董明鉴
<Date>          ：2024/10/10
<Update>        ：
<Copyright>     ：Panasonic Co. , Ltd.
******************************************************************/
static void Err_Jdg_OffsetErr(uint16_t* u16p_errflg)
{
    uint32_t u32_offset;
    static uint8_t u8l_offset_cnt = 0x00;
    u32_offset = PWM_Handle_M1.PhaseOffset;
    /*相电流零点漂移*/
    if (pwmcHandle[M1]->offsetCalibStatus == true)              /*偏置计算已完成*/
    {
        if((u32_offset < OFFSET_MIN) || (u32_offset > OFFSET_MAX)) /* (コメント対応必要)OFFSET_MINとOFFSET_MAXの設計の考え方をコメントに記載すること */
        {
            if(u8l_offset_cnt >= OFFSET_NUM)
            {
                (*u16p_errflg) |= F_ERR_OFFSET;
                MCI_FaultProcessing(&Mci[M1],MC_OFS_ERROR,~MC_OFS_ERROR);
                u8g_LED_err = LED_ERR_OFFSET;
            }
            else
            {
                u8l_offset_cnt++;
            }
        }
        else
        {
            u8l_offset_cnt = 0;
        }
    }
    
}

/******************************************************************
<Title>         ：Motor_Curr_get(void)
<Function>      ：读取电流值
<Description>   ：调用接口读取Ia，Ib;计算Ic = -（Ia + Ib；并取绝对值
<Input>         ：
<Return>        ：
<Author>        ：董明鉴
<Date>          ：2024/10/12
<Update>        ：
<Copyright>     ：Panasonic Co. , Ltd.
******************************************************************/
static void Motor_Curr_get(void)
{
    /*--------------电流采样值获取----------------------*/
    ab_f_t stl_curr;
    stl_curr = MC_GetIabMotor1_F();           /*读取Ia，Ib*/
    fs_currA = stl_curr.a;
    fs_currB = stl_curr.b;
    fs_currC = -fs_currA - fs_currB;
    if(fs_currA < 0.0F)
    {
        fs_currA = -fs_currA;
    }
    if(fs_currB < 0.0F)
    {
        fs_currB = -fs_currB;
    }
    if(fs_currC < 0.0F)
    {
        fs_currC = -fs_currC;
    }
}

/******************************************************************
<Title>         ：Motor_Err_RemianCnt(void)
<Function>      ：异常持续时间计数
<Description>   ：5min异常持续时间计数
<Input>         ：
<Return>        ：
<Author>        ：董明鉴
<Date>          ：2024/10/12
<Update>        ：
<Copyright>     ：Panasonic Co. , Ltd.
******************************************************************/
static void Motor_Err_RemainCnt(void)
{
    static uint16_t u16l_cnt_ms = 0x0000;       /*毫秒计数*/
    static uint8_t u8l_cnt_s = 0x00;            /*秒计数*/
    static uint8_t u8l_cnt_min = 0x00;          /*分钟计数*/

    if(u8g_ERR_remain == TRUE)                  /*异常持续中*/
    {
        if(u16l_cnt_ms >= 1000)
        {
            u16l_cnt_ms = 0;
            u8l_cnt_s++;
            if(u8l_cnt_s >= 60)
            {
                u8l_cnt_s = 0;
                u8l_cnt_min++;
                if(u8l_cnt_min >= ERR_REMAIN_CNT)   /*异常持续时间 5min*/
                {
                    u8l_cnt_min = 0;
                    u8g_ERR_remain = FALSE;         /*异常持续标志清除*/
                    u8g_LED_err = LED_ERR_NONE;     /*led异常代码清除*/
                    u8g_hog_5minstop_cnt ++;        /*保護停止5minカウントをｲﾝｸﾘﾒﾝﾄ*/
                    SET_LED_DISP(LED_STATE_OFF);    /*LED关闭*/
                    #ifndef OLP_None
                        u8g_OLPflg = FALSE;         /*有OLP保护器时，5分钟清除OLP标志位*/
                    #endif
                }
            }
        }
        else
        {
            u16l_cnt_ms += 10;
        }
    }
}


/******************************************************************
<Title> 		：SET_MotorErr_ucHOG_movF(void)
<Function> 		：u16p_errflgをucHOG_movFに変換して保存
<Description> 	：u16p_errflgのビットをucHOG_movFに変換して保存
<Input> 		：none
<Return>        ：none
<Author> 		：Sakamoto
<Date> 			：2026/6/3
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
void SET_MotorErr_ucHOG_movF(void)
{
    ucHOG_movF = 0x00;

    /* bit1(0x02): コンプレッサ巻線保護 */
    if(((*st_motor_err.u16_errflg) & F_ERR_OVERPWR) != 0)
    {
        ucHOG_movF |= 0x02;
    }

    /* bit3(0x03): 過電圧 */
    if(((*st_motor_err.u16_errflg) & F_ERR_OVERVOLT) != 0)
    {
        ucHOG_movF |= 0x04;
    }

    /* bit3(0x08): 不足電圧低下保護 */
    if(((*st_motor_err.u16_errflg) & F_ERR_UNDERVOLT) != 0)
    {
        ucHOG_movF |= 0x08;
    }

    /* bit4(0x10): IPM保護 EMG error */
    if(((*st_motor_err.u16_errflg) & (F_ERR_IPMERR | F_ERR_OVERCUR)) != 0)
    {
        ucHOG_movF |= 0x10;
    }

    /* bit5(0x20): ロック保護(欠相保護) */
    if(((*st_motor_err.u16_errflg) & (F_ERR_PHASELOSS | F_ERR_OFFSET | F_ERR_OUTCTRL)) != 0)
    {
        ucHOG_movF |= 0x20;
    }

    /* bit6(0x40): 強制転流失敗 error */
    if((MC_GetSTMStateMotor1() == FAULT_OVER) != 0)
    {
        ucHOG_movF |= 0x40;
    }

    /*保護停止5分間経過していれば、データセットをクリア*/
    if (u8g_ERR_remain == FALSE){
        ucHOG_movF = 0x00; 
    }

    vHOG_5minwt();								/* ucHOG_sttF反映 */
}

/******************************************************************
<Title> 		：IsResetByHogo5minStop(void)
<Function> 		：保護停止5分待ちによるリセット判定
<Description> 	：保護停止5分待ちが規定回数以上発生しているかを判定し、リセットフラグを返す
<Input> 		：none
<Return>        ：judge(ﾘｾｯﾄﾌﾗｸﾞ)
<Author> 		：Takida
<Date> 			：2026/09/09
<Copyright> 	：Panasonic Co. , Ltd.
******************************************************************/
uint8_t IsResetByHogo5minStop(void)
{
    uint8_t judge;
    if(u8g_hog_5minstop_cnt >= NUM_HOGO5MINSTP_RST)			/*保護停止による5min待ちが3回以上発生なら *//*保護が3回以上(≒保護停止による強制停止が合計15分以上)の場合は各ポート設定異常発生の可能性を無くすためにもリセット */
	{
        judge = SET;					            /*ﾘｾｯﾄする(ﾌﾗｸﾞｾｯﾄ) */
        u8g_hog_5minstop_cnt = NUM_HOGO5MINSTP_RST; /*リセット発生時にカウントをｸﾘｯﾌﾟ(ｵｰﾊﾞｰﾌﾛｰ対策) */
    }else{
        judge = RESET;                              /*ﾘｾｯﾄしない(ﾌﾗｸﾞｸﾘｱ) *//*"RESET"と"CLR"は両方0の定義であり値は一致する*/
    }
    return judge;
}
