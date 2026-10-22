#include "sto_volt_speed_pos_fdbk.h"





/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/**
  * @defgroup SpeednPosFdbk_STO_PLL State Observer PLL Speed & Position Feedback.
  * @brief State Observer with PLL Speed & Position Feedback component of MCSDK.
  *
  * This component uses a State Observer coupled with a software PLL to provide an estimation of
  * the speed and the position of the rotor of the motor.
  *
  * See the [Speed & position feedback sensorless chapter of the User Manual](speed_pos_sensorless_bemf_reconstruction.md) for more details on the sensorless algorithm.
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define C6_COMP_CONST1  ((int32_t)1043038)
#define C6_COMP_CONST2  ((int32_t)10430)

/* Private function prototypes -----------------------------------------------*/
static void STO_VOLT_Store_Rotor_Speed(STO_VOLT_PLL_Handle_t *pHandle, int16_t hRotor_Speed);
static int16_t STO_VOLT_ExecutePLL(STO_VOLT_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est);
static inline void STO_VOLT_InitSpeedBuffer(STO_VOLT_PLL_Handle_t *pHandle); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */


/**
  * @brief  Initializes the @p pHandle of STate Observer (STO) PLL component.
  *
  */
__weak void STO_VOLT_PLL_Init(STO_VOLT_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif

    pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
    pHandle->EnableDualCheck = true;

    STO_VOLT_PLL_Clear(pHandle);

    PID_HandleInit(&pHandle->PIRegulator);

    /* Acceleration measurement set to zero */
    pHandle->_Super.hMecAccelUnitP = 0;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
  return;
}

/**
  * @brief  Necessary empty return to implement fictitious IRQ_Handler.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  uint8_t: Fictitious interrupt flag.
  */
/*cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect*/
__weak void STO_VOLT_PLL_Return(STO_VOLT_PLL_Handle_t *pHandle, uint8_t flag)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == (void *)pHandle) || ((uint8_t)0 == flag))
  {
    /* Nothing to do */
  }
  else
  {
    /* Nothing to do */
  }
#endif
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Calculates the estimated electrical angle.
  * 
  * Executes Luenberger state observer equations and calls
  * PLL to compute a new speed estimation and
  * update the estimated electrical angle.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  pInput: Pointer to the observer inputs structure.
  * @retval int16_t Rotor electrical angle (s16Degrees).
  */
/*cstat !MISRAC2012-Rule-8.13*/
__weak int16_t STO_VOLT_PLL_CalcElAngle(STO_VOLT_PLL_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{
  int16_t retValue;

  if ((MC_NULL == pHandle) || (MC_NULL == pInputs))
  {
    retValue = 0;
  }
  else
  {
    int32_t wAux;
    int32_t wAux1;
    int32_t wBemf_d_est_Next;
    int32_t wBemf_q_est_Next;
    int32_t wAlpha;
    int16_t hAux;
    int16_t hAlpha;
    int16_t hsgn_Rotor_Speed;
    int16_t hFai;
    int16_t hRotor_Speed;
    int16_t abs_hRotor_Speed;


/***********************以下是Vdref*********************/		
    wBemf_d_est_Next = ((int32_t)pHandle->hVC1) * pInputs->Vbus;                   
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
  wBemf_d_est_Next = (int32_t)(wBemf_d_est_Next >> pHandle->FV1LOG);                 /*hVF1=4096*/
#else
    wBemf_d_est_Next = (int32_t)(wBemf_d_est_Next / pHandle->hVF1);
#endif
  wBemf_d_est_Next = (int32_t)(wBemf_d_est_Next * pInputs->Vdq.d);         /*vd=(3277)时，*/

/***********************以下是RSH*idref*********************/

    wAux = ((int32_t)pHandle->hVC2) * pInputs->Iqd.d;

    wBemf_d_est_Next -= wAux;


/***********************以下是LSH*imax*W1*********************/
  wAux = ((int32_t)pHandle->hVC4) *  pInputs->Iqd.q;                   /*iq*Ld对吗？后面确认*/
    wAux = ((int32_t)wAux) * pHandle->_Super.hElSpeedDpp;
    wBemf_d_est_Next += wAux;

/***********************以下是Vqref*********************/				
    wBemf_q_est_Next = ((int32_t)pHandle->hVC1) * pInputs->Vbus;
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
    wBemf_q_est_Next = (int32_t)(wBemf_q_est_Next >> pHandle->FV1LOG);
#else
    wBemf_q_est_Next = (int32_t)(wBemf_q_est_Next / pHandle->hVF1);
#endif
    wBemf_q_est_Next = (int32_t)(wBemf_q_est_Next * pInputs->Vdq.q);

/***********************以下是RSH*iqref*********************/
    wAux = ((int32_t)pHandle->hVC2) * pInputs->Iqd.q;
    wBemf_q_est_Next -= wAux;

/***********************以下是LSHD*imax*W1*********************/
    wAux = ((int32_t)pHandle->hVC3) *  pInputs->Iqd.d;
/*		wAux += pHandle->hVC11;*/
    wAux = ((int32_t)wAux) *  pHandle->_Super.hElSpeedDpp;
    wBemf_q_est_Next -= wAux;


#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
  pHandle->hBemf_d_est = (int16_t)(wBemf_d_est_Next >> pHandle->FV3LOG);      /*hVF3==65535*/
#else
    pHandle->hBemf_d_est = (int16_t)(wBemf_d_est_Next / pHandle->hVF3);              
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
    pHandle->hBemf_q_est = (int16_t)(wBemf_q_est_Next >> pHandle->FV3LOG);
#else
    pHandle->hBemf_q_est = (int16_t)(wBemf_q_est_Next / pHandle->hVF3);
#endif
    
/****************计算fsgn(W1)和fabs(W1)*******************************/
    if(pHandle->_Super.hElSpeedDpp >= 0 )
    {
      abs_hRotor_Speed = pHandle->_Super.hElSpeedDpp;
      hsgn_Rotor_Speed = 8192;
    }
    else
    {
      abs_hRotor_Speed = -pHandle->_Super.hElSpeedDpp;
      hsgn_Rotor_Speed = -8192;
    }
/*****************************计算alpha=ALPHAO+2*lamda*fabs(w1)***********************/
  wAlpha = ((int32_t)pHandle->hVC7) * abs_hRotor_Speed;                       /*hElSpeedDppMAX=4800*3/60*1.2/5000=3775*/
    wAlpha += ((int32_t)pHandle->wVC5);
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
  hAlpha = (int16_t)(wAlpha >> pHandle->FV1LOG);                               /*MAX=24905*/
#else
    hAlpha = (int16_t)(wAlpha / pHandle->hVF1);
#endif

/****************************计算W1+=T*alpha*((eq-lamda*fsgn(w1)*ed)/faim-w1)************/		

    wAux  = ((int32_t)pHandle->hBemf_d_est) *  hsgn_Rotor_Speed;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    wAux = (int16_t)(wAux >> pHandle->FV2LOG);
#else
    wAux = (int16_t)(wAux / pHandle->hVF2);
#endif
    wAux = ((int32_t)pHandle->hBemf_q_est) - wAux;

    wAux = ((int32_t)pHandle->hVC8) * wAux;

    wAux1 = ((int32_t)pHandle->hVC10) * pInputs->Iqd.d;
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
    hAux = (int16_t)(wAux1 >> pHandle->FV1LOG);
#else
    hAux = (int16_t)(wAux1 / pHandle->hVF1);
#endif
  hFai = ((int16_t)pHandle->hVC9) + hAux;	                               /*MAX=5734*/

    wAux = wAux / hFai;
    wAux = wAux - pHandle->_Super.hElSpeedDpp;
    wAux = wAux *hAlpha;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
    hAux = (int16_t)(wAux >> pHandle->FV4LOG);
#else
    hAux = (int16_t)(wAux / pHandle->hVF4);
#endif

    /*Bemf LPF*/
    Filter_OneOrderLPF(pHandle->hBemf_d_est,&pHandle->hBemf_d_LPF);
    Filter_OneOrderLPF(pHandle->hBemf_q_est,&pHandle->hBemf_q_LPF);
    /*LPF end*/

    hRotor_Speed  = pHandle->_Super.hElSpeedDpp + hAux;

    pHandle->_Super.InstantaneousElSpeedDpp = hRotor_Speed;
    pHandle->_Super.hElAngle += hRotor_Speed;

    STO_VOLT_Store_Rotor_Speed(pHandle, hRotor_Speed);

    retValue = pHandle->_Super.hElAngle;
  }
  return (retValue);
}

/**
  * @brief  Computes and returns the average mechanical speed.
  * 
  * This method must be called - at least - with the same periodicity
  * on which speed control is executed. It computes and returns - through
  * parameter hMecSpeedUnit - the rotor average mechanical speed,
  * expressed in #SPEED_UNIT. Average is computed considering a FIFO depth
  * equal to bSpeedBufferSizeUnit. Moreover it also computes and returns
  * the reliability state of the sensor.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  pMecSpeedUnit: Pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in #SPEED_UNIT).
  * @retval True if the sensor information is reliable, false otherwise.
  */

__weak bool STO_VOLT_PLL_CalcAvrgMecSpeedUnit(STO_VOLT_PLL_Handle_t *pHandle, int16_t *pMecSpeedUnit) /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
{
  bool bAux; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pMecSpeedUnit))
  {
    bAux = false;
  }
  else
  {
#endif
    int32_t wAvrSpeed_dpp = (int32_t)0;
    int32_t wError;
    int32_t wAux;
    int32_t wAvrSquareSpeed;
    int32_t wAvrQuadraticError = 0;
    /*int32_t wObsBemf, wEstBemf;*/
    /*int32_t wObsBemfSq = 0;*/
    /*int32_t wEstBemfSq = 0;*/
    /*int32_t wEstBemfSqLo;*/
    bool bIs_Speed_Reliable = false; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    /*bool bIs_Bemf_Consistent = false;*/
    uint8_t i, bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;
    /* Calculate the average mechanial speed*/
    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wAvrSpeed_dpp += (int32_t)(pHandle->Speed_Buffer[i]);
    }

    if (0U == bSpeedBufferSizeUnit)
    {
      /* Nothing to do */
    }
    else
    {
      wAvrSpeed_dpp = wAvrSpeed_dpp / ((int16_t)bSpeedBufferSizeUnit);
    }

    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wError = ((int32_t)pHandle->Speed_Buffer[i]) - wAvrSpeed_dpp;
      wError = (wError * wError);
      wAvrQuadraticError += wError;
    }

    /* It computes the measurement variance */
    wAvrQuadraticError = wAvrQuadraticError / ((int16_t)bSpeedBufferSizeUnit);

    /* The maximum variance acceptable is here calculated as a function of average speed */
    wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
    int64_t lAvrSquareSpeed = (int64_t)(wAvrSquareSpeed) * (int64_t)pHandle->VariancePercentage; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    wAvrSquareSpeed = (int32_t)(lAvrSquareSpeed / (int64_t)128);
  
    if (wAvrQuadraticError < wAvrSquareSpeed)
    {
      bIs_Speed_Reliable = true;
    }
    if((START == MC_GetSTMStateMotor1()) || (SWITCH_OVER == MC_GetSTMStateMotor1()))        /*定位、启动阶段采用瞬时速度值*/
    {
        wAvrSpeed_dpp = pHandle->_Super.InstantaneousElSpeedDpp;
    }
            
    /* Computation of Mechanical speed Unit */
    wAux = wAvrSpeed_dpp * ((int32_t)pHandle->_Super.hMeasurementFrequency);
    wAux = wAux * ((int32_t)pHandle->_Super.SpeedUnit);
    wAux = wAux / ((int32_t)pHandle->_Super.DPPConvFactor);
    wAux = wAux / ((int16_t)pHandle->_Super.bElToMecRatio);

    *pMecSpeedUnit = (int16_t)wAux;
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wAux;
    pHandle->IsSpeedReliable = bIs_Speed_Reliable;
    bAux = bIs_Speed_Reliable;

  /* Bemf Consistency Check algorithm
   * if (true == pHandle->EnableDualCheck)
   * {
   *   wAux = ((wAux < 0) ? (-wAux) : (wAux));
   *   if (wAux < (int32_t)(pHandle->MaxAppPositiveMecSpeedUnit))
   *   {
   *     wObsBemf = (int32_t)pHandle->hBemf_d_est;
   *     wObsBemfSq = wObsBemf * wObsBemf;
   *     wObsBemf = (int32_t)pHandle->hBemf_q_est;
   *     wObsBemfSq += wObsBemf * wObsBemf;
   *     wEstBemf = (wAux * 32767) / ((int16_t)pHandle->_Super.hMaxReliableMecSpeedUnit);
   *     wEstBemfSq = (wEstBemf * ((int32_t)pHandle->BemfConsistencyGain)) / 64;
   *     wEstBemfSq *= wEstBemf;
   *     wEstBemfSqLo = wEstBemfSq - ((wEstBemfSq / 64) * ((int32_t)pHandle->BemfConsistencyCheck));
   *     if (wObsBemfSq > wEstBemfSqLo)
   *     {
   *       bIs_Bemf_Consistent = true;
   *     }
   *   }
   *   pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
   *   pHandle->Obs_Bemf_Level = wObsBemfSq;
   *   pHandle->Est_Bemf_Level = wEstBemfSq;
   * }
   * else
   * {
   *   bIs_Bemf_Consistent = true;
   * }
   * if (false == pHandle->IsAlgorithmConverged)
   * {
   *   bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
   * }
   * else
   * {
   *   if ((false == pHandle->IsSpeedReliable) || (false == bIs_Bemf_Consistent))
   *   {
   *     pHandle->ReliabilityCounter++;
   *     if (pHandle->ReliabilityCounter >= pHandle->_Super.bMaximumSpeedErrorsNumber)
   *     {
   *       pHandle->ReliabilityCounter = 0U;
   *       pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
   *       bAux = false;
   *     }
   *     else
   *     {
   *       bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
   *     }
   *   }
   *   else
   *   {
   *     pHandle->ReliabilityCounter = 0U;
   *     bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
   *   }
   * }
   */
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
  return (bAux);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  Computes and updates the average electrical speed.
  * 
  * This method must be called - at least - with the same periodicity
  * on which speed control is executed. It computes and update component
  * variable hElSpeedDpp that is estimated average electrical speed
  * expressed in dpp used for instance in observer equations.
  * Average is computed considering a FIFO depth equal to
  * bSpeedBufferSizedpp.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_VOLT_PLL_CalcAvrgElSpeedDpp(STO_VOLT_PLL_Handle_t *pHandle, int16_t hSpeedRefdpp)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wSum = pHandle->DppBufferSum;
    /*int32_t wAvrSpeed_dpp;*/
    int16_t hSpeedBufferSizedpp = (int16_t)pHandle->SpeedBufferSizeDpp;
    int16_t hSpeedBufferSizeUnit = (int16_t)pHandle->SpeedBufferSizeUnit;
    int16_t hBufferSizeDiff;
    int16_t hIndexNew = (int16_t)pHandle->Speed_Buffer_Index;
    int16_t hIndexOld;
    int16_t hIndexOldTemp;

    hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

    if (0 == hBufferSizeDiff)
    {
      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->SpeedBufferOldestEl;
    }
    else
    {
      hIndexOldTemp = hIndexNew + hBufferSizeDiff;

      if (hIndexOldTemp >= hSpeedBufferSizeUnit)
      {
        hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
      }
      else
      {
        hIndexOld = hIndexOldTemp;
      }

      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->Speed_Buffer[hIndexOld];
    }

    pHandle->_Super.hElSpeedDpp = pHandle->_Super.InstantaneousElSpeedDpp;

    pHandle->DppBufferSum = wSum;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}
/******************************************************************
<Title>       ：STO_VOLT_PLL_CalcTorqueRatio(SpeednTorqCtrl_Handle_t *tpHandle, STO_VOLT_PLL_Handle_t *pHandle, Observer_Inputs_t *pInput)
<Function>    ：转矩补偿
<Description> ：根据当前功率计算补偿系数，对转矩控制(iq)进行补偿
<Input>       ：
<Return>      ：
<Author>      ：董明鉴、曾祥海
<Date>        ：2025/3/10
<Update>      ：
<Copyright>   ：Panasonic Co. , Ltd. 
******************************************************************/

__weak int16_t STO_VOLT_PLL_CalcTorqueRatio(SpeednTorqCtrl_Handle_t *tpHandle, STO_VOLT_PLL_Handle_t *pHandle,Observer_Inputs_t *pInput)
{

    int16_t hElSpeedDppRef;
    int16_t hElSpeedErr;
    int16_t hAux;
    int16_t TorqueRatio;
  int16_t TorqueMeasurement=0;    /*16384*_Super.hMeasurementFrequency/(Super.bElToMecRatio*65536*40)=83; 40代表1转的误差增加0.025的比例系数,实际差值：+-8以内*/
    uint16_t hVbus;
    hVbus = pInput->Vbus * (uint16_t)(3.3 * 154);
    hVbus = hVbus / 65536U;
    if((hVbus >= 180) && (hVbus < 239))
    {
        TorqueMeasurement = hVbus *13 / 239;
        TorqueMeasurement = ((int16_t)MC_GetAveragePowerMotor1_F()) / TorqueMeasurement;	
    }
    else
    {
        TorqueMeasurement = ((int16_t)MC_GetAveragePowerMotor1_F()) / 13;
    }
    if(TorqueMeasurement>20)
    {
      TorqueMeasurement=20;
    }      
    hElSpeedDppRef  = (int16_t)((tpHandle->SpeedRefUnitExt)	/ (((int32_t)SPEED_UNIT) * ((int32_t)pHandle->_Super.hMeasurementFrequency)));
    hElSpeedDppRef *= ((int16_t)pHandle->_Super.bElToMecRatio);
    hElSpeedErr = hElSpeedDppRef - pHandle->_Super.hElSpeedDpp;
    hAux = 16384 + hElSpeedErr * TorqueMeasurement;
    pHandle->_Super.hElSpeedDppRef = hElSpeedDppRef;
  if(hAux > 20480)              /* 1.2=19661,1.5=24576 1.15=18841 1.25=20480 */
    {
      TorqueRatio = 20480;
    }
  else if(hAux < 12288)        /*0.85=13926 0.8=13107  0.75=12288*/
    {
      TorqueRatio = 12288;
    }
    else 		
    {
      TorqueRatio = hAux;
    }
    return (TorqueRatio);
}

/**
  * @brief  Clears state observer component by re-initializing private variables in @p pHandle.
  * 
  */
__weak void STO_VOLT_PLL_Clear(STO_VOLT_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Ialfa_est = (int32_t)0;
    pHandle->Ibeta_est = (int32_t)0;
    pHandle->hBemf_d_est = (int16_t)0;
    pHandle->hBemf_q_est = (int16_t)0;
    pHandle->_Super.hElAngle = -10922;
    pHandle->_Super.hElSpeedDpp = (int16_t)0;
    pHandle->_Super.InstantaneousElSpeedDpp = (int16_t)0;
    pHandle->ConsistencyCounter = 0u;
    pHandle->ReliabilityCounter = 0u;
    pHandle->IsAlgorithmConverged = false;
    pHandle->IsBemfConsistent = false;
    pHandle->Obs_Bemf_Level = (int32_t)0;
    pHandle->Est_Bemf_Level = (int32_t)0;
    pHandle->DppBufferSum = (int32_t)0;
    pHandle->ForceConvergency = false;
    pHandle->ForceConvergency2 = false;

    STO_VOLT_InitSpeedBuffer(pHandle);
    PID_SetIntegralTerm(& pHandle->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Stores in @p pHandle the latest calculated value of @p hRotor_Speed.
  * 
  */
inline static void STO_VOLT_Store_Rotor_Speed(STO_VOLT_PLL_Handle_t *pHandle, int16_t hRotor_Speed)
{
  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

  bBuffer_index++;
  if (bBuffer_index == pHandle->SpeedBufferSizeUnit)
  {
    bBuffer_index = 0U;
  }
  else
  {
    /* Nothing to do */
  }

  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  pHandle->Speed_Buffer_Index = bBuffer_index;
}

/**
  * @brief  Executes PLL algorithm for rotor position extraction from B-emf alpha and beta.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hBemf_alfa_est: Estimated Bemf alpha on the stator reference frame.
  * @param  hBemf_beta_est: Estimated Bemf beta on the stator reference frame.
  * @retval 
  */
inline static int16_t STO_VOLT_ExecutePLL(STO_VOLT_PLL_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est)
{
  int32_t wAlfa_Sin_tmp;
  int32_t wBeta_Cos_tmp;
  int16_t hAux1;
  int16_t hAux2;
  int16_t hOutput;
  Trig_Components Local_Components;

  Local_Components = MCM_Trig_Functions(pHandle->_Super.hElAngle);

  /* Alfa & Beta BEMF multiplied by Cos & Sin */
  wAlfa_Sin_tmp = ((int32_t )hBemf_alfa_est) * ((int32_t )Local_Components.hSin);
  wBeta_Cos_tmp = ((int32_t )hBemf_beta_est) * ((int32_t )Local_Components.hCos);

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  hAux1 = (int16_t)(wBeta_Cos_tmp >> 15); /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
#else
  hAux1 = (int16_t)(wBeta_Cos_tmp / 32768);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15); /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
#else
  hAux2 = (int16_t)(wAlfa_Sin_tmp / 32768);
#endif

  /* Speed PI regulator */
  hOutput = PI_Controller(& pHandle->PIRegulator, (int32_t)(hAux1 ) - hAux2);
  return (hOutput);
}

/**
  * @brief  Clears the estimated speed buffer in @p pHandle.
  *
  */
static inline void STO_VOLT_InitSpeedBuffer(STO_VOLT_PLL_Handle_t * pHandle)
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize = pHandle->SpeedBufferSizeUnit;

  /* Init speed buffer */
  for (b_i = 0U; b_i < bSpeedBufferSize; b_i++)
  {
    pHandle->Speed_Buffer[b_i] = (int16_t)0;
  }
  pHandle->Speed_Buffer_Index = 0U;
  pHandle->SpeedBufferOldestEl = (int16_t)0;
}

/**
  * @brief  Checks if the state observer algorithm converged. コンパイラをARM Compiler/GCCから変更する際は見直し必要
  * 
  * Internally performs a set of checks necessary to state whether
  * the state observer algorithm converged. To be periodically called
  * during motor open-loop ramp-up (e.g. at the same frequency of
  * SPD_CalcElAngle), it returns true if the estimated angle and speed
  * can be considered reliable, false otherwise.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hForcedMecSpeedUnit: Mechanical speed in 0.1Hz unit as forced by VSS.
  * @retval bool True if the estimated angle and speed are reliables, false otherwise.
  */
__weak bool STO_VOLT_PLL_IsObserverConverged(STO_VOLT_PLL_Handle_t *pHandle, int16_t *phForcedMecSpeedUnit)
{
  bool bAux = false;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == phForcedMecSpeedUnit))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wAux;
    int32_t wtemp;
    int16_t hEstimatedSpeedUnit;
    int16_t hUpperThreshold;
    int16_t hLowerThreshold; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

    if (true == pHandle->ForceConvergency2) /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    {
      *phForcedMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
    }
    else
    {
      /* Nothing to do */
    }

    if (true == pHandle->ForceConvergency)
    {
      bAux = true;
      pHandle->IsAlgorithmConverged = true;
      pHandle->_Super.bSpeedErrorNumber = 0U;
    }
    else
    {
      hEstimatedSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;

      wtemp = ((int32_t)hEstimatedSpeedUnit) * ((int32_t)*phForcedMecSpeedUnit);

      if (wtemp > 0)
      {
        if (hEstimatedSpeedUnit < 0)
        {
          hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
        }

        if (*phForcedMecSpeedUnit < 0)
        {
          *phForcedMecSpeedUnit = -*phForcedMecSpeedUnit;
        }
        wAux = ((int32_t)*phForcedMecSpeedUnit) * ((int16_t)pHandle->SpeedValidationBand_H);
        hUpperThreshold = (int16_t)(wAux / ((int32_t)16));

        wAux = ((int32_t)*phForcedMecSpeedUnit) * ((int16_t)pHandle->SpeedValidationBand_L);
        hLowerThreshold = (int16_t)(wAux / ((int32_t)16));

        /* If the variance of the estimated speed is low enough... */
        if (true == pHandle->IsSpeedReliable)
        {
          if ((uint16_t)hEstimatedSpeedUnit > pHandle->MinStartUpValidSpeed)
          {
            /* ...and the estimated value is quite close to the expected value... */
            if (hEstimatedSpeedUnit >= hLowerThreshold)
            {
              if (hEstimatedSpeedUnit <= hUpperThreshold)
              {
                pHandle->ConsistencyCounter++;

                /* ...for hConsistencyThreshold consecutive times... */
                if (pHandle->ConsistencyCounter >= pHandle->StartUpConsistThreshold)
                {

                  /* The algorithm converged */
                  bAux = true;
                  pHandle->IsAlgorithmConverged = true;
                  pHandle->_Super.bSpeedErrorNumber = 0U;
                }
              }
              else
              {
                pHandle->ConsistencyCounter = 0U;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0U;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0U;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0U;
        }
      }
      else
      {
        /* Nothing to do */
      }
    }
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
  return (bAux);
}

/**
  * @brief  Exports estimated Bemf alpha-beta from @p pHandle.
  * 
  * @retval alphabeta_t Bemf alpha-beta.
  */
__weak alphabeta_t STO_VOLT_PLL_GetEstimatedBemf(STO_VOLT_PLL_Handle_t *pHandle)
{
  alphabeta_t vaux;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    vaux.alpha = 0;
    vaux.beta = 0;
  }
  else
  {
#endif
    vaux.alpha = pHandle->hBemf_d_est;
    vaux.beta = pHandle->hBemf_q_est;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
  return (vaux);
}


/**
  * @brief  Exports from @p pHandle the stator current alpha-beta as estimated by state observer.
  * 
  * @retval alphabeta_t State observer estimated stator current Ialpha-beta.
  */
__weak alphabeta_t STO_VOLT_PLL_GetEstimatedCurrent(STO_VOLT_PLL_Handle_t *pHandle)
{
  alphabeta_t iaux;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    iaux.alpha = 0;
    iaux.beta = 0;
  }
  else
  {
#endif
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
  iaux.alpha = (int16_t)(pHandle->Ialfa_est >> pHandle->FV1LOG);
#else
  iaux.alpha = (int16_t)(pHandle->Ialfa_est / pHandle->hVF1);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
  /*cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6*/
  iaux.beta = (int16_t)(pHandle->Ibeta_est >> pHandle->FV1LOG);
#else
  iaux.beta = (int16_t)(pHandle->Ibeta_est / pHandle->hVF1);
#endif
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
  return (iaux);
}

/**
  * @brief  Exports current observer gains from @p pHandle and to parameters @p phC2 and @p phC4.
  * 
  */
__weak void STO_VOLT_PLL_GetObserverGains(STO_VOLT_PLL_Handle_t *pHandle, int16_t *phC2, int16_t *phC4)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == phC2) || (MC_NULL == phC4))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *phC2 = pHandle->hVC2;
    *phC4 = pHandle->hVC4;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Stores in @p pHandle the new values @p hhC1 and @p hhC2 for observer gains.
  * 
  */
__weak void STO_VOLT_PLL_SetObserverGains(STO_VOLT_PLL_Handle_t *pHandle, int16_t hhC1, int16_t hhC2)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hVC2 = hhC1;
    pHandle->hVC4 = hhC2;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Exports current PLL gains from @p pHandle to @p pPgain and @p pIgain.
  * 
  */
__weak void STO_VOLT_GetPLLGains(STO_VOLT_PLL_Handle_t *pHandle, int16_t *pPgain, int16_t *pIgain)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == pHandle) || (MC_NULL == pPgain) || (MC_NULL == pIgain))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *pPgain = PID_GetKP(& pHandle->PIRegulator);
    *pIgain = PID_GetKI(& pHandle->PIRegulator);
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Empty function. Could be declared to set instantaneous information on rotor mechanical angle.
  * 
  * Note: Mechanical angle management is not implemented in this
  * version of State observer sensor class.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  hMecAngle: Instantaneous measure of rotor mechanical angle.
  */
/*cstat !MISRAC2012-Rule-8.13 !RED-func-no-effect*/
__weak void STO_VOLT_PLL_SetMecAngle(STO_VOLT_PLL_Handle_t *pHandle, int16_t hMecAngle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if ((MC_NULL == (void *)pHandle) || ((int16_t)0 == hMecAngle))
  {
    /* Nothing to do */
  }
  else
  {
    /* nothing to do */
  }
#endif
}

/**
  * @brief  Resets the PLL integral term during on-the-fly startup.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_VOLT_OTF_ResetPLL(STO_Handle_t * pHandle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
  STO_VOLT_PLL_Handle_t *pHdl = (STO_VOLT_PLL_Handle_t *)pHandle->_Super; /*cstat !MISRAC2012-Rule-11.3*/
    PID_SetIntegralTerm(&pHdl->PIRegulator, (int32_t)0);
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Enables/Disables additional reliability check based on observed Bemf.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @param  bSel Enable/Disable check.
  */
__weak void STO_VOLT_PLL_BemfConsistencyCheckSwitch(STO_VOLT_PLL_Handle_t *pHandle, bool bSel)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->EnableDualCheck = bSel;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}
 /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
/**
  * @brief  Checks if the Bemf is consistent.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @retval bool True when observed Bemfs are consistent with expectation, false otherwise.
  */
__weak bool STO_VOLT_PLL_IsBemfConsistent(STO_VOLT_PLL_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  return ((MC_NULL == pHandle) ? false : pHandle->IsBemfConsistent);
#else
  return (pHandle->IsBemfConsistent);
#endif
}

/**
  * @brief  Checks the value of the variance.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  * @retval bool True if the speed measurement variance is lower than threshold VariancePercentage, false otherwise.
  */
__weak bool STO_VOLT_PLL_IsVarianceTight(const STO_Handle_t *pHandle) /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
{
  bool tempStatus;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    tempStatus = false;
  }
  else
  {
#endif
  const STO_VOLT_PLL_Handle_t *pHdl = (STO_VOLT_PLL_Handle_t *)pHandle->_Super; /*cstat !MISRAC2012-Rule-11.3*/
    tempStatus = pHdl->IsSpeedReliable;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  return (tempStatus);
} /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

/**
  * @brief  Forces the state-observer to declare convergency.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_VOLT_PLL_ForceConvergency1(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
  STO_VOLT_PLL_Handle_t *pHdl = (STO_VOLT_PLL_Handle_t *)pHandle->_Super; /*cstat !MISRAC2012-Rule-11.3*/
    pHdl->ForceConvergency = true;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Forces the state-observer to declare convergency.
  * 
  * @param  pHandle: Handler of the current instance of the STO component.
  */
__weak void STO_VOLT_PLL_ForceConvergency2(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
  STO_VOLT_PLL_Handle_t *pHdl = (STO_VOLT_PLL_Handle_t *)pHandle->_Super; /*cstat !MISRAC2012-Rule-11.3*/
    pHdl->ForceConvergency2 = true;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  Sets the rotation @p direction in @p pHandle.
  */
__weak void STO_VOLT_SetDirection(STO_VOLT_PLL_Handle_t *pHandle, int8_t direction)
{
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hForcedDirection = direction;
#ifdef NULL_PTR_CHECK_STO_PLL_SPD_POS_FDB
  }
#endif
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
