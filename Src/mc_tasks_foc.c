
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mc_app_hooks.h"
#include "motor_process.h"
#include "dead_zone_compensation.h"
#include "max_torque_per_ampere.h"

/* USER CODE BEGIN Includes */
#include "motor_error.h"
#include "lowspeed_ctrl.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
#define FDBK_ERR_NUM              2000            /* 運転中のフィードバック異常判定時間: 2s */
#define STRAT_ERR_NUM             500             /* 起動中の異常判定時間: 500ms */
#define SPEED_PID_MIN             (0 * SPEED_UNIT)  /* 速度PI切替の下限速度: 0rps（SPEED_UNIT形式） */
#define SPEED_PID_MAX             (80 * SPEED_UNIT) /* 速度PI切替の上限速度: 80rps（SPEED_UNIT形式） */
#define SPEED_PID_SHIFT_LOW       (25 * SPEED_UNIT) /* 速度PI切替ポイント(LOW): 25rps（SPEED_UNIT形式） */
#define SPEED_PID_SHIFT_MID       (49 * SPEED_UNIT) /* 速度PI切替ポイント(MID): 49rps（SPEED_UNIT形式） */
#define SPEED_PID_CURR_MID        (32 * SPEED_UNIT) /* 電流PI切替ポイント(MID): 32rps（SPEED_UNIT形式） */
#define SPEED_PID_CURR_LOW        (26 * SPEED_UNIT) /* 電流PI切替ポイント(LOW): 26rps（SPEED_UNIT形式） */
#define SPEED_FLT_SHIFT           (45 * SPEED_UNIT) /* 速度フィルタ係数切替ポイント: 45rps（SPEED_UNIT形式） */
#define OBS_SHIFT_ANGLE           (int16_t)((int32_t)(5)* 65536/360)      /* オブザーバ切替角度しきい値: 5度 */
/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0.000\
                                      * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0\
                                      * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))

/* USER CODE BEGIN Private Variables */
extern uint8_t ucg_LOW_LowSpdCtrlState;
/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */
    MTPA_init(&MTPA_M1);
    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R1_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    /*STO_PLL_Init (&STO_PLL_M1);*/

    STO_VOLT_PLL_Init (&STO_VOLT_PLL_M1);
    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    /*STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_PLL_M1._Super);*/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_VOLT_PLL_M1._Super);
    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    /*******************************************************/
    /*   Flux weakening component initialization           */
    /*******************************************************/
    PID_HandleInit(&PIDFluxWeakeningHandle_M1);
    FW_Init(pFW[M1],&PIDSpeedHandle_M1,&PIDFluxWeakeningHandle_M1);
    /*******************************************************/
    /*   dead zone compensation initialization           */
    /*******************************************************/
    dead_zone_comp_Init(&dead_zone_comp_M1);

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
    R1_SwitchOffPWM(pwmcHandle[motor]);

  FOC_Clear(motor);

  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  /*int16_t wAux = 0;*/
  /*bool IsSpeedReliable = STO_PLL_CalcAvrgMecSpeedUnit(&STO_PLL_M1, &wAux);*/
  int16_t wAux1 = 0;
  (void)STO_VOLT_PLL_CalcAvrgMecSpeedUnit(&STO_VOLT_PLL_M1, &wAux1);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
              R1_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R1_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R1_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );

              /*STO_PLL_Clear(&STO_PLL_M1);*/
              STO_VOLT_PLL_Clear(&STO_VOLT_PLL_M1);
              FOC_Clear( M1 );
              TSK_SetStartFlagDelayM1(STARTFLAG_TICKS);
              Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

            /* Execute the Rev Up procedure */
            if(! RUC_Exec(&RevUpControlM1))
            {
            /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /*��λ�׶���δ����������۲�������*/
              if(false == RUC_FirstAccelerationStageReached(&RevUpControlM1))
              {
                STO_VOLT_PLL_Clear(&STO_VOLT_PLL_M1);
              }
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M1]);
              IqdRef.d = FOCVars[M1].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M1].Iqdref = IqdRef;
           }

            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);

            /* ��λ�׶ν�������ʼ���� */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
            {
                MC_ProgramSpeedRampMotor1(SPD_START * SPEED_UNIT,200); 		

#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1)) / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */

                /* USER CODE END MediumFrequencyTask M1 1 */
                STC_SetSpeedSensor(pSTC[M1], &STO_VOLT_PLL_M1._Super); /* �۲�������Ϊ��̬��ѹ�۲��� */
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                Mci[M1].State = SWITCH_OVER;
                break;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

              MCI_ExecBufferedCommands(&Mci[M1]);

              FOC_CalcCurrRef(M1);
          }
          if(TSK_StartFlagHasElapsedM1()==true)                   /*�����׶ν�����������ת�׶�*/
          {
            Mci[M1].State = RUN;
          }
          else
          {
            break;
          }
         
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

              MCI_ExecBufferedCommands(&Mci[M1]);   /*ִ��ָ������*/

              FOC_CalcCurrRef(M1);
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  STC_Clear(pSTC[bMotor]);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  if (NULL == pFW[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FW_Clear(pFW[bMotor]);
  }

  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;
  int16_t hTargetSpeed = (int16_t)0;

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    /*Speed Kp/Ki adjust*/
    if(RUN == Mci[M1].State)          /*Run state*/
    {
      hTargetSpeed = MC_GetMecSpeedReferenceMotor1();
      vLowSpeed_Task_1ms(SPD_GetAvrgMecSpeedUnit(pSTC[M1]->SPD), hTargetSpeed);     /* 低速制御中　1ms処理 */
      if(hTargetSpeed<= SPEED_PID_SHIFT_LOW)    /*Speed < 25rps*/
      {
        if(ucg_LOW_LowSpdCtrlState == LOWSPD_STATE_ECO_ENABLE){                     /* 低速制御中　省エネ制御許可判定状態の場合　→　速度制御ゲインを低く設定 */
          PID_SetKP(pSTC[bMotor]->PISpeed,PID_SPEED_KP_RUN_ECO_CTRL);
          PID_SetKI(pSTC[bMotor]->PISpeed,PID_SPEED_KI_RUN_ECO_CTRL);
        }else{
          PID_SetKP(pSTC[bMotor]->PISpeed,PID_SPEED_KP_RUN_LOW);
          PID_SetKI(pSTC[bMotor]->PISpeed,PID_SPEED_KI_RUN_LOW);
        }
      }
			else if(hTargetSpeed <= SPEED_PID_SHIFT_MID) 
			{
			  PID_SetKP(pSTC[bMotor]->PISpeed,PID_SPEED_KP_RUN_MID);
        PID_SetKI(pSTC[bMotor]->PISpeed,PID_SPEED_KI_RUN_MID);
			}
      else
      {
        PID_SetKP(pSTC[bMotor]->PISpeed,PID_SPEED_KP_RUN_HIGH);
        PID_SetKI(pSTC[bMotor]->PISpeed,PID_SPEED_KI_RUN_HIGH);
      }


      
    }
    else          /*Start state*/
    {
      PID_SetKP(pSTC[bMotor]->PISpeed,PID_SPEED_KP_DEFAULT);
      PID_SetKI(pSTC[bMotor]->PISpeed,PID_SPEED_KI_DEFAULT);
    }
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;
    if(RUN == Mci[M1].State)          /*Run state*/
      {
	    	MTPA_CalcCurrRefFromIq(&MTPA_M1, &IqdTmp);
      }
    else
      {
         IqdTmp.d = FOCVars[bMotor].UserIdref;
      }



    if (NULL == pFW[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
    }
  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
  int32_t wAux;
  /* USER CODE BEGIN HighFrequencyTask 0 */
  int16_t hSpeedref;
  /* USER CODE END HighFrequencyTask 0 */

  Observer_Inputs_t STO_Inputs; /* Only if sensorless main */

  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
  if (SWITCH_OVER == Mci[M1].State)
  {
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
    STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
    STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
    STO_Inputs.Iqd = FOCVars[M1].Iqd;
    STO_Inputs.Vdq = FOCVars[M1].Vqd;
      
    hSpeedref = (int16_t)(pSTC[M1]->SpeedRefUnitExt >> 16);

    (void)STO_VOLT_PLL_CalcElAngle (&STO_VOLT_PLL_M1, &STO_Inputs);

    STO_VOLT_PLL_CalcAvrgElSpeedDpp (&STO_VOLT_PLL_M1, hSpeedref); /*  Only in case of Sensor-less */
    /* Only for sensor-less */
    if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State))
    {
      if (START == Mci[M1].State )
      {
        if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 1);
        }
      }
      else
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
      }

      int16_t hObsAngle = SPD_GetElAngle(&STO_VOLT_PLL_M1._Super); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */
    
    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }

  return (bMotorNbr);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
inline uint16_t FOC_CurrControllerM1(void) /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError;
  int16_t hMeasuredSpeed_Instantaneous;
  SpeednPosFdbk_Handle_t *speedHandle;
  SpeednTorqCtrl_Handle_t *TorqCtrlHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  hMeasuredSpeed_Instantaneous = SPD_GetAvrgMecSpeedUnit(pSTC[M1]->SPD);
  vLowSpeed_Task_200us(hElAngle, hMeasuredSpeed_Instantaneous); /* 低速制御中　200μs処理 */
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  RCM_ExecNextConv();
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
  int16_t hTargetSpeed = STC_GetMecSpeedRefUnit(pSTC[M1]); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
		if(TSK_StartFlagHasElapsedM1()==true)
	{
	  	if(hTargetSpeed < SPEED_PID_CURR_LOW )

	{
			pPIDIq[M1]->hKpGain=PID_TORQUE_KP_LOW;   
			pPIDIq[M1]->hKiGain=PID_TORQUE_KI_LOW;   
	  	pPIDId[M1]->hKpGain=PID_FLUX_KP_LOW;    
			pPIDId[M1]->hKiGain=PID_FLUX_KI_LOW;	
	}
	else if(hTargetSpeed <= SPEED_PID_CURR_MID)
	{
			pPIDIq[M1]->hKpGain=PID_TORQUE_KP_MID;   
			pPIDIq[M1]->hKiGain=PID_TORQUE_KI_MID;   
	  	pPIDId[M1]->hKpGain=PID_FLUX_KP_MID;    
			pPIDId[M1]->hKiGain=PID_FLUX_KI_MID;
	}
	else
  {
		  pPIDIq[M1]->hKpGain=PID_TORQUE_KP_HIGH;   
			pPIDIq[M1]->hKiGain=PID_TORQUE_KI_HIGH;   
	  	pPIDId[M1]->hKpGain=PID_FLUX_KP_HIGH;    
			pPIDId[M1]->hKiGain=PID_FLUX_KI_HIGH;
  }

	}
	else
	{
			pPIDIq[M1]->hKpGain=PID_TORQUE_KP_DEFAULT;   
			pPIDIq[M1]->hKiGain=PID_TORQUE_KI_DEFAULT;   
	  	pPIDId[M1]->hKpGain=PID_FLUX_KP_DEFAULT;    
			pPIDId[M1]->hKiGain=PID_FLUX_KI_DEFAULT;		
	}
  Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
  Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  RCM_ReadOngoingConv();
  uint16_t hDeadZoneVbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  abc_t Vabc = dead_zone_compensation(&dead_zone_comp_M1,Iab,hDeadZoneVbus); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  alphabeta_t Valphabeta_Comp = MCM_Clarke_Comp(Vabc); /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  Valphabeta = voltage_limit(Valphabeta_Comp,Valphabeta);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta,Iab);
  PWMC_CalcPhaseCurrentsEst(pwmcHandle[M1],Iqd, hElAngle);

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  FW_DataProcess(pFW[M1], Vqd);
  return (hCodeError);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
