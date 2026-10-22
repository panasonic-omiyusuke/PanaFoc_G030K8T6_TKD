/**
  ******************************************************************************
  * @file    max_torque_per_ampere.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Maximum Torque Per Ampere (MTPA) Control component of the Motor Control SDK:
  *
  *           * Initialize the parameter for  MTPA
  *           * Calculate and output id and iq reference based on torque input
  *           * Calculate and output id based on iq input
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * @ingroup MTPA
  */

/* Includes ------------------------------------------------------------------*/
#include "max_torque_per_ampere.h"
#include <stdint.h>
/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MTPA Maximum Torque Per Ampere Control
  * @brief Maximum Torque Per Ampere (MTPA) Control component of the Motor Control SDK
  *
  *  The torque of the PMSM can be expressed with the equation shown below:
  *
  *  @f[
  *  T_e = \frac{3}{2}\times p \times \phi \times i_q + \frac{3}{2}\times p\times(L_d-L_q)\times i_q\times i_d
  *  @f]
  *
  *    When the motor is a surface mount permanent magnet synchronous motor (SM-PMSM), the @f$ L_d @f$ and @f$ L_q @f$
  *  are almost the same, so only the @f$ i_q @f$ can influence the torque. For internal permanent
  *  magnet synchronous motor (I-PMSM), the @f$ L_d @f$ is not equal to @f$ L_q @f$. Both @f$ i_d @f$ and @f$ i_q @f$ will influence
  *  the torque.
  *    The aim of the MTPA (maximum-torque-per-ampere) control is to calculate the
  *  reference currents which maximize the ratio between produced
  *  electromagnetic torque and copper losses.
  *    The input of this component
  *  is the @f$ i_q @f$ reference. The output of this component is @f$ i_d @f$ reference.
  *
  * @{
  */
MTPA_Handle_t MTPA_M1;
static int16_t segment = 0;
/**
  * @brief  Calculates the Id current reference based on input Iq current reference
  * @param  pHandle Handle on the MTPA component
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed
  *         in the qd_t format.
  *
  */
void MTPA_init(MTPA_Handle_t *pHandle)
{

  pHandle->SegDiv = (int16_t)SEGDIV;
  pHandle->AngCoeff[0] = 0;
  pHandle->AngCoeff[1] = -3;
  pHandle->AngCoeff[2] = -11;
  pHandle->AngCoeff[3] = -26;
  pHandle->AngCoeff[4] = -46;
  pHandle->AngCoeff[5] = -72;
  pHandle->AngCoeff[6] = -103;
  pHandle->AngCoeff[7] = -140;
  pHandle->AngCoeff[8] = -182;
  pHandle->AngCoeff[9] = -230;
  pHandle->AngCoeff[10] = -283;
  pHandle->AngCoeff[11] = -342;
  pHandle->AngCoeff[12] = -406;
  pHandle->AngCoeff[13] = -474;
  pHandle->AngCoeff[14] = -548;
  pHandle->AngCoeff[15] = -627;
  pHandle->AngCoeff[16] = -710;
  pHandle->AngCoeff[17] = -794;
  pHandle->AngCoeff[18] = -879;
  pHandle->AngCoeff[19] = -965;
  pHandle->AngCoeff[20] = -1052;
  pHandle->AngCoeff[21] = -1140;
  pHandle->AngCoeff[22] = -1229;
  pHandle->AngCoeff[23] = -1319;
  pHandle->AngCoeff[24] = -1410;
  pHandle->AngCoeff[25] = -1502;
  pHandle->AngCoeff[26] = -1595;
  pHandle->AngCoeff[27] = -1689;
  pHandle->AngCoeff[28] = -1784;
  pHandle->AngCoeff[29] = -1880;
  pHandle->AngCoeff[30] = -1977;
  pHandle->AngCoeff[31] = -2075;
  pHandle->AngCoeff[32] = -2174;
  pHandle->AngCoeff[33] = -2274;
  pHandle->AngCoeff[34] = -2375;
  pHandle->AngCoeff[35] = -2477;
  pHandle->AngCoeff[36] = -2580;
  pHandle->AngCoeff[37] = -2684;
  pHandle->AngCoeff[38] = -2789;
  pHandle->AngCoeff[39] = -2895;
  pHandle->AngCoeff[40] = -3002;
  pHandle->AngCoeff[41] = -3110;
  pHandle->AngCoeff[42] = -3219;
  pHandle->AngCoeff[43] = -3329;
  pHandle->AngCoeff[44] = -3440;
  pHandle->AngCoeff[45] = -3552;
  pHandle->AngCoeff[46] = -3665;
  pHandle->hId_lowpass = 0;
}

__weak void MTPA_CalcCurrRefFromIq(MTPA_Handle_t *pHandle, qd_t *Iqdref)
{
#ifdef NULL_PTR_CHECK_MAX_TRQ_PER_AMP
  if ((NULL == pHandle) || (NULL == Iqdref))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int16_t id;
    //int16_t aux;
    int16_t iq;
    //uint8_t segment;

    iq = ((Iqdref->q < 0) ? (-Iqdref->q) : (Iqdref->q));       /* Teref absolute value */

    if(iq > ((segment + 1) * pHandle->SegDiv))
    {
        segment += 1;
    }
    else if(iq < ((segment * pHandle->SegDiv) - MTPA_HYSTERESIS))
    {
        segment -=1;
    }
    else
    {
    }
    if(segment < 0)
    { 
       segment = 0;
    }
    else
    {
    }

    if (segment > SEGMENT_NUM)
    {
      segment = SEGMENT_NUM;
    }
    else
    {
      /* Nothing to do */
    }

/*#ifndef FULL_MISRA_C_COMPLIANCY_MAX_TOR
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    id = ((pHandle->AngCoeff[segment] * iq) >> 15) + pHandle->Offset[segment];
#else
    id = ((pHandle->AngCoeff[segment] * iq) / 32768) + pHandle->Offset[segment];
#endif
    Iqdref->d = (int16_t)id;*/
    id = pHandle->AngCoeff[segment];
    MTPA_Id_lowpass(&MTPA_M1,id);
    Iqdref->d = pHandle->hId_lowpass;
#ifdef NULL_PTR_CHECK_MAX_TRQ_PER_AMP
  }
#endif
}


void MTPA_Id_lowpass(MTPA_Handle_t *pHandle,int16_t Id_raw)
{

    int32_t wAux;
    int32_t lowPassFilterBW = (int32_t)255 ;
    //static int16_t Id_Lowpass = 0;

    wAux = (int32_t)(pHandle->hId_lowpass) * lowPassFilterBW;
    wAux += Id_raw;
    pHandle->hId_lowpass = (int16_t)(wAux >> 8);
    //return pHandle->hId_lowpass;

}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
