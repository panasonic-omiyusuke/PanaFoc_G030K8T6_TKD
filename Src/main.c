/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_process.h"
#include "Sys_type.h"
#include "Dtyctrl.h"
#include "Timea.h"
#include "Torque.h"
#include "Trans_Inv.h"
#include "Speedcnt.h"	/* 速度カウンタ処理 *//* @259swDOD[23/06/30] */
#include "angle_control.h"
#include "system_ram.h"
#include "InvCommonFunc.h"
#include "Inv_fct.h"
#include "RefSpeed.h"
#include "brake.h"
#include "Softr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch4;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart1;
/*UART_HandleTypeDef huart2;*/
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
static void vSPD_set(void);
/* USER CODE BEGIN PFP */
uint8_t ucTRNS_sfrq_dec;
UI08 ucMAIN_chkint;
uint8_t u8g_TRNSset_spd_flg;		/*メインからの回転数指令セット確認フラグ*//*マイコン起動時からの通信異常カウントのために必要*/
#ifdef DEBUG /*DEBUG*/
UI08 test_chg_speed_flg = 1;
UI08 test_speed = 0x00;
#endif /*DEBUG*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**************************************************************************
<Function>		:異常時ＳＷリセット処理(強制的にマイコンリセットさせる)
<Author>		:Takida
<Date>			:2022.02.08
<Update>		:Ver REF081-0.00 2008-06-28 -Outi ﾙﾈｻｽ指摘0627対応(機能向上) @005awORM[08/06/27]
 **************************************************************************/
void vMAIN_swreset(void){					/* 異常時ＳＷリセット処理 */
 #ifdef RENESAS_TO_ST32
	HAL_NVIC_SystemReset();					/* システムリセット処理 Initiate a system reset request to reset the MCU. */
	while(1)								/* ソフトウェアリセット動作までの待機処理	 */
	{										/* 動作まで本関数から脱出しないようにして	 */
    ;									/* 異常状態での通常処理実行を回避する。		 */
	}										/* 万一無限loop陥ってもWDT動作でSWﾘｾｯﾄ同様	 */
 #else	/* RENESAS_TO_ST32 */
	ucCPU_PRCR = 0x02;						/* protect OFF */
	PM0_addr.bit.b3 = 1;					/* ソフトウェアリセット */
	ucCPU_PRCR = 0x00;						/* protect ON */
	while(1)								/* ソフトウェアリセット動作までの待機処理	 */
	{										/* 動作まで本関数から脱出しないようにして	 */
		;									/* 異常状態での通常処理実行を回避する。		 */
	}										/* 万一無限loop陥ってもWDT動作でSWﾘｾｯﾄ同様	 */
 #endif	/* RENESAS_TO_ST32 */
}

/**************************************************************************
<Function>		:ウォッチドッグタイマのリセット処理
<Author>		:Takeoka
<Date>			:2021.12.02
<Update>		:2022.02.02-Takida
 **************************************************************************/
void ResetWatchDogTimer(void){
	if(ucMAIN_wdrfg == 0){					/*正常なら */
		HAL_IWDG_Refresh(&hiwdg);
	}
}

/**************************************************************************
<Function>		:RAMクリア処理
<Author>		:Takeoka
<Date>			:2021.12.02
<Update>		:
 **************************************************************************/
void ClearAllRam(void)	{
	uint32_t * ram_address;
  ram_address = (uint32_t *)RAM_data_END_ADDRESS;	/* .data領域はクリア対象外（.mapのRW_IRAM1内 Data RW終端を使用） */
                      /* ※ .data増減時は RAM_data_END_ADDRESS を更新すること */
	while(ram_address < (uint32_t *)(RAM_ACCESS_END_ADDRESS)){ /* pipe line 処理になっているため、最後のアドレスで比較すると、範囲外にアクセスしてしまうため、手前で停止 */
		*ram_address = 0;
		ram_address++;			/* クリアするRAMアドレスを1つ(4byte)進める */
	}
	/* RAMの残りをクリア */
	*(uint32_t *)(RAM_ACCESS_END_ADDRESS1) = 0;
	*(uint32_t *)(RAM_ACCESS_END_ADDRESS2) = 0;
	*(uint32_t *)(RAM_ACCESS_END_ADDRESS3) = 0;
	/*さくじょはじめ*/
  /*	*(uint32_t *)(0x20008ffc) = 0; */
	/*さくじょおわり*/
}

/**************************************************************************
<Function>		:回転数指令更新処理
<Description>	:前回のdec指令を保持後、通信受信値(BCD)をdecへ変換して回転数指令へ反映
 **************************************************************************/
static void vSPD_set(void)
{
  UI08 u8l_speed_cmd;
  static UI08 u8l_speed_cmd_offline;    /*断続運転中のコンプON回転数(rps)*/
  
  /*-----回転数設定-----*/
  ucTRNS_sfrq_dec = vSPD_Bcd2nrm(ucTRNS_sfrq_b);
  u8l_speed_cmd = MOTOR_SelectSpeedCommand(u8g_MOTOR_ctrlflg, u8g_TRNSset_spd_flg, ucTRNS_sfrq_dec, u8l_speed_cmd_offline);  /* オンライン/オフラインに応じた回転数を設定 */
  u8l_speed_cmd = SpdLimitOLP(u8l_speed_cmd);	/* OLP制限/復帰時の速度制限 */
  u8l_speed_cmd = MOTOR_SanitizeSpeedCommand(u8l_speed_cmd);	/* 回転数設定&ModeTime変換処理 */
  u8l_speed_cmd = SpdLimitDline(u8l_speed_cmd);	/* Dライン共振保護制限 */

  u8g_SPD_cmd = u8l_speed_cmd;

   /*-----前回回転数の保持および断続運転中のコンプON回転数の設定-----*/
  if(u8g_TRNSset_spd_flg != 0){            	/* マイコン起動後に一度でも回転数セットしているなら */
    if(u8l_speed_cmd != SPD_ZERO){          /* 前回回転数がゼロでないなら */
      u8l_speed_cmd_offline = u8l_speed_cmd;/* 前回回転数を断続運転中のコンプON回転数としてセット */
    }else{                                  /* 前回回転数がゼロなら */
        /* 何もしない（前回回転数がゼロなら断続運転中のコンプON回転数は変更しない。メインからの回転数ゼロ指令を格納しないようにするため。） */
    }
  }else{	                                /* メインからの回転数指令がセットされていない(マイコン起動してから一度も回転数セットしていない)なら */
    u8l_speed_cmd_offline = SPD_DEFAULT;    /* デフォルト回転数（断続運転回転数）をセット */
  } 
}
/*ついかおわり*/
UCHAR checknum;
/*------------------------------------------------------------------------------*/
/*  関数名      : main() */
/* @brief      : スタートアップ完了後の基板の設定を行い、プログラムを実行するための土台の処理※一部のフラグや変数について、値の代入がそのまま存在するため、関数化を検討する */
/* @li           (起動時)ブレーキ時の処理が初期化されていること( Brake_Init() ) */
/* @li           (起動時)動作フラグ(F_WakeUp_MainTm, ucr_Fg_StartUp等)が初期化されていること */
/* @li           (暫定) - STのハードウェア初期化が問題なく行われていること( HAL_Init() ) */
/* @li           (暫定) - MCUのクロック設定が意図通りの設定が行われていること(SystemClock_Config() ->PLL - 48MHz) */
/* @li           メインタイマの割込みが禁止されること ( cSYSTIC_IRQ_DIS() ) */
/* @li           パラメータデータが意図通りに設定されていること( vEEPROM_init_param_read() / ConvertSet() / ModeSet() ) */
/* @li           各種ペリフェラルの設定が意図通りに行われていること (Tuning_init() / PMD1G_Init() / Timer_Init()) */
/* @li           メインタイマの開始から1s間待機すること(LED点灯) */
/* @li           各機能(PWM, ADトリガ) を開始したときに、意図通りに開始されること( Drive_SetReg_Pmd1g() ) */
/* @li           メインループを継続すること、意図しないmain処理からの脱出がないこと( main_loop() ) */
/* @param [in] : none */
/* @return     : none */
/* @date       : 22/06/06 */
/* @author     : */
/*------------------------------------------------------------------------------*/
/* @remark (変更履歴) */
/* @remark     : 2022/10/13_CHG013 デバッグ出力機能を追加 */
/*------------------------------------------------------------------------------*/
int main(void)
{

  /* USER CODE BEGIN 1 */
  /*------500ms待機------*/
  uint16_t u16l_delaycnt;
  uint16_t u16l_delaynum;
  for(u16l_delaynum = 0;u16l_delaynum < 125;u16l_delaynum++)          
  {
    for(u16l_delaycnt = 0;u16l_delaycnt <= 16000;u16l_delaycnt++)
    {
      __NOP();
    }
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* HAL_GPIO_WritePin(GPIOB, debug_SPIN1_Pin, GPIO_PIN_RESET); */ /* A */
  /* USER CODE BEGIN Init */
  ClearAllRam();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_MotorControl_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  Ram_Init();
  __enable_irq();
  MX_USART1_UART_Init();

  /*from HY3_RIBU_sin_soft*/
  /*------------------------------------------------
  送信データリフレッシュ処理
  ------------------------------------------------*/
	vTRNS_Outputi();						/*送信ﾃﾞｰﾀｾｯﾄ処理 */ /*＃ 午後 02:26 REF061-0.01 */

	ucg_TR_stbyct = 0;						/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀをｸﾘｱ(初期化)  */	/* @086sxDOD[20/04/10] */
	ucMINtm_stbyct = 0;						/*ｽﾀﾝﾊﾞｲ連続異常計測ﾀｲﾏｶｳﾝﾀをｸﾘｱ */		/* @088swRRD[20/04/22] */
	/*------------------------------------------------
		メイン通信確定フラグ初期化
	------------------------------------------------*/
	#ifdef ONECHIP	/*ﾒｲﾝ-INV 1ﾏｲｺﾝ*/														/* @002MxOOM[17/02/21] */
			ucTRNS_sfrg &= 0xef;				/* 通信ﾃﾞｰﾀ 強制確定(0設定)				*/
	#else			/*INV専用ﾏｲｺﾝ */
			ucTRNS_sfrg |= 0x10;				/*通信ﾃﾞｰﾀ 確定処理準備(1設定)			*/ /* @001MzDOD[16/09/01] */
	#endif

	while ((un100MSt_typect <= MAINAPP_UART_WAIT_100MS_COUNT_MAX) &&
         ((ucTRNS_sfrg & MAINAPP_TRNS_READY_MASK) == MAINAPP_TRNS_READY_MASK)){  /*100msec * 6000回 = 600sec or 通信データ未確定 */ /* @001MzDOD[16/09/01] */
		
		ucTYPE_uart	= 0;				/*通信機種判別"未"確定	*/
		/* ------------------ ｽﾀｯｸﾁｪｯｸ処理 ------------------> */
    	if( *(uint32_t *)(STACK_ADDRESS) != 0 ){					/* ｽﾀｯｸﾁｪｯｸ 0x20001A40(STM32G030K8T6) */
			vMAIN_swreset();						/* 異常時⇒ﾘｾｯﾄ */ /* @005awORM[08/06/27] */
		}
		/*asm("MOV.W	#20008A40H,A0");*/			/* ｽﾀｯｸﾁｪｯｸ 0x20008A40(STM32G071C8T6M) */
		/*asm("MOV.B	[A0],R0L");*/					/* R0L←0(ｽﾀｯｸ非accessﾃﾞｰﾀ) */
		/*asm("CMP.B	#0,R0L");*/					/* stack ﾁｪｯｸ */
		/*asm("JNE	_vMAIN_swreset");*/			/* 異常時⇒ﾘｾｯﾄ */ /* @005awORM[08/06/27] */
		/* <------------------ ｽﾀｯｸﾁｪｯｸ処理 ------------------ */
		vMAIN_Softr();

        while((ucMAIN_10up & 0x01) == 0){	/*10msタイムアップまで待機 */
                if(ucg_2ms_flg == true){	/* 2ms check */

                    vTIME_Timea();			/*タイマ制御処理 */
                    vTRNS_Outputi();		/*送信ﾃﾞｰﾀｾｯﾄ処理 */
                    vTRNS_Cnttrns();		/*通信ﾀｲﾏｶｳﾝﾄ処理 */
                    ucg_2ms_flg = false;

                    MainLoop_PulseOutput();
                }							/* */
                vTRNS_Hw_trns();			/*UART通信処理(2Chip用) 1Chip用では本処理通過しないので不要 */	/* @000MzDOD[10/08/26]_07 */
                
                vMAIN_Msec10();				/*10msec 判断 */
                HAL_IWDG_Refresh(&hiwdg);
            }					 			/* */
            ucMAIN_10up &= 0xfe;			/*10msﾀｲﾑｱｯﾌﾟﾌﾗｸﾞ←ｸﾘｱ */
    /*			vMAIN_Input();			*/		/*入力処理			( p_inpt ) */	/* @001MzDOD[16/09/01] */
#ifdef DEBUG /*DEBUG*/
       un100MSt_typect = 6001;  /*デバッグ*/
        ucTRNS_sfrg &= ~(0x10);			/*通信データ確定状態セット*//* デバッグ */
        }
        //ucTRNS_type = 0x0C; /* TKD100E20PBF用 */
        ucTRNS_type = 0x0E; /* TKD100E23DAH用 */
#else  /*DEBUG*/ 
        }
#endif /*DEBUG*/ 
	vMAIN_type_uart();					/*通信機種判定処理 */

	ucg_TR_stbyct = 0;						/*ｽﾀﾝﾊﾞｲ異常判定ｶｳﾝﾀをｸﾘｱ(初期化)  */					
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    ResetWatchDogTimer();
    main_loop();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**************************************************************************
<Title>			:vMAIN_Msec10 		;main
<Function>		:10ms 監視処理
<Description>	:	 _Timea()subﾙｰﾁﾝvMAIN_Cnt10msでSETされる従ﾌﾗｸﾞを監視し"0"になった時に主ﾌﾗｸﾞSET
<Input>			:ucMAIN_10up
<Return>		:ucMAIN_10up
<Author>		:-Outi	作成
<Date>			:2002-0215
<Update>		:Ver REF02A-0.00 2002-03-26
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void  vMAIN_Msec10(void){					/*10ms 監視処理                              */

	if((ucMAIN_10up & 0x12) == 0x12){		/*10ms経過ﾌﾗｸﾞ(従1)&&(従4)なら               */
		ucMAIN_10up = 0x01;					/*10ms経過ﾌﾗｸﾞ(主)←ｾｯﾄ (従)←全ｸﾘｱ          */
	}else if((ucMAIN_10up & 0x82) == 0x82){	/*10ms経過ﾌﾗｸﾞ(従1)&&(従7)なら               */
		ucMAIN_10up = 0x01;					/*10ms経過ﾌﾗｸﾞ(主)←ｾｯﾄ (従)←全ｸﾘｱ          */
	}else if((ucMAIN_10up & 0x90) == 0x90){	/*10ms経過ﾌﾗｸﾞ(従4)&&(従7)なら               */
		ucMAIN_10up = 0x01;					/*10ms経過ﾌﾗｸﾞ(主)←ｾｯﾄ (従)←全ｸﾘｱ          */
	}else{									/*10ms経過ﾌﾗｸﾞ(従)が２つ以上立っていないなら */
		ucMAIN_10up &= 0xfe;		}		/*10ms経過ﾌﾗｸﾞ(主)←ｸﾘｱ                          */

}

/**************************************************************************
<Title>			:mainloop 		;main
<Function>		:ﾒｲﾝﾙｰﾌﾟ処理
<Description>	:	 ｿﾌﾄ ﾘｾｯﾄ処理、基準ﾀｲﾏ2ms毎割込ﾌﾗｸﾞ←ｸﾘｱ処理、通信処理、10msec 判断処理、入力処理、ｷｬﾘｱ周期 set
<Input>			:
<Return>		:
<Author>		:-Tomo	作成
<Date>			:2007-0418
<Update>		:Ver REF070-0.04 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF070-0.10 2007-05-17 -Outi 山田ＧＭご指摘対応、機種切換（通信）ループにタイムセーフ追加
				:Ver REF081-0.03 2008-06-28 -Outi ﾙﾈｻｽ指摘0627対応(機能向上) @005awORM[08/06/27]
				:10/09/29 BW465 ｽﾀｯｸﾁｪｯｸｱﾄﾞﾚｽ変更 岩崎
				:Ver REF1010-0.01 2010-12-13 -Mura 通信関数名変更 @038MwDOD[10/12/13]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void main_loop( void )
{
	/* ------------------ ｽﾀｯｸﾁｪｯｸ処理 ------------------> */
	if( *(uint32_t *)(STACK_ADDRESS) != 0 ){					/* ｽﾀｯｸﾁｪｯｸ 0x20001A40(STM32G030K8T6) */
		vMAIN_swreset();						/* 異常時⇒ﾘｾｯﾄ */ /* @005awORM[08/06/27] */
	}
  /* asm("MOV.W	#20008A40H,A0"); */			/* ｽﾀｯｸﾁｪｯｸ 0x20008A40(STM32G071C8T6M) */
  /* asm("MOV.B	[A0],R0L"); */					/* R0L←0(ｽﾀｯｸ非accessﾃﾞｰﾀ) */
  /* asm("CMP.B	#0,R0L"); */					/* stack ﾁｪｯｸ */
  /* asm("JNE	_vMAIN_swreset"); */			/* 異常時⇒ﾘｾｯﾄ */ /* @005awORM[08/06/27] */
	/* <------------------ ｽﾀｯｸﾁｪｯｸ処理 ------------------ */

	uc10MSt_intct = 0x0ff;					/*ﾒｲﾝﾙｰﾌﾟ確定 設定 00-0526 -ak */
    vMAIN_Softr();							/*ｿﾌﾄ ﾘｾｯﾄ処理 */
	while((ucMAIN_10up & 0x01) == 0){	/*10ms経過までは */
		if(ucg_2ms_flg){	/* 2ms check */
			ucg_2ms_flg = false;
			vTIME_Timea();							/*タイマ制御処理｜≧19.5us */
			/*INV系処理*/
#ifdef DEBUG /*DEBUG*/
            if(test_chg_speed_flg == 1){  /* デバッグ */
				ucTRNS_sfrq_b = test_speed;
			}
#endif /*DEBUG*/
			ucMAIN_chkint = 0;				/*割込ﾁｪｯｸｶｳﾝﾀ←ｸﾘｱ */
			vSPD_set();
			v2LV_Ctrl2lv();							/*２レベル電圧制御処理 */
			/*通信関連*/
			vTRNS_Outputi();						/*送信ﾃﾞｰﾀｾｯﾄ処理 */
			vTRNS_Cnttrns();						/*通信ﾀｲﾏｶｳﾝﾄ処理 */
			vPMD_SoftStopDuringFreeRun();			/*フリーラン中のソフトストップ処理*/
			/*オプションバイト確認用発振*/
			MainLoop_PulseOutput();
			vMAIN_MainInttm51();					/*Main制御 2ms割込み*/
			if(ucg_MAIN_40up == SET){				/*通常制御*/			/* @094HxOOM[11/07/02] */
				vMAIN_MainPros();					/*Main制御 40ms毎処理 *//* @094HxOOM[11/07/02] */
			}
			un100MSt_typect = 0;			/*機種切換（通信）ﾙｰﾌﾟﾀｲﾑｾｰﾌ(6000,600s)ｶｳﾝﾀ←ｸﾘｱ */ /* @011axORT[07/05/17] */
		}
		vTRNS_Hw_trns();					/* 通信処理				*/	/* @000MzDOD[10/08/26]_07 */
		vMAIN_Msec10();						/*10ms 監視処理*/
	}
	ucMAIN_10up &= 0xfe;					/*10msﾀｲﾑｱｯﾌﾟﾌﾗｸﾞ←ｸﾘｱ */
	MOTOR_main_process();

}

/**************************************************************************
<Title>			:vTRNS_Cnttrns 		;vMAIN_Inttc1pros
<Function>		:通信ﾀｲﾏｶｳﾝﾄ処理
<Description>	:	 送信ﾃﾞｰﾀ間隔のｶｳﾝﾄ処理と連続送信異常の判定を行う｡
<Input>			:ucTRNS_trer.0	通信ｴﾗｰﾌﾗｸﾞ
<Return>		:ucMINtm_trerct	通信異常継続時間積算ｶｳﾝﾀ
<Author>		:-Outi	作成
<Date>			:2002-0222
<Update>		:Ver REF02A-0.00 2002-02-22
				:Ver REF045-3.00 2004-07-06 -Outi 通信異常時強制運転仕様追加
				:Ver REF070-0.03 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.09 2007-05-11 -Outi H63=Lの場合D_TRERRJUDGETM分後に必ずH50断続運転動作する部分を修正
	<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vTRNS_Cnttrns(void){					/*通信ﾀｲﾏｶｳﾝﾄ処理 */
	if(ucg_TR_stbyct >= TRNSFAULTNUM){       			 /*stbyコマンド連続受信状態なら */
		ucg_TR_stbyct = TRNSFAULTNUM;           		/*ucg_TR_stbyctをTRNSFAULTNUMに固定 */
		u8g_MOTOR_ctrlflg = MOTOR_CTRL_OFFLINE; 		/*RW 通信状態フラグをオフラインに設定（通信異常とみなす）*/
        if(ucMINtm_stbyct >= 255){
            ucMINtm_stbyct = 255;              			/*最大値でクリップ(オーバーフロー対策) */
        }
	}else if((ucTRNS_trer & 0x01) != 0){				/*"通信が異常なら  F_TRERFG(ucTRNS_trer.0) At HW_TRNS.c" OR "通信機種判別"未"確定なら" */ /*＠~07-0511 -Outi H50断続運転不具合修正 ＃ 午後 04:52 REF070-0.09 */ /* @010ahORM[07/05/11] */
		if(ucMINtm_trerct >= D_TRERRJUDGETM){			/*1hr(=1分*D_TRERRJUDGETM)経過？ */
			ucMINtm_trerct = D_TRERRJUDGETM;			/*通信異常継続時間←D_TRERRJUDGETM固定 */
		/*	ucTRNS_sfrq_b = 0;	*/						/*通信異常時にsfrqをCLRﾘﾌﾚｯｼｭ *//*正弦波ソフトでは回転数設定は別モジュールに集約*/
			u8g_MOTOR_ctrlflg = MOTOR_CTRL_OFFLINE;    	/*RW通信状態フラグをオフラインに設定*/
		}else if((ucMINtm_trerct > 1)&&(u8g_TRNSset_spd_flg == 0)){	/*通信異常が少なくとも1min継続 かつ マイコン起動時から通信異常(一度もメインからの回転数指令をセットしていない)*/
			u8g_MOTOR_ctrlflg = MOTOR_CTRL_OFFLINE;    	/*RW通信状態フラグをオフラインに設定*/
		}else{											/*1hr(=1分*D_TRERRJUDGETM)未達？ */
			ucHOG_sttF &= 0xfe;							/*通信異常１時間経過ﾌﾗｸﾞ←ｸﾘｱ F_TRERHG(ucHOG_sttF.0) */
			u8g_MOTOR_ctrlflg = MOTOR_CTRL_ONLINE;    	/*RW通信状態フラグをオンラインに設定*/
		}												/* */
	}else{												/*通信が正常なら */
		ucMINtm_trerct = 0;								/*通信異常継続時間積算ｶｳﾝﾀ←ｸﾘｱ */
		ucHOG_sttF &= 0xfe;								/*通信異常１時間経過ﾌﾗｸﾞ←ｸﾘｱ F_TRERHG(ucHOG_sttF.0) */
		u8g_MOTOR_ctrlflg = MOTOR_CTRL_ONLINE;    		/*RW通信状態フラグをオンラインに設定*/
	}													/* */
#ifdef DEBUG /*DEBUG*/
	u8g_MOTOR_ctrlflg = MOTOR_CTRL_ONLINE;
#endif /*DEBUG*/
}				

/**************************************************************************
<Title>			:vMAIN_type_uart 		;main
<Function>		:通信機種判定処理
<Description>	:	 受信ﾃﾞｰﾀから駆動仕様の判定を行う｡
<Input>			:ucTRNS_type	受信ﾃﾞｰﾀ
<Return>		:
<Author>		:-tomo	作成
<Date>			:2007-0417
<Update>		:Ver REF070-0.03 2007-04-17 -Tomo 機種切り替え対応
				:Ver REF070-0.04 2007-04-18 -Tomo 機種切り替え対応
				:Ver REF090-0.00 2009-05-08 -Outi 回路異常検出仕様追加 @004axOOM[09/05/08]
				:Ver REF1000-0.01 2010-04-01 -Mura 2Level対応	@000MwDOD[10/04/01]
				:Ver REF1000-0.02 2010-04-05 -Mura 不足電圧保護2Level対応 @001MwDOD[10/04/05]
<Copyright> 	:Matsushita Electric Industrial Co. , Ltd.
 **************************************************************************/
void vMAIN_type_uart(void){						/*通信機種判定処理	*/

	if((ucTRNS_sfrg & 0x10) == 0){			/*Mainから通信確定確定ﾌﾗｸﾞ受信*/			/* @019MzDOD[17/03/06] */
		/*------------------------------*/
		/*	INV仕様識別コード			*/		/* 3V : INVKDT(0x61) → (0x6B) */
		/*------------------------------*/
		ucTYPE_overlap 			=  (ucTRNS_type & 0x01);		/* bit0：ｵｰﾊﾞｰﾗｯﾌﾟ及び矩形波ﾍﾞｸﾄﾙ		 0：無し 		1：有り				*/
		ucTYPE_position_duty	= ((ucTRNS_type & 0x02)>>1); 	/* bit1：起動Duty						 0：低起動DUTY 	1：高起動DUTY		*/
		ucTYPE_global 			= ((ucTRNS_type & 0x04)>>2); 	/* bit2：国内/海外切替					 0：国内		1：海外				*/
		ucTYPE_Dline 			= ((ucTRNS_type & 0x08)>>3); 	/* bit3：Dﾗｲﾝ共振保護制御 				 0：無			1：有				*/
		ucTYPE_Dline_point 		= ((ucTRNS_type & 0x10)>>4); 	/* bit4：Dﾗｲﾝ共振ﾎﾟｲﾝﾄ 					 0：44?47r/s	1：45?49r/s		*/
		ucTYPE_carrier_freq	 	= ((ucTRNS_type & 0x20)>>5);	/* bit5：基本ｷｬﾘｱ周波数仕様				 0：10kHz 		1：6.5kHz			*/	/* @250swDOD[23/05/18] */
		ucTYPE_2level		 	= ((ucTRNS_type & 0x40)>>6);	/* bit6：2ﾚﾍﾞﾙ制御	 					 0：無 			1：有				*/

		ucTYPE_undvlt_int = (1);										/*直流電圧低下保護仕様	1:外部割込み仕様*//* @250swDOD[23/05/18] */
																		/* 直流電圧低下保護外部割込みに仕様統一化、ucTRNS_typeﾌﾗｸﾞによるucTYPE_undvlt_intの設定廃止のため *//* @250swDOD[23/05/18] */
																		/* ﾍﾞｸﾄﾙ制御有りなら直流電圧低下保護外部割込みにする処理廃止（削除） *//* @250swDOD[23/05/18] */

		ucTYPE_uart	= 1;												/*通信機種判別確定	*/

    }else{
    /* 通信異常時のデフォルト設定 */
		ucTYPE_overlap 			= 0;	/* bit0：ｵｰﾊﾞｰﾗｯﾌﾟ及び矩形波ﾍﾞｸﾄﾙ		 0：無し 		1：有り				*/
		ucTYPE_position_duty	= 1; 	/* bit1：起動Duty						 0：低起動DUTY 	1：高起動DUTY		*/
		ucTYPE_global 			= 0; 	/* 国内/海外切替 "国内"				*/	/* 他仕様識別ｺｰﾄﾞと対応異なるが国内機種に強制設定する*/	/* @019MzDOD[17/03/06] *//* bit2：国内/海外切替					 0：国内		1：海外				*/
    ucTYPE_Dline 			= 0; 	/* 要検討 保護であるなら回避すべきでは 現行ソフトは保護なし */ /* bit3：Dﾗｲﾝ共振保護制御 				 0：無			1：有				*/
    ucTYPE_Dline_point 		= 0; 	/* 現状意味はないが 0が選択されている */ /* bit4：Dﾗｲﾝ共振ﾎﾟｲﾝﾄ 					 0：44?47r/s	1：45?49r/s		*/
		ucTYPE_carrier_freq	 	= 1;	/* 基本ｷｬﾘｱ周波数は制御性の観点からデフォルトは6.5kHzとする	*/	/* @250swDOD[23/05/18] */
		ucTYPE_undvlt_int		= 1;	/* 内臓ｺﾝﾊﾟﾚｰﾀありﾏｲｺﾝ採用が前提であり、デフォルトは外部割込み仕様で統一する	*/	/* @250swDOD[23/05/18] */
    ucTYPE_2level		 	= 0;	/* ハードが備わっていない可能性があるため、デフォルトは無し */ /* bit6：2ﾚﾍﾞﾙ制御	 					 0：無 			1：有				*/

		ucTRNS_stopctrl			= 18;   /* 通信異常中はブレーキ制御なし */
		ucTYPE_uart				= 0;	/* Main-Inv通信確定ﾌﾗｸﾞ"未確定"設定	*/															/* @019MzDOD[17/03/06] */
	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  uint32_t Timeout ;

	/*=========================G0оƬRTC��λ=============================*/

  SET_BIT(RCC->APBENR1, RCC_APBENR1_PWREN); 						/*enable clock to pwr control subsystem*/

  /*Start LSI oscillator used by RTC and IWDG*/

	SET_BIT(PWR->CR1, PWR_CR1_DBP);           						/* enable access to backup power domain*/

	Timeout = SYSTEM_CLOCK_CONF_RDY_TIMEOUT_MS;

	while(READ_BIT(PWR->CR1, PWR_CR1_DBP) == RESET)

	{

		 if(Timeout-- == 0)

		 {

				Error_Handler();           											/* call NVIC_SystemReset */   

		 }

	}

/* See Errata 2.2.10: Unstable LSI after main power domain reset*/

/* if reset was due to POR or BOR, reset the backup power domain*/

    if (RCC->CSR & RCC_CSR_PWRRSTF)

    {

      /* SET_BIT(RCC->BDCR, RCC_BDCR_BDRST); */      
		  /* reset backup power domain*/

			/*a. Writing 0x0001 0000 in the RCC_BDCR register, which sets the BDRST bit and clears other*/

			/*register bits that might not be reset.*/     

      WRITE_REG(RCC->BDCR,0x00010000);        

      Timeout = SYSTEM_CLOCK_CONF_RDY_TIMEOUT_MS;

			/*. reading the RCC_BDCR register, to make the reset time long enough*/

			while(READ_BIT(RCC->BDCR, RCC_BDCR_BDRST) == RESET)

			{

					 if(Timeout-- == 0)

					 {

							Error_Handler();    										/*  call NVIC_SystemReset */

					 }

			}

			/*c. writing 0x0000 0000 in the RCC_BDCR register, to clear the BDRST bit */

        WRITE_REG(RCC->BDCR,0x00000000);

      /* CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST);*/

    } 

		/*d. Clear the BORRSTF flag by setting the RMVF bit of the RCC_CSR register */   

    SET_BIT(RCC->CSR, RCC_CSR_RMVF); 									/*CLEAR_BIT*/    

    /* Start LSI and wait until ready */ 

    SET_BIT(RCC->CSR, RCC_CSR_LSION);         				/* turn on LSI oscillator*/

    Timeout = SYSTEM_CLOCK_CONF_RDY_TIMEOUT_MS;   

    while (!(RCC->CSR & RCC_CSR_LSIRDY))     					/* wait for LSI ready */

    {

       if(Timeout-- == 0)

       {

          Error_Handler();    												/*  call NVIC_SystemReset */ 

       }

    } 

    CLEAR_BIT(PWR->CR1, PWR_CR1_DBP);  

/*==============================================================================*/	
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; /* コンパイラをARM Compiler/GCCから変更する際は見直し必要 */

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Ch4_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);
  /* ADC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
  ucMAIN_wdrfg = 0;
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC5REF_RISING_OC6REF_RISING;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) + 1);
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 12; /* 矩形波駆動(HY2)と同じ設定 */
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 12;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0xffff;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0xffff;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0xffff;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0xffff;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 16000 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, unused_Pin|unusedC15_Pin|HEAT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, unusedA0_Pin|FREQ_PULSE_OUT_Pin|DRV_RELAY_Pin|HEAT2_Pin
                          |unusedA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, unusedB3_Pin|unusedB4_Pin|unusedB5_Pin|unusedB8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin unusedB3_Pin unusedB4_Pin unusedB5_Pin
                           unusedB8_Pin */
  GPIO_InitStruct.Pin = LED_Pin|unusedB3_Pin|unusedB4_Pin|unusedB5_Pin
                          |unusedB8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : unused_Pin unusedC15_Pin HEAT1_Pin */
  GPIO_InitStruct.Pin = unused_Pin|unusedC15_Pin|HEAT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : unusedA0_Pin FREQ_PULSE_OUT_Pin DRV_RELAY_Pin HEAT2_Pin
                           unusedA15_Pin */
  GPIO_InitStruct.Pin = unusedA0_Pin|FREQ_PULSE_OUT_Pin|DRV_RELAY_Pin|HEAT2_Pin
                          |unusedA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
