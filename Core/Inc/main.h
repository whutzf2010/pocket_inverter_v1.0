/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
	* @Author         : whutzf2010
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "arm_math.h"	// ????arm_math.h
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_comp.h"
#include "stm32f3xx_ll_dac.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */


/* Private Includes Define ----------------------------------------------------------*/
#include "GM_adc.h"


/* Private Macro Define ----------------------------------------------------------*/
#define PWM_2KHZ    36000
#define PWM_4KHZ    18000
#define PWM_5KHZ    14400
#define PWM_10KHZ   7200
#define PWM_20KHZ   3600
#define PWM_25KHZ   2880
#define PWM_30KHZ   2400
#define PWM_40KHZ   1800
#define PWM_50KHZ   1440
#define PWM_100KHZ  720
#define PWM_150KHZ  480
#define PWM_200KHZ  360

#define PWM_10KHZ_HALF   PWM_10KHZ/2
#define PWM_20KHZ_HALF   PWM_20KHZ/2
#define PWM_25KHZ_HALF   PWM_25KHZ/2
#define PWM_30KHZ_HALF   PWM_30KHZ/2
#define PWM_40KHZ_HALF   PWM_40KHZ/2
#define PWM_50KHZ_HALF   PWM_50KHZ/2

#define INVERT_PWM PWM_50KHZ
#define INVERT_PWM_HALF PWM_50KHZ_HALF

#define TIM2_FREQ 4000

#define AC_FREQ_5HZ    	TIM2_FREQ/5  //f.TIM2/5Hz
#define AC_FREQ_10HZ    TIM2_FREQ/10
#define AC_FREQ_25HZ    TIM2_FREQ/25
#define AC_FREQ_40HZ    TIM2_FREQ/40
#define AC_FREQ_50HZ    TIM2_FREQ/50
#define AC_FREQ_65HZ    TIM2_FREQ/65
#define AC_FREQ_500HZ   TIM2_FREQ/500

#define INVERT_AC_FREQ AC_FREQ_50HZ

#define PWM_DUTY PWM_20KHZ*0.5

#define DT_42NS    3
#define DT_84NS    6
#define DT_138NS   10
#define DT_250NS   18
#define DT_500NS   36
#define DT_600NS   43
#define DT_750NS   54
#define DT_1000NS  72
#define DT_1200NS  87
#define DT_1500NS  108

#define BAUDRATE  115200 //do not use 9600->transmit error, 19200, 115200 is OK
#define ADC_BUFFER_SIZE ((uint32_t)    5) 

#define UPPERLIMIT_VOL 100.0
#define LOWERLIMIT_VOL -100.0

#define UPPERLIMIT_CUR 10.0
#define LOWERLIMIT_CUR -10.0

//-------Filtering -------//

#define VAC_FILTER_K 0.72    //K=0.6,f.c = 2.4KHz,t=66us;  K=0.5,f.c=1.6KHz,t= 99us
#define VAC_FILTER_1_K (1-VAC_FILTER_K)

#define VIN_FILTER_K 0.25 //0.2 ok for OnGrid Control   //K=0.6,f.c = 2.4KHz,t=66us;  K=0.5,f.c=1.6KHz,t= 99us
#define VIN_FILTER_1_K (1-VIN_FILTER_K)

#define IIN_FILTER_K 0.35    //K=0.6,f.c = 2.4KHz,t=66us;  K=0.5,f.c=1.6KHz,t= 99us
#define IIN_FILTER_1_K (1-IIN_FILTER_K)

#define IL_FILTER_K 0.85    //K=0.6,f.c = 2.4KHz,t=66us;  K=0.5,f.c=1.6KHz,t= 99us
#define IL_FILTER_1_K (1-IL_FILTER_K)

#define IG_FILTER_K 0.85    //K=0.6,f.c = 2.4KHz,t=66us;  K=0.5,f.c=1.6KHz,t= 99us
#define IG_FILTER_1_K (1-IG_FILTER_K)

//-------AC Volt-PI---------//
#define VAC_KP 0.00001    
#define VAC_KI 0.0005
#define VAC_KD 0.1

//-------AC Current-PI--------//
#define IAC_KP 0.01    
#define IAC_KI 0.2
#define IAC_KD 0.1

/*------- uart select -------------------------------------------*/
typedef enum
{
	VOFA,   // InvertOffline
	Serialplot
}SerialPortTypedef;
extern SerialPortTypedef SerialPort;
extern int16_t u16SerialPortTypedef;
/*------- Operation Mode ----------------------------------------------*/
typedef enum
{
	InvertOffGrid,   // InvertOffline
	InvertOnGrid,    // Bidirecton ACDC          //PFC
	InvertOffGridDroop // Invert Output Parallel


}ModeTypedef;
extern ModeTypedef Mode;
extern int16_t u16Mode;
//--------------Normal & Fault ---------------------------------------//
typedef struct
{
	int8_t Fault;
	int8_t UV;
	int8_t OV;
	int8_t OC;
		
}StatusTypedef;
extern StatusTypedef Status;


/*------- Closed Loop Mode ----------------------------------------------*/
typedef enum
{

	InvertOnGridDQ, // Invert Output Parallel
	InvertOnGridPR // Invert Output Parallel	
	
}LoopModeTypedef;
extern LoopModeTypedef LoopMode;
extern int16_t u16LoopMode;

/*-------AC LF Switch State-Machine----------------------------------------------*/
typedef enum
{
	POS1, // 0-->Vth
	POS2, // Vth-->Vth  
	POS3, // Vth-->0
	NEG1, // 0-->-Vth
	NEG2, // -Vth-->-Vth
	NEG3  // -Vth-->0
		
}ACStateTypedef;
extern ACStateTypedef ACState;
extern int16_t u16VacState;

typedef enum
{
	Ready,
	Run,
	Stop
		
}OnGridStateTypedef;
extern OnGridStateTypedef OnGridState;
extern int16_t u16OnGridState;
/* Private Variables Define ----------------------------------------------------------*/
extern uint16_t ADCValue_VIN;
extern uint16_t ADCValue_IL;
extern uint16_t ADCValue_VAC;
extern uint16_t ADCValue_IIN;
extern uint16_t ADCValue_IG;

extern uint16_t   aADCxConvertedValues[ADC_BUFFER_SIZE];

extern float f32_VIN;
extern float f32_VIN_filter;
extern float f32_VIN_filtered;
extern float f32_IIN;
extern float f32_IIN_filter;
extern float f32_IIN_filtered;

extern float f32_VAC;
extern float f32_VAC_filter;
extern float f32_VAC_filtered;
extern float f32_VAC_K;
extern float f32_IIN;
extern float f32_IL;
extern float f32_IL_filter;
extern float f32_IL_filtered;
extern float f32_IG;
extern float f32_IG_filter;
extern float f32_IG_filtered;

extern float f32_VAC_Rms[50];
extern int16_t f32_VAC_Rms_Counter;
extern float f32SUMValue;
extern float f32RMSValue;
extern float f32_VDC_Set;
extern float f32_VRms_Set;
extern float f32_IAC_Ref;
extern float f32_VAC_Set;
extern float f32_VAC_Set_Step;
extern float f32_VAC_Set_Update;
extern float f32_IAC_Set;
extern float f32_IAC_Set_Step;
extern float f32_IAC_Set_Update;
extern double f32_VAC_Gain;

extern uint16_t UsartTxValue;
extern uint16_t UsartRxValue;

extern int16_t TIM2_Counter;
extern int16_t TIM2_Delay_Counter;

//--------------- Fiter & PWM ------------//
extern float VacRmsVal;
extern int16_t VacPeakVal;
extern float SinVal;
extern float f32VacVal;
extern float f32VacSin;
extern int16_t u16VacVal;
extern int16_t u16VacAbsVal;
extern int16_t u16VacFreqCounter;
extern float f32VacFreqCounter;
extern float f32VacFreq;
extern double VacTimer;
extern float VacGain;
extern int16_t u16PwmUpdate;
extern int16_t u16PwmUpdateCH2;
extern int16_t u16PwmUpdateCH3;
extern float f32PwmUpdate;

extern float OnGridDuty;
extern float OffGridDuty;
/* Exported types ------------------------------------------------------------*/

/* USER CODE BEGIN ET */
/*typedef struct 
{
    int16_t       _SciCount;
    //--TX Sending Buffer------------------
    float         _fDataBufferTx[3];
    int16_t      *_cDataSendTx;    //tx data to be send
    char          _cDataTailSendTx[4]; //tx tail data for  VOFA "jusifloat" format
    int16_t       _ChannelTx;
    float         _VOFAGain;

    //--RX Receiving Buffer------------------
    char        _cDataBufferRx[5];
    int16_t     _iDataBufferRx[5];
    float       _fDataBufferRx[5];
}UARTStructTypeDef;*/
//extern UARTStructTypeDef SciCom;



extern uint8_t USARTBufferTx[16]; //VOFA Length  
extern uint8_t USARTBufferRx[12];
extern float USARTf32Tx[4]; // VOFA length
extern uint8_t *USARTf32TxADDr;

extern uint8_t DataTailSendTx;
/* USER CODE END ET */

/* RMS Value --------------------------------------------------------*/
extern float f32MaxValue;
extern float f32MaxValueBefore;
extern float f32MinValue;
extern float f32MinValueBefore;

//--------------- Off Grid Voltage Control ------------//
typedef struct
{
	float Kp_Vol;
	float Ki_Vol;
	float Kd_Vol;
	float Err0_Vol;
	float Err1_Vol;
	float Out0_Vol;
	float Out1_Vol;
	float Set_Vol;
	float Feedback_Vol;
  float Ts;	
	float Kp_Cur;	
	float Ki_Cur;
	float Kd_Cur;
	float Err0_Cur;
	float Err1_Cur;
	float Out0_Cur;
	float Out1_Cur;
	float Set_Cur;
	float Feedback_Cur;
		
}PIDStructTypedef;
extern PIDStructTypedef OffGridPID;
extern PIDStructTypedef OnGridPID;

//--------------- On Grid Voltage SOGI-PLL ------------//
typedef struct
{
	float w0;
	float k;
	float Ts;
	float lamda;
	float x;
	float y;
	
	float b0;
	float a1;
	float a2;
	
	float u;
	float u1;
	float u2;
	float alpha0;
	float beta0;
	float alpha1;
	float beta1;
	float alpha2;
	float beta2;
	
	float d;
	float q;
	float theta;
	
	float kp;
	float ki;
	float error0;
	float error1;
	float out0;
	float out1;
		
}SOGIPLLStructTypedef;
extern SOGIPLLStructTypedef SOGIPLL;
extern SOGIPLLStructTypedef SOGIPLL_Ig;

//--------------- On Grid Voltage DQ Decouple ------------//
typedef struct
{
	float error0_Volt;
	float error1_Volt;
	float out0_Volt;
	float out1_Volt;
	float ref_Volt;
	float feedback_Volt;
	float kp_Volt;
	float ki_Volt;
	
	float error0_Id;
	float error1_Id;
	float out0_Id;
	float out1_Id;
	float max_Id;
	float min_Id;
	float ref_Id;
	float feedback_Id;
	float kp_Id;
	float ki_Id;
	
	float error0_Iq;
	float error1_Iq;
	float out0_Iq;
	float out1_Iq;
	float max_Iq;
	float min_Iq;
	float ref_Iq;
	float feedback_Iq;
	float kp_Iq;
	float ki_Iq;
	
	float Vd;
	float Vq;
	float theta;
	float Vc;
	float alpha;
	float beta;
	float duty;
	float Ts;
		
}DQStructTypedef;
extern DQStructTypedef OnGridDQ;




typedef struct
{
	float Err0;
	float Err1;
	float Err2;
	float Err3;
	
	float Out0;
	float Out1;
	float Out2;
	float Out3;
	
	float Kp;
	float Kr;
	
	float wc;
	float w0;
	float Ts;
	
	float A1;
	float A2;
	float A3;
	float A4;
	float A5;
	float A6;
		
}PRStructTypedef;
extern PRStructTypedef OnGridPR;


typedef struct
{
	float Err0;
	float Err1;
	float Err2;
	float Err3;
	
	float Out0;
	float Out1;
	float Out2;
	float Out3;
	
	float Ts;	
	float Kp;
	float p1;
	float p2;
	float z1;
	float z2;
	
	float A1;
	float A2;
	float A3;
	float B1;
	float B2;
	float B3;
		
}_2P2ZStructTypedef;
extern _2P2ZStructTypedef OnGrid2P2Z;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private Constant Defines ----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
