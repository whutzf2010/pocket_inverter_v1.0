/**
  ******************************************************************************
  * @file    	stm32f3xx_it.c
  * @brief   	Interrupt Service Routines.
	* @Author   Power Bell
  ******************************************************************************

  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
#include "arm_math.h"	// 

/* Private typedef -----------------------------------------------------------*/
/* -------Serial Select Define -------------*/
SerialPortTypedef SerialPort;
int16_t u16SerialPortTypedef;
/* -------Mode Select Define -------------*/
ACStateTypedef ACState;
int16_t u16VacState;

OnGridStateTypedef OnGridState;
int16_t u16OnGridState;

ModeTypedef Mode;
int16_t u16Mode;
/* -------Loop Select Define -------------*/
LoopModeTypedef LoopMode;
int16_t u16LoopMode;
StatusTypedef Status; 

/* -------Loop Control Define -------------*/
DQStructTypedef OnGridDQ;
PIDStructTypedef OffGridPID;
PIDStructTypedef OnGridPID;
PRStructTypedef OnGridPR;
_2P2ZStructTypedef OnGrid2P2Z;
SOGIPLLStructTypedef SOGIPLL;   //Vgrid
SOGIPLLStructTypedef SOGIPLL_Ig;  //Igrid

//UARTStructTypeDef SciCom;

/* -------Interrupt Define -------------*/
int16_t TIM2_Counter;
int16_t TIM2_Delay_Counter; //10KHz 


/* -------Loop Calculate Define -------------*/
float VacRmsVal;
int16_t VacPeakVal;
float SinVal;
float f32VacVal;
float f32VacSin;
int16_t u16VacVal;
int16_t u16VacAbsVal;
int16_t u16VacFreqCounter;
float f32VacFreqCounter;
float f32VacFreq;
double VacTimer;
float VacGain;
int16_t u16PwmUpdate;
int16_t u16PwmUpdateCH2;
int16_t u16PwmUpdateCH3;
float f32PwmUpdate;

float OnGridDuty;
float OffGridDuty;

/* -------Serial Context Define -------------*/
uint8_t DataTailSendTx;
uint8_t USARTBufferTx[16];
uint8_t USARTBufferRx[12];
uint8_t *USARTf32TxADDr;
float USARTf32Tx[4];

/* ------- RMS Calculate Define -------------*/
float f32MaxValue;
float f32MaxValueBefore;
float f32MinValue;
float f32MinValueBefore;


//**************************************************************//
//	     	Digital Power Control Algrithum		            	//
//								Power Bell															//
//								2023.12.30															//
//**************************************************************//


void TIM2_IRQHandler(void)
{
	
	if(LL_TIM_IsActiveFlag_UPDATE(TIM2))
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);	

		if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13))
		{
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14);	
		}
		else
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14);
//---------VOFA Transmission Data--------------//
		if(u16SerialPortTypedef == VOFA)	
		{
			USARTf32Tx[0] = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13);//````````````````````````````````````````````````````````````````		f32_VAC_filtered;//OnGridPR.Out0;//OnGridDuty;//u16PwmUpdateCH2;//OnGridPR.Out0;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//f32_VAC_filtered;//SOGIPLL.theta;//
			USARTf32Tx[1] = f32_IL_filtered;//(float)(arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol);//SOGIPLL_Ig.d;//SOGIPLL_Ig.d;//OnGridDuty;//OnGridPID.Out0_Vol;//OnGridPR.Err0;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//SOGIPLL.alpha0;//f32_IAC_Ref;
			USARTf32Tx[2] = f32_IG_filtered;//(float)(-f32_IG_filtered);//SOGIPLL_Ig.q;//SOGIPLL_Ig.q;//Status.Fault;//OnGridPR.Out0;//-f32_IAC;//- f32_VIN_filtered;//SOGIPLL.beta0;//f32_VAC_filtered;
			//USARTf32Tx[3] = arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//0;
			
//----------VOFA USART Communication Proctol-2kHz--------------//			
			USARTf32TxADDr = (uint8_t *)&USARTf32Tx;
			USARTBufferTx[0] = USARTf32TxADDr[0];
			USARTBufferTx[1] = USARTf32TxADDr[1];
			USARTBufferTx[2] = USARTf32TxADDr[2];
			USARTBufferTx[3] = USARTf32TxADDr[3];
			
			USARTBufferTx[4] = USARTf32TxADDr[4];
			USARTBufferTx[5] = USARTf32TxADDr[5];
			USARTBufferTx[6] = USARTf32TxADDr[6];
			USARTBufferTx[7] = USARTf32TxADDr[7];
		
			USARTBufferTx[8] = USARTf32TxADDr[8];
			USARTBufferTx[9] = USARTf32TxADDr[9];
			USARTBufferTx[10] = USARTf32TxADDr[10];
			USARTBufferTx[11] = USARTf32TxADDr[11];
	

			//USARTBufferTx[12] = USARTf32TxADDr[12];
			//USARTBufferTx[13] = USARTf32TxADDr[13];
			//USARTBufferTx[14] = USARTf32TxADDr[14];
			//USARTBufferTx[15] = USARTf32TxADDr[15];
		}	
		
		if(u16SerialPortTypedef == Serialplot)
		{
			USARTf32Tx[0] = f32_VIN;//u16PwmUpdateCH2;//OnGridPR.Out0;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//f32_VAC_filtered;//SOGIPLL.theta;//
			USARTf32Tx[1] = f32_VAC;//OnGridPID.Out0_Vol;//OnGridPR.Err0;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//SOGIPLL.alpha0;//f32_IAC_Ref;
			USARTf32Tx[2] = f32_IIN;//OnGridPR.Out0;//-f32_IAC;//- f32_VIN_filtered;//SOGIPLL.beta0;//f32_VAC_filtered;
			USARTf32Tx[3] = arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//0;
			
//----------VOFA USART Communication Proctol-2kHz--------------//			
			USARTf32TxADDr = (uint8_t *)&USARTf32Tx;
			USARTBufferTx[0] = USARTf32TxADDr[0];
			USARTBufferTx[1] = USARTf32TxADDr[1];
			USARTBufferTx[2] = USARTf32TxADDr[2];
			USARTBufferTx[3] = USARTf32TxADDr[3];
			
			USARTBufferTx[4] = USARTf32TxADDr[4];
			USARTBufferTx[5] = USARTf32TxADDr[5];
			USARTBufferTx[6] = USARTf32TxADDr[6];
			USARTBufferTx[7] = USARTf32TxADDr[7];
		
			USARTBufferTx[8] = USARTf32TxADDr[8];
			USARTBufferTx[9] = USARTf32TxADDr[9];
			USARTBufferTx[10] = USARTf32TxADDr[10];
			USARTBufferTx[11] = USARTf32TxADDr[11];
	

			USARTBufferTx[12] = USARTf32TxADDr[12];
			USARTBufferTx[13] = USARTf32TxADDr[13];
			USARTBufferTx[14] = USARTf32TxADDr[14];
			USARTBufferTx[15] = USARTf32TxADDr[15];
		}
			
//------------ADC Sampling Result Update--3%---------------//	
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);			
		
		
		ADCValue_VIN = aADCxConvertedValues[0]; //VIN
		ADCValue_IG = aADCxConvertedValues[1]; //IG
		ADCValue_IL = aADCxConvertedValues[2]; //IL
		ADCValue_VAC =  aADCxConvertedValues[3]; //VAC
		ADCValue_IIN = 	aADCxConvertedValues[4];  //IIN
		
    f32_VIN = (float)ADCValue_VIN;
    f32_VIN = f32_VIN * 0.0169189453; //  f32_VIN = ADCValue_VIN/4096*3.3*21;
    f32_VIN = f32_VIN * 1.0; // calibrate
    f32_VIN += 1.2;          // calibrate
    if(f32_VIN <= 0.01)
        f32_VIN = 0.01;

    f32_VAC = (float)(ADCValue_VAC - (2048.0));
    f32_VAC = f32_VAC * 0.0245016; //  f32_VAC = ADCValue_VAC/4096*3.3*30.41176;
    f32_VAC = f32_VAC + 4.7; // calibrate 2023.12.17
    //f32_VAC = f32_VAC * (1.168);

    f32_IIN = (float)ADCValue_IIN;
    f32_IIN = f32_IIN * 0.004028; //  f32_IIN = ADCValue_IIN/4096*3.3*5;

		//CC6900-10A
    f32_IL = (float)(ADCValue_IL - 1551.515);//1440);//1450);
    f32_IL = f32_IL * (-0.00805664);//0.00644; //  f32_IAC = ADCValue_IAC/4096*3.3*20;
		f32_IL = f32_IL - 0.5; // calibrate 2023.12.17
		
		//CC6900-20A
    f32_IG = (float)(ADCValue_IG - 1551.515);//1440);//1450);
    //f32_IG = f32_IG * (-0.00644);//0.00644; //  f32_IAC = ADCValue_IAC/4096*3.3*8;
    f32_IG = f32_IG * (-0.00805664);//0.00644; //  f32_IAC = ADCValue_IAC/4096*3.3*20;
		f32_IG = f32_IG - 0.57; // calibrate 2023.12.17
		f32_IG = f32_IG * 2.06;//0.00644; //  f32_IAC = ADCValue_IAC/4096*3.3*20;
		
		//TIM_OC_InitStruct.CompareValue = 2000;	
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);	
		//}

		
//-------- Low Pass Filter ---------------//
		f32_VAC_filter = f32_VAC * VAC_FILTER_K;
		f32_VAC_filter += f32_VAC_filtered * VAC_FILTER_1_K;
		f32_VAC_filtered = f32_VAC_filter;
		
		f32_IIN_filter = f32_IIN * IIN_FILTER_K;
		f32_IIN_filter += f32_IIN_filtered * IIN_FILTER_1_K;
		f32_IIN_filtered = f32_IIN_filter;
	
		f32_VIN_filter = f32_VIN * VIN_FILTER_K;
		f32_VIN_filter += f32_VIN_filtered * VIN_FILTER_1_K;
		f32_VIN_filtered = f32_VIN_filter;	
		
		f32_IL_filter = f32_IL * IL_FILTER_K;
		f32_IL_filter += f32_IL_filtered * IL_FILTER_1_K;
		f32_IL_filtered = f32_IL_filter;
		
		f32_IG_filter = f32_IG * IG_FILTER_K;
		f32_IG_filter += f32_IG_filtered * IG_FILTER_1_K;
		f32_IG_filtered = f32_IG_filter;
		//if(f32_VAC_filtered > 15.0)
		//	f32_VAC_filtered = 15.0;
		//if(f32_VAC_filtered < -15.0)
		//	f32_VAC_filtered = -15.0;	
		//if(f32_VAC >= 0.0)
		//	f32MaxValueBefore = f32_VAC;
		//else 
		//	f32MinValueBefore = f32_VAC;
		
//----------- MAX/MIN Value ------------------//			
		//if(f32MinValue >= f32_VAC_filtered)
		//	f32MinValue = f32_VAC_filtered;
		//if(f32MaxValue <= f32_VAC_filtered)
		//	f32MaxValue = f32_VAC_filtered;


//------------Voltage Protect-----------------------//

		if(f32_VIN_filtered >= 31.0)
		{
			Status.Fault = 1;
		}
		if(f32_VIN_filtered <= 0.0)
		{
			Status.Fault = 0;
		}
		if(Status.Fault == 1)
		{
			//LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12); //AC relay off
			LL_TIM_DisableAllOutputs(TIM1);
		}
		if(Status.Fault == 0)
		{
			//LL_GPIO_SaetOutputPin(GPIOB, LL_GPIO_PIN_12); //AC relay off
			//LL_TIM_EnableAllOutputs(TIM1);
		}

//--------RMS Value Calculation---------------------//
		TIM2_Counter++;
		if(TIM2_Counter == 4)
		{
			TIM2_Counter = 0;  
			
			f32_VAC_Rms_Counter ++;
			if(f32_VAC_Rms_Counter == 50) 
				f32_VAC_Rms_Counter = 0;
			f32_VAC_Rms[f32_VAC_Rms_Counter] = (float)(f32_VAC*f32_VAC);
			f32SUMValue = 0.0;
			int16_t i=0;
			while(i<=50)
			{
				f32SUMValue += f32_VAC_Rms[i];
				i++;
			}
			f32SUMValue = f32SUMValue/50.0;
			f32RMSValue = sqrtf(f32SUMValue);
			if(f32RMSValue <= 0.5)
				f32RMSValue = 0.5;
			else if(f32RMSValue >= 30.1)
				f32RMSValue = 30.1;			
		}
		
//**************************************************************//
//		Digital Power Off Grid Unidirectional AC-DC Mode		//
//								Power Bell															//
//								2023.12.30															//
//**************************************************************//
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_13);		//GPIOB-13 LED2	

			if(u16Mode == InvertOffGrid)
			{
				
			
//-------------------AC Soft Start --------------------------//	
//				f32_VAC_Set_Update += f32_VAC_Set_Step;
//				if(f32_VAC_Set_Update >= f32_VAC_Set)
//					f32_VAC_Set_Update = f32_VAC_Set;
//				f32_VAC_Gain = f32_VAC_Set_Update/f32_VIN;
		
//----------------Voltage Loop PI Controller------------------------//
//                         G(s)=kp+ki/s                             //
//           G[2/Ts*(z-1)/(z+1)]=ki*Ts/2+kp+ki*Ts/(z-1)             //
//       Y(k)=Y(k-1)+x(k)*(ki*Ts/2+kp)+x(k-1)*(-kp+ki*Ts/2)         //
//------------------------------------------------------------------//
				OffGridPID.Err0_Vol = f32_VRms_Set - f32RMSValue;
				OffGridPID.Out0_Vol = OffGridPID.Out1_Vol + (float)(OffGridPID.Err0_Vol*(OffGridPID.Ki_Vol*OffGridPID.Ts/2 + OffGridPID.Kp_Vol));
				OffGridPID.Out0_Vol = OffGridPID.Out0_Vol + (float)(OffGridPID.Err1_Vol*(OffGridPID.Ki_Vol*OffGridPID.Ts/2 - OffGridPID.Kp_Vol));

				if(OffGridPID.Out0_Vol > 2.0)
					OffGridPID.Out0_Vol  = 2.0;
				else if(OffGridPID.Out0_Vol < 0.0)
					OffGridPID.Out0_Vol = 0.0;
				
				//if(PID.Out1_Vol >= 2.0)
				//	PID.Out1_Vol  = 2.0;
				//else if(PID.Out1_Vol < 0.0)
				//	PID.Out1_Vol = 0.0;
		
				OffGridPID.Err1_Vol = OffGridPID.Err0_Vol;
				OffGridPID.Out1_Vol = OffGridPID.Out0_Vol;
			
//				f32_VAC_Gain = (float) (f32_VAC_Gain *PID.Out0_Vol);
				f32_VAC_Gain = OffGridPID.Out0_Vol;
				if(f32_VAC_Gain >= 1.0)
					f32_VAC_Gain  = 1.0;
				else if(f32_VAC_Gain  < 0.2)
					f32_VAC_Gain  = 0.2;
			//Open Loop Test 2023.12.17
					f32_VAC_Gain  = 1.0;


//------------------ Current Loop PI Controller---------------------//
//                         G(s)=kp+ki/s                             //
//           G[2/Ts*(z-1)/(z+1)]=ki*Ts/2+kp+ki*Ts/(z-1)             //
//       Y(k)=Y(k-1)+x(k)*(ki*Ts/2+kp)+x(k-1)*(-kp+ki*Ts/2)         //
//------------------------------------------------------------------//				
				//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);		

/*
				f32_IAC_Ref = PID.Out0_Vol/f32RMSValue;// 
				f32_IAC_Ref = f32_IAC_Ref/1;// 
				f32_IAC_Ref = f32_IAC_Ref *SinVal;
				//f32_IAC_Ref = fabs(f32_IAC_Ref);
				//f32_IL_filtered = fabs(f32_IL_filtered);
				if(f32_IAC_Ref > 0.0)
					f32_IAC_Ref = f32_IAC_Ref;
				else if(f32_IAC_Ref <= 0.0)
					f32_IAC_Ref = -f32_IAC_Ref;
				if(f32_IAC_filtered > 0.0)
					f32_IL_filtered = f32_IL_filtered;
				else if(f32_IL_filtered <= 0.0)
					f32_IL_filtered = -f32_IL_filtered;			
				//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);					


				OffGridPID.Err0_Cur = f32_IAC_Ref - f32_IAC_filtered;
				
				if(OffGridPID.Err0_Cur >10.0)  // Avoid PIout NaN
					OffGridPID.Err0_Cur = 10.0;
				else if(OffGridPID.Err0_Cur <-10.0)
					OffGridPID.Err0_Cur = -10.0;
					
				OffGridPID.Out0_Cur = OffGridPID.Out1_Cur + (float)(OffGridPID.Err0_Cur*(OffGridPID.Ki_Cur*OffGridPID.Ts/2 + OffGridPID.Kp_Cur));
				OffGridPID.Out0_Cur = OffGridPID.Out0_Cur + (float)(OffGridPID.Err1_Cur*(OffGridPID.Ki_Cur*OffGridPID.Ts/2 - OffGridPID.Kp_Cur));
		
				if(PID.Out0_Cur > 1.0)
					PID.Out0_Cur  = 1.0;
				else if(PID.Out0_Cur < 0.0)
					PID.Out0_Cur = 0.0;

				f32VacVal = VacPeakVal*PID.Out0_Cur;
				PID.Err1_Cur = PID.Err0_Cur;
				PID.Out1_Cur = PID.Out0_Cur;
	*/		
//==================== SPWM Modulation ======================//
				u16VacFreqCounter ++;
				if (u16VacFreqCounter == INVERT_AC_FREQ)
				{
						u16VacFreqCounter = 0;
				}
				f32VacFreqCounter = (float)u16VacFreqCounter;
				f32VacFreq = (float)INVERT_AC_FREQ;
				VacTimer = f32VacFreqCounter/f32VacFreq;
				SinVal = arm_sin_f32(6.2831852*(float)VacTimer);
				f32VacVal = VacPeakVal*SinVal;
				//u16PwmUpdate = (int16_t)VacVal;
				
				f32VacVal = (float)(f32VacVal*f32_VAC_Gain);
				f32VacVal =  f32VacVal/2;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

//				if (f32VacVal >= 0.0)
//				{
					u16PwmUpdateCH2 = (int16_t)(f32VacVal+INVERT_PWM_HALF);
					u16PwmUpdateCH3 = (int16_t)(f32VacVal+INVERT_PWM_HALF);
//				}
//				else
//				{
//						u16PwmUpdateCH2 = PWM_20KHZ_HALF-u16VacVal;
//						u16PwmUpdateCH3 = (int16_t)(VacPeakVal + f32VacVal);	
//				}
				LL_TIM_OC_SetCompareCH2(TIM1, u16PwmUpdateCH2);
				LL_TIM_OC_SetCompareCH3(TIM1, u16PwmUpdateCH3);	
		


			}	
	
//**************************************************************//
//		Digital Power On Grid Bidirectional AC-DC Mode			//
//								Power Bell															//
//								2023.12.30															//
//**************************************************************//
//====================Full HF SPWM Modulation ======================//
	  	//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);		//GPIOB-12 LED3		

			if(u16Mode == InvertOnGrid)
			{
				
				if(u16LoopMode == InvertOnGridPR)
				{
//==================== Grid Voltage SOGI-PLL Function======================//
					SOGIPLL.u = f32_VAC_filtered;
					SOGIPLL.alpha0 = SOGIPLL.b0*SOGIPLL.u - SOGIPLL.b0*SOGIPLL.u2 + SOGIPLL.a1*SOGIPLL.alpha1 + SOGIPLL.a2*SOGIPLL.alpha2 ;
					SOGIPLL.beta0 = SOGIPLL.a1*SOGIPLL.beta1 + SOGIPLL.a2*SOGIPLL.beta2 + SOGIPLL.lamda * SOGIPLL.b0 *(SOGIPLL.u + 2*SOGIPLL.u1 + SOGIPLL.u2);
					SOGIPLL.u2 = SOGIPLL.u1;
					SOGIPLL.u1 = SOGIPLL.u;
					SOGIPLL.alpha2 = SOGIPLL.alpha1;
					SOGIPLL.alpha1 = SOGIPLL.alpha0;
					SOGIPLL.beta2 = SOGIPLL.beta1;
					SOGIPLL.beta1 = SOGIPLL.beta0;

//				SOGIPLL.d = arm_cos_f32(SOGIPLL.theta)*SOGIPLL.alpha + arm_sin_f32(SOGIPLL.beta);
					SOGIPLL.q = (-arm_sin_f32(SOGIPLL.theta)*SOGIPLL.alpha0) + (arm_cos_f32(SOGIPLL.theta)* SOGIPLL.beta0);

          SOGIPLL.error0 = SOGIPLL.q;
          SOGIPLL.out0 = SOGIPLL.out1 + (float)(SOGIPLL.error0*(SOGIPLL.ki*SOGIPLL.Ts/2 + SOGIPLL.kp));
          SOGIPLL.out0 = SOGIPLL.out0 + (float)(SOGIPLL.error1*(SOGIPLL.ki*SOGIPLL.Ts/2 - SOGIPLL.kp));
					
					if(SOGIPLL.out0 > 10.0)
						SOGIPLL.out0  = 10.0;
					else if(SOGIPLL.out0 < -10.0)
						SOGIPLL.out0 = -10.0;
						
					SOGIPLL.error1 = SOGIPLL.error0;
					SOGIPLL.out1 = SOGIPLL.out0;
				
					SOGIPLL.out0 = SOGIPLL.out0 + 314.15926;
					SOGIPLL.theta = SOGIPLL.theta + SOGIPLL.out0*SOGIPLL.Ts;
					if(SOGIPLL.theta > 6.28318)
						SOGIPLL.theta = SOGIPLL.theta - 6.28318;
					else if(SOGIPLL.theta < -6.28318)
						SOGIPLL.theta = SOGIPLL.theta + 6.28318;		
				
					
//================== On Grid PI Voltage Loop =======================//
//                         G(s)=kp+ki/s                             //
//           G[2/Ts*(z-1)/(z+1)]=ki*Ts/2+kp+ki*Ts/(z-1)             //
//       Y(k)=Y(k-1)+x(k)*(ki*Ts/2+kp)+x(k-1)*(-kp+ki*Ts/2)         //
//==================================================================//

          OnGridPID.Err0_Vol = f32_VDC_Set - f32_VIN_filtered;
          OnGridPID.Out0_Vol = OnGridPID.Out1_Vol + (float)(OnGridPID.Err0_Vol*(OnGridPID.Ki_Vol*OnGridPID.Ts/2 + OnGridPID.Kp_Vol));
          OnGridPID.Out0_Vol = OnGridPID.Out0_Vol + (float)(OnGridPID.Err1_Vol*(OnGridPID.Ki_Vol*OnGridPID.Ts/2 - OnGridPID.Kp_Vol));

					if(OnGridPID.Out0_Vol > 10.0)
						OnGridPID.Out0_Vol  = 10.0;
					else if(OnGridPID.Out0_Vol < -10.0)
						OnGridPID.Out0_Vol = -10.0;
		
					OnGridPID.Err1_Vol = OnGridPID.Err0_Vol;
					OnGridPID.Out1_Vol = OnGridPID.Out0_Vol;			

//=====================PR Control =========================//
	/*	
					OnGridPR.Err0 = (float)(arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol + f32_IG_filtered); //-(-f32_IL) //IL/IG sample has 180 degree with true current
					
					
					OnGridPR.Out0 = -OnGridPR.A2*OnGridPR.Out1-OnGridPR.A3*OnGridPR.Out2+OnGridPR.A4*OnGridPR.Err0+OnGridPR.A5*OnGridPR.Err1+OnGridPR.A6*OnGridPR.Err2;
					if (OnGridPR.Out0 > 0.95)
						OnGridPR.Out0 = 0.95;
					if (OnGridPR.Out0 < -0.95)
						OnGridPR.Out0 = -0.95;
				
					OnGridPR.Err3 = OnGridPR.Err2;
					OnGridPR.Err2 = OnGridPR.Err1;
					OnGridPR.Err1 = OnGridPR.Err0;
				
					OnGridPR.Out3 = OnGridPR.Out2;
					OnGridPR.Out2 = OnGridPR.Out1;
					OnGridPR.Out1 = OnGridPR.Out0;
					
					OnGridDuty = OnGridPR.Out0;
		*/			
//=====================2P2Z Control =========================//		
//                         G(s)=Kp*(z1+s)*(z2+s) /(p1+s) /(p2+s)                          //
//                       Y[z]/X[z]= Kp*(A1 + A2*z^-1 + A3*z^-2) /(B1 + B2*z^-1 + B3*z^-2)        //
//                 Y(k)*B1 + Y(k-1)*B2 + Y(k-2)*B3 = X(k)*A1*Kp + X(k-1)*A2*Kp + X(k-2)*A3*Kp
//==================================================================//

					OnGrid2P2Z.Err0 = (float)(arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol + f32_IG_filtered); //-(-f32_IL) //IL/IG sample has 180 degree with true current
					
					
					OnGrid2P2Z.Out0 = (OnGrid2P2Z.Err0*OnGrid2P2Z.A1*OnGrid2P2Z.Kp + OnGrid2P2Z.Err1*OnGrid2P2Z.A2*OnGrid2P2Z.Kp + OnGrid2P2Z.Err2*OnGrid2P2Z.A3*OnGrid2P2Z.Kp - OnGrid2P2Z.Out1*OnGrid2P2Z.B2 - OnGrid2P2Z.Out2*OnGrid2P2Z.B3)/OnGrid2P2Z.B1;
					if (OnGrid2P2Z.Out0 > 0.95)
						OnGrid2P2Z.Out0 = 0.95;
					if (OnGrid2P2Z.Out0 < -0.95)
						OnGrid2P2Z.Out0 = -0.95;
				
					OnGrid2P2Z.Err3 = OnGrid2P2Z.Err2;
					OnGrid2P2Z.Err2 = OnGrid2P2Z.Err1;
					OnGrid2P2Z.Err1 = OnGrid2P2Z.Err0;
				
					OnGrid2P2Z.Out3 = OnGrid2P2Z.Out2;
					OnGrid2P2Z.Out2 = OnGrid2P2Z.Out1;
					OnGrid2P2Z.Out1 = OnGrid2P2Z.Out0;
					
					OnGridDuty = OnGrid2P2Z.Out0;
											
				}
				
				if(u16LoopMode == InvertOnGridDQ)
				{
//==================== Grid Voltage SOGI-PLL Function======================//
					SOGIPLL.u = f32_VAC_filtered;
					SOGIPLL.alpha0 = SOGIPLL.b0*SOGIPLL.u - SOGIPLL.b0*SOGIPLL.u2 + SOGIPLL.a1*SOGIPLL.alpha1 + SOGIPLL.a2*SOGIPLL.alpha2 ;
					SOGIPLL.beta0 = SOGIPLL.a1*SOGIPLL.beta1 + SOGIPLL.a2*SOGIPLL.beta2 + SOGIPLL.lamda * SOGIPLL.b0 *(SOGIPLL.u + 2*SOGIPLL.u1 + SOGIPLL.u2);
					SOGIPLL.u2 = SOGIPLL.u1;
					SOGIPLL.u1 = SOGIPLL.u;
					SOGIPLL.alpha2 = SOGIPLL.alpha1;
					SOGIPLL.alpha1 = SOGIPLL.alpha0;
					SOGIPLL.beta2 = SOGIPLL.beta1;
					SOGIPLL.beta1 = SOGIPLL.beta0;

					SOGIPLL.d = arm_cos_f32(SOGIPLL.theta)*SOGIPLL.alpha0 + arm_sin_f32(SOGIPLL.theta)*(SOGIPLL.beta0);
					SOGIPLL.q = (-arm_sin_f32(SOGIPLL.theta)*SOGIPLL.alpha0) + (arm_cos_f32(SOGIPLL.theta)* SOGIPLL.beta0);

          SOGIPLL.error0 = SOGIPLL.q;
          SOGIPLL.out0 = SOGIPLL.out1 + (float)(SOGIPLL.error0*(SOGIPLL.ki*SOGIPLL.Ts/2 + SOGIPLL.kp));
          SOGIPLL.out0 = SOGIPLL.out0 + (float)(SOGIPLL.error1*(SOGIPLL.ki*SOGIPLL.Ts/2 - SOGIPLL.kp));
					
					if(SOGIPLL.out0 > 10.0)
						SOGIPLL.out0  = 10.0;
					else if(SOGIPLL.out0 < -10.0)
						SOGIPLL.out0 = -10.0;
						
					SOGIPLL.error1 = SOGIPLL.error0;
					SOGIPLL.out1 = SOGIPLL.out0;
				
					SOGIPLL.out0 = SOGIPLL.out0 + 314.15926;
					SOGIPLL.theta = SOGIPLL.theta + SOGIPLL.out0*SOGIPLL.Ts;
					if(SOGIPLL.theta > 6.28318)
						SOGIPLL.theta = SOGIPLL.theta - 6.28318;
					else if(SOGIPLL.theta < -6.28318)
						SOGIPLL.theta = SOGIPLL.theta + 6.28318;		

//==================== Grid Current SOGI-PLL Function======================//
					SOGIPLL_Ig.u = f32_IG_filtered;
					SOGIPLL_Ig.alpha0 = SOGIPLL_Ig.b0*SOGIPLL_Ig.u - SOGIPLL_Ig.b0*SOGIPLL_Ig.u2 + SOGIPLL_Ig.a1*SOGIPLL_Ig.alpha1 + SOGIPLL_Ig.a2*SOGIPLL_Ig.alpha2 ;
					SOGIPLL_Ig.beta0 = SOGIPLL_Ig.a1*SOGIPLL_Ig.beta1 + SOGIPLL_Ig.a2*SOGIPLL_Ig.beta2 + SOGIPLL_Ig.lamda * SOGIPLL_Ig.b0 *(SOGIPLL_Ig.u + 2*SOGIPLL_Ig.u1 + SOGIPLL.u2);
					SOGIPLL_Ig.u2 = SOGIPLL_Ig.u1;
					SOGIPLL_Ig.u1 = SOGIPLL_Ig.u;
					SOGIPLL_Ig.alpha2 = SOGIPLL_Ig.alpha1;
					SOGIPLL_Ig.alpha1 = SOGIPLL_Ig.alpha0;
					SOGIPLL_Ig.beta2 = SOGIPLL_Ig.beta1;
					SOGIPLL_Ig.beta1 = SOGIPLL_Ig.beta0;

					SOGIPLL_Ig.d = arm_cos_f32(SOGIPLL_Ig.theta)*SOGIPLL_Ig.alpha0 + arm_sin_f32(SOGIPLL_Ig.theta)*(SOGIPLL_Ig.beta0);
					SOGIPLL_Ig.q = (-arm_sin_f32(SOGIPLL_Ig.theta)*SOGIPLL_Ig.alpha0) + (arm_cos_f32(SOGIPLL_Ig.theta)* SOGIPLL_Ig.beta0);

          SOGIPLL_Ig.error0 = SOGIPLL_Ig.q;
          SOGIPLL_Ig.out0 = SOGIPLL_Ig.out1 + (float)(SOGIPLL_Ig.error0*(SOGIPLL_Ig.ki*SOGIPLL_Ig.Ts/2 + SOGIPLL_Ig.kp));
          SOGIPLL_Ig.out0 = SOGIPLL_Ig.out0 + (float)(SOGIPLL_Ig.error1*(SOGIPLL_Ig.ki*SOGIPLL_Ig.Ts/2 - SOGIPLL_Ig.kp));
					
					if(SOGIPLL_Ig.out0 > 10.0)
						SOGIPLL_Ig.out0  = 10.0;
					else if(SOGIPLL_Ig.out0 < -10.0)
						SOGIPLL_Ig.out0 = -10.0;
						
					SOGIPLL_Ig.error1 =SOGIPLL_Ig.error0;
					SOGIPLL_Ig.out1 = SOGIPLL_Ig.out0;
				
					SOGIPLL_Ig.out0 = SOGIPLL_Ig.out0 + 314.15926;
					SOGIPLL_Ig.theta = SOGIPLL_Ig.theta + SOGIPLL_Ig.out0*SOGIPLL.Ts;
					if(SOGIPLL_Ig.theta > 6.28318)
						SOGIPLL_Ig.theta = SOGIPLL_Ig.theta - 6.28318;
					else if(SOGIPLL_Ig.theta < -6.28318)
						SOGIPLL_Ig.theta = SOGIPLL_Ig.theta + 6.28318;

//==================== On Grid QD Voltage Loop ====================//
//                         G(s)=kp+ki/s                             //
//           G[2/Ts*(z-1)/(z+1)]=ki*Ts/2+kp+ki*Ts/(z-1)             //
//       Y(k)=Y(k-1)+x(k)*(ki*Ts/2+kp)+x(k-1)*(-kp+ki*Ts/2)         //
//==================================================================//
         OnGridDQ.error0_Volt = f32_VDC_Set - f32_VIN_filtered;
         OnGridDQ.out0_Volt = OnGridDQ.out1_Volt + (float)(OnGridDQ.error0_Volt*(OnGridDQ.ki_Volt*OnGridDQ.Ts/2 + OnGridDQ.kp_Volt));
         OnGridDQ.out0_Volt = OnGridDQ.out0_Volt + (float)(OnGridDQ.error1_Volt*(OnGridDQ.ki_Volt*OnGridDQ.Ts/2 - OnGridDQ.kp_Volt));


					if(OnGridDQ.out0_Volt > 0.98)
						OnGridDQ.out0_Volt  = 0.98;
					else if(OnGridDQ.out0_Volt < -0.98)
						OnGridDQ.out0_Volt = -0.98;
		
					OnGridDQ.error1_Volt = OnGridDQ.error0_Volt;
					OnGridDQ.out1_Volt = OnGridDQ.out0_Volt;	
				
//==================== DQ Control ======================//	
					
//======================== Q Axis PI Control ====================   //
//                         G(s)=kp+ki/s                             //
//           G[2/Ts*(z-1)/(z+1)]=ki*Ts/2+kp+ki*Ts/(z-1)             //
//       Y(k)=Y(k-1)+x(k)*(ki*Ts/2+kp)+x(k-1)*(-kp+ki*Ts/2)         //
//==================================================================//
          OnGridDQ.ref_Iq = 0.0;
          OnGridDQ.feedback_Iq = SOGIPLL_Ig.q;
          OnGridDQ.error0_Iq = OnGridDQ.ref_Iq - OnGridDQ.feedback_Iq;
          OnGridDQ.out0_Iq = OnGridDQ.out1_Iq + (float)(OnGridDQ.error0_Iq*(OnGridDQ.ki_Iq*OnGridDQ.Ts/2 + OnGridDQ.kp_Iq));
          OnGridDQ.out0_Iq = OnGridDQ.out0_Iq + (float)(OnGridDQ.error1_Iq*(OnGridDQ.ki_Iq*OnGridDQ.Ts/2 - OnGridDQ.kp_Iq));
					
// =============== Band Limit =================
					if(OnGridDQ.out0_Iq > 10.0)
						OnGridDQ.out0_Iq  = 10.0;
					else if(OnGridDQ.out0_Iq < -10.0)
						OnGridDQ.out0_Iq = -10.0;
// =============== Iterative =================
					OnGridDQ.error1_Iq = OnGridDQ.error0_Iq;
					OnGridDQ.out1_Iq = OnGridDQ.out0_Iq;	
//========= Q Axis DQ decoupling =============
					OnGridDQ.out0_Iq = -(OnGridDQ.out0_Iq + 0.03*SOGIPLL_Ig.d);
					if(OnGridDQ.out0_Iq > 10.0)
						OnGridDQ.out0_Iq  = 10.0;
					else if(OnGridDQ.out0_Iq < -10.0)
						OnGridDQ.out0_Iq = -10.0;		
					
//========= Q Axis Grid Voltage Feed-forward		
					OnGridDQ.out0_Iq = OnGridDQ.out0_Iq + SOGIPLL.q;			
					
//======================== D Axis PI Control =======================//
//                         G(s)=kp+ki/s                             //
//           G[2/Ts*(z-1)/(z+1)]=ki*Ts/2+kp+ki*Ts/(z-1)             //
//       Y(k)=Y(k-1)+x(k)*(ki*Ts/2+kp)+x(k-1)*(-kp+ki*Ts/2)         //
//==================================================================//
          OnGridDQ.ref_Id = OnGridDQ.out0_Volt;
          OnGridDQ.feedback_Id = SOGIPLL_Ig.d;
          OnGridDQ.error0_Id = OnGridDQ.ref_Id - OnGridDQ.feedback_Id;//100*pi*47*10e-6
          //OnGridDQ.error0_Id = OnGridDQ.ref_Id - OnGridDQ.feedback_Id;
          OnGridDQ.out0_Id = OnGridDQ.out1_Id + (float)(OnGridDQ.error0_Id*(OnGridDQ.ki_Id*OnGridDQ.Ts/2 + OnGridDQ.kp_Id));
          OnGridDQ.out0_Id = OnGridDQ.out0_Id + (float)(OnGridDQ.error1_Id*(OnGridDQ.ki_Id*OnGridDQ.Ts/2 - OnGridDQ.kp_Id));
					
// =============== Band Limit =================
					if(OnGridDQ.out0_Id > 10.0)
						OnGridDQ.out0_Id  = 10.0;
					else if(OnGridDQ.out0_Id < -10.0)
						OnGridDQ.out0_Id = -10.0;
// =============== Iterative =================		
					OnGridDQ.error1_Id = OnGridDQ.error0_Id;
					OnGridDQ.out1_Id = OnGridDQ.out0_Id;	
					
//========= D Axis DQ decoupling =============
					OnGridDQ.out0_Id = -OnGridDQ.out0_Id + 0.03*SOGIPLL_Ig.q; //100*pi*47*10e-6
					if(OnGridDQ.out0_Id > 10.0)
						OnGridDQ.out0_Id  = 10.0;
					else if(OnGridDQ.out0_Id < -10.0)
						OnGridDQ.out0_Id = -10.0;
					
//========= D Axis Grid Voltage Feed-forward
					OnGridDQ.out0_Id = OnGridDQ.out0_Id + SOGIPLL.d;
								
//========= DQ - Alpha,Beta Transforming				
					OnGridDQ.alpha = arm_cos_f32(SOGIPLL_Ig.theta)*OnGridDQ.out0_Id - arm_sin_f32(SOGIPLL_Ig.theta)*(OnGridDQ.out0_Iq);
					OnGridDQ.beta = arm_sin_f32(SOGIPLL_Ig.theta)*OnGridDQ.out0_Id + arm_cos_f32(SOGIPLL_Ig.theta)*(OnGridDQ.out0_Iq);
					OnGridDQ.duty = OnGridDQ.alpha;
					
					if (OnGridDQ.duty > 0.98)
						OnGridDQ.duty = 0.98;
					if (OnGridDQ.duty < -0.98)
						OnGridDQ.duty = -0.98;
					
					OnGridDuty = OnGridDQ.duty;			
				}
				

				
//==================== SPWM Modulation ======================//
					u16VacFreqCounter ++;
					if (u16VacFreqCounter == INVERT_AC_FREQ)
					{
							u16VacFreqCounter = 0;
					}
					f32VacFreqCounter = (float)u16VacFreqCounter;
					f32VacFreq = (float)INVERT_AC_FREQ;
					VacTimer = f32VacFreqCounter/f32VacFreq;
					//SinVal = arm_sin_f32(6.2831852*(float)VacTimer);
					f32VacVal = VacPeakVal*OnGridDuty;
					//u16PwmUpdate = (int16_t)VacVal;

					f32VacVal =  f32VacVal/2; // Here need a inverse, CH3 is PWM mode 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             

					u16PwmUpdateCH2 = (int16_t)(f32VacVal+INVERT_PWM_HALF);
					u16PwmUpdateCH3 = (int16_t)(f32VacVal+INVERT_PWM_HALF);

					LL_TIM_OC_SetCompareCH2(TIM1, u16PwmUpdateCH2);
					LL_TIM_OC_SetCompareCH3(TIM1, u16PwmUpdateCH3);				
				
				}				
			}
		
	}	



void USART2_IRQHandler(void)
{		
   
	LL_USART_ClearFlag_TC(USART2);
	//if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
   //{
    //UsartRxValue = (uint16_t)LL_USART_ReceiveData8(USART2);		
   //}
     
}




/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
