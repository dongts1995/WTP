

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_6Steps
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define COUNTER_FREQUENCY			80    // kHz
#define PERIOD_COUNTER 				72000/COUNTER_FREQUENCY-1
#define DEAD_TIME							200
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
NVIC_InitTypeDef nvicConfig;
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;	
NVIC_InitTypeDef   NVIC_InitStructure;
uint16_t debug = 0;
uint16_t pri_debug = 0;


__IO uint16_t ADC_Value[3];
int CCR1_Val = 899, CCR2_Val = 0;
int get_ccr1 = 0, get_ccr2 = 0;
uint16_t teta = 100;
int count_test = 0;
int i = 0;
int i_control = 0;
int control_status = 0;

float i_adc_avg = 0.0, current_value = 0.0, current_offset = 2.0;
int a0_val[100] = {0};
uint32_t sum_i_adc = 0;
float ek=0, ek_1=0, uk=0, uk_1=0;
float Kp = 0.0005;
float Ki = 22345.00;
float Kd = 0.00;
const float dt = 0.00025;
#define MIN_INTEGRAL		0
#define MAX_INTEGRAL		180
static float _integral;



/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void PhaseShift_Config(void);
void ADC_DMA_Config(void);
void EXTI0_Config(void);	     // extternal interupt
void TimerInterrupt_Config(void);
// TIM2 Interrupt Function
void TIM2_IRQHandler(void);



//void NVIC_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* Reset Clock Control Configuration */
	// Config clock TIM1, GPIOA, GPIOB for phase_shift
	// Config ADC clock (max14Mhz), DMA for ADC
  RCC_Configuration();

  /* GPIO Configuration */
	// A8, A9 -> Channel 1, 2 and  B13, B14 -> Channel 1N, 2N for phase_shift
	// Config A0,A1,A2 for ADC
	/* Configure PA.01 in input pull up */
  GPIO_Configuration();
	
  /* SysTick Configuration */
  SysTick_Configuration();
	
	/* pulse phase shift config */
	// use:
	//			TIM_SetCompare1(TIM1,CCR1_Val);
	//			TIM_SetCompare2(TIM1,CCR2_Val);
	//			get_ccr1 = TIM_GetCapture1(TIM1);
	//			get_ccr2 = TIM_GetCapture2(TIM1);
	PhaseShift_Config();
	
	/* adc and dma config*/
	// use:
	// 			ADC_Value[0] = A0 voltage
	// 			ADC_Value[1] = A1 voltage
	// 			ADC_Value[2] = A2 voltage
	ADC_DMA_Config();
	
	TimerInterrupt_Config();
	
  while (1)
  {
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==0)
			{
				control_status = 1;
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
			}
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)==1)
			{
				control_status = 0;
				GPIO_ResetBits(GPIOA, GPIO_Pin_2);
			}
		
			
		if (i_control == 1)
			{
				i_control = 0;
				
				// get avg value adc
				sum_i_adc = 0;
				for (i=0; i<1000; i++)
				{
					sum_i_adc += ADC_Value[0];
				}
				i_adc_avg = sum_i_adc/1000;
				
				
				//coverter to current value
				current_value = ((float)(i_adc_avg+1)/4096)*3.3;
				//current_value = (current_value*2+1)/1.4142;
				ek = current_offset - current_value;
				
				// current control
				_integral += Ki*ek*dt;
				if(_integral > MAX_INTEGRAL) _integral = MAX_INTEGRAL;
				else if (_integral < MIN_INTEGRAL) _integral = MIN_INTEGRAL;
				uk = Kp*ek + _integral + Kd*(ek - ek_1)/dt;
				ek_1 = ek;
				if(uk>=175) uk=175;
				else if(uk<=5) uk=5;
				
				if(control_status == 1)
				{
					teta = (uint16_t)uk;
				}
				else teta = 70;
								
				TIM_SetCompare2(TIM1,(180 - teta)*5);   // TIM_SetCompare2(TIM1,(180 - teta)*5);
			}
	}
}

void RCC_Configuration(void)
{  
	// Config clock TIM1, GPIOA, GPIOB for phase_shift; GPIOA for Led
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);
	
	// Config ADC clock (max14Mhz), DMA for ADC
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 12Mhz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration: A8, A9 - > Channel 1, 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOB Configuration: B13, B14 -> Channel 1N, 2N as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Config A0 for ADC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure PA.01 pin as input pull up */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;														// Pin_1 in PA01
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;								//
  GPIO_Init(GPIOA, &GPIO_InitStructure);															// GPIOA in PA01
	
	// config A2 for led
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  

	
}

void SysTick_Configuration(void)
{

}


void PhaseShift_Config(void)
{
	// config counter
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = PERIOD_COUNTER;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	//-------
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	// Set the TIM Output compare preload.
	TIM1->CCMR1 |= 0x00000008;
	TIM1->CCMR1 |= 0x00000800;
  /* Channel 1, 2 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	//--------------
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInitStructure.TIM_DeadTime = DEAD_TIME;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;     
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
	//------------
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
		
	TIM1->CCR1 = CCR1_Val;
	TIM1->CCR2 = CCR2_Val;
}

void ADC_DMA_Config(void)
{
	/* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


void TimerInterrupt_Config(void)
	{
		// init Timer2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
    timerInitStructure.TIM_Prescaler = 10; // Chia tan so cho x 
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 1799;    // count from 0 to x
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM_Cmd(TIM2, ENABLE);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		
		// Init Interrupt;
		
		nvicConfig.NVIC_IRQChannel = TIM2_IRQn;
		nvicConfig.NVIC_IRQChannelPreemptionPriority = 0;
		nvicConfig.NVIC_IRQChannelSubPriority = 1;
		nvicConfig.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvicConfig);
	}
	
	void TIM2_IRQHandler(void)
{
  /* Generate TIM1 COM event by software */
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

			i_control = 1;
    }
}
	
	
/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
