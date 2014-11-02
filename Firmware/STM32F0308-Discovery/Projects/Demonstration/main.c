/**f
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    02-October-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include "DS18b20.h"

#include <stdio.h>
#include <math.h>

/** @addtogroup STM32F0308-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// if SOFTWARE_BEEP defined then buzzer without generator connected
//#define SOFTWARE_BEEP

// if PA9_AS_ZCD defined then zero-cross signal connected to PA9 instead of PA14
// (to enable SWD debug) 
//#define PA9_AS_ZCD

// if defined, then temperature data send to UART
//#define UART_SEND_TEMP

// if defined, then test pin used for PA10
//#define PA10_AS_TEST_PIN

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t BlinkSpeed = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Beep ctrl -----------------------------------------------------------------*/


#define BTN_BEEP_GPIO_PORT           GPIOA
#define BTN_BEEP_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define BTN_BEEP_GPIO_PIN            GPIO_Pin_6

unsigned char bBeep = 0; // if =1, then beep

void Beep_Config()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the BTN_BEEP_GPIO_CLK Clock */
  RCC_AHBPeriphClockCmd(BTN_BEEP_GPIO_CLK, ENABLE);
  /* Configure the BTN_BEEP_GPIO_PIN pin */
  GPIO_InitStructure.GPIO_Pin = BTN_BEEP_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BTN_BEEP_GPIO_PORT, &GPIO_InitStructure);
}

unsigned char bBeepState = 0;
void Beep_Toggle()
{
  if (bBeepState)
    GPIO_SetBits(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN);
  else
    GPIO_ResetBits(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN);
  
  bBeepState = !bBeepState;
}

void Beep_On()
{
  bBeep = 1;
  GPIO_SetBits(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN);
}

void Beep_Off()
{
  bBeep = 0;
  GPIO_ResetBits(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN);
}


/* Triac ctrl -------------------------------------------------------------------*/
// duty cycle for full power
// supposed that all timers has 100 kHz tick -- full cycle is 10 ms
#define MAX_TRIAC_DUTY 1000
int iTriacDuty = 250; // 0..1000 -- from no to full power

// Zero cross detection
#define ZCD_GPIO_PORT             GPIOA
#define ZCD_GPIO_CLK              RCC_AHBPeriph_GPIOA
#ifdef PA9_AS_ZCD
  #define ZCD_GPIO_PIN              GPIO_Pin_9
#else
  #define ZCD_GPIO_PIN              GPIO_Pin_14
#endif

#define ZCD_EXTI_PORT             EXTI_PortSourceGPIOA
#ifdef PA9_AS_ZCD
  #define ZCD_EXTI_PIN              EXTI_PinSource9
  #define ZCD_EXTI_LINE             EXTI_Line9
#else
  #define ZCD_EXTI_PIN              EXTI_PinSource14
  #define ZCD_EXTI_LINE             EXTI_Line14
#endif

// Triac out
#define TRIAC_GPIO_PORT           GPIOB
#define TRIAC_GPIO_CLK            RCC_AHBPeriph_GPIOB
#define TRIAC_GPIO_PIN            GPIO_Pin_1


// Test out
#ifdef PA10_AS_TEST_PIN
  #define TEST_GPIO_PORT           GPIOA
  #define TEST_GPIO_CLK            RCC_AHBPeriph_GPIOA
  #define TEST_GPIO_PIN            GPIO_Pin_10
#endif

void TriacPins_Config()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // ZCD pin for input
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  RCC_AHBPeriphClockCmd(ZCD_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = ZCD_GPIO_PIN;
  GPIO_Init(ZCD_GPIO_PORT, &GPIO_InitStructure);
  
  // EXTI
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  // Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(ZCD_EXTI_PORT, ZCD_EXTI_PIN);
  /* Configure EXTI Line4 */
  EXTI_InitStructure.EXTI_Line = ZCD_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* Enable the TRIAC_GPIO_CLK Clock */
  RCC_AHBPeriphClockCmd(TRIAC_GPIO_CLK, ENABLE);
  /* Configure the TRIAC_GPIO_PIN pin */
  GPIO_InitStructure.GPIO_Pin = TRIAC_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(TRIAC_GPIO_PORT, &GPIO_InitStructure);
  
  #ifdef PA10_AS_TEST_PIN
    /* Enable the TEST_GPIO_CLK Clock */
    RCC_AHBPeriphClockCmd(TEST_GPIO_CLK, ENABLE);
    /* Configure the TEST_GPIO_PIN pin */
    GPIO_InitStructure.GPIO_Pin = TEST_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TEST_GPIO_PORT, &GPIO_InitStructure);
  #endif
}

// variables for timer configuration
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
__IO uint16_t CCR1_Val;

// timer for triac control
void TriacTimer_Config()
{
  uint16_t PrescalerValue = 0;
  
  CCR1_Val = iTriacDuty; 

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);  
 
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock  / 100000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM16, PrescalerValue, TIM_PSCReloadMode_Immediate);
  
  /* Output Compare Timing Mode configuration: Channel1 */
  // according to http://www-micrel.deis.unibo.it/LABARCH_2012/slidecorso2012/lab4.pdf
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OC1Init(TIM16, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Disable);
   
  /* Enable the TIM16 Trigger and commutation interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   

  /* TIM3 enable counter */
  TIM_Cmd(TIM16, ENABLE);
  
  TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
}

#define Timer_Restart() { TIM16->CNT = 0; TIM_Cmd(TIM16, ENABLE); }
#define Timer_Stop() { TIM_Cmd(TIM16, DISABLE); }
#define Timer_LoadCompare(x) { TIM_OCInitStructure.TIM_Pulse = x; TIM_OC1Init(TIM16, &TIM_OCInitStructure); }


unsigned char bTriac = 0;

// duration of tick in sec for all timers
#define TIMER_TICK 0.00001

// default AC grid frequency
#define DEFAULT_GRID_FREQ 60

// default AC grid period in ticks
#define DEFAULT_GRID_PERIOD ((int)(((1.0/((double)(DEFAULT_GRID_FREQ)))/TIMER_TICK)/2.0 + 0.5))

// acceptible frequency deviation -- 10% from DEFAULT_GRID_FREQ
#define MAX_PERIOD_DEVIATION ((int)(DEFAULT_GRID_PERIOD*0.1 + 0.5))

// period count for averaging
#define PERIOD_COUNT 50
int iPeriodCount = 0;
int iPeriod = DEFAULT_GRID_PERIOD;
int iPeriodNew = 0;
unsigned char bPeriodUpdate = 0;
int iPeriodSum = 0;

int iWidth = 100; // width of pulse
int iWidthCount = 0;
int iWidthSum = 0; // sum of widthes
int iWidthPrevTimer = 0;
  
void EXTI4_15_IRQHandler(void) 
{
  EXTI_ClearITPendingBit(ZCD_EXTI_LINE);

#ifdef PA10_AS_TEST_PIN
  if (GPIO_ReadInputDataBit(ZCD_GPIO_PORT, ZCD_GPIO_PIN))
    GPIO_SetBits(TEST_GPIO_PORT, TEST_GPIO_PIN);
  else
    GPIO_ResetBits(TEST_GPIO_PORT, TEST_GPIO_PIN);
#endif
  
  if (GPIO_ReadInputDataBit(ZCD_GPIO_PORT, ZCD_GPIO_PIN))
  {
    // rising edge!
    
    TIM14->CNT = 0; // restart shift delay timer 
    
    iPeriodNew = TIM17->CNT;
    TIM17->CNT = 0;
    
    // if this is acceptible
    if (abs(iPeriodNew - iPeriod) < MAX_PERIOD_DEVIATION)
    {
      // accept measurement
      iPeriodSum = iPeriodSum + iPeriodNew; 
      iPeriodCount++;
    }
    
    
    // period measurement
    if (iPeriodCount >= PERIOD_COUNT)
    {
      iPeriod = (iPeriodSum + (PERIOD_COUNT/2)) / PERIOD_COUNT;
      iPeriodSum = 0;
      iPeriodCount = 0;
      bPeriodUpdate = 1;
      
      // update compare value for triac timer
      CCR1_Val = iPeriod - iPeriod*iTriacDuty / MAX_TRIAC_DUTY;
      TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
      TIM_OC1Init(TIM16, &TIM_OCInitStructure);
    }
    
    // storing prev timer value for pulse width measurement
    iWidthPrevTimer = TIM17->CNT;
  }
  else
  {
    iWidthSum = iWidthSum + (TIM17->CNT - iWidthPrevTimer);
    iWidthCount++;
    if (iWidthCount >= PERIOD_COUNT)
    {
      iWidth = (iWidthSum + (PERIOD_COUNT/2)) / PERIOD_COUNT;
      iWidthSum = 0;
      iWidthCount = 0;
      
      // update compare value for delay timer
      CCR1_Val = iWidth/2 - iWidth/4;
      TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
      TIM_OC1Init(TIM14, &TIM_OCInitStructure);
    }
  }
}



// TIM16 compare interrupt -- triac open
void TIM16_IRQHandler()
{
  if(TIM_GetITStatus(TIM16, TIM_IT_CC1) == SET) 
  {
    TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
    Timer_Stop();
    if (iTriacDuty != 0)
      GPIO_SetBits(TRIAC_GPIO_PORT, TRIAC_GPIO_PIN);
  }
}

// timer for grid AC voltage period measurement
void PeriodTimer_Config()
{
  uint16_t PrescalerValue = 0; 

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);  
 
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock  / 100000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM17, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* TIM3 enable counter */
  TIM_Cmd(TIM17, ENABLE);
}


void ShiftTimer_Config()
{
  uint16_t PrescalerValue = 0;
  CCR1_Val = 1000; 

  /* TIM1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);  
 
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock  / 100000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM14, PrescalerValue, TIM_PSCReloadMode_Immediate);
  
  /* Output Compare Timing Mode configuration: Channel1 */
  // according to http://www-micrel.deis.unibo.it/LABARCH_2012/slidecorso2012/lab4.pdf
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OC1Init(TIM14, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Disable);
  
  /* Enable the TIM14 Trigger and commutation interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   

  /* TIM14 enable counter */
  TIM_Cmd(TIM14, ENABLE);
  
  TIM_ITConfig(TIM14, TIM_IT_CC1, ENABLE);
}


// TIM14 compare interrupt -- test signal
unsigned char bOut = 0;
unsigned char b50HzOut = 1;
unsigned char bBtnPressed50HzOut = 0;

void TIM14_IRQHandler()
{
  if(TIM_GetITStatus(TIM14, TIM_IT_CC1) == SET) 
  {
    TIM_ClearITPendingBit(TIM14, TIM_IT_CC1);
    //TIM14->CNT = 0;

    // restart triac timer
    if (iTriacDuty != MAX_TRIAC_DUTY)
      GPIO_ResetBits(TRIAC_GPIO_PORT, TRIAC_GPIO_PIN);
    Timer_Restart();

    /*
    if (bOut)
      GPIO_SetBits(TEST_GPIO_PORT, TEST_GPIO_PIN);
    else
      GPIO_ResetBits(TEST_GPIO_PORT, TEST_GPIO_PIN);
    
    bOut = !bOut;
    */
    /*
    if (bBtnPressed50HzOut) 
    { 
      b50HzOut = !b50HzOut;
      if (b50HzOut)
        CCR1_Val = 1000;
      else
        CCR1_Val = 833;
      
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
      TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
      TIM_OC1Init(TIM14, &TIM_OCInitStructure);
      TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Disable);
      
      bBtnPressed50HzOut = 0;
    }
    */
  }
}
/* PI-controller auto-tuning-----------------------------------------------------*/
// total point count for PI-controller auto-tuning
#define TUNE_POINT_COUNT 40
// difference between points
#define TUNE_TEMP_DELTA 0.5

// value of test step amplitude (100% of power as default)
#define TUNE_INPUT_VALUE 100.0

// desired closed loop response parameters
#define zeta_cl 1.0
#define w_0_cl_factor 1.5 // w_0 = w_0_cl_factor/tau

// time marks for step response
float fPoints_T[TUNE_POINT_COUNT];
float fPoints_Y[TUNE_POINT_COUNT];

// actual point count from measurement
int iPointCount = 0;

float fY_start = 0; // first point of grid

float fY_ss = 0; // steady-state value
float fY_0 = 0; // initial value

// first order approximation results
double fTau = 0;
double fK = 0;

float fK_p = 16.23729;
float fK_i = 0.019;

unsigned char bTuningMode = 0; // if =1 then automatic tuning activated

// start auto-tuning, InitialValue -- process output in idle steady-state
void Tuning_Start(float InitialValue)
{
  iPointCount = 0;
  fY_0 = InitialValue;

  fY_start = ceil(InitialValue/TUNE_TEMP_DELTA)*TUNE_TEMP_DELTA;
}

// add new measurement to recorded response
void Tuning_AddPoint(float Time, float Value)
{
  if (iPointCount >= TUNE_POINT_COUNT) return;

  if (Value >= (fY_start + TUNE_TEMP_DELTA*iPointCount))
  {
    fPoints_T[iPointCount] = Time;
    fPoints_Y[iPointCount] = Value;
    iPointCount++;
  }
}

// error codes for auto-tuning
#define TUNING_RESULT_OK 0
#define TUNING_RESULT_NOT_ENOUGH_DATA 1
#define TUNING_RESULT_WRONG_SHAPE 2
#define TUNING_RESULT_REGR_FAULT 3
// finilize auto-tuning and calculate coefficients, EndValue -- steady state value
// returns error code described above
unsigned char Tuning_Finilize(float EndValue)
{
  int i;
  float fMean_T = 0;
  float fMean_Y = 0;
  double fNum = 0;
  double fDen = 0;
  double fB;
  
  fY_ss = EndValue;

  // shape response to match steady state value
  i = iPointCount-1;
  while (i >= 0)
  {
    if (fPoints_Y[i] >= fY_ss) 
      iPointCount--;
    else
      break;

    i--;
  }

  if (iPointCount < 2) return TUNING_RESULT_NOT_ENOUGH_DATA;

  // calculate logarithms -- should be a line in result
  for (i = 0; i < iPointCount; i++)
  {
    if (fY_ss <= fPoints_Y[i]) return TUNING_RESULT_WRONG_SHAPE;
    if (fY_ss <= fY_0) return TUNING_RESULT_WRONG_SHAPE;

    fPoints_Y[i] = -log((fY_ss - fPoints_Y[i])/(fY_ss - fY_0));
  }

  // linear regression for slope calculation

  for (i = 0; i < iPointCount; i++)
    fMean_T = fMean_T + fPoints_T[i];
  fMean_T = fMean_T / iPointCount;

  for (i = 0; i < iPointCount; i++)
    fMean_Y = fMean_Y + fPoints_Y[i];
  fMean_Y = fMean_Y / iPointCount;

  for (i = 0; i < iPointCount; i++)
  {
    fNum = fNum + (fPoints_T[i] - fMean_T)*(fPoints_Y[i] - fMean_Y);
    fDen = fDen + (fPoints_T[i] - fMean_T)*(fPoints_T[i] - fMean_T);
  }
  if (fDen == 0) return TUNING_RESULT_REGR_FAULT;
  fB = fNum / fDen;
  
  // first order system parameters
  fTau = 1/fB;  
  fK = (fY_ss - fY_0)/TUNE_INPUT_VALUE;
  
  // PI controller parameters
  fK_i = w_0_cl_factor*w_0_cl_factor*fB/fK;
  fK_p = (2*zeta_cl*w_0_cl_factor - 1) / fK;

  return TUNING_RESULT_OK;
}

// function calculates maximal relative error between actual data and first-order approximation
float Tuning_CalculateMaxError()
{
  float fRange = fY_ss - fY_0;
  float fError = 0;
  int i;
  float fModelY; // output of model for given input
  
  // convert back to temperature values
  for (i = 0; i < iPointCount; i++)
    fPoints_Y[i] = fY_ss - exp(-fPoints_Y[i])*(fY_ss - fY_0);

  for (i = 0; i < iPointCount; i++)
  {
    fModelY = fY_0 + fK*TUNE_INPUT_VALUE*(1 - exp(-fPoints_T[i]/fTau));
    if (fabs(fModelY - fPoints_Y[i]) > fError)
      fError = fabs(fModelY - fPoints_Y[i]);
  }

  return (fError/fRange); // relative error
}


/* USART ------------------------------------------------------------------------*/
#ifdef UART_SEND_TEMP

  #define USART                        USART1
  #define USART_CLK                    RCC_APB2Periph_USART1

  #define USART_TX_PIN                 GPIO_Pin_9
  #define USART_TX_GPIO_PORT           GPIOA
  #define USART_TX_GPIO_CLK            RCC_AHBPeriph_GPIOA
  #define USART_TX_SOURCE              GPIO_PinSource9
  #define USART_TX_AF                  GPIO_AF_1

  #define USART_RX_PIN                 GPIO_Pin_10
  #define USART_RX_GPIO_PORT           GPIOA
  #define USART_RX_GPIO_CLK            RCC_AHBPeriph_GPIOA
  #define USART_RX_SOURCE              GPIO_PinSource10
  #define USART_RX_AF                  GPIO_AF_1

  void USART_Config()
  {
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(USART_TX_GPIO_CLK | USART_RX_GPIO_CLK, ENABLE);

    /* Enable USART clock */
    RCC_APB2PeriphClockCmd(USART_CLK, ENABLE); 

    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(USART_TX_GPIO_PORT, USART_TX_SOURCE, USART_TX_AF);

    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(USART_RX_GPIO_PORT, USART_RX_SOURCE, USART_RX_AF);
    
    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USART_TX_GPIO_PORT, &GPIO_InitStructure);
      
    /* Configure USART Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
    GPIO_Init(USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(USART, &USART_InitStructure);
      
    /* Enable USART */
    USART_Cmd(USART, ENABLE);
  }

  void USART_SendStr_Blocking(char* Str)
  {
    while (*Str != 0)
    {
      USART_SendData(USART, *Str);
      /* Wait till holding buffer empty */
      while(USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET) {};
      Str++;
    }
  }
#endif
/* ------------------------------------------------------------------------------*/
// sampling time [sec]
#define t_s 5
// K_i for discrete action
#define K_i (fK_i*t_s)

// output limits
#define u_max 100
#define u_min 0

// integrator
float fInt = 0; // maube use double?

// returns % of triac power: 0..100
float PI_Controller(float e)
{
  float u;
  
  // control
  u = K_i*fInt + fK_p*e;

  // integrator action
  if (u > u_max || u < u_min)
      ; // do nothing with integrator
  else
      fInt = fInt + e;

  // input saturation
  if (u > u_max)
      u = u_max;
  if (u < u_min)
      u = u_min;

  return u;
}

/* System variables ----------------------------------------------------------*/
int iTriacDutyPrev = 0; 
int iTimerDelayPrev = 0;

float fTemprRef = 35.5; // temperature reference
float fTempr = 35.5; // current temperature

int iPeriodCycle = 0; // period cycle counter for PI-auto-tuning
#define SENSOR_CONVERSION_DURATION 750 // DS18B20 conversion time in ms
//#define CYCLE_DURATION 5000 // cycle duration in ms
#define CYCLE_DURATION (t_s*1000) // cycle duration in ms
#define END_PAUSE_DURATION (CYCLE_DURATION - SENSOR_CONVERSION_DURATION)

// rotate timer
unsigned short int iRotatePeriod = 60; // in minutes
unsigned short int iRotateDuration = 20; // in seconds

// sensor calibration
float fSensK = 1; // gain
float fSensO = 0; // offset

float fE; // temperature feedback error
  

// state of main menu
typedef enum {MENU_MAIN_NONE = 0, MENU_MAIN_BASIC, MENU_MAIN_ADVANCED, MENU_MAIN_LAST} main_menu_state_t;
main_menu_state_t bMainMenuState = MENU_MAIN_NONE;
char* sMainMenuCaptions[] = {"", "Basic", "Advancd"}; // main menu captions
// state of basic menu
typedef enum {MENU_BASIC_T_REF = 0, MENU_BASIC_ROTATE_INTERVAL, MENU_BASIC_ROTATE_DURATIOIN, MENU_BASIC_LAST} basic_menu_state_t;
basic_menu_state_t bBasicMenuState = MENU_BASIC_T_REF;
char* sBasicMenuCaptions[] = {"Temp.ref", "Rot.int", "Rot.dur",}; // basic menu captions
char sBasicMenuStartDigit[] = {3, 3, 3}; // digit that start to modify first

// state of advanced menu
typedef enum {MENU_ADV_O = 0, MENU_ADV_K, MENU_ADV_PID_K_P, MENU_ADV_PID_K_I, MENU_ADV_PID_AUTO, MENU_ADV_LAST} adv_menu_state_t;
adv_menu_state_t bAdvMenuState = MENU_ADV_O;
char* sAdvMenuCaptions[] = {"T. Ofst", "T. Gain", "PID K_P", "PID K_I", "PID Auto"}; // basic menu captions
char sAdvMenuStartDigit[] = {7, 7, 7, 7, 7};

unsigned char bSubMenu = 0; // if =1 then basic or advanced menu selected
unsigned char bParameterSet = 0; // if =1 then show parameter

unsigned char bUpdateLCD = 0; // if =1 then LCD should be updated ASAP
unsigned char bStoreParams = 0; // if =1 then request to store parameters in flash

unsigned char bBlinkEn = 0; // if =1 then enable blink digit at LCD
unsigned char bBlinkLCD = 0; // blink now
unsigned char bBlinkIndex = 0; // index of digit to blink


// error handler variables
#define TEMP_ERROR_COUNT 5 // max concecutive ds18b20 errors before beep
unsigned char iError_TempCount = 0; // counter for ds18b20 errors
#define HEATER_ERROR_INTERVAL (30*60) // duration of output saturation before error signal [in sec]
#define HEATER_ERROR_COUNT (HEATER_ERROR_INTERVAL/t_s)
int iError_HeaterCount = 0; // counter for output saturation
#define MAX_TEMP_ERROR 2.0 // maximum acceptable temperature error for heater fault check [oC]


int iSecTimer; // timer for second
char sLine1[16]; // LCD line 1
char sLine2[16]; // LCD line 2
/* Buttons handler -----------------------------------------------------------*/

unsigned char bBtnMode = 0;
unsigned char bPrevBtnMode = 0;
unsigned char bBtnSet = 0;
unsigned char bPrevBtnSet = 0;
unsigned char bBtnInc = 0;
unsigned char bPrevBtnInc = 0;

unsigned char bBtnIndex = 0; // index of tested button

#define BTN_MODE_PRESSED ((bBtnMode != bPrevBtnMode) && (bBtnMode == 1))
#define BTN_SET_PRESSED ((bBtnSet != bPrevBtnSet) && (bBtnSet == 1))
#define BTN_INC_PRESSED ((bBtnInc != bPrevBtnInc) && (bBtnInc == 1))

char sTmp[8];
float IncDigit(char* S, int Index, float Div)
{
  float fRes = 0;
  signed char i;
  unsigned char iLast = 7;
  float fFactor = 1;
  
  // decrement digit
  if (S[Index] == '9') 
    S[Index] = '0';
  else
    S[Index] = S[Index] + 1;

  // last character
  for (i = 0; i < 8; i++)
    if (S[i] == ' ') 
    {
      iLast = i-1; break;
    }

  for (i = iLast; i >= 0; i--)
    if (S[i] != '.')
    {
      if (S[i] == '+' || S[i] == '-') continue;
      fRes = fRes + (S[i] - 0x30)*fFactor; 
      fFactor = fFactor*10;
    }
    
  return fRes/Div;
}


void Btns_Check()
{  
  GPIO_InitTypeDef  GPIO_InitStructure;

  if (bBeep) return; // if beep, then reset
  
  /* Configure the BTN_BEEP_GPIO_PIN pin as input */
  GPIO_InitStructure.GPIO_Pin = BTN_BEEP_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BTN_BEEP_GPIO_PORT, &GPIO_InitStructure);
  
  if (bBtnIndex == 0)
  {
    LCD_HAL_Write(0x10);
    if (GPIO_ReadInputDataBit(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN))
      bBtnMode = 1;
    else
      bBtnMode = 0;
  }
  else
  if (bBtnIndex == 1)
  {
    LCD_HAL_Write(0x20);
    if (GPIO_ReadInputDataBit(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN))
      bBtnSet = 1;
    else
      bBtnSet = 0;
  }
  else
  if (bBtnIndex == 2)
  {
    LCD_HAL_Write(0x40);
    if (GPIO_ReadInputDataBit(BTN_BEEP_GPIO_PORT, BTN_BEEP_GPIO_PIN))
      bBtnInc = 1;
    else
      bBtnInc = 0;
  }
  
  bBtnIndex++;
  if (bBtnIndex == 3) bBtnIndex = 0;

  
  if (BTN_MODE_PRESSED)
  {
    if (bSubMenu == 0)
    {
      // main menu selection
      bMainMenuState++;
      if (bMainMenuState == MENU_MAIN_LAST) bMainMenuState = 0;
    }
    else
    {
      // sub menu selection
      if (bMainMenuState == MENU_MAIN_BASIC)
      {
        bBasicMenuState++;
        if (bBasicMenuState == MENU_BASIC_LAST) { bBasicMenuState = 0; bSubMenu = 0; }      
      }
      else
      if (bMainMenuState == MENU_MAIN_ADVANCED)
      {
        bAdvMenuState++;
        if (bAdvMenuState == MENU_ADV_LAST) { bAdvMenuState = 0; bSubMenu = 0; }      
      }
    }
    
    bParameterSet = 0;
    bBlinkEn = 0;
    bBlinkLCD = 0;
    
    bUpdateLCD = 1;
  }
  
  if (BTN_SET_PRESSED)
  {
    if (bMainMenuState != MENU_MAIN_NONE)
    {
      if (bSubMenu == 0)
      {
        bSubMenu = 1;
      }
      else
      {
        if (bParameterSet == 0)
        {
          bParameterSet = 1;
          if (bMainMenuState == MENU_MAIN_ADVANCED && bAdvMenuState == MENU_ADV_PID_AUTO)
            sprintf(sLine2, "Ready  .");
        }
        else
        {
          if (bBlinkEn == 0)
          {
            if (bMainMenuState == MENU_MAIN_BASIC)
            {
              bBlinkIndex = sBasicMenuStartDigit[bBasicMenuState];
            }
            else
            if (bMainMenuState == MENU_MAIN_ADVANCED)
            {
              bBlinkIndex = sAdvMenuStartDigit[bAdvMenuState];
            }
            
            bBlinkEn = 1;
          }
          else
          {
            if (bBlinkIndex > 0)
            {
              if (bMainMenuState == MENU_MAIN_ADVANCED && bAdvMenuState == MENU_ADV_PID_AUTO)
              {
              }
              else
              {
                bBlinkIndex--;
                if (sLine2[bBlinkIndex] == '.') bBlinkIndex--;
              }
            }
            else
            {
              if (bMainMenuState == MENU_MAIN_BASIC)
              {
                bBlinkIndex = sBasicMenuStartDigit[bBasicMenuState];
              }
              else
              if (bMainMenuState == MENU_MAIN_ADVANCED)
              {
                bBlinkIndex = sAdvMenuStartDigit[bAdvMenuState];
              }
            }
          }
        }
      }
      
      bUpdateLCD = 1;
    }
  }
  
  if (BTN_INC_PRESSED)
  {
    // update parameter
    if (bBlinkEn)
    {
      bUpdateLCD = 1;
      bStoreParams = 1;
      
      if (bMainMenuState == MENU_MAIN_BASIC)
      {
        if (bBasicMenuState == MENU_BASIC_T_REF)
        {
          fTemprRef = IncDigit(sLine2, bBlinkIndex, 10);
        }
        if (bBasicMenuState == MENU_BASIC_ROTATE_INTERVAL)
        {
          iRotatePeriod = IncDigit(sLine2, bBlinkIndex, 1);
        }
        if (bBasicMenuState == MENU_BASIC_ROTATE_DURATIOIN)
        {
          iRotateDuration = IncDigit(sLine2, bBlinkIndex, 10);
        }
      }
      else
      if (bMainMenuState == MENU_MAIN_ADVANCED)
      {
        if (bAdvMenuState == MENU_ADV_O)
        {
          if (bBlinkIndex == 0)
            fSensO = -fSensO;
          else
            fSensO = IncDigit(sLine2, bBlinkIndex, 100000);
        }
        else
        if (bAdvMenuState == MENU_ADV_K)
          fSensK = IncDigit(sLine2, bBlinkIndex, 1000000);
        else
        if (bAdvMenuState == MENU_ADV_PID_K_P)
          fK_p = IncDigit(sLine2, bBlinkIndex, 100000);
        else
        if (bAdvMenuState == MENU_ADV_PID_K_I)
          fK_i = IncDigit(sLine2, bBlinkIndex, 1000000);
        else
        if (bAdvMenuState == MENU_ADV_PID_AUTO)
        {
          if (bTuningMode == 0)
          {
            // activate auto-tuning mode
            bTuningMode = 1;
            // full turn-on of heater
            iTriacDuty = MAX_TRIAC_DUTY;
            Timer_LoadCompare(iPeriod - (iPeriod*iTriacDuty / MAX_TRIAC_DUTY));
            // prepare for auto-tuning
            iPeriodCycle = 0;
            Tuning_Start(fTempr);
          }
          else
          {
            // finilizing auto-tuning            
            if (Tuning_Finilize(fTempr) == TUNING_RESULT_OK)
            {
              fE = Tuning_CalculateMaxError();
              sprintf(sLine2, "ok! %2.1f %%", fE);
            }
            else
            {
              sprintf(sLine2, "failed:(");
              bStoreParams = 0;
            }
            LCD_DisplayStringLine(1, sLine2);
            bUpdateLCD = 0;
            bTuningMode = 0;
          }
        }
      }
      
      
    }
  }

  
  bPrevBtnMode = bBtnMode;
  bPrevBtnSet = bBtnSet;
  bPrevBtnInc = bBtnInc;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(BTN_BEEP_GPIO_PORT, &GPIO_InitStructure);
  

}

char sTmpLine[10];
void UpdateLCD()
{
  // display temperature in normal case
  if (bMainMenuState == MENU_MAIN_NONE)
  {
    sprintf(sLine1, "%2.1f oC ", fTempr);
    LCD_DisplayStringLine(0, sLine1);
    
    sprintf(sLine2, "%d%%  ", iTriacDuty/10);
    LCD_DisplayStringLine(1, sLine2);
    
    return;
  }
  
  // update LCD according to current menu state
  if (bParameterSet == 0)
  {
    if (bSubMenu == 0)
    {
      sprintf(sLine1, "%-8s", sMainMenuCaptions[bMainMenuState]);      
      sprintf(sLine2, "%-8s", "");
    }
    else
    if (bSubMenu == 1)
    {
      sprintf(sLine1, "%-8s", sMainMenuCaptions[bMainMenuState]);        
      if (bMainMenuState == MENU_MAIN_BASIC)
      {
        sprintf(sLine2, "%-8s", sBasicMenuCaptions[bBasicMenuState]);
      }
      else
      if (bMainMenuState == MENU_MAIN_ADVANCED)
      {
        sprintf(sLine2, "%-8s", sAdvMenuCaptions[bAdvMenuState]);
      }
    }
  }
  else
  {
    if (bMainMenuState == MENU_MAIN_BASIC)
    {
      sprintf(sLine1, "%-8s", sBasicMenuCaptions[bBasicMenuState]);
      
      if (bBasicMenuState == MENU_BASIC_T_REF)
        sprintf(sLine2, "%04.1f  oC", fTemprRef);
      else
      if (bBasicMenuState == MENU_BASIC_ROTATE_INTERVAL)
        sprintf(sLine2, "%04d min", iRotatePeriod);
      else
      if (bBasicMenuState == MENU_BASIC_ROTATE_DURATIOIN)
        sprintf(sLine2, "%04d sec", iRotateDuration);
    }
    else
    if (bMainMenuState == MENU_MAIN_ADVANCED)
    {
      sprintf(sLine1, "%-8s", sAdvMenuCaptions[bAdvMenuState]);
      
      if (bAdvMenuState == MENU_ADV_O)
        sprintf(sLine2, "%+8.6f", fSensO);
      else
      if (bAdvMenuState == MENU_ADV_K)
        sprintf(sLine2, "%8.6f", fSensK);
      else
      if (bAdvMenuState == MENU_ADV_PID_K_P)
        sprintf(sLine2, "%8.5f", fK_p);
      else
      if (bAdvMenuState == MENU_ADV_PID_K_I)
        sprintf(sLine2, "%8.6f", fK_i);
      else
      if (bAdvMenuState == MENU_ADV_PID_AUTO)
      {
        if (bTuningMode)
        {
          sprintf(sLine1, "%2.1f oC", fTempr);
          sprintf(sLine2, "Tuning..");
        }
      }
    }
  }
  
  strcpy(sTmpLine, sLine2);
  if (bBlinkLCD)
  {
    sTmpLine[bBlinkIndex] = ' ';
  }
  
  LCD_DisplayStringLine(0, sLine1);
  LCD_DisplayStringLine(1, sTmpLine);
}

/* Parameters store/recall ---------------------------------------------------*/
#define FLASH_PARAM_BASE 0x08003C00 // last page in program memory for parameters
#define FLASH_TEMP_REF_OFST 0
#define FLASH_ROTATE_PERIOD_OFST 1
#define FLASH_ROTATE_DURATION_OFST 2
#define FLASH_SENS_K_OFST 3
#define FLASH_SENS_O_OFST 4
#define FLASH_PID_K_P_OFST 5
#define FLASH_PID_K_I_OFST 6



void Flash_StoreParameters()
{
  FLASH_Unlock();
  
  if (FLASH_ErasePage(FLASH_PARAM_BASE) != FLASH_COMPLETE) return; // TODO: fault handling here
  
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_TEMP_REF_OFST*4, *((uint32_t*)(&fTemprRef)));
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_ROTATE_PERIOD_OFST*4, iRotatePeriod); 
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_ROTATE_DURATION_OFST*4, iRotateDuration);
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_SENS_K_OFST*4, *((uint32_t*)(&fSensK))); 
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_SENS_O_OFST*4, *((uint32_t*)(&fSensO))); 
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_PID_K_P_OFST*4, *((uint32_t*)(&fK_p)));
  FLASH_ProgramWord(FLASH_PARAM_BASE + FLASH_PID_K_I_OFST*4, *((uint32_t*)(&fK_i)));
  
  FLASH_Lock();
}

void Flash_LoadParameters()
{
  int iTmp;
  iTmp = *((int*)(FLASH_PARAM_BASE + FLASH_TEMP_REF_OFST*4));
  if (iTmp == -1) 
  { 
    Flash_StoreParameters(); 
    return; 
  }

  fTemprRef = *((float*)(FLASH_PARAM_BASE + FLASH_TEMP_REF_OFST*4));
  iRotatePeriod = *((int*)(FLASH_PARAM_BASE + FLASH_ROTATE_PERIOD_OFST*4)); 
  iRotateDuration = *((int*)(FLASH_PARAM_BASE + FLASH_ROTATE_DURATION_OFST*4));
  fSensK = *((float*)(FLASH_PARAM_BASE + FLASH_SENS_K_OFST*4)); 
  fSensO = *((float*)(FLASH_PARAM_BASE + FLASH_SENS_O_OFST*4)); 
  fK_p = *((float*)(FLASH_PARAM_BASE + FLASH_PID_K_P_OFST*4));
  fK_i = *((float*)(FLASH_PARAM_BASE + FLASH_PID_K_I_OFST*4));
}
/* Rotation ------------------------------------------------------------------*/
#define ROTATION_GPIO_PORT           GPIOA
#define ROTATION_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define ROTATION_GPIO_PIN            GPIO_Pin_10

void Rotation_Config()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the ROTATION_GPIO_CLK Clock */
  RCC_AHBPeriphClockCmd(ROTATION_GPIO_CLK, ENABLE);
  /* Configure the ROTATION_GPIO_PIN pin */
  GPIO_InitStructure.GPIO_Pin = ROTATION_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ROTATION_GPIO_PORT, &GPIO_InitStructure);
}

#define Rotation_On() GPIO_SetBits(ROTATION_GPIO_PORT, ROTATION_GPIO_PIN);
#define Rotation_Off() GPIO_ResetBits(ROTATION_GPIO_PORT, ROTATION_GPIO_PIN);

int iSecRotation = 0; // for 1 sec duration
int iMinRotation = 0; // for 1 min duration
unsigned char bRotationOn = 0;

unsigned short int iRotatePeriod_Timer = 0; // in minutes
unsigned short int iRotateDuration_Timer = 0; // in seconds

// called every 1 sec for rotation timer function
void Rotate_Check()
{
  iMinRotation++;
  if (iMinRotation == 60)
  {
    iMinRotation = 0;
    
    iRotatePeriod_Timer++;
    if (iRotatePeriod_Timer >= iRotatePeriod)
    {
      iRotatePeriod_Timer = 0;
      bRotationOn = 1;
      Rotation_On();
    }
  }
  
  if (bRotationOn)
  {
    iRotateDuration_Timer++;
    if (iRotateDuration_Timer >= iRotateDuration)
    {
      Rotation_Off();
      iRotateDuration_Timer = 0;
    }
  }
}



void IWDT_Init(void)
{
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/256, ~6.4ms */
  IWDG_SetPrescaler(IWDG_Prescaler_256);

  IWDG_SetReload(1250); // IWDG timeout 6.4ms*1250 = ~ 8 sec

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();

}
/* MAIN ----------------------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  char sTmp[8];
  int iRes = 0;
  unsigned char bData[8];
  RCC_ClocksTypeDef RCC_Clocks;
  
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
 
  Delay(100);
  
  Beep_Config();
  
  LCD_Setup();
  LCD_DisplayStringLine(0, "ver 1.00");
  Delay(1000);
/* 
while (1)
{
  Btns_Check();
  
  if (bUpdateLCD)
  {
    UpdateLCD();
    bUpdateLCD = 0;
  }
  
  Delay(10);
}
*/

  Flash_LoadParameters();
	
  TriacPins_Config();
  TriacTimer_Config();
  PeriodTimer_Config();
  ShiftTimer_Config();
  
  Rotation_Config();
  IWDT_Init();
  
  #ifdef UART_SEND_TEMP
    USART_Config();
  #endif
  
  ds_init();

  while (1)
  {    
    ds_start_convert_single();
    ds_strong_pull_up(1);
    
    
    TimingDelay = SENSOR_CONVERSION_DURATION; // we need wait 750 ms in total
    
    // slow actions during waiting for 750 ms:
    
    if (bTuningMode == 0)
    {
      // normal operation -- PI action
      fE = fTemprRef - fTempr;
      iTriacDuty = PI_Controller(fE)*10;
      if (iTriacDutyPrev != iTriacDuty)
      {
        // update compare value for triac timer
        if (iTriacDuty > 1)
          Timer_LoadCompare(iPeriod - (iPeriod*iTriacDuty / MAX_TRIAC_DUTY));
      }
      iTriacDutyPrev = iTriacDuty;
      
      // heater fault check
      if (fabs(fE) > MAX_TEMP_ERROR)
      {
        iError_HeaterCount++; 
        if (iError_HeaterCount >= HEATER_ERROR_COUNT) 
        {
          Beep_On();
        }
      }
      else
      {
        iError_HeaterCount = 0;
        Beep_Off();
      }
      
    }
    else
    {
      Tuning_AddPoint(iPeriodCycle*CYCLE_DURATION*0.001, fTempr);
      iPeriodCycle++;
    }
    
    
    // output temperature
    UpdateLCD();
    
    #ifdef UART_SEND_TEMP
      sprintf(sTmp, "%2.1f\r\n", fTempr);
      USART_SendStr_Blocking(sTmp);
    #endif

/*    
    // show period
    if (bPeriodUpdate)
    {
      sprintf(sTmp, "%04d %02d", iPeriod, iWidth);
      //LCD_DisplayStringLine(0, sTmp);
      bPeriodUpdate = 0; 
    }
*/
    
    // wait for end delay and check push buttons
    iTimerDelayPrev = TimingDelay;
    while (TimingDelay != 0)
    {
      Btns_Check();
      if (bUpdateLCD)
      {
        UpdateLCD();
        bUpdateLCD = 0;
        
        if (bStoreParams) 
        {
          Flash_StoreParameters(); bStoreParams = 0;
        }
      }
      
      while (TimingDelay == iTimerDelayPrev) ; // 1ms wait here
      iTimerDelayPrev = TimingDelay;
      
    }
    
    // wait for some time for full cycle
    TimingDelay = END_PAUSE_DURATION;
    
    ds_strong_pull_up(0);
    if (ds_read_data_single(bData) == 0)
    {
      iRes = ds_conv_to_temperature(bData);
      fTempr = iRes / 10.0;
    
      fTempr = fSensK*fTempr + fSensO; // correction with gain and offset
      iError_TempCount = 0;
    }
    else
    {
      iError_TempCount++;
      if (iError_TempCount >= TEMP_ERROR_COUNT)
        Beep_On();
    }
    
    while (TimingDelay != 0)
    {
      Btns_Check();
      if (bUpdateLCD)
      {
        UpdateLCD();
        bUpdateLCD = 0;
        
        if (bStoreParams) 
        {
          Flash_StoreParameters(); bStoreParams = 0;
        }
      }
    }
    
    IWDG_ReloadCounter();
    
  }
  
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 1 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
  
  #ifdef SOFTWARE_BEEP
    if (bBeep)
      Beep_Toggle();
  #endif
    
  if (bBlinkEn)
  {
    iSecTimer++;
    if (iSecTimer == 250)
    {
      iSecTimer = 0;
      
      bBlinkLCD = !bBlinkLCD;
      bUpdateLCD = 1;
    }
  }
  
  iSecRotation++;
  if (iSecRotation == 1000)
  {
    iSecRotation = 0;
    Rotate_Check();
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
