#include "DS18b20.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"


//*********************************************************************************************
// Hardware defines
// timer for us delay
#define TIMER       TIM3
#define TIMER_CLK   RCC_APB1Periph_TIM3

// 1-wire pin   
#define PORT        GPIOC
#define PIN         GPIO_Pin_13
#define PORT_CLK    RCC_AHBPeriph_GPIOC

//*********************************************************************************************
// Timer
// restart timer
#define timer_reset()      TIMER->CNT = 0
// wait for specific interval in us from timer restart
#define timer_wait_us(x)   while (TIMER->CNT < (x)) {};


//*********************************************************************************************
// function  Reset pulse to ds18b20
// argument  none
// return    0 - device is ok, 1 - device not found, 2 - short circuit to ground
//*********************************************************************************************
uint8_t ds_reset_pulse()
{
   uint16_t result;   
 
   
   if ( GPIO_ReadInputDataBit(PORT, PIN) == 0 )  return 2; // if short circuit to ground
   GPIO_ResetBits(PORT, PIN);
   timer_reset();
   timer_wait_us(480);
   GPIO_SetBits(PORT, PIN);
   timer_wait_us(550); 
   result = GPIO_ReadInputDataBit(PORT, PIN); // reading the bus 
   timer_wait_us(960);  // wait for end of initialization (960 us)
   if(result) return 1; // no response from device
   return 0; 
}

//*********************************************************************************************
// function  Transmitting of one bit
// argument  bit value (0/1)
// return    none
//*********************************************************************************************
void ds_write_bit(uint8_t bit)
{
   timer_reset();
   GPIO_ResetBits(PORT, PIN);
   timer_wait_us(2);
   if (bit) GPIO_SetBits(PORT, PIN);
   timer_wait_us(60);
   GPIO_SetBits(PORT, PIN);
}

//*********************************************************************************************
// function  Reading of one bit
// argument  none
// return    bit value (0/1)
//*********************************************************************************************
uint16_t ds_read_bit()
{
   uint16_t result;
 
   timer_reset();
   GPIO_ResetBits(PORT, PIN);
   timer_wait_us(2);
   GPIO_SetBits(PORT, PIN);
   timer_wait_us(15);
   result = GPIO_ReadInputDataBit(PORT, PIN);
   timer_wait_us(60); 
   return result;
}

//*********************************************************************************************
// function  Transmitting of single byte
// argument  byte value
// return    none
//*********************************************************************************************
void ds_write_byte(uint8_t byte)
{
   uint8_t i;
   for (i=0;i<8;i++) ds_write_bit(byte&(1<<i));
}

//*********************************************************************************************
// function  Reading of single byte
// argument  none
// return    byte value
//*********************************************************************************************
uint8_t ds_read_byte()
{
   uint8_t i,result = 0;
   for(i=0;i<8;i++) 
   if(ds_read_bit()) result |= 1<<i; 
   return result;
}
//*********************************************************************************************
// function  Start temperature comversion for ds18b20
// argument  none
// return    0 - succeeded, 1 - device not found, 2 - short circuit to ground
//*********************************************************************************************
uint8_t ds_start_convert_single()
{
  uint8_t result;
  result = ds_reset_pulse();      // reset pulse
  if(result) return result;       // return error code
  ds_write_byte(0xCC);            // eneable access without ROM ID
  ds_write_byte(0x44);            // start command
  return 0;
}

//*********************************************************************************************
// function  Reading memory data of ds18b20
// argument  buff - pointer to array (8 bytes)
// return    0 - succeeded, 1 - device not found, 2 - short circuit to ground, 3 - CRC error
//*********************************************************************************************
uint8_t ds_read_data_single(uint8_t *buff)
{
  uint8_t crc = 0;
  uint8_t data;
  uint8_t i,j;
  uint8_t tmp;
 
  tmp = ds_reset_pulse();         // reset pulse
  if(tmp) return tmp;             // return error code
  ds_write_byte(0xCC);            // eneable access without ROM ID
 
  ds_write_byte(0xBE);            // memory contents request
 
  // reading 8 bytes
  for( i=0; i<8; i++) 
  {
    data = ds_read_byte();     
    buff[i] = data; // store in buffer
 
    // CRC calculations
    for( j=0; j<8; j++)         
    {
      tmp = (crc ^ data) & 0x01;
      if (tmp==0x01) crc = crc ^ 0x18;
      crc = (crc >> 1) & 0x7F;
      if (tmp==0x01) crc = crc | 0x80;
      data = data >> 1;
    }		
  }
 
  data = ds_read_byte();         // CRC from device
  if(crc==data) return 0;        // CRC check
  return 3; // CRC error
}

//*********************************************************************************************
// function  Hardware initialization for ds18b20 access
// argument  none
// return    none
//*********************************************************************************************
void ds_init()
{
/*
int i;
unsigned char b = 0;
int iSec;
*/
  GPIO_InitTypeDef  GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  RCC_APB1PeriphClockCmd(TIMER_CLK, ENABLE); // timer clock enable
  // one tick of timer = 1 us
  TIM_TimeBaseStructure.TIM_Prescaler = 48-1;
  //TIM_TimeBaseStructure.TIM_Prescaler = 8-1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1000;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);
  TIM_Cmd(TIMER, ENABLE);  // enable timer
  
  
  RCC_AHBPeriphClockCmd(PORT_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PORT, &GPIO_InitStructure);
  GPIO_SetBits(PORT, PIN);

/*
ds_strong_pull_up(1);
while (1)
{
timer_reset();
timer_wait_us(100);
  
if (b)
GPIO_SetBits(PORT, PIN);
else
GPIO_ResetBits(PORT, PIN);

b = !b;
}
*/
/*
for (iSec = 0; iSec < 10; iSec++)
{
for (i = 0; i < 2000; i++)
{
timer_reset();
timer_wait_us(500);
}

if (b)
GPIO_SetBits(PORT, PIN);
else
GPIO_ResetBits(PORT, PIN);

b = !b;
}
*/
//ds_strong_pull_up(0);

}

//*********************************************************************************************
// function  Strong pull-up enable for power on ds18b20 during temperature conversion
// argument  if enable = 1, then connect the pin to VCC, otherwise leave floating (external pull-up)
// return    none
//*********************************************************************************************
void ds_strong_pull_up(uint8_t enable)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  if (enable)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  else
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PORT, &GPIO_InitStructure);
  GPIO_SetBits(PORT, PIN);
}

//*********************************************************************************************
// function  Conversion of the read data into temperature
// argument  buff - pointer to buffer with ds18b20 memory contens
// return    signed temperature value in 0.1 C (ex, 28.3 C = 283)
//*********************************************************************************************
signed int ds_conv_to_temperature(uint8_t *buff)
{
  signed char integer = 0;
  signed char frac;
  signed int  result; 
 
  frac    = buff[0] & 0x0f;                            // fractional part of temperature
  integer = (buff[0]>>4) | ((buff[1] & 0x0f)<<4);      // integer part of temperature

  // integer part
  if (integer < 0) // if temperature is negative
  {
    integer = 0 - integer - 1;
    result  = integer *10;
    frac = frac | 0xf0;
    frac = 0 - frac ;
  }
  else 
    result  = integer *10; // if positive

  // fractional part
  result = result + ((frac*10)/16);

  return result;
}
