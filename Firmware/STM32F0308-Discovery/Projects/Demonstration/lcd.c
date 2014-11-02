/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : stm32f10x_lcd.c
* Author             : MCD Application Team & IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : This file includes the LCD driver for AM-240320LTNQW00H 
*                      (LCD_HX8312) and AM-240320L8TNQW00H (LCD_ILI9320) 
*                      Liquid Crystal Display Module of STM3210B-EVAL board.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"

#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  /* Global variables to set the written text color */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LCD_Setup
* Description    : Setups the LCD.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_Setup(void)
{ 
/* Configure the LCD Control pins --------------------------------------------*/
  LCD_CtrlLinesConfig();

  LCD_HAL_Write(0x30);
  LCD_HAL_RS(0);        //Selected command register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);
  LCD_HAL_Write(0x30);
  LCD_HAL_RS(0);        //Selected command register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);
  LCD_HAL_Write(0x30);
  LCD_HAL_RS(0);        //Selected command register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);
  LCD_HAL_Write(0x20);
  LCD_HAL_RS(0);        //Selected command register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);


  LCD_SendCmd(0x28);     //Function set: 2 Line, 8-bit, 5x8 dots
  LCD_SendCmd(0x0C);     //Display on, Curson blinking command
  LCD_SendCmd(0x01);     //Clear LCD
  LCD_SendCmd(0x06);     //Entry mode, auto increment with no shift 
}

/*******************************************************************************
* Function Name  : LCD_ClearLine
* Description    : Clears the selected line.
* Input          : - Line: the Line to be cleared.
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_ClearLine(char Line)
{
  LCD_DisplayStringLine(Line, "                    ");
}

/*******************************************************************************
* Function Name  : LCD_Clear
* Description    : Clears the hole LCD.
* Input          : Color: the color of the background.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_Clear(void)
{
  LCD_SendCmd(0x01);     //Clear LCD
}

/*******************************************************************************
* Function Name  : LCD_SetCursor
* Description    : Sets the cursor position.
* Input          : - Xpos: specifies the X position.
*                  - Ypos: specifies the Y position. 
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_SetCursor(char Xpos, int Ypos)
{
  unsigned char bShift = 0;
  if (Xpos == 0) bShift = 0;
  if (Xpos == 1) bShift = 0x40;
  if (Xpos == 2) bShift = 0x14;
  if (Xpos == 3) bShift = 0x54;

  LCD_SendCmd(0x80 | (bShift+Ypos));
  //x--; y--;
  //LCD_SendCmd((0x80+Xpos)|(Ypos<<6)); -- for 2 line interface
}

/*******************************************************************************
* Function Name  : LCD_DisplayChar
* Description    : Displays one character (16dots width, 24dots height).
* Input          : - Line: the Line where to display the character shape .
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - Column: start column address.
*                  - Ascii: character ascii code, must be between 0x20 and 0x7E.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayChar(char Line, int Column, char Ascii)
{
  LCD_SetCursor(Line, Column);

  LCD_SendData(Ascii);
}

/*******************************************************************************
* Function Name  : LCD_DisplayStringLine
* Description    : Displays a maximum of 20 char on the LCD.
* Input          : - Line: the Line where to display the character shape .
*                    This parameter can be one of the following values:
*                       - Linex: where x can be 0..9
*                  - *ptr: pointer to string to display on LCD.
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_DisplayStringLine(char Line, char *ptr)
{
  LCD_SetCursor(Line, 0);

  while(*ptr)              //till string ends
    LCD_SendData(*ptr++);  //send characters one by one
}

/*******************************************************************************
* Function Name  : LCD_CtrlLinesConfig
* Description    : Configures LCD control lines in Output Push-Pull mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
  #define LCD_RS_GPIO_PORT          GPIOA
  #define LCD_RS_GPIO_CLK           RCC_AHBPeriph_GPIOA   
  #define LCD_RS_GPIO_PIN           GPIO_Pin_0

  #define LCD_E_GPIO_PORT           GPIOA
  #define LCD_E_GPIO_CLK            RCC_AHBPeriph_GPIOA    
  #define LCD_E_GPIO_PIN            GPIO_Pin_1

  #define LCD_D4_GPIO_PORT          GPIOA
  #define LCD_D4_GPIO_CLK           RCC_AHBPeriph_GPIOA    
  #define LCD_D4_GPIO_PIN           GPIO_Pin_2

  #define LCD_D5_GPIO_PORT          GPIOA
  #define LCD_D5_GPIO_CLK           RCC_AHBPeriph_GPIOA    
  #define LCD_D5_GPIO_PIN           GPIO_Pin_3

  #define LCD_D6_GPIO_PORT          GPIOA
  #define LCD_D6_GPIO_CLK           RCC_AHBPeriph_GPIOA   
  #define LCD_D6_GPIO_PIN           GPIO_Pin_4

  #define LCD_D7_GPIO_PORT          GPIOA
  #define LCD_D7_GPIO_CLK           RCC_AHBPeriph_GPIOA  
  #define LCD_D7_GPIO_PIN           GPIO_Pin_5

void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  RCC_AHBPeriphClockCmd(LCD_RS_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LCD_RS_GPIO_PIN;
  GPIO_Init(LCD_RS_GPIO_PORT, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(LCD_E_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LCD_E_GPIO_PIN;
  GPIO_Init(LCD_E_GPIO_PORT, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(LCD_D4_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LCD_D4_GPIO_PIN;
  GPIO_Init(LCD_D4_GPIO_PORT, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(LCD_D5_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LCD_D5_GPIO_PIN;
  GPIO_Init(LCD_D5_GPIO_PORT, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(LCD_D6_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LCD_D6_GPIO_PIN;
  GPIO_Init(LCD_D6_GPIO_PORT, &GPIO_InitStructure);

  RCC_AHBPeriphClockCmd(LCD_D7_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LCD_D7_GPIO_PIN;
  GPIO_Init(LCD_D7_GPIO_PORT, &GPIO_InitStructure);
/*
GPIO_SetBits(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN);
GPIO_ResetBits(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN);
GPIO_SetBits(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN);
GPIO_ResetBits(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN);
GPIO_SetBits(LCD_D4_GPIO_PORT, LCD_D4_GPIO_PIN);
GPIO_ResetBits(LCD_D4_GPIO_PORT, LCD_D4_GPIO_PIN);
GPIO_SetBits(LCD_D5_GPIO_PORT, LCD_D5_GPIO_PIN);
GPIO_ResetBits(LCD_D5_GPIO_PORT, LCD_D5_GPIO_PIN);
GPIO_SetBits(LCD_D6_GPIO_PORT, LCD_D6_GPIO_PIN);
GPIO_ResetBits(LCD_D6_GPIO_PORT, LCD_D6_GPIO_PIN);
GPIO_SetBits(LCD_D7_GPIO_PORT, LCD_D7_GPIO_PIN);
GPIO_ResetBits(LCD_D7_GPIO_PORT, LCD_D7_GPIO_PIN);
*/
}


void LCD_delay()
{
  int j;
  for(j=0;j<1005;j++);
}

void LCD_HAL_Write(unsigned char Data)
{
  int iData;
  if (Data & 0x80) GPIO_WriteBit(LCD_D7_GPIO_PORT, LCD_D7_GPIO_PIN, Bit_SET); else GPIO_WriteBit(LCD_D7_GPIO_PORT, LCD_D7_GPIO_PIN, Bit_RESET);
  if (Data & 0x40) GPIO_WriteBit(LCD_D6_GPIO_PORT, LCD_D6_GPIO_PIN, Bit_SET); else GPIO_WriteBit(LCD_D6_GPIO_PORT, LCD_D6_GPIO_PIN, Bit_RESET);
  if (Data & 0x20) GPIO_WriteBit(LCD_D5_GPIO_PORT, LCD_D5_GPIO_PIN, Bit_SET); else GPIO_WriteBit(LCD_D5_GPIO_PORT, LCD_D5_GPIO_PIN, Bit_RESET);
  if (Data & 0x10) GPIO_WriteBit(LCD_D4_GPIO_PORT, LCD_D4_GPIO_PIN, Bit_SET); else GPIO_WriteBit(LCD_D4_GPIO_PORT, LCD_D4_GPIO_PIN, Bit_RESET);

LCD_delay();
}


void LCD_HAL_RS(unsigned char Data)
{  
  //if (Data) { LCD_RS_GPIO_PORT->BSRRL = LCD_RS_GPIO_PIN; } else { LCD_RS_GPIO_PORT->BSRRH = LCD_RS_GPIO_PIN; };
  if (Data) 
    GPIO_WriteBit(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, Bit_SET);
  else 
    GPIO_WriteBit(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, Bit_RESET);

LCD_delay();
}

void LCD_HAL_EN(unsigned char Data)
{
  if (Data)
    GPIO_WriteBit(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, Bit_SET);
  else 
    GPIO_WriteBit(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, Bit_RESET);
  
LCD_delay();
}


void LCD_busy()
{
  unsigned char i,j;
  for(i=0;i<50;i++)
    for(j=0;j<255;j++);
}

void LCD_SendCmd(unsigned char var)
{
  LCD_HAL_Write(var);      //Function set: 2 Line, 8-bit, 5x8 dots
  LCD_HAL_RS(0);        //Selected command register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);

  LCD_HAL_Write(var << 4);      //Function set: 2 Line, 8-bit, 5x8 dots
  LCD_HAL_RS(0);        //Selected command register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);

  LCD_busy();          //Wait for LCD to process the command
}

void LCD_SendData(unsigned char var)
{
  LCD_HAL_Write(var);      //Function set: 2 Line, 8-bit, 5x7 dots
  LCD_HAL_RS(1);        //Selected data register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);

  LCD_HAL_Write(var << 4);      //Function set: 2 Line, 8-bit, 5x7 dots
  LCD_HAL_RS(1);        //Selected data register
  LCD_HAL_EN(1);        //Enable H->L
  LCD_HAL_EN(0);

  LCD_busy();          //Wait for LCD to process the command
}  



/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
