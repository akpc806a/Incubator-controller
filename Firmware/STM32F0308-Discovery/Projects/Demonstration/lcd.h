
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void LCD_Setup(void);
void LCD_ClearLine(char Line);
void LCD_Clear(void);
void LCD_SetCursor(char Xpos, int Ypos);
void LCD_DisplayChar(char Line, int Column, char Ascii);
void LCD_DisplayStringLine(char Line, char *ptr);

/*----- Low layer function -----*/
void LCD_CtrlLinesConfig(void);
void LCD_SendCmd(unsigned char var);
void LCD_SendData(unsigned char var);

void LCD_HAL_Write(unsigned char Data);
void LCD_HAL_RS(unsigned char Data);
void LCD_HAL_EN(unsigned char Data);


#endif /* __LCD_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
