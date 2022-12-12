#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

int c1 ;
uint16_t Xpos,Ypos;
uint8_t n5,n4,n3,n2,n1,n0 ;

void var_print(int c1 , uint16_t Xpos, uint16_t Ypos)
{

 n5 =  c1 / 100000 + 48;
 c1 %= 100000;
 n4 =  c1 / 10000 + 48;
 c1 %= 10000;
 n3 =  c1 / 1000 + 48;
 c1 %= 1000;
 n2 =  c1 / 100 + 48;
 c1 %= 100;
 n1 =  c1 / 10 + 48;
 c1 %= 10;
 n0 = c1 + 48;
 //BSP_LCD_DisplayChar(Xpos+0 ,Ypos, n5 );
 //BSP_LCD_DisplayChar(Xpos+13,Ypos, n4 );
 BSP_LCD_DisplayChar(Xpos+26,Ypos, n3 );
 BSP_LCD_DisplayChar(Xpos+39,Ypos, n2 );
 BSP_LCD_DisplayChar(Xpos+52,Ypos, n1 );
 BSP_LCD_DisplayChar(Xpos+65,Ypos, n0 );
 }
