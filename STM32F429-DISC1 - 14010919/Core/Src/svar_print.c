#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

uint8_t sign;
int32_t c11 ;
uint16_t Xpos1,Ypos1;
uint8_t n51,n41,n31,n21,n11,n01 ;

void svar_print(int32_t c11 , uint16_t Xpos1, uint16_t Ypos1)
{
 if ( c11 < 0 ) {
		sign = 1;
		c11 = 4294967296 - c11;
		BSP_LCD_DisplayChar(Xpos1+0 ,Ypos1, 45 );
	}
 else {
		sign = 0;
		BSP_LCD_DisplayChar(Xpos1+0 ,Ypos1, 43 );
	}
 n51 =  c11 / 100000 + 48;
 c11 %= 100000;
 n41 =  c11 / 10000 + 48;
 c11 %= 10000;
 n31 =  c11 / 1000 + 48;
 c11 %= 1000;
 n21 =  c11 / 100 + 48;
 c11 %= 100;
 n11 =  c11 / 10 + 48;
 c11 %= 10;
 n01 = c11 + 48;
 BSP_LCD_DisplayChar(Xpos1+13,Ypos1, n51 );
 BSP_LCD_DisplayChar(Xpos1+26,Ypos1, n41 );
 BSP_LCD_DisplayChar(Xpos1+39,Ypos1, n31 );
 BSP_LCD_DisplayChar(Xpos1+52,Ypos1, n21 );
 BSP_LCD_DisplayChar(Xpos1+65,Ypos1, n11 );
 BSP_LCD_DisplayChar(Xpos1+78,Ypos1, n01 );
}
