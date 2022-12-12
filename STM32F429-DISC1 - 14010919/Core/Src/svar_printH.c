#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

uint8_t signH;
int32_t c11H ;
uint16_t Xpos1H,Ypos1H;
uint8_t n51H,n41H,n31H,n21H,n11H,n01H ;
uint8_t space2;

void svar_printH(int32_t c11H , uint8_t space2, uint16_t Xpos1H, uint16_t Ypos1H)
{
 if ( c11H < 0 ) {
		signH = 1;
		c11H = 4294967296 - c11H;
		BSP_LCD_DisplayCharH(Xpos1H+0 ,Ypos1H, 45 );
	}
 else {
		signH = 0;
		BSP_LCD_DisplayCharH(Xpos1H+0 ,Ypos1H, 43 );
	}
 n51H =  c11H / 100000 + 48;
 c11H %= 100000;
 n41H =  c11H / 10000 + 48;
 c11H %= 10000;
 n31H =  c11H / 1000 + 48;
 c11H %= 1000;
 n21H =  c11H / 100 + 48;
 c11H %= 100;
 n11H =  c11H / 10 + 48;
 c11H %= 10;
 n01H = c11H + 48;
 BSP_LCD_DisplayCharH(Xpos1H,Ypos1H-space2, n51H );
 BSP_LCD_DisplayCharH(Xpos1H,Ypos1H-2*space2, n41H );
 BSP_LCD_DisplayCharH(Xpos1H,Ypos1H-3*space2, n31H );
 BSP_LCD_DisplayCharH(Xpos1H,Ypos1H-4*space2, n21H );
 BSP_LCD_DisplayCharH(Xpos1H,Ypos1H-5*space2, n11H );
 BSP_LCD_DisplayCharH(Xpos1H,Ypos1H-6*space2, n01H );
}
