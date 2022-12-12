#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

uint8_t c18H ;
uint16_t Xpos8H,Ypos8H;
uint8_t n28H,n18H,n08H ;
uint8_t space18;

void var8_printH(uint8_t c18H , uint8_t space18, uint16_t Xpos8H, uint16_t Ypos8H)
{
 n28H =  c18H / 100 + 48;
 c18H %= 100;
 n18H =  c18H / 10 + 48;
 c18H %= 10;
 n08H = c18H + 48;
 BSP_LCD_DisplayCharH(Xpos8H,Ypos8H-0, n28H );
 BSP_LCD_DisplayCharH(Xpos8H,Ypos8H-space18, n18H );
 BSP_LCD_DisplayCharH(Xpos8H,Ypos8H-2*space18, n08H );


}