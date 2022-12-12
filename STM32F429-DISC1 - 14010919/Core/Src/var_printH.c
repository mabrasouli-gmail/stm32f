#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

int c1H ;
uint16_t XposH,YposH;
uint8_t n5H,n4H,n3H,n2H,n1H,n0H ;
uint8_t space1;
uint8_t digit;

void var_printH(int c1H , uint8_t digit, uint8_t space1, uint16_t XposH, uint16_t YposH)
{
if ( digit == 1 )	{
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n0H );
 }
else if ( digit == 2 )	{
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n0H );
 }
else if ( digit == 3 )	{
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n0H );
 }
else if ( digit == 30 )	{
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, 46 );  //fraction
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, n0H );
 }
else if ( digit == 31 )	{
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, 46 );  //fraction
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, n0H );
 }
else if ( digit == 4 )	{
 n3H =  c1H / 1000 + 48;
 c1H %= 1000;
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n3H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, n0H );
 }
else if ( digit == 40 )	{
 n3H =  c1H / 1000 + 48;
 c1H %= 1000;
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n3H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, 46 );  //fraction
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-4*space1, n0H );
 }
else if ( digit == 41 )	{
 n3H =  c1H / 1000 + 48;
 c1H %= 1000;
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n3H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, 46 );  //fraction
 BSP_LCD_DisplayCharH(XposH,YposH-4*space1, n0H );
 }
else if ( digit == 5 )	{
 n4H =  c1H / 10000 + 48;
 c1H %= 10000;
 n3H =  c1H / 1000 + 48;
 c1H %= 1000;
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n4H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n3H );
//BSP_LCD_DisplayCharH(XposH,YposH-2*space1, 46 );  //fraction
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-4*space1, n0H );
 }
else {
 n5H =  c1H / 100000 + 48;
 c1H %= 100000;
 n4H =  c1H / 10000 + 48;
 c1H %= 10000;
 n3H =  c1H / 1000 + 48;
 c1H %= 1000;
 n2H =  c1H / 100 + 48;
 c1H %= 100;
 n1H =  c1H / 10 + 48;
 c1H %= 10;
 n0H = c1H + 48;
 BSP_LCD_DisplayCharH(XposH,YposH-0, n5H );
 BSP_LCD_DisplayCharH(XposH,YposH-space1, n4H );
 BSP_LCD_DisplayCharH(XposH,YposH-2*space1, n3H );
 BSP_LCD_DisplayCharH(XposH,YposH-3*space1, n2H );
 BSP_LCD_DisplayCharH(XposH,YposH-4*space1, n1H );
 BSP_LCD_DisplayCharH(XposH,YposH-5*space1, n0H );
 }
}