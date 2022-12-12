#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

char *txt ;
uint16_t XposHH,YposHH;
uint8_t num,space,t;

void txt_printH( uint8_t *txt, uint8_t num, uint8_t space, uint16_t XposHH, uint16_t YposHH)
{
	for (t=0;t<num;t++){
		BSP_LCD_DisplayCharH(XposHH ,YposHH - t*space ,*txt );
		txt++;
	}
}
