/**
  ******************************************************************************
  * @file    Display/LTDC_Paint/Src/ts_calibration.c 
  * @author  MCD Application Team
  * @version V1.3.1
  * @date    09-October-2015
  * @brief   This application code shows how to calibrate the touchscreen.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @addtogroup STM32F4xx_HAL_Applications
  * @{
  */

/** @addtogroup LTDC_Paint
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t Calibration_Done = 0;
static int16_t  A1, A2, B1, B2;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Performs the TS calibration
  * @param  None
  * @retval None
  */
void Touchscreen_Calibration(void)
{ 

      A1 = 981;//(1000 * ( aLogX[1] - aLogX[0]))/ ( aPhysX[1] - aPhysX[0]); 
      B1 = 3228;//(1000 * aLogX[0]) - A1 * aPhysX[0]; 
      
      A2 = 983;//(1000 * ( aLogY[1] - aLogY[0]))/ ( aPhysY[1] - aPhysY[0]); 
      B2 = 255;//(1000 * aLogY[0]) - A2 * aPhysY[0]; 
      
      Calibration_Done = 1;
      return;
    }
   

/**
  * @brief  Calibrate X position
  * @param  x: X position
  * @retval calibrated x
  */
uint16_t Calibration_GetX(uint16_t x)
{
  return (((A1 * x) + B1)/1000);
}

/**
  * @brief  Calibrate Y position
  * @param  y: Y position
  * @retval calibrated y
  */
uint16_t Calibration_GetY(uint16_t y)
{
  return (((A2 * y) + B2)/1000);
}

/**
  * @brief  Check if the TS is calibrated
  * @param  None
  * @retval calibration state
  */
uint8_t IsCalibrationDone(void)
{
  return (Calibration_Done);
}

/**
  * @}
  */ 
  
/**
  * @}
  */
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
