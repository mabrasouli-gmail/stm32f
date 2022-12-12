/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd1.h"
#include "stm32f429i_discovery_ts.h"
#include "color2.h"
#include "save.h"
#include "stlogo.h"

//#include "arm_math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	void LED_Toggle(void);
#define LCD_FRAME_BUFFER_LAYER0                  (LCD_FRAME_BUFFER+0x130000)
#define LCD_FRAME_BUFFER_LAYER1                  LCD_FRAME_BUFFER
#define CONVERTED_FRAME_BUFFER                   (LCD_FRAME_BUFFER+0x260000)
void Touchscreen_Calibration (void);
void var_print(uint16_t,uint16_t,uint16_t);
void svar_print(int32_t,uint16_t,uint16_t);
void var_printH(uint16_t,uint8_t,uint8_t,uint16_t,uint16_t);
void var8_printH(uint8_t,uint8_t,uint16_t,uint16_t);
void svar_printH(int32_t,uint8_t,uint16_t,uint16_t);
void txt_printH(uint8_t *, uint8_t, uint8_t, uint16_t, uint16_t);
void disp( char, uint8_t, uint16_t, uint16_t, char, char, char, char, char, char );
void disp1( char ,char ,char ,char ,char ,char );

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
 struct  stm_packet												//34byte
		{
			uint8_t start;												//start byte	(0x55)
			uint8_t stm_com;											//command : "idel=0","get_status=1","wait for charge=2","charge_OK=3","charge_NOK=4","wait for shock=5","shock_OK=6","shock_NOK=7","...
			uint8_t stm_status;	//uint16_t stm_status;									//stm board internal status
			uint8_t date_year;										// year (binary), offset=2000
			uint8_t date_month;										// month (binary)
			uint8_t date_day;											// day (binary)
			uint8_t time_hour;										// hour (binary)
			uint8_t time_minute;									// minute (binary)
			uint8_t time_second;									// second (binary)
			uint16_t HR;	     										// heart rate (binary)15------0
			uint8_t pad_type;									    //(paddel type :"none =0", "adult = 1", "pediatric =2","neonate1 = 3","neonate2 = 4")4---0
																						//("PADELL_open,PADELL_bad,PADELL_weak,PADELL_shrt" = pad_type bit 7---4
			uint16_t lead_off;										//{( lead off :("PADELL_open,PADELL_weak,PADELL_shrt,RL,RA,V1,V5,V4,V3,V2,LL,LA,V6")11---0       0 = none  ,1 = present
																						// (lead type :("none =0", "3lead = 1", "5lead = 2","12lead= 3")15---12}  ("12LEAD: no PADDEL" , "5lead :no V2,V3,V4,V5,V6" , "3LEAD : no V1,V2,V3,V4,V5,V6")
			uint8_t hv_status;										//HV board internal status
			uint8_t pace_status;									//pace board internal status
			uint8_t pace_time;										//pace priode time done(binary) "30 ----- 180" with 5 interval
			uint8_t pace_curr;										//pace current done(binary) "0 ----- 150" with 5 interval
			uint16_t pad_imp_meas;									//impedance measure value (binary) "0 ----- 65535"
			uint8_t gain21_def;										//default gain ch2,ch1 (2*4bit binary) ( gain=6 --> 1 ),( gain=1 --> 1 ),( gain=2 --> 2 )	
			uint8_t gain43_def;										//default gain ch4,ch3 (2*4bit binary) ( gain=3 --> 3 ),( gain=4 --> 4 ),( gain=8 --> 5 )	
			uint8_t gain65_def;										//default gain ch6,ch5 (2*4bit binary) ( gain=12 --> 6 )
			uint8_t gain87_def;										//default gain ch8,ch7 (2*4bit binary)
			uint8_t lead_status;									// lead status (8bit) lead 8---1 ( active=0 /deactive=1 )
			uint16_t energy_meas;									//energy measure value (binary) 0 ---- 360 jole
			uint8_t bat_meas;											// battery charge value percentage 
			uint8_t reserve;	     								// reserve
			uint32_t crc_stm;											// crc check 32bit for 28byte (from "stm_com" to "bat_meas")
			uint8_t stop;													//stop byte	(0xAA)
		} s1;

//------------------------------
 extern   struct  bit_flag												
		{
			uint8_t PC_DAT_RCV	;		//flag for recieve packet form pc
		} f1 ;

  uint8_t pc_buf[64];         //data recieve buffer 
	uint16_t pc_buf_size;       //number of bytes recieve size
	uint8_t num_packet = 8 ;		//number of bytes in packet for recieve data
	uint8_t stm_buf[30]={0x55,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,254,0xAA}; //sample packet with 30 byte
//------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t retSD; /* Return value for SD */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile1,SDFile2; /* File object for SD */
extern char SDPath[4]; /* SD logical drive path */
		
		
		
static void GetPosition(void);
static void Update_ColorSize(void);
static uint32_t radius = 1; 
 
static char mod_flag = 0;
static char wav_flag = 0;
static char bpm_flag = 0;
static char wid_flag = 0;
static char amp_flag = 0;
static char range_flag = 0;
static char tim_flag = 0;
static char r_flag = 0;
	
static char MODE = 0;										// mode input    ( "0" = ECG-SIM , "1" = DEFITST ,  ECG-SIM default )
static char RANGE = 1;										// range input   ( "0" = 1.5Kv , "1" = 3.0Kv ,  4kv default )
static char TIME = 0;										// time input    ( "0" = 20ms, "1" = 40ms ,  20ms default 
static char AMPL = 1;										// amplitude     ( 5 mv default )
static uint8_t CURVE_TYPE = 1;							// type of curve ( qrs default )
static uint16_t BPM = 60;								// bpm					 ( 75 bpm default )
static uint16_t WIDTH = 100;							// pulse width	 ( 500 ms default )
static float R = 50;										//load resistance
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize the LCD */
  BSP_LCD_Init();
  /* Layer2 Init */
  BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER1);
  /* Set Foreground Layer */
  BSP_LCD_SelectLayer(1);
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);  
  BSP_LCD_SetColorKeying(1, LCD_COLOR_WHITE);
  BSP_LCD_SetLayerVisible(1, DISABLE);
  
  /* Layer1 Init */
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER0);
  
  /* Set Foreground Layer */
  BSP_LCD_SelectLayer(0);
 
  /* Enable The LCD */
  BSP_LCD_DisplayOn();

  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);


  Touchscreen_Calibration();
  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

//-----------------------------------------------------  /* Draw Bitmap */------------------
//BSP_LCD_DrawBitmap(40, 2, (uint8_t *)arm3);
//HAL_Delay(3000);
//-----------------------------------------------------  /* Draw Bitmap */------------------

BSP_LCD_SelectLayer(0);//select on which layer we write
BSP_LCD_DisplayOn();//turn on LCD
BSP_LCD_SetFont(&Font24);
BSP_LCD_Clear(LCD_COLOR_BLUE);//clear the LCD on blue color
BSP_LCD_SetBackColor(LCD_COLOR_BLUE);//set text background color

BSP_LCD_DisplayStringAtLineH(0," TPD Demo tester");

disp ( MODE , CURVE_TYPE , BPM , WIDTH , AMPL , mod_flag , wav_flag , bpm_flag , wid_flag , amp_flag );
BSP_LCD_SetTextColor(LCD_COLOR_WHITE);//set text color 
BSP_LCD_SetBackColor(LCD_COLOR_RED);//set text background color
txt_printH("SET",3,11,200,35);
//---------------------------------------------------------------------------------------------------
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)	{								//mount disk
		Error_Handler();
	}
	make_folder();																													//making folder & directory
	w_file_name();																													//create file name from date
	
	if (f_open(&SDFile1, NFILE, FA_CREATE_ALWAYS | FA_WRITE ) != FR_OK) {		//open file
		Error_Handler();
	}
	
	if (f_lseek(&SDFile1, f_size(&SDFile1))!= FR_OK)	{											//seek addres file
		Error_Handler();
	}
	f1.file_new_create = 0;
	f1.STM_DAT_STP = 0;
//---------------------------------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if ( f1.PC_DAT_RCV == 1 )  {
			CDC_Transmit_HS(pc_buf,pc_buf_size);
			f1.PC_DAT_RCV = 0;	
		}
		LED_Toggle();
		HAL_RTC_GetTime(&hrtc, &currentTime, FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &currentDate, FORMAT_BIN);
	  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);

		s1.date_year = currentDate.Year ;	
		s1.date_month = currentDate.Month ;
	  s1.date_day = currentDate.Date ;
	  s1.time_hour = currentTime.Hours ;
		s1.time_minute = currentTime.Minutes ;
	  s1.time_second = currentTime.Seconds ;				
    var_printH(s1.date_year,2,11,100,220);
    var_printH(s1.date_month,2,11,100,198);
    var_printH(s1.date_day,2,11,100,176);
    var_printH(s1.time_hour,2,11,100,154);
    var_printH(s1.time_minute,2,11,100,132);
    var_printH(s1.time_second,2,11,100,110);
		
	if	( HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ) == 1 	)	{		//Blue push botton press
				sTime.Hours = 19;
				sTime.Minutes = 5;
				sTime.Seconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_SET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)	{
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
				sDate.Month = 5;	
				sDate.Date = 29;
				sDate.Year = 22;
				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
			}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LED_Toggle(void)
{
	  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_Delay(500);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
