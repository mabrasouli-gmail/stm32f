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
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	rd_step1 2100;
#define	rd_step2 256;
#define	rd_step3 4200;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//ADS-----------------------------
	uint8_t tx_buff1[30] ;
	uint8_t tx_buff2[28] ;//= {0,1,2,3,4,5,6,7};
	uint8_t rx_buff[28];
	
	uint8_t rcv_buff[28];
	uint16_t sample_counter;        //sample counter
	uint16_t n_sample = 800;        //number of sample
	uint32_t in_signal[800];			  // input signal vector
//ADS-----------------------------
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
 struct  stm_packet												//34byte
		{
			uint8_t start;												//start byte	(0x55)
			uint8_t stm_com;											//command : "idel=0","get_status=1","wait for charge=2","charge_OK=3","charge_NOK=4","wait for shock=5","shock_OK=6","shock_NOK=7","...
			uint8_t stm_status;	//uint16_t stm_status;									//stm board internal status
			uint16_t date_year;										// year (binary), offset=2000
			uint8_t date_month;										// month (binary)
			uint8_t date_day;											// day (binary)
			uint8_t time_hour;										// hour (binary)
			uint8_t time_minute;									// minute (binary)
			uint8_t time_second;									// second (binary)
			uint8_t stop;													//stop byte	(0xAA)
		} s1;
		
struct  PcPacket{
    uint8_t     start;			//start byte
		uint8_t     CommandType;     //command : "charge = 1","shock = 2 ","sync_shock = 3 ",...
    uint8_t     status;         //PC internal status
    uint8_t     year;           // year (binary), offset=2000
    uint8_t     month;          // month (binary)
    uint8_t     day;            // day (binary)
    uint8_t     hour;           // hour (binary)
    uint8_t     minute;         // minute (binary)
    uint8_t     second;         // second (binary)
    uint8_t     stop;			//stop byte
}pc,rcv;				//228byte

 extern   struct  bit_flag												
		{
			uint8_t file_end;
			uint8_t file_new_create;
			uint8_t PC_DAT_RCV	;		//flag for recieve packet form pc
			uint8_t STM_DAT_STP;
			uint8_t error_handler;
			uint8_t history ;
			uint8_t download ;
			uint8_t settime ;
			uint8_t format ;
			unsigned sampling_state :1;
			unsigned data_rcv_flg	  :1;		
			unsigned data_dump_flg  :1;
			unsigned data_set_flg   :1;
			unsigned spi4_strt_flg	:1;																				// priode timer flag for transmit spi4
			unsigned spi4_end_flg 	:1;																				// priode timer flag for transmit spi4
			unsigned tim10_flg 			:1;
			unsigned tim11_flg 			:1;
		} f1 ;

  uint8_t pc_buf[228];         //data recieve buffer 
	uint16_t pc_buf_size;        //number of bytes recieve size
	uint8_t num_packet = 10 ;	  //number of bytes in packet for recieve data
	uint8_t stm_buf[30]={0x55,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,254,0xAA}; //sample packet with 30 byte
	uint8_t stm_buf_DSP[32];
  static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */

extern uint8_t SET_DATA[4];          //setting data
		
		
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Touchscreen_Calibration (void);
void var_print(uint16_t,uint16_t,uint16_t);
void svar_print(int32_t,uint16_t,uint16_t);
void var_printH(uint16_t,uint8_t,uint8_t,uint16_t,uint16_t);
void var8_printH(uint8_t,uint8_t,uint16_t,uint16_t);
void svar_printH(int32_t,uint8_t,uint16_t,uint16_t);
void txt_printH(uint8_t *, uint8_t, uint8_t, uint16_t, uint16_t);
void disp( char, uint8_t, uint16_t, uint16_t, char, char, char, char, char, char );
void disp1( char ,char ,char ,char ,char ,char );
		
void rd_disk(void);
void rd_file (void);
FRESULT scan_files ( char* path );	
void make_folder(void);		
void w_file_name (void);
void r_file_name (void);
void cpypc_pcbuf (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t retSD; /* Return value for SD */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile1,SDFile2; /* File object for SD */
extern char SDPath[4]; /* SD logical drive path */
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t rtext[8400];                                  /* Read File buffer */
	uint32_t icnt ;
	uint32_t diff_dat ;
	uint32_t file_size ;
	uint16_t rd_step = rd_step1;
	char NFILE[28];   		  // file name
	char SDpat[15];					// temp path
	char str_table[20] = {0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x30,0x31,0x32,0x33,0x34,0x35, 0x2E,0x42,0x49,0x4E}; // " . B I N "
  char SDpat_r[15];   		// input path buffer
	char file[8];				    // temp buffer for path files array
	char file_arr[750][8];	// path files array
	char rfile[28];					//file name for read from file
	uint16_t f_cnt = 0;			// scan found files counter
//==================================================================================		
		
		
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
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
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
  	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET );     //led G off
  	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET );     //led R off
		f1.file_end = 0;
	  f1.error_handler = 0;
//mSD------------------------------------------------------------------------------------------------
	HAL_RTC_GetTime(&hrtc, &currentTime, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, FORMAT_BIN);
	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)	{								//mount disk
		Error_Handler();
	}
//	BSP_LED_On(LED3);
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
//mSD------------------------------------------------------------------------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
//ADS----------------------------------------------------------------------------	
		if ( f1.sampling_state == 1 )	{	
			if	( sample_counter != n_sample )	{
				if ( f1.data_rcv_flg == 1 )  {
///////// READ DATA (9 byte)				
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET );					//CS_P	 			active
					HAL_SPI_TransmitReceive_DMA(&hspi4,tx_buff1,rcv_buff,9);				//27 byte (216 bit)	RECIEVE
					in_signal[sample_counter] = (rcv_buff[3]<<24)+(rcv_buff[4]<<16)+(rcv_buff[5]<<8);
					f1.data_rcv_flg = 0;
///////// READ DATA (9 byte)	
					sample_counter++;
				}
			}
			else	{
				sample_counter = 0;
				f1.sampling_state = 0;
			}
		}
//ADS----------------------------------------------------------------------------		
//mSD----------------------------------------------------------------------------new file create		
	if ( (currentTime.Minutes == 0) && (currentTime.Seconds <= 10) && (f1.file_new_create == 0) )	{
		if (f_close(&SDFile1) != FR_OK )	{
			Error_Handler();
		}
		make_folder();
		w_file_name();
		
		if (f_open(&SDFile1, NFILE, FA_CREATE_ALWAYS | FA_WRITE ) != FR_OK) {
			Error_Handler();
		}
		
		if (f_lseek(&SDFile1, f_size(&SDFile1))!= FR_OK)	{
			Error_Handler();
		}
		f1.file_new_create = 1;
	}else	if ( (currentTime.Minutes == 0) && (currentTime.Seconds > 10) ) {
		f1.file_new_create = 0;
	}
//mSD----------------------------------------------------------------------------
//DSP---------------------------------------------------		
										if ( f1.data_dump_flg == 0 ) {
												CDC_Transmit_HS(stm_buf_DSP,6);
										}
										else	{}
//mSD write------------------------------------------------------------------										
		res = f_write(&SDFile1, stm_buf_DSP, 6 , (void *)&byteswritten);
		if((byteswritten == 0) || (res != FR_OK)) {
			Error_Handler();
		}
		res = f_sync(&SDFile1);
		if( res != FR_OK )	{
			Error_Handler();
		}
//mSD write------------------------------------------------------------------										
			
//DSP---------------------------------------------------		
		
//RTC------------------------------------------------------------------------------------------------
	if	( HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ) == 1 	)	{		//Blue push botton press
				sTime.Hours = 17;
				sTime.Minutes = 17;
				sTime.Seconds = 0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_SET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)	{
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
				sDate.Month = 12;	
				sDate.Date = 9;
				sDate.Year = 22;
				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
			}
//RTC------------------------------------------------------------------------------------------------
			
//PC Request-----------------------------------------------------------------------------------------
	if (f1.PC_DAT_RCV == 1)		{
		
		if ( pc_buf[0] == 0x55 && pc_buf[9] == 0xAA )	{		
			cpypc_pcbuf();																			// load "pc_buff to "pc structure"	
				
//---------------------------------
			if (pc.CommandType == 0x01)	{																						//setting signal
				f1.download = 1;
			}

//---------------------------------
			if (pc.CommandType == 0x02)	{																						//set time
				f1.settime = 1;
				HAL_TIM_Base_Stop_IT(&htim10);																		//timer stop blink G led
				htim10.Init.Period = 3000;
				HAL_TIM_Base_Init(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);																		//timer start blink G led
				sTime.Hours = pc.hour;
				sTime.Minutes = pc.minute;
				sTime.Seconds = pc.second;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_SET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)	{
					Error_Handler();
				}
				sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
				sDate.Month = pc.month;	
				sDate.Date = pc.day;
				sDate.Year = pc.year;
				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
					Error_Handler();
				}
				HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
				HAL_TIM_Base_Stop_IT(&htim10);																		//timer stop blink G led
				htim10.Init.Period = 9999;
				HAL_TIM_Base_Init(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);																		//timer start blink G led
				f1.settime = 0;
			}
			
//---------------------------------
			if (pc.CommandType == 0x03)	{																						//format SD DISK
				f1.format = 1;
				htim10.Init.Period = 5000;
				HAL_TIM_Base_Init(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);																		//timer start blink G led
				if(f_mkfs((TCHAR const*)SDPath, 0, 0, buffer, sizeof(buffer)) != FR_OK)	{		/* FatFs Format  */
						Error_Handler();		
				}
				HAL_Delay(10000);																			//10s delay
				if(FATFS_LinkDriver(&SD_Driver, SDPath) != FR_OK)	{  /*Register the file system object to the FatFs module*/
					Error_Handler();
				}
				HAL_Delay(1000);																			//1s delay
				if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)	{		/*mount disk*/
					Error_Handler();
				}
				
				make_folder();																							/*making folder & directory*/
				w_file_name();																							/*create file name from date*/
	
				if (f_open(&SDFile1, NFILE, FA_CREATE_ALWAYS | FA_WRITE ) != FR_OK) {		/*open file*/
					Error_Handler();
				}
	
				if (f_lseek(&SDFile1, f_size(&SDFile1))!= FR_OK)	{					/*seek addres file*/
					Error_Handler();
				}
				htim10.Init.Period = 9999;
				HAL_TIM_Base_Init(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);																		//timer start blink G led
				f1.file_new_create = 0;
				f1.format = 0;
			}
//---------------------------------
			if (pc.CommandType == 0x04)	{																						//history reguest
				f1.history = 1;
				htim10.Init.Period = 1000;
				HAL_TIM_Base_Init(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);																		//timer start blink N led

				SDpat_r[0] = 0x2F;        											 //  "/"
				if ( pc.year <= currentDate.Year ) {
					SDpat_r[1] = 0x32;             //ascii "2"
					SDpat_r[2] = 0x30;             //ascii "0"
					SDpat_r[3] = str_table[(pc.year /10)];
					SDpat_r[4] = str_table[(pc.year %10)];
					if ( pc.month <= 12 && pc.month != 0 )	{
						SDpat_r[5] = 0x2F;        										//  "/"
						SDpat_r[6] = str_table[( pc.month /10 )];
						SDpat_r[7] = str_table[( pc.month %10 )];
						if ( pc.day <= 31 && pc.day != 0 )	{
							SDpat_r[8] = 0x2F;        									//  "/"
							SDpat_r[9] = str_table[( pc.day /10 )];
							SDpat_r[10] = str_table[( pc.day %10 )];
							if ( pc.hour <= 23 ) {
								SDpat_r[11] = 0x2F;       							  //  "/"
								SDpat_r[12] = str_table[( pc.hour /10 )];
								SDpat_r[13] = str_table[( pc.hour %10 )];
								SDpat_r[14] = 0x00;       							  //  "0"
							}else	{
								SDpat_r[11] = 0x00;        //  "0"
								SDpat_r[12] = 0x00;        //  "0"
								SDpat_r[13] = 0x00;        //  "0"
								SDpat_r[14] = 0x00;        //  "0"
							}
						}else	{
							SDpat_r[8] = 0x00;         //  "0"
							SDpat_r[9] = 0x00;         //  "0"
							SDpat_r[10] = 0x00;        //  "0"
							SDpat_r[11] = 0x00;        //  "0"
							SDpat_r[12] = 0x00;        //  "0"
							SDpat_r[13] = 0x00;        //  "0"
							SDpat_r[14] = 0x00;        //  "0"
						}
					}else	{
						SDpat_r[5] = 0x00;         //  "0"
						SDpat_r[6] = 0x00;         //  "0"
						SDpat_r[7] = 0x00;         //  "0"
						SDpat_r[8] = 0x00;         //  "0"
						SDpat_r[9] = 0x00;         //  "0"
						SDpat_r[10] = 0x00;        //  "0"
						SDpat_r[11] = 0x00;        //  "0"
						SDpat_r[12] = 0x00;        //  "0"
						SDpat_r[13] = 0x00;        //  "0"
						SDpat_r[14] = 0x00;        //  "0"
					}
				}else	{
					SDpat_r[1] = 0x00;				 //  "0"
					SDpat_r[2] = 0x00;				 //  "0"
					SDpat_r[3] = 0x00;				 //  "0"
					SDpat_r[4] = 0x00;				 //  "0"
					SDpat_r[5] = 0x00;         //  "0"
					SDpat_r[6] = 0x00;         //  "0"
					SDpat_r[7] = 0x00;         //  "0"
					SDpat_r[8] = 0x00;         //  "0"
					SDpat_r[9] = 0x00;         //  "0"
					SDpat_r[10] = 0x00;        //  "0"
					SDpat_r[11] = 0x00;        //  "0"
					SDpat_r[12] = 0x00;        //  "0"
					SDpat_r[13] = 0x00;        //  "0"
					SDpat_r[14] = 0x00;        //  "0"
				}

				f_cnt = 0;
				res = scan_files(SDpat_r);
			
				for (int fi_cnt =0 ; fi_cnt< f_cnt ; fi_cnt++)	{
//						strcpy ( file , file_arr[fi_cnt]);
					file[0]= file_arr[fi_cnt][0];file[1]= file_arr[fi_cnt][1];file[2]= file_arr[fi_cnt][2];file[3]= file_arr[fi_cnt][3];
					file[4]= file_arr[fi_cnt][4];file[5]= file_arr[fi_cnt][5];file[6]= file_arr[fi_cnt][6];file[7]= file_arr[fi_cnt][7];
					r_file_name();						
					rd_file ();
				}
				htim10.Init.Period = 9999;
				HAL_TIM_Base_Init(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);																		//timer start blink G led
				f1.file_end = 1;	                   //read file end
				f1.history = 0;
			}				
			if (pc.CommandType == 0x05)	{																						//watchdog test
				f1.error_handler = 1;
			}
//---------------------------------
			if (pc.CommandType == 0x06)	{																						//data dump enable
				f1.data_dump_flg = 0;
			}
//---------------------------------
			if (pc.CommandType == 0x07)	{																						//data dump disable
				f1.data_dump_flg = 1;
			}
		}
		f1.PC_DAT_RCV = 0;
	}
//PC Request-----------------------------------------------------------------------------------------
	
//watchdog refresh-----------------------------------------------------------------------------------------
		if ( f1.error_handler == 0 )  {     // no error & fault
		}
//watchdog refresh-----------------------------------------------------------------------------------------
}
  /* USER CODE END 3 */

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
void make_folder(void)
{
	uint16_t ttt;
	SDpat[0] = 0x2F;        //  "/"
	SDpat[1] = str_table[((currentDate.Year + 2000)/1000)];
	ttt = currentDate.Year + 2000;
	ttt %= 1000;
	SDpat[2] = str_table[(ttt /100)];
	ttt %= 100;
	SDpat[3] = str_table[(ttt /10)];
	SDpat[4] = str_table[(ttt %10)];
	SDpat[5] = 0x00;				//	"0"
	res = f_mkdir ((TCHAR const*)SDpat);
	if( res != FR_OK )	{
		if ( res != FR_EXIST )  {
			Error_Handler();
		}
	}
	SDpat[5] = 0x2F;        //  "/"
	SDpat[6] = str_table[( currentDate.Month/10 )];
	SDpat[7] = str_table[( currentDate.Month %10 )];
	SDpat[8] = 0x00;				//	"0"
	res = f_mkdir ((TCHAR const*)SDpat);
	if( res != FR_OK )	{
		if ( res != FR_EXIST )  {
			Error_Handler();
		}
	}
	SDpat[8] = 0x2F;        //  "/"
	SDpat[9] = str_table[( currentDate.Date /10 )];
	SDpat[10] = str_table[( currentDate.Date %10 )];
	SDpat[11] = 0x00;				//	"0"
	res = f_mkdir ((TCHAR const*)SDpat);
	if( res != FR_OK )	{
		if ( res != FR_EXIST )  {
			Error_Handler();
		}
	}
	SDpat[11] = 0x2F;        //  "/"
	SDpat[12] = str_table[( currentTime.Hours /10 )];
	SDpat[13] = str_table[( currentTime.Hours %10 )];
	SDpat[14] = 0x00;				//	"0"
	res = f_mkdir ((TCHAR const*)SDpat);
	if( res != FR_OK )	{
		if ( res != FR_EXIST )  {
			Error_Handler();
		}
	}
}
//-------------------------------------------------------write file name provide
//mSD----------------------------------------------------------------------------
void w_file_name (void)
{
//	uint16_t ttt;
//	SDpat[0] = 0x2F;        //  "/"
//	SDpat[1] = str_table[((calendar.w_year)/1000)];
//	ttt = calendar.w_year;
//	ttt %= 1000;
//	SDpat[2] = str_table[(ttt /100)];
//	ttt %= 100;
//	SDpat[3] = str_table[(ttt /10)];
//	SDpat[4] = str_table[(ttt %10)];
//	SDpat[5] = 0x2F;        //  "/"
//	SDpat[6] = str_table[( calendar.w_month/10 )];
//	SDpat[7] = str_table[( calendar.w_month%10 )];
//	SDpat[8] = 0x2F;        //  "/"
//	SDpat[9] = str_table[( calendar.w_date /10 )];
//	SDpat[10] = str_table[( calendar.w_date %10 )];
//	SDpat[11] = 0x2F;        //  "/"
//	SDpat[12] = str_table[( calendar.hour /10 )];
//	SDpat[13] = str_table[( calendar.hour %10 )];

		NFILE[0] = SDpat[0];			//					FOLDER																																			
		NFILE[1] = SDpat[1];
		NFILE[2] = SDpat[2];
		NFILE[3] = SDpat[3];
		NFILE[4] = SDpat[4];
		NFILE[5] = SDpat[5];
		NFILE[6] = SDpat[6];
		NFILE[7] = SDpat[7];
		NFILE[8] = SDpat[8];
		NFILE[9] = SDpat[9];
		NFILE[10] = SDpat[10];
		NFILE[11] = SDpat[11];
		NFILE[12] = SDpat[12];
		NFILE[13] = SDpat[13];
		
		NFILE[14] = 0x2F;         //  "/"
		
		NFILE[15] = SDpat[3];		  //	year		FILE
		NFILE[16] = SDpat[4];
		
		NFILE[17] = SDpat[6];		  //	month
		NFILE[18] = SDpat[7];

		NFILE[19] = SDpat[9];		  //	date
		NFILE[20] = SDpat[10];

		NFILE[21] = SDpat[12];	  //	hour
		NFILE[22] = SDpat[13];
		NFILE[23] = str_table[16];//	"."
		NFILE[24] = str_table[17];//	"B"
		NFILE[25] = str_table[18];//	"I"
		NFILE[26] = str_table[19];//	"N"
		NFILE[27] = 0x00;				  //	"0"
}

//-------------------------------------------------------write file name provide
void r_file_name (void)
{
	rfile[0] = 0x2F;       		   //  "/"
	rfile[1] = str_table[2];		 //  "2"
	rfile[2] = str_table[0];		 //	 "0"
	rfile[3] = file[0];
	rfile[4] = file[1];
	rfile[5] = 0x2F;        		 //  "/"
	rfile[6] = file[2];
	rfile[7] = file[3];
	rfile[8] = 0x2F;        		 //  "/"
	rfile[9] = file[4];
	rfile[10] = file[5];
	rfile[11] = 0x2F;       		 //  "/"
	rfile[12] = file[6];
	rfile[13] = file[7];

		
		rfile[14] = 0x2F;        	 //  "/"
		
		rfile[15] = file[0];		 	 //	 year		FILE
		rfile[16] = file[1];
		
		rfile[17] = file[2];		 	 //	 month
		rfile[18] = file[3];

		rfile[19] = file[4];		   //	 date
		rfile[20] = file[5];

		rfile[21] = file[6];	  	 //	 hour
		rfile[22] = file[7];
		rfile[23] = str_table[16]; //  "."
		rfile[24] = str_table[17]; //	 "B"
		rfile[25] = str_table[18]; //	 "I"
		rfile[26] = str_table[19]; //	 "N"
		rfile[27] = 0x00;				   //	 "0"
}

//-------------------------------------------------------dir1
FRESULT scan_files (
    char* path        /* Start node to be scanned (***also used as work area***) */
)
{
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;


    res = f_opendir(&dir, path);                      		 /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
						if (fno.fname[0] == '.') continue;             /* Ignore dot entry */	
						if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                res = scan_files(path);                    /* Enter the directory */
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
//                sprintf(file,"%s/%s\0", path, fno.fname);
								file[0]=fno.fname[0];file[1]=fno.fname[1];file[2]=fno.fname[2];file[3]=fno.fname[3];
								file[4]=fno.fname[4];file[5]=fno.fname[5];file[6]=fno.fname[6];file[7]=fno.fname[7];
//								strcpy(file_arr[f_cnt],file);
								file_arr[f_cnt][0]= file[0];file_arr[f_cnt][1]= file[1];file_arr[f_cnt][2]= file[2];file_arr[f_cnt][3]= file[3];
								file_arr[f_cnt][4]= file[4];file_arr[f_cnt][5]= file[5];file_arr[f_cnt][6]= file[6];file_arr[f_cnt][7]= file[7];
								f_cnt++;
            }
        }
        f_closedir(&dir);
    }

    return res;
}

//-------------------------------------------------------rd_file
	void rd_file (void)
	{
		f1.STM_DAT_STP = 1;
		if (f_close(&SDFile1) != FR_OK )	{																		//close writing file
			Error_Handler();
		}
		icnt = 0;
//-----		
		res = f_open(&SDFile2, rfile, FA_READ);
		if (res != FR_OK)	{
			Error_Handler();
		}
		file_size = f_size(&SDFile2);
		for ( ;; )	{																			//loop for entire data read & transmit
			if ( file_size == 0 )	 {												// if size of file zero break in loop 99/07/16
				break;
			}
				diff_dat = file_size - icnt;
				if ( diff_dat > rd_step ) { 
						if (f_lseek(&SDFile2, icnt)!= FR_OK)	{
							Error_Handler();
						}
						res = f_read(&SDFile2, rtext, rd_step, (UINT*)&bytesread );
						if((bytesread == 0) || (res != FR_OK)) {
							Error_Handler();
						}
						icnt += rd_step;
						CDC_Transmit_HS(rtext,rd_step);
						HAL_Delay(60);
				
				}else if( diff_dat == rd_step ) {
						if (f_lseek(&SDFile2, icnt)!= FR_OK)	{
							Error_Handler();
						}
						res = f_read(&SDFile2, rtext, rd_step, (UINT*)&bytesread );
						if((bytesread == 0) || (res != FR_OK)) {
							Error_Handler();
						}
						icnt += rd_step;
						CDC_Transmit_HS(rtext,rd_step);
						HAL_Delay(60);
						break;
						
				}else  {
						if (f_lseek(&SDFile2, icnt)!= FR_OK)	{
							Error_Handler();
						}

						res = f_read(&SDFile2, rtext, diff_dat, (UINT*)&bytesread );
						if((bytesread == 0) || (res != FR_OK)) {
							Error_Handler();
						}
						icnt += diff_dat;
						CDC_Transmit_HS(rtext,diff_dat);
						HAL_Delay(60);
						break;
				}
		}	
				
		if (f_close(&SDFile2) != FR_OK )	{
			Error_Handler();
		}
//-----		
		if (f_open(&SDFile1, NFILE, FA_OPEN_EXISTING | FA_WRITE ) != FR_OK) {	//open writing file
			Error_Handler();
		}
		
		if (f_lseek(&SDFile1, f_size(&SDFile1))!= FR_OK)	{
			Error_Handler();
		}
		f1.STM_DAT_STP = 0;
	}
	
//-----------------------------------------------------------------copy "pc_buff" to "pc structure"--------------------------------------------------------		
	void cpypc_pcbuf (void)
	{
				pc.start = pc_buf[0];
				pc.CommandType = pc_buf[1];    //command : "charge = 1","shock = 2 ","sync_shock = 3 ",...
				pc.status = pc_buf[2];         //PC internal status
				pc.year = pc_buf[3];           // year (binary), offset=2000
				pc.month = pc_buf[4];          // month (binary)
				pc.day = pc_buf[5];            // day (binary)
				pc.hour = pc_buf[6];           // hour (binary)
				pc.minute = pc_buf[7];         // minute (binary)
				pc.second = pc_buf[8];         // second (binary)
				pc.stop = pc_buf[9];
	}

//mSD----------------------------------------------------------------------------
//---------------------------------------------------------------TIMER--------------------------------------------		
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if 						(htim->Instance == TIM7)  {
			__NOP();
		}
		else if 			(htim->Instance == TIM8)  {
			__NOP();
		}
		else if 			(htim->Instance == TIM10)  {
		f1.tim10_flg = 1;
	  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13 );	
	  HAL_RTC_GetTime(&hrtc, &currentTime, FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &currentDate, FORMAT_BIN);
		s1.date_year = currentDate.Year + 2000 ;	
		s1.date_month = currentDate.Month ;
	  s1.date_day = currentDate.Date ;
	  s1.time_hour = currentTime.Hours ;
		s1.time_minute = currentTime.Minutes ;
	  s1.time_second = currentTime.Seconds ;				
    var_printH(s1.date_year,4,11,100,242);
    var_printH(s1.date_month,2,11,100,198);
    var_printH(s1.date_day,2,11,100,176);
    var_printH(s1.time_hour,2,11,100,154);
    var_printH(s1.time_minute,2,11,100,132);
    var_printH(s1.time_second,2,11,100,110);
		}
		else if 			(htim->Instance == TIM11)  {
				f1.tim11_flg = 1;
//				HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14 );	
		}
		else	{
			__NOP();
		}
	}
//-----------------------------------------------------------------SPI--------------------------------------------------------		
	void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
	{
		if 			(hspi->Instance == SPI4)  {
			f1.spi4_strt_flg = 1;
		}
	}
//----------------------------------------------------------------EXTI----------------------------	
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		 if (GPIO_Pin == GPIO_PIN_3) {								//lcd  interrupt 
				f1.data_rcv_flg = 1;
			}				
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
  	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET );     //led R on
  	f1.error_handler = 1;
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
