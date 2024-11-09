/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include <stdio.h>
#include "ds1307_for_stm32_hal.h"
#include "tm1637.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BT_KEY_UP 1
#define BT_KEY_DOWN 2
#define BT_KEY_UP_AND_DOWN 3
#define BT_KEY_UP_DRAG 4
#define BT_KEY_DOWN_DRAG 5
#define BT_KEY_UP_AND_DOWN_DRAG 6

#define EEPROM_ADDR 0xA0
#define PAGE_SIZE 64     // in Bytes จำนวน ต่อ 1 หน้า
#define PAGE_NUM  512    // number of pages จำนวนหน้า
#define EEPROM_I2C &hi2c1

#define CONFIG_ADDR 0x700

#define MPU6050_ADDR 0xD0

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define acc_CONFIG_REG 0x1B
#define acc_XOUT_H_REG 0x43

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct config {

	uint8_t Limit_value[16];
	uint8_t Period;
};

struct config Config;
uint8_t sinnal_PWN;
uint8_t MSG[50] = { '\0' };
uint8_t X = 0;
uint8_t gyloXYZ[14];
const uint16_t i2c_timeout = 100;
uint8_t Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float genDegreefromAcc(float ac_value) {
	return ac_value * 90;
}
uint8_t Getstategylo(float x, float y, float z, float torrent) {
	uint8_t bufferx, buffery, bufferz, buffer_finish;
	return buffer_finish;
} // ใช้ตัวเลข 6 bit จากขวาสุด หลัก 1-3-5 แสดงสถานะว่าสมดุลหรือไม่  1 คือ x , 3 คือ y , 5 คือ z,0=สมดุุล,1=ไม่สมดุล พิจารณาจากว่าเกินค่า offset
//หลัก 2-4-6 แสดงทิศทาง 0 คือบวก 1 คือลบ
void EEPROM_Read(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size) {
	int paddrposition = log(PAGE_SIZE) / log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size + offset) / PAGE_SIZE);

	uint16_t numofpages = (endPage - startPage) + 1;
	uint16_t pos = 0;

	for (int i = 0; i < numofpages; i++) {
		uint16_t MemAddress = startPage << paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos],
				bytesremaining, 1000);
		startPage += 1;
		offset = 0;
		size = size - bytesremaining;
		pos += bytesremaining;
	}
}
void EEPROM_Write(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size) {

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(PAGE_SIZE) / log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size + offset) / PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage - startPage) + 1;
	uint16_t pos = 0;

	// write the data
	for (int i = 0; i < numofpages; i++) {
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage << paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset); // calculate the remaining bytes to be written

		HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos],
				bytesremaining, 1000);  // write the data to the EEPROM

		startPage += 1; // increment the page, so that a new page address can be selected for further write
		offset = 0; // since we will be writing to a new page, so offset will be 0
		size = size - bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay(5);  // Write cycle delay (5ms)
	}
}
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
	uint8_t check;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1,
			i2c_timeout);

	if (check == 104) // 0x68 will be returned by the sensor if everything goes well
			{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0x0;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1,
				i2c_timeout);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1,
				i2c_timeout);

		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x1A, 1, &Data, 1, i2c_timeout);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g

		Data = 0x0;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1,
				i2c_timeout);

		// Set accscopic configuration in acc_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
		Data = 0x0;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, acc_CONFIG_REG, 1, &Data, 1,
				i2c_timeout);

		Data = 0x07;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x68, 1, &Data, 1, i2c_timeout);

		Data = 0xff;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x23, 1, &Data, 1, i2c_timeout);

		sprintf(MSG, "OKx1x \r\n");
		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
		return 0;
	}
	return 1;
}
void read_gylo() {

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, gyloXYZ,
			14, 1);
}

void read_Config() {
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, CONFIG_ADDR, 2, &Config,
			sizeof(Config), 1000);
	uint8_t *x;
	x = &Config;
	for (int i = 0; i < sizeof(Config); i++) {
		sprintf(MSG, "Read %d = %d \r\n", i, *(x + i));
		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
	}
}
void write_Config() {
	for (uint8_t i = 0; i < 16; i++) {
		if (Config.Limit_value[i] < 10) {
			Config.Limit_value[i] = 128;
		}
		sprintf(MSG, "L %d = %d \r\n", i, Config.Limit_value[i]);
		HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
	}
	if (Config.Period < 2) {
		Config.Period = 8;
	}
//	sprintf(MSG, "PL : %d \r\n", Config.Period);
//	HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
//	sprintf(MSG, "SizeConfig : %d \r\n", sizeof(Config));
//	HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
//	HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, CONFIG_ADDR, 2, &Config,
//			sizeof(Config), 1000);
}
void Set_limitvalue() {

}
uint8_t read_bt() {
	static uint8_t Bt_key;
	static bool statecout;
	static uint64_t timetrick;
	GPIO_PinState Bt_up, Bt_Down;

	Bt_up = HAL_GPIO_ReadPin(Bt_UP_GPIO_Port, Bt_UP_Pin);
	Bt_Down = HAL_GPIO_ReadPin(Bt_DOWN_GPIO_Port, Bt_DOWN_Pin);
	while (!Bt_up) {

	}

	return 0;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_CAN_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_RTC_Init();
	MX_USART2_UART_Init();
	MX_FATFS_Init();
	TM1637_Init();
	TM1637_SetBrightness(7);
	TM1637_ClearDisplay();
	while (MPU6050_Init(&hi2c1) == 1)
		;
	/* USER CODE BEGIN 2 */
//	write_Config();
	/* Start DS1307 timing. Pass user I2C handle pointer to function. */
//	DS1307_Init(&hi2c1);
//	/* To test leap year correction. */
//	DS1307_SetTimeZone(+7, 00);
//	DS1307_SetDate(30);
//	DS1307_SetMonth(10);
//	DS1307_SetYear(2024);
//	DS1307_SetHour(19);
//	DS1307_SetMinute(45);
//	DS1307_SetSecond(10);
//	read_Config();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	float acc_x;
	float acc_y;
	float acc_z;
	short gylo_x_raw;
	short gylo_y_raw;
	short gylo_z_raw;
	float gylo_x;
	float gylo_y;
	float gylo_z;
	while (1) {
		static uint32_t tk;
		static bool state_tk;
		static bool show_togleamp;
		if (!state_tk) {
			tk = HAL_GetTick();
			state_tk = true;
			show_togleamp = true;
//			uint8_t date = DS1307_GetDate();
//			uint8_t month = DS1307_GetMonth();
//			uint16_t year = DS1307_GetYear();
//			uint8_t hour = DS1307_GetHour();
//			uint8_t minute = DS1307_GetMinute();
//			uint8_t second = DS1307_GetSecond();
//			int8_t zone_hr = DS1307_GetTimeZoneHour();
//			uint8_t zone_min = DS1307_GetTimeZoneMin();
//			sprintf(MSG,
//					"TZ:+%02d:%02d,%04d-%02d-%02d,%02d:%02d:%02d%: tk = %d \r\n",
//					zone_hr, zone_min, year, month, date, hour, minute, second,
//					tk);
//			/* May show warning below. Ignore and proceed. */
			read_gylo();
			acc_x_raw = (gyloXYZ[0] << 8) | gyloXYZ[1];
			acc_y_raw = (gyloXYZ[2] << 8) | gyloXYZ[3];
			acc_z_raw = (gyloXYZ[4] << 8) | gyloXYZ[5];
			acc_x = acc_x_raw / 16384.f;
			acc_y = acc_y_raw / 16384.f;
			acc_z = acc_z_raw / 16384.f;

			gylo_x_raw = (gyloXYZ[8] << 8) | gyloXYZ[9];
			gylo_y_raw = (gyloXYZ[10] << 8) | gyloXYZ[11];
			gylo_z_raw = (gyloXYZ[12] << 8) | gyloXYZ[13];

			gylo_x = gylo_x_raw / 131.f;
			gylo_y = gylo_y_raw / 131.f;
			gylo_z = gylo_z_raw / 131.f;
			sprintf(MSG, "tk = %d ax = %.2f ,ay = %.2f , az = %.2f \r\n", tk,
					acc_x, acc_y, acc_z);
			/* May show warning below. Ignore and proceed. */
			HAL_UART_Transmit(&huart1, MSG, strlen(MSG), 100);

			sprintf(MSG, "tk = %d glx = %.2f ,gly = %.2f , glz = %.2f \r\n", tk,
					gylo_x, gylo_y, gylo_z);
			/* May show warning below. Ignore and proceed. */
			HAL_UART_Transmit(&huart1, MSG, strlen(MSG), 100);
			TM1637_displayDecimal_only(genDegreefromAcc(acc_x));

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		} else {
			uint32_t tt = HAL_GetTick();

			if ((tt - tk) >= 250) {
				if (show_togleamp) {
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					show_togleamp = false;
				}
			}
			if ((tt - tk) >= 500) {
				state_tk = false;
			}
		}

//		HAL_Delay(50);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 16;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef DateToUpdate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SIGNAL_LAMP_Pin_GPIO_Port, SIGNAL_LAMP_Pin_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, TM_CLK_Pin | TM_DIO_Pin | GPIO_PIN_2,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : LAMP_SIGNAL_Pin_Pin */
	GPIO_InitStruct.Pin = SIGNAL_LAMP_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SIGNAL_LAMP_Pin_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TM_CLK_Pin TM_DIO_Pin PB2 */
	GPIO_InitStruct.Pin = TM_CLK_Pin | TM_DIO_Pin | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 PB14 PB15
	 PB3 Bt_UP_Pin Bt_DOWN_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
			| GPIO_PIN_3 | Bt_UP_Pin | Bt_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
