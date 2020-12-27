/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bitmap.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
	uint16_t raw;
	struct {
		unsigned int r :5;
		unsigned int g :6;
		unsigned int b :5;
	} rgb;
	struct {
		unsigned int high :8;
		unsigned int low :8;
	} split;
} OLEDColorUnion;

typedef union {
	uint16_t raw;
	struct {
		unsigned int high :8;
		unsigned int low :8;
	} split;
} transmit16;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PORTRAIT
#define LANDSCAPE

#define MAC_PORTRAIT 0xe8
#define MAC_LANDSCAPE 0x48

#if defined(PORTRAIT)
#define MAC_CONFIG MAC_PORTRAIT
#define OLED_WIDTH 63
#define OLED_HEIGHT 95
#else
#define MAC_CONFIG MAC_LANDSCAPE
#define OLED_WIDTH 95
#define OLED_HEIGHT 63
#endif

#define CMD_DISPLAY_OFF 0xAE
#define CMD_DISPLAY_ON 0xAF
#define CMD_COLUMN_ADDRESS 0x15
#define CMD_ROW_ADDRESS 0x75
#define CMD_SET_POWER_SAVE_MODE 0xB0
#define CMD_POWER_SAVE_EN 0x1A
#define CMD_SET_START_LINE 0xA1
#define CMD_SET_OFFSET 0xA2
#define CMD_SET_MODE_NORMAL 0xA4

#define OLED_background	0x0000
#define OLED_white 0xF7DE
#define Major_white 0b0111101111101111
#define miner_white 0b0011100111100111
#define background_white	0xFFFF

#define OLED_NAVY        0x000F      /*   0,   0, 128 */
#define OLED_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define OLED_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define OLED_MAROON      0x7800      /* 128,   0,   0 */
#define OLED_PURPLE      0x780F      /* 128,   0, 128 */
#define OLED_OLIVE       0x7BE0      /* 128, 128,   0 */
#define OLED_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define OLED_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define OLED_CYAN        0x07FF      /*   0, 255, 255 */
#define OLED_RED         0xF800      /* 255,   0,   0 */
#define OLED_MAGENTA     0xF81F      /* 255,   0, 255 */
#define OLED_ORANGE      0xFD20      /* 255, 165,   0 */
#define OLED_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define OLED_PINK        0xF81F

#ifndef M_PI
#define M_PI 3.14159265358979
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t firstEdgePin = 0;
int16_t value = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void OLED_init();
void OLED_clear(uint8_t, uint8_t, uint8_t, uint8_t);
void OLED_copy(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
void OLED_set_rect(uint8_t, uint8_t, uint8_t, uint8_t);
void OLED_fill_rect(uint8_t, uint8_t, uint8_t, uint8_t, uint16_t, uint16_t);
void OLED_draw_line(uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
void displayStringBitmap(uint8_t, uint8_t, const char _character[], uint16_t,
		uint16_t, uint8_t);
void displayCharacterBitmap8(uint8_t, uint8_t, char, uint16_t, uint16_t);
void displayCharacterBitmap5(uint8_t, uint8_t, char, uint16_t, uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//HW-040(KY-040)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//rotary encoder
	if (firstEdgePin == 0) {
		return;
	}

	GPIO_PinState REA = HAL_GPIO_ReadPin(GPIOB, RE_A_Pin);
	GPIO_PinState REB = HAL_GPIO_ReadPin(GPIOB, RE_B_Pin);
	GPIO_PinState RESW = HAL_GPIO_ReadPin(GPIOA, RE_SW_Pin);

	if (REA && REB) {
		if (firstEdgePin == RE_A_Pin) { // Means pin A Changed first - We're Rotating Clockwise
			if (RESW) {
				value++;
			} else {
				value += 10;
			}
		} else { // Otherwise B changed first and we're moving CCW
			if (RESW) {
				value--;
			} else {
				value -= 10;
			}
		}
		firstEdgePin = 0;
	}
	return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case RE_A_Pin:
	case RE_B_Pin:
		if (firstEdgePin == 0) {
			firstEdgePin = GPIO_Pin;
		}
		break;
	}
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
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	//variable
	int16_t scaleInterval = 10;
	int16_t resolution = 1;
	uint8_t center_y = 32;
	uint8_t mode = 1;
	/*
	 * mode 1: Linear meter(like micrometer)
	 * mode 2: Arc meter
	 */

	//OLED initilizetion
	HAL_GPIO_WritePin(GPIOB, OLED_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOB, OLED_RESET_Pin, GPIO_PIN_SET);

	OLED_init();
	OLED_clear(0, OLED_WIDTH, 0, OLED_HEIGHT);
	OLED_fill_rect(0, OLED_WIDTH, 0, OLED_HEIGHT, OLED_background,
	OLED_background);
	//OLED initilizetion end
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int16_t prev_value = 0;

	uint8_t changeScale = 1;
	uint8_t changePalet = 1;
	uint8_t changeMode = 1;

	uint8_t prevScale = 100;
	uint8_t prevPalet = 100;

	uint16_t color_text = OLED_white;
	uint16_t color_scale = miner_white;
	uint16_t color_background = OLED_background;

	transmit16 transmitData;
	char measure[7];

	while (1) {
		//palet
		if (!HAL_GPIO_ReadPin(GPIOB, color_palet_Pin)) { //dark
			color_text = OLED_white;
			color_scale = miner_white;
			color_background = OLED_background;

			changePalet = (prevPalet == 1) ? 0 : 1;
			prevPalet = 1;
		} else { //white
			color_text = OLED_background;
			color_scale = OLED_background;
			color_background = background_white;

			changePalet = (prevPalet == 0) ? 0 : 1;
			prevPalet = 0;
		}

		//scale
		if (HAL_GPIO_ReadPin(GPIOA, resolution_Pin)) {
			scaleInterval = 10;
			resolution = 1;
			changeScale = (prevScale == 1) ? 0 : 1;
			prevScale = 1;
		} else {
			scaleInterval = 1000;
			resolution = 100;
			changeScale = (prevScale == 0) ? 0 : 1;
			prevScale = 0;
		}

		//display meter
		if ((prev_value != value) || (changeScale) || (changePalet)
				|| (changeMode)) {
			prev_value = value; //Since the value may change after this, set prev to the displayed value.
			HAL_GPIO_WritePin(GPIOC, Board_LED_Pin, GPIO_PIN_RESET); //status LED

			//color palet changed
			if (changePalet) {
				OLED_fill_rect(0, OLED_WIDTH, 0, OLED_HEIGHT, color_background,
						color_background);
			}

			//create bitmap
			uint8_t bitmap[256] = { 0 };
			if (mode == 0) {
				drawCircleMode(prev_value, scaleInterval, resolution, center_y,
						bitmap);
			} else if (mode == 1) {
				drawLinerMode(prev_value, scaleInterval, resolution, center_y,
						bitmap);
			}

			uint8_t spi_tx_buff[128];
			uint8_t spi_buff_cnt;
			uint8_t index = 0;

			//draw bitmap
			OLED_set_rect(0, 31, 0, OLED_HEIGHT);
			for (uint8_t i = 0; i < 32; i++) {
				spi_buff_cnt = 0;
				for (uint8_t j = 0; j < 8; j++) {
					uint16_t fontByte;
					fontByte = bitmap[index++];

					for (uint8_t x = 0; x < 8; x++) {
						uint16_t pixelOutput;
						pixelOutput =
								(fontByte & 0x80) ?
										color_scale : color_background;
						fontByte = fontByte << 1;

						transmitData.raw = pixelOutput;
						spi_tx_buff[spi_buff_cnt++] = transmitData.split.low;
						spi_tx_buff[spi_buff_cnt++] = transmitData.split.high;
					}
				}
				HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 128);
				while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
					;
			}

			//fixed needle
			OLED_draw_line(27, 42, center_y, center_y, OLED_RED);

			sprintf(measure, "%6d", prev_value);
			displayStringBitmap(45, 29, measure, color_text, color_background,
					1);

			changeScale = 0;
			changePalet = 0;
			changeMode = 0;
			HAL_GPIO_WritePin(GPIOC, Board_LED_Pin, GPIO_PIN_SET); //status LED
		}
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 500;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 500;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, OLED_DC_Pin | OLED_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Board_LED_Pin */
	GPIO_InitStruct.Pin = Board_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OLED_DC_Pin OLED_RESET_Pin */
	GPIO_InitStruct.Pin = OLED_DC_Pin | OLED_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : resolution_Pin color_palet_Pin mode_Pin */
	GPIO_InitStruct.Pin = resolution_Pin | color_palet_Pin | mode_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : RE_SW_Pin */
	GPIO_InitStruct.Pin = RE_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(RE_SW_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RE_A_Pin RE_B_Pin */
	GPIO_InitStruct.Pin = RE_A_Pin | RE_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void OLED_set_rect(uint8_t _x1, uint8_t _x2, uint8_t _y1, uint8_t _y2) {
	uint8_t spi_tx_buff[6];

	spi_tx_buff[0] = CMD_COLUMN_ADDRESS;
	spi_tx_buff[1] = _x1;
	spi_tx_buff[2] = _x2;
	spi_tx_buff[3] = CMD_ROW_ADDRESS;
	spi_tx_buff[4] = _y1;
	spi_tx_buff[5] = _y2;

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 6);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);
}

void OLED_copy(uint8_t _x1, uint8_t _x2, uint8_t _y1, uint8_t _y2, uint8_t _x,
		uint8_t _y) {
	uint8_t spi_tx_buff[7];

	spi_tx_buff[0] = 0x23;
	spi_tx_buff[1] = _x1;
	spi_tx_buff[2] = _y1;
	spi_tx_buff[3] = _x2;
	spi_tx_buff[4] = _y2;
	spi_tx_buff[5] = _x;
	spi_tx_buff[6] = _y;

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 7);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);
}

void OLED_fill_rect(uint8_t _x1, uint8_t _x2, uint8_t _y1, uint8_t _y2,
		uint16_t _outline, uint16_t _innner) {
	OLEDColorUnion color;
	uint8_t spi_tx_buff[11];

	spi_tx_buff[0] = 0x26;
	spi_tx_buff[1] = 0b00000001;

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 2);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);

	spi_tx_buff[0] = 0x22;
	spi_tx_buff[1] = _x1;
	spi_tx_buff[2] = _y1;
	spi_tx_buff[3] = _x2;
	spi_tx_buff[4] = _y2;

	color.raw = _outline;
	spi_tx_buff[5] = (uint8_t) (color.rgb.b << 1);
	spi_tx_buff[6] = (uint8_t) (color.rgb.g);
	spi_tx_buff[7] = (uint8_t) (color.rgb.r << 1);
	color.raw = _innner;
	spi_tx_buff[8] = (uint8_t) (color.rgb.b << 1);
	spi_tx_buff[9] = (uint8_t) (color.rgb.g);
	spi_tx_buff[10] = (uint8_t) (color.rgb.r << 1);

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 11);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void OLED_draw_line(uint8_t _x1, uint8_t _x2, uint8_t _y1, uint8_t _y2,
		uint16_t _color) {
	OLEDColorUnion color;
	uint8_t spi_tx_buff[8];

	spi_tx_buff[0] = 0x21;
	spi_tx_buff[1] = _x1;
	spi_tx_buff[2] = _y1;
	spi_tx_buff[3] = _x2;
	spi_tx_buff[4] = _y2;

	color.raw = _color;
	spi_tx_buff[5] = (uint8_t) (color.rgb.b << 1);
	spi_tx_buff[6] = (uint8_t) (color.rgb.g);
	spi_tx_buff[7] = (uint8_t) (color.rgb.r << 1);

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 8);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void OLED_clear(uint8_t _x0, uint8_t _x1, uint8_t _y0, uint8_t _y1) {
	uint8_t spi_tx_buff[5];
	spi_tx_buff[0] = 0x25;
	spi_tx_buff[1] = _x0;
	spi_tx_buff[2] = _y0;
	spi_tx_buff[3] = _x1;
	spi_tx_buff[4] = _y1;

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 5);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void displayStringBitmap(uint8_t _x, uint8_t _y, const char _character[],
		uint16_t _color, uint16_t _background_color, uint8_t size) {
	uint8_t c = 0;
	while (_character[c]) {
		if (size == 0) {
			displayCharacterBitmap5(_x + (c * 4), _y, _character[c], _color,
					_background_color);
		} else {
			displayCharacterBitmap8(_x + (c * 8), _y, _character[c], _color,
					_background_color);
		}
		c++;
	}
}

void displayCharacterBitmap8(uint8_t _x, uint8_t _y, char _character,
		uint16_t _color, uint16_t _background_color) {
	uint8_t end_x, end_y;

	transmit16 transmitData;
	uint8_t spi_tx_buff[128];
	uint8_t spi_buff_cnt = 0;

	end_x = _x + 8 - 1;
	end_y = _y + 8 - 1;

	OLED_set_rect(_x, end_x, _y, end_y);
	for (uint8_t y = 0; y < 8; y++) {
		uint16_t fontByte;
		fontByte = FONT8x8[_character - 0x1F][y];

		for (uint8_t x = 0; x < 8; x++) {
			uint16_t pixelOutput;
			pixelOutput = (fontByte & 0x80) ? _color : _background_color;
			fontByte = fontByte << 1;

			transmitData.raw = pixelOutput;
			spi_tx_buff[spi_buff_cnt++] = transmitData.split.low;
			spi_tx_buff[spi_buff_cnt++] = transmitData.split.high;
		}
	}

	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 128);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
}

void displayCharacterBitmap5(uint8_t _x, uint8_t _y, char _character,
		uint16_t _color, uint16_t _background_color) {
	uint8_t code = 12;
	if ((0x30 <= _character) && (_character <= 0x39)) {
		code = _character - 0x30;
	} else if (_character == 0x2d) {
		code = 10;
	} else if (_character == 0x2e) {
		code = 11;
	}
	uint8_t end_x, end_y;

	transmit16 transmitData;
	uint8_t spi_tx_buff[32];
	uint8_t spi_buff_cnt = 0;

	end_x = _x + 3 - 1;
	end_y = _y + 5 - 1;

	OLED_set_rect(_x, end_x, _y, end_y);
	for (uint8_t array = 0; array < 2; array++) {
		uint16_t fontByte;
		fontByte = FONT5x3[code][array];

		for (uint8_t i = 0; i < 8; i++) {
			uint16_t pixelOutput;
			pixelOutput = (fontByte & 0x80) ? _color : _background_color;
			fontByte = fontByte << 1;

			transmitData.raw = pixelOutput;
			spi_tx_buff[spi_buff_cnt++] = transmitData.split.low;
			spi_tx_buff[spi_buff_cnt++] = transmitData.split.high;
		}
	}

	HAL_SPI_Transmit_DMA(&hspi1, spi_tx_buff, 30);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
}

void OLED_init() {
	uint8_t spi_tx_buff[1];

	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_RESET);

	spi_tx_buff[0] = CMD_DISPLAY_OFF; //Set Display Off
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xA0; //Remap & Color Depth setting
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0b01110010; //A[7:6] = 00; 256 color. A[7:6] = 01; 65k color format
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_SET_START_LINE; //Set Display Start Line
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_SET_OFFSET; //Set Display Offset
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_SET_MODE_NORMAL; //Set Display Mode (Normal)
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xA8; //Set Multiplex Ratio
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 63; //15-63
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xAD; //Set Master Configration
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0b10001110; //a[0]=0 Select external Vcc supply, a[0]=1 Reserved(reset)
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_SET_POWER_SAVE_MODE; //Power Save Mode
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_POWER_SAVE_EN; //0x1A Enable power save mode. 0x00 Disable
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xB1; //Phase 1 and 2 period adjustment
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x74;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xB3; //Display Clock DIV
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xF0;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x8A; //Pre Charge A
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x81;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x8B; //Pre Charge B
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x82;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x8C; //Pre Charge C
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x83;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xBB; //Set Pre-charge level
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x3A;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0xBE; //Set VcomH
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x3E;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x87; //Set Master Current Control
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x06;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_COLUMN_ADDRESS; //Set Column Address
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = OLED_HEIGHT;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_ROW_ADDRESS; //Set Row Address
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = OLED_HEIGHT;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x81; //Set Contrast for Color A
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 255;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x82; //Set Contrast for Color B
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 255;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 0x83; //Set Contrast for Color C
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = 255;
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);
	spi_tx_buff[0] = CMD_DISPLAY_ON; //Set Display On
	HAL_SPI_Transmit(&hspi1, spi_tx_buff, 1, 1000);

	HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin, GPIO_PIN_SET);
	HAL_Delay(150); //datasheet required after command 0xAF(display on)
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
