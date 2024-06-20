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
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "obc_interface.h"
#include "radio_driver.h"
#include "com_debug.h"
#include "stdio.h"
#include "ax25_generator.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PAYLOAD_LENGTH 		(102)
#define OBC_PAYLOAD_LENGTH	(83)
#define GS_CMD_LENGTH		(35)

#define FREQ_435_MHZ            (435313000) //DLink
#define FREQ_437_MHZ			(437375000)	//UPLINK

#define PA_DUTY_CYCLE           (0x04)
#define HP_MAX                  (0x00)
#define PA_SEL                  (0x01)

#define POWER                   (0x0E)
#define RAMP_TIME               (0x02)

#define DEMO_DEFAULT_BR_1200            1200
#define DEMO_DEFAULT_FDEV_1200          3000

#define DEMO_DEFAULT_BR_4800            4800
#define DEMO_DEFAULT_FDEV_4800          12000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PacketParams_t pkt_params;
ModulationParams_t mod_params;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SUBGHZ_HandleTypeDef hsubghz;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

int OBC_HANDSHAKE_FLAG = 0;
int OBC_SUCCESS_DATA_RX_FLAG = 0;

uint8_t obc_plen = OBC_PAYLOAD_LENGTH;
uint8_t p_len = PAYLOAD_LENGTH;
uint8_t gs_cmd_plen = GS_CMD_LENGTH;

uint8_t OBC_UART_RX[OBC_PAYLOAD_LENGTH];
uint8_t tx_cmd[PAYLOAD_LENGTH];
uint8_t rx_cmd[GS_CMD_LENGTH];

uint8_t rssi_value = 0;
int packetLen = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DioIrqHndlr(RadioIrqMasks_t radioIrq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_SubGHz_Phy_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);

	myDebug("########## Slippers2Sat SATELLITE COM: BEGIN ##########\r\n");
	myDebug("########## COMMUNICATION PARAMETERS ##########\r\n ");
	myDebug("Modulation: GFSK PACKET\r\n");
	myDebug("FREQUENCY MODES: DOWNLINK FREQ: %luHz and UPLINK FREQ: %lu Hz\r\n",
	FREQ_437_MHZ, FREQ_435_MHZ);
	myDebug("STM32 BSP_SubGHz-WL Radio: Low Power\n ");
	myDebug("POWER CONFIG:: \n "
			"PA_DUTY_CYCLE: %x, HP_MAX: %x, PA_SEL: %x, POWER TX: %u dBm\r\n",
	PA_DUTY_CYCLE, HP_MAX, PA_SEL, POWER);

	myDebug("########## Operation Starts ##########\r\n");
	myDebug("________________Waiting Handshake Command____________\r\n");

	WAIT_FOR_HANDSHAKE();

	HAL_UART_Receive_DMA(&huart2, OBC_UART_RX, obc_plen);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		delay_us(1000000);

		if (OBC_SUCCESS_DATA_RX_FLAG) {

			packetLen = AX_25PacketFormation(OBC_UART_RX);

			myDebug("\nAX.25 packet_buffer: 0x%x\r\n", tx_cmd);
			myDebug(" Tx_Buffer Length: %d\r\n", packetLen);
			for (int i = 0; i < packetLen; i++) {
				myDebug(" %0x", tx_cmd[i]);
			}
			myDebug("\r\n");

			pkt_params.PacketType = PACKET_TYPE_GFSK;
			pkt_params.Params.Gfsk.PayloadLength = PAYLOAD_LENGTH;
			pkt_params.Params.Gfsk.PreambleLength = 8; /*Convert byte into bit*/
			pkt_params.Params.Gfsk.PreambleMinDetect =
					RADIO_PREAMBLE_DETECTOR_08_BITS;
			pkt_params.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
			pkt_params.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
			pkt_params.Params.Gfsk.HeaderType = RADIO_PACKET_FIXED_LENGTH;
			pkt_params.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
			pkt_params.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

			mod_params.PacketType = PACKET_TYPE_GFSK;
			mod_params.Params.Gfsk.Bandwidth = RX_BW_29300; /*Not used in TX*/
			mod_params.Params.Gfsk.BitRate = DEMO_DEFAULT_BR_4800;
			mod_params.Params.Gfsk.Fdev = DEMO_DEFAULT_FDEV_4800;
			mod_params.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;

			SUBGRF_Init(DioIrqHndlr);
			SUBGRF_SetBufferBaseAddress(0x00, 0x00);
			SUBGRF_SetPayload(tx_cmd, PAYLOAD_LENGTH);
			SUBGRF_SetPacketParams(&pkt_params);
			SUBGRF_SetSyncWord(( uint8_t[] ) { 0xC1, 0x94, 0xC1, 0x00, 0x00,
							0x00, 0x00, 0x00 });
			SUBGRF_SetWhiteningSeed(0x01FF);
			SUBGRF_SetRfFrequency(FREQ_437_MHZ);
			SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
			//SUBGRF_SetTxParams(RFO_HP, POWER, RAMP_TIME);
			SUBGRF_SetTxParams(RFO_LP, POWER, RAMP_TIME); // Set to RFO_LP for low power
			SUBGRF_SetModulationParams(&mod_params);
			SUBGRF_SetDioIrqParams(
					IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
							| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
					IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
							| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
					IRQ_RADIO_NONE, IRQ_RADIO_NONE);

			myDebug("########## TX Configuration: ##########\n ");

			myDebug("FREQUENCY MODS: DOWNLINK FREQ: %lu Hz\r\n", FREQ_437_MHZ);
			myDebug("Bit Rate: 	%d\n\r", mod_params.Params.Gfsk.BitRate);
			myDebug("Frequency Deviation: 	%d\n\r",
					mod_params.Params.Gfsk.Fdev);
			myDebug("RECEVING BANDWIDTH: 	%d\n\r",
					mod_params.Params.Gfsk.Bandwidth);
			myDebug("Packet Type 			%d\n\r", pkt_params.PacketType);
			myDebug("PayloadLength 			%d\n\r",
					pkt_params.Params.Gfsk.PayloadLength);
			myDebug("PreambleLength 		%d\n\r",
					pkt_params.Params.Gfsk.PreambleLength);
			myDebug("PreambleMinDetect		%d\n\r",
					pkt_params.Params.Gfsk.PreambleMinDetect);
			myDebug("HeaderType 			%d\n\r", pkt_params.Params.Gfsk.HeaderType);
			myDebug("__________*******************__________\r\n");

			SUBGRF_SetRfFrequency(FREQ_437_MHZ);
			//SUBGRF_SetSwitch(RFO_HP, RFSWITCH_TX); /*Set RF switch*/
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX); /*Set RF switch*/
			SUBGRF_SendPayload(tx_cmd, p_len, 0);

		}

		HAL_UART_Receive_DMA(&huart2, OBC_UART_RX, obc_plen);

		/* USER CODE END WHILE */
		MX_SubGHz_Phy_Process();

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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 6;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SUBGHZ Initialization Function
 * @param None
 * @retval None
 */
void MX_SUBGHZ_Init(void) {

	/* USER CODE BEGIN SUBGHZ_Init 0 */

	/* USER CODE END SUBGHZ_Init 0 */

	/* USER CODE BEGIN SUBGHZ_Init 1 */

	/* USER CODE END SUBGHZ_Init 1 */
	hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
	if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SUBGHZ_Init 2 */

	/* USER CODE END SUBGHZ_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 48 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
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

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 48 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000001 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin | FE_CTRL2_Pin | FE_CTRL1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
	GPIO_InitStruct.Pin = FE_CTRL3_Pin | FE_CTRL2_Pin | FE_CTRL1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : B1_Pin B2_Pin */
	GPIO_InitStruct.Pin = B1_Pin | B2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : B3_Pin */
	GPIO_InitStruct.Pin = B3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (OBC_HANDSHAKE_FLAG) {
			myDebug("\n--> Command Received form OBC: 0x%x\r\n", OBC_UART_RX);
			for (int i = 0; i < sizeof(OBC_UART_RX); i++) {
				myDebug("%x ", OBC_UART_RX[i]);
			}
			myDebug("\r\n");
			if (OBC_UART_RX[0] == 0x53 && OBC_UART_RX[82] == 0x7E) {
				myDebug("--> Correct command received from OBC\n");
				OBC_SUCCESS_DATA_RX_FLAG = 1;
			} else {
				myDebug("*** Incorrect command received from OBC\n");
				if (HAL_UART_Transmit(&huart2, OBC_UART_RX, obc_plen, 2000)
						== HAL_OK) {
					myDebug("*** Incorrect command re-transmit to OBC\n");
					memset(OBC_UART_RX, '\0', sizeof(OBC_UART_RX));
					OBC_SUCCESS_DATA_RX_FLAG = 0;
				}
			}
			HAL_UART_Receive_DMA(&huart2, OBC_UART_RX, obc_plen);
		}
	}
}

void DioIrqHndlr(RadioIrqMasks_t radioIrq) {
	if (radioIrq == IRQ_TX_DONE) {
		OBC_SUCCESS_DATA_RX_FLAG = 0;
		myDebug("\n\rCommand Transmitted Successful:  \r");
		for (int i = 0; i < PAYLOAD_LENGTH; i++) {
			myDebug(" %02x", tx_cmd[i]);
		}
		myDebug("\n\n");

		delay_us(1000000);

		pkt_params.PacketType = PACKET_TYPE_GFSK;
		pkt_params.Params.Gfsk.PayloadLength = GS_CMD_LENGTH;
		pkt_params.Params.Gfsk.PreambleLength = 8; /*Convert byte into bit*/
		pkt_params.Params.Gfsk.PreambleMinDetect =
				RADIO_PREAMBLE_DETECTOR_08_BITS;
		pkt_params.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
		pkt_params.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
		pkt_params.Params.Gfsk.HeaderType = RADIO_PACKET_FIXED_LENGTH;
		pkt_params.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
		pkt_params.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

		mod_params.PacketType = PACKET_TYPE_GFSK;
		mod_params.Params.Gfsk.Bandwidth = RX_BW_29300; /*Not used in TX*/
		mod_params.Params.Gfsk.BitRate = DEMO_DEFAULT_BR_1200;
		mod_params.Params.Gfsk.Fdev = DEMO_DEFAULT_FDEV_1200;
		mod_params.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;

		SUBGRF_Init(DioIrqHndlr);
		SUBGRF_SetBufferBaseAddress(0x00, 0x00);
		SUBGRF_SetPayload(rx_cmd, GS_CMD_LENGTH);
		SUBGRF_SetPacketParams(&pkt_params);
		SUBGRF_SetSyncWord(( uint8_t[] ) { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00,
						0x00, 0x00 });
		SUBGRF_SetWhiteningSeed(0x01FF);
		SUBGRF_SetRfFrequency(FREQ_435_MHZ);
		SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
		//SUBGRF_SetTxParams(RFO_HP, POWER, RAMP_TIME);
		SUBGRF_SetTxParams(RFO_LP, POWER, RAMP_TIME); // Set to RFO_LP for low power
		SUBGRF_SetModulationParams(&mod_params);
		SUBGRF_SetDioIrqParams(
				IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
						| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
				IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
						| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
				IRQ_RADIO_NONE, IRQ_RADIO_NONE);

		myDebug("########## RX Configuration: ##########\n ");

		myDebug("FREQUENCY MODS: UPLINK FREQ: %lu Hz\r\n", FREQ_435_MHZ);
		myDebug("Bit Rate: 	%d\n\r", mod_params.Params.Gfsk.BitRate);
		myDebug("Frequency Deviation: 	%d\n\r", mod_params.Params.Gfsk.Fdev);
		myDebug("RECEVING BANDWIDTH: 	%d\n\r",
				mod_params.Params.Gfsk.Bandwidth);
		myDebug("Packet Type 			%d\n\r", pkt_params.PacketType);
		myDebug("PayloadLength 			%d\n\r",
				pkt_params.Params.Gfsk.PayloadLength);
		myDebug("PreambleLength 		%d\n\r",
				pkt_params.Params.Gfsk.PreambleLength);
		myDebug("PreambleMinDetect		%d\n\r",
				pkt_params.Params.Gfsk.PreambleMinDetect);
		myDebug("HeaderType 			%d\n\r", pkt_params.Params.Gfsk.HeaderType);
		myDebug("__________*******************__________\r\n");

		myDebug("_____________________*******************______________\r\n");
		myDebug("____Waiting command from Ground Station____\r\n");
		myDebug("__________________________________________________\r\n");

		SUBGRF_SetRfFrequency(FREQ_435_MHZ);
		SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX); /*Set RF switch*/
		SUBGRF_SetRxBoosted(0xFFFFFF);

		HAL_UART_Receive_DMA(&huart2, OBC_UART_RX, obc_plen);
	}

	if (radioIrq == IRQ_RX_DONE) {
		SUBGRF_GetPayload(rx_cmd, &gs_cmd_plen, GS_CMD_LENGTH);
		rssi_value = SUBGRF_GetRssiInst();
		myDebug("\nGS Command Received\r\n");
		for (int i = 0; i < sizeof(rx_cmd); i++) {
			myDebug(" %x", rx_cmd[i]);
		}
		myDebug("\r\n");

		myDebug("Sending to OBC \r\n");

		HAL_UART_Transmit(&huart2, rx_cmd, sizeof(rx_cmd), 2000);

		HAL_UART_Receive_DMA(&huart2, OBC_UART_RX, obc_plen);
	}
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
