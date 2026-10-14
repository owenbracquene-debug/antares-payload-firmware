/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t C[8];          // kalibratiecoëfficiënten C0..C6 (C7 = 0, voor de CRC)
  uint8_t  crc_ok;
  uint8_t  ver;           // versiecode uit C0
  uint8_t  is_02ba;       // 1 = 02BA, 0 = 30BA
  HAL_StatusTypeDef st;
  int32_t  temp;          // 0,01 °C
  int32_t  pres;          // Pa
} MS5837_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAUNCH_THR      2943   // drempel in 0,01 m/s² (3 g = 3 × 981)
#define LAUNCH_MS  100    // zo lang moet de versnelling boven de drempel blijven (ms)
#define MS5837_ADDR  (0x76 << 1)
#define FLASH_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t bno_id = 0;
volatile HAL_StatusTypeDef st_id;
volatile uint8_t acc_cfg = 0;
volatile int16_t acc_x, acc_y, acc_z;   // eenheid: 0,01 m/s²
volatile HAL_StatusTypeDef st_acc;
uint8_t rx_buf[8];
volatile HAL_StatusTypeDef st_tx, st_rx;
volatile uint8_t launched = 0;
volatile uint8_t high_count = 0;
volatile uint8_t ack_ok = 0;
MS5837_t ps1 = { .hi2c = &hi2c1 };   // tanksensor op bus 1
MS5837_t ps2 = { .hi2c = &hi2c2 };   // tanksensor op bus 2
uint32_t t_acc = 0, t_pres = 0, above_since = 0;
uint8_t flash_id[3];
volatile HAL_StatusTypeDef st_flash;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t ms5837_crc4(const uint16_t *prom)
{
  uint16_t n[8];
  for (int i = 0; i < 8; i++) n[i] = prom[i];
  n[0] &= 0x0FFF;
  n[7] = 0;
  uint16_t rem = 0;
  for (int i = 0; i < 16; i++) {
    if (i % 2) rem ^= (n[i >> 1] & 0x00FF);
    else       rem ^= (n[i >> 1] >> 8);
    for (int b = 8; b > 0; b--) {
      if (rem & 0x8000) rem = (rem << 1) ^ 0x3000;
      else              rem = (rem << 1);
    }
  }
  return (rem >> 12) & 0x000F;
}

static HAL_StatusTypeDef ms5837_cmd(MS5837_t *s, uint8_t cmd)
{
  return HAL_I2C_Master_Transmit(s->hi2c, MS5837_ADDR, &cmd, 1, 100);
}

static void ms5837_init(MS5837_t *s)
{
  s->st = ms5837_cmd(s, 0x1E);          // reset
  HAL_Delay(10);
  for (int i = 0; i < 7 && s->st == HAL_OK; i++) {
    uint8_t buf[2] = {0};
    s->st = ms5837_cmd(s, 0xA0 + 2 * i);  // PROM-woord i
    if (s->st == HAL_OK)
      s->st = HAL_I2C_Master_Receive(s->hi2c, MS5837_ADDR, buf, 2, 100);
    s->C[i] = (buf[0] << 8) | buf[1];
  }
  s->C[7] = 0;
  s->crc_ok = (s->st == HAL_OK) && ((s->C[0] >> 12) == ms5837_crc4(s->C));
  s->ver = (s->C[0] >> 5) & 0x7F;
  s->is_02ba = 0;   // versiecode = 0, maar metingen tonen 30BA-gedrag
}
static HAL_StatusTypeDef ms5837_read_adc(MS5837_t *s, uint8_t conv_cmd, uint32_t *out)
{
  HAL_StatusTypeDef st = ms5837_cmd(s, conv_cmd);   // conversie starten
  if (st != HAL_OK) return st;
  HAL_Delay(3);                                     // OSR 1024: max 2,28 ms
  st = ms5837_cmd(s, 0x00);                         // ADC uitlezen
  if (st != HAL_OK) return st;
  uint8_t b[3] = {0};
  st = HAL_I2C_Master_Receive(s->hi2c, MS5837_ADDR, b, 3, 100);
  *out = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
  return st;
}

static void ms5837_measure(MS5837_t *s)
{
  uint32_t D1 = 0, D2 = 0;
  s->st = ms5837_read_adc(s, 0x44, &D1);                       // druk, OSR 1024
  if (s->st == HAL_OK) s->st = ms5837_read_adc(s, 0x54, &D2);  // temperatuur, OSR 1024
  if (s->st != HAL_OK) return;

  int32_t dT   = (int32_t)D2 - ((int32_t)s->C[5] << 8);
  int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * s->C[6]) >> 23);
  int64_t OFF, SENS, Ti = 0, OFFi = 0, SENSi = 0;
  int64_t t = TEMP - 2000;

  if (s->is_02ba) {
    OFF  = ((int64_t)s->C[2] << 17) + (((int64_t)s->C[4] * dT) >> 6);
    SENS = ((int64_t)s->C[1] << 16) + (((int64_t)s->C[3] * dT) >> 7);
    if (TEMP < 2000) {
      Ti    = (11LL * dT * dT) >> 35;
      OFFi  = (31 * t * t) >> 3;
      SENSi = (63 * t * t) >> 5;
    }
  } else {
    OFF  = ((int64_t)s->C[2] << 16) + (((int64_t)s->C[4] * dT) >> 7);
    SENS = ((int64_t)s->C[1] << 15) + (((int64_t)s->C[3] * dT) >> 8);
    if (TEMP < 2000) {
      Ti    = (3LL * dT * dT) >> 33;
      OFFi  = (3 * t * t) >> 1;
      SENSi = (5 * t * t) >> 3;
      if (TEMP < -1500) {
        int64_t u = TEMP + 1500;
        OFFi  += 7 * u * u;
        SENSi += 4 * u * u;
      }
    } else {
      Ti    = (2LL * dT * dT) >> 37;
      OFFi  = (t * t) >> 4;
    }
  }

  OFF  -= OFFi;
  SENS -= SENSi;
  s->temp = TEMP - (int32_t)Ti;                                    // 0,01 °C

  int64_t P = ((((int64_t)D1 * SENS) >> 21) - OFF);
  if (s->is_02ba) s->pres = (int32_t)(P >> 15);                    // Pa
  else            s->pres = (int32_t)(P >> 13) * 10;               // 0,1 mbar → Pa
}
static void led_set(uint16_t pin, int on)
{
  HAL_GPIO_WritePin(GPIOB, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void leds_update(uint32_t now)
{
  // LED0: hartslag
  led_set(GPIO_PIN_0, (now / 500) % 2);

  // LED1: sensoren in orde -> aan, anders snel knipperen
  int sens_ok = (bno_id == 0xA0) && (st_acc == HAL_OK)
             && ps1.crc_ok && ps2.crc_ok
             && (ps1.st == HAL_OK) && (ps2.st == HAL_OK);
  led_set(GPIO_PIN_1, sens_ok ? 1 : (now / 100) % 2);

  // LED2: lancering en camera
  if (!launched)    led_set(GPIO_PIN_2, 0);
  else if (ack_ok)  led_set(GPIO_PIN_2, 1);
  else              led_set(GPIO_PIN_2, (now / 100) % 2);
}
static HAL_StatusTypeDef flash_read_id(uint8_t *id)
{
  uint8_t cmd = 0x9F;                       // JEDEC ID
  FLASH_CS_LOW();
  HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  if (st == HAL_OK) st = HAL_SPI_Receive(&hspi1, id, 3, 100);
  FLASH_CS_HIGH();
  return st;
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Opstartanimatie (~0,7 s) + wachttijd: samen ruim de 650 ms die de BNO055 nodig heeft
  for (int r = 0; r < 3; r++) {
    for (int i = 0; i < 3; i++) {
      led_set(GPIO_PIN_0 << i, 1);
      HAL_Delay(80);
      led_set(GPIO_PIN_0 << i, 0);
    }
  }
  HAL_Delay(300);

  // Chip-ID controleren (moet 0xA0 zijn)
  uint8_t id = 0;
  for (int i = 0; i < 5; i++) {
      st_id = HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
      if (st_id == HAL_OK) break;
      HAL_Delay(10);
  }
  bno_id = id;

  // Versnellingsmeter instellen: 16 g, ACCONLY-modus
  uint8_t v;
  v = 0x00; HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x3D, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  // CONFIG-modus
  HAL_Delay(25);
  v = 0x01; HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  // page 1
  v = 0x0F; HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x08, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  // 16 g, 62,5 Hz
  HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x08, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);             // terug lezen
  acc_cfg = v;
  v = 0x00; HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x07, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  // page 0
  v = 0x01; HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x3D, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);  // ACCONLY
  HAL_Delay(20);

  ms5837_init(&ps1);
  ms5837_init(&ps2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // /RESET van de flash hoog
  HAL_Delay(1);
  st_flash = flash_read_id(flash_id);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    uint32_t now = HAL_GetTick();

	    // 1. Versnelling en lanceerdetectie, elke 10 ms
	    if (now - t_acc >= 10) {
	      t_acc = now;
	      uint8_t a[6];
	      st_acc = HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x08, I2C_MEMADD_SIZE_8BIT, a, 6, 100);
	      if (st_acc == HAL_OK) {
	        acc_x = (int16_t)((a[1] << 8) | a[0]);
	        acc_y = (int16_t)((a[3] << 8) | a[2]);
	        acc_z = (int16_t)((a[5] << 8) | a[4]);

	        if (!launched) {
	          uint32_t mag2 = (uint32_t)((int32_t)acc_x * acc_x)
	                        + (uint32_t)((int32_t)acc_y * acc_y)
	                        + (uint32_t)((int32_t)acc_z * acc_z);
	          uint32_t thr2 = (uint32_t)LAUNCH_THR * LAUNCH_THR;

	          if (mag2 > thr2) {
	            if (above_since == 0) above_since = now;
	            if (now - above_since >= LAUNCH_MS) {
	              launched = 1;
	              const char msg[] = "START\n";
	              for (int i = 0; i < 5 && !ack_ok; i++) {
	                HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, 100);
	                if (HAL_UART_Receive(&huart1, rx_buf, 4, 200) == HAL_OK &&
	                    rx_buf[0] == 'A' && rx_buf[1] == 'C' && rx_buf[2] == 'K') {
	                  ack_ok = 1;
	                }
	              }
	            }
	          } else {
	            above_since = 0;
	          }
	        }
	      }
	    }

	    // 2. Druk, elke 50 ms
	    if (now - t_pres >= 50) {
	      t_pres = now;
	      ms5837_measure(&ps1);
	      ms5837_measure(&ps2);
	    }

	    // 3. LED's bijwerken
	    leds_update(now);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x301029FF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00402D41;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
