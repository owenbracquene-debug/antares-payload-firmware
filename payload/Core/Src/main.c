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
#include "flash/flash_helpers.h"
#include "flash/flash_dump.h"
#include <string.h>

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
#define RUN_FLASH_READBACK_TEST  1   // readout tester bij het opstarten (raakt de vluchtdata niet)
#define ERASE_FLASH_ON_BOOT      0   // 1 = wist de VOLLEDIGE flash bij elke start (80-400 s). In TEST_MODE niet nodig, zie hieronder

// Landingsdetectie: |a| stabiel rond 1 g, pas na een minimale vluchtduur
#define LANDED_MIN_FLIGHT_MS   300000UL  // min. 5 min na lancering (voorkomt trigger onder de parachute)
#define LANDED_STILL_MS         10000UL  // zo lang moet |a| stilstaan
#define LANDED_MAX_FLIGHT_MS  1800000UL  // vangnet: na 30 min altijd "geland"
#define LANDED_G_LOW              785    // 0,8 g in 0,01 m/s²
#define LANDED_G_HIGH            1177    // 1,2 g
#define LANDED_SPREAD2         100000UL  // max. spreiding van |a|² in het venster (~0,05 g)
#define DUMP_RETRY_MS           10000UL  // dump opnieuw proberen als de ESP32 niet antwoordt

// TESTMODUS: vervangt lancerings- en landingsdetectie.
// Direct na het opstarten START naar de camera + loggen, na TEST_LOG_MS "geland" -> dump via UART.
#define TEST_MODE                   0    // !! op 0 zetten voor de vlucht !!
#define TEST_LOG_MS             20000UL  // zo lang loggen in testmodus
#define TEST_START_DELAY_MS      5000UL  // eerst de XIAO laten opstarten (camera + SD) voor START
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
// Diagnose LED1: welke sensorcontrole faalt (bitmasker, zie leds_update)
volatile uint8_t sens_fault = 0, sens_fault_seen = 0;
uint32_t t_retry = 0;
uint8_t flash_id[3];
volatile HAL_StatusTypeDef st_flash;
const CS_Pin_t flash_cs = { GPIOA, GPIO_PIN_4 };   // W25Q256JV /CS op PA4
volatile HAL_StatusTypeDef st_log_init, st_log;
volatile uint8_t flash_test_ok = 0;
volatile uint8_t landed = 0;
volatile uint8_t dump_done = 0;
volatile HAL_StatusTypeDef st_dump = HAL_BUSY;
volatile uint32_t dump_records = 0;
uint32_t launch_time = 0, still_since = 0, still_min2 = 0, still_max2 = 0, t_dump = 0;
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
// BNO055: chip-ID controleren en instellen op 16 g, ACCONLY-modus.
// Wordt bij het opstarten aangeroepen en opnieuw (vóór de lancering) als het mislukte.
static void bno055_init(void)
{
  // Chip-ID controleren (moet 0xA0 zijn)
  uint8_t id = 0;
  for (int i = 0; i < 5; i++) {
      st_id = HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
      if (st_id == HAL_OK) break;
      HAL_Delay(10);
  }
  bno_id = id;
  if (id != 0xA0) return;   // niet gevonden: later opnieuw proberen

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
  // sens_fault toont welke controle faalt; sens_fault_seen onthoudt ook korte fouten
  uint8_t f = 0;
  if (bno_id != 0xA0)    f |= 0x01;   // BNO055 niet herkend (chip-ID)
  if (st_acc != HAL_OK)  f |= 0x02;   // versnelling lezen faalt
  if (!ps1.crc_ok)       f |= 0x04;   // MS5837 bus 1: kalibratie (PROM/CRC) mislukt
  if (!ps2.crc_ok)       f |= 0x08;   // MS5837 bus 2: kalibratie (PROM/CRC) mislukt
  if (ps1.st != HAL_OK)  f |= 0x10;   // MS5837 bus 1: meting faalt
  if (ps2.st != HAL_OK)  f |= 0x20;   // MS5837 bus 2: meting faalt
  sens_fault = f;
  sens_fault_seen |= f;
  led_set(GPIO_PIN_1, f == 0 ? 1 : (now / 100) % 2);

  // LED2: lancering en camera
  if (!launched)    led_set(GPIO_PIN_2, 0);
  else if (ack_ok)  led_set(GPIO_PIN_2, 1);
  else              led_set(GPIO_PIN_2, (now / 100) % 2);
}
// Wacht op een tekstregel van de XIAO die 'needle' bevat (bv. "OK START")
static uint8_t uart_wait_line(const char *needle, uint32_t timeout_ms)
{
  char line[24];
  uint8_t n = 0;
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < timeout_ms) {
    uint8_t c;
    if (HAL_UART_Receive(&huart1, &c, 1, 5) != HAL_OK) continue;
    if (c == '\r') continue;
    if (c == '\n') {
      line[n] = '\0';
      if (strstr(line, needle)) return 1;
      n = 0;
    } else if (n < sizeof(line) - 1) {
      line[n++] = (char)c;
    }
  }
  return 0;
}

// Lancering: camera starten (START, wachten op "OK START") en vanaf nu loggen
static void start_flight(uint32_t now)
{
  launched = 1;
  launch_time = now;
  const char msg[] = "START\n";
  for (int i = 0; i < 10 && !ack_ok; i++) {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);   // oude "READY" e.d. weggooien
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, 100);
    if (uart_wait_line("OK START", 300)) ack_ok = 1;
  }
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

  bno055_init();   // chip-ID + 16 g ACCONLY (zie USER CODE 0)

  ms5837_init(&ps1);
  ms5837_init(&ps2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // /RESET van de flash hoog
  HAL_Delay(1);
  st_flash = flash_read_id(flash_id);

#if ERASE_FLASH_ON_BOOT
  st_flash = W25Q_ChipErase(&flash_cs);        // blokkeert tot de chip leeg is
#endif
#if RUN_FLASH_READBACK_TEST
  flash_test_ok = FlashLogger_TestReadBack(&flash_cs);
#endif
  st_log_init = FlashLogger_Init(&flash_cs);   // gaat verder na de laatst beschreven pagina

#if TEST_MODE
  // Testmodus: alleen het gebruikte deel wissen (snel), zodat de dump enkel deze test bevat
  for (uint32_t a = 0; a < flashLogger.write_addr; a += 4096UL)
    W25Q_EraseSector(a, &flash_cs);
  st_log_init = FlashLogger_Init(&flash_cs);   // nu vanaf adres 0
#endif
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

	        uint32_t mag2 = (uint32_t)((int32_t)acc_x * acc_x)
	                      + (uint32_t)((int32_t)acc_y * acc_y)
	                      + (uint32_t)((int32_t)acc_z * acc_z);

	        if (TEST_MODE) {
	          // testmodus: detectoren uitgeschakeld (zie hieronder)
	        } else if (!launched) {
	          uint32_t thr2 = (uint32_t)LAUNCH_THR * LAUNCH_THR;

	          if (mag2 > thr2) {
	            if (above_since == 0) above_since = now;
	            if (now - above_since >= LAUNCH_MS) {
	              start_flight(now);
	            }
	          } else {
	            above_since = 0;
	          }
	        } else if (!landed) {
	          // 1b. Landingsdetectie: |a| binnen 0,8-1,2 g en nauwelijks variërend
	          uint32_t lo2 = (uint32_t)LANDED_G_LOW * LANDED_G_LOW;
	          uint32_t hi2 = (uint32_t)LANDED_G_HIGH * LANDED_G_HIGH;
	          if (mag2 >= lo2 && mag2 <= hi2) {
	            if (still_since == 0) {
	              still_since = now; still_min2 = still_max2 = mag2;
	            } else {
	              if (mag2 < still_min2) still_min2 = mag2;
	              if (mag2 > still_max2) still_max2 = mag2;
	              if (still_max2 - still_min2 > LANDED_SPREAD2) {   // beweegt nog: venster opnieuw
	                still_since = now; still_min2 = still_max2 = mag2;
	              }
	            }
	            if (now - launch_time >= LANDED_MIN_FLIGHT_MS &&
	                now - still_since >= LANDED_STILL_MS)
	              landed = 1;
	          } else {
	            still_since = 0;
	          }
	          if (now - launch_time >= LANDED_MAX_FLIGHT_MS)
	            landed = 1;
	        }
	      }
	    }

	    // 1a. Testmodus: meteen "lancering", na TEST_LOG_MS "geland"
	    if (TEST_MODE) {
	      if (!launched && now >= TEST_START_DELAY_MS) start_flight(now);
	      if (launched && !landed && now - launch_time >= TEST_LOG_MS) landed = 1;
	    }

	    // 1c. Na de landing: volledige flash naar de ESP32 sturen (blokkeert)
	    if (landed && !dump_done && (t_dump == 0 || now - t_dump >= DUMP_RETRY_MS)) {
	      led_set(GPIO_PIN_2, 1);
	      uint32_t n = 0;
	      st_dump = FlashDump_SendAll(&huart1, &flash_cs, &n);
	      dump_records = n;
	      if (st_dump == HAL_OK) dump_done = 1;
	      t_dump = HAL_GetTick();
	      if (t_dump == 0) t_dump = 1;
	    }

	    // 2. Druk, elke 50 ms
	    if (!landed && now - t_pres >= 50) {   // na de landing niet meer loggen
	      t_pres = now;
	      ms5837_measure(&ps1);
	      ms5837_measure(&ps2);

	      // 2b. Naar flash, pas vanaf de lancering (= camerastart): één record per druksensor
	      if (launched) {
	        Flash_Payload rec = { .acc_x = acc_x, .acc_y = acc_y, .acc_z = acc_z };
	        rec.temp = ps1.temp; rec.pres = ps1.pres;
	        st_log = FlashLogger_LogPayload(&rec, 1, now);
	        rec.temp = ps2.temp; rec.pres = ps2.pres;
	        st_log = FlashLogger_LogPayload(&rec, 2, now);
	      }
	    }

	    // 2c. Vóór de lancering: sensoren die bij het opstarten niet antwoordden elke seconde
	    //     opnieuw initialiseren (blokkeert kort, daarom niet meer tijdens de vlucht)
	    if (!launched && now - t_retry >= 1000) {
	      t_retry = now;
	      if (bno_id != 0xA0) bno055_init();
	      if (!ps1.crc_ok)    ms5837_init(&ps1);
	      if (!ps2.crc_ok)    ms5837_init(&ps2);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
