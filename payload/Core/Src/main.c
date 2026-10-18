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
#define ERASE_FLASH_ON_BOOT      0   // 1 = wist de VOLLEDIGE flash bij elke start (80-400 s). Alleen eenmalig vóór de vlucht!

// WERKING: vanaf het inschakelen camera aan + elke 50 ms loggen naar de flash
// (+ live naar de XIAO voor de SD-kaart). De lanceerdetectie zet ENKEL LED2 aan
// en onthoudt launch_time; ze start of stopt niets.
#define CAM_START_DELAY_MS       5000UL  // eerst de XIAO laten opstarten (camera + SD) voor START
#define CAM_RETRY_MS            10000UL  // START opnieuw proberen zolang de camera niet bevestigde (vóór de lancering)
#define LIVE_TO_XIAO                1    // 1 = elke meting ook live via UART naar de XIAO (CSV op de SD)

// HERSTELMODUS na de vlucht: 1 = bij het opstarten de volledige flash naar de XIAO sturen
// (zelfde dump als vroeger), daarna niets meer doen. Niet loggen, geen camera.
// LED2 aan = dump gelukt, LED2 snel knipperend = dump mislukt.  !! op 0 voor de vlucht !!
#define DUMP_ON_BOOT                0
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
// Busherstel: aantal mislukte transfers na elkaar (zie 2c in de hoofdlus)
uint8_t i2c1_fail = 0, ps2_fail = 0;
volatile uint16_t i2c1_recoveries = 0, i2c2_recoveries = 0;
uint8_t flash_id[3];
volatile HAL_StatusTypeDef st_flash;
const CS_Pin_t flash_cs = { GPIOA, GPIO_PIN_4 };   // W25Q256JV /CS op PA4
volatile HAL_StatusTypeDef st_log_init, st_log;
volatile uint8_t flash_test_ok = 0;
volatile HAL_StatusTypeDef st_dump = HAL_BUSY;   // alleen in herstelmodus (DUMP_ON_BOOT)
volatile uint32_t dump_records = 0;
volatile uint32_t launch_time = 0;                // ms sinds opstarten, 0 = geen lancering gezien
volatile uint8_t  cam_attempts = 0;               // aantal START-pogingen
uint32_t t_cam = 0;
volatile HAL_StatusTypeDef st_live = HAL_OK;      // laatste live-zending naar de XIAO
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
// Vastgelopen I2C-bus vrijmaken: I2C uit, 9 klokpulsen op SCL (een slave die SDA vasthoudt
// laat dan los), een STOP-conditie, en daarna de I2C-module opnieuw initialiseren.
static void i2c_delay(void)
{
  for (volatile int d = 0; d < 20; d++) { }
}

static void i2c_bus_recover(I2C_HandleTypeDef *hi2c,
                            GPIO_TypeDef *sda_port, uint16_t sda_pin,
                            GPIO_TypeDef *scl_port, uint16_t scl_pin)
{
  HAL_I2C_DeInit(hi2c);                          // pinnen terug naar GPIO

  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);
  g.Pin = sda_pin; HAL_GPIO_Init(sda_port, &g);
  g.Pin = scl_pin; HAL_GPIO_Init(scl_port, &g);
  i2c_delay();

  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET); i2c_delay();
    HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);   i2c_delay();
  }
  // STOP: SDA laag -> hoog terwijl SCL hoog is
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET); i2c_delay();
  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);   i2c_delay();
  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET);   i2c_delay();

  if (hi2c->Instance == I2C1) MX_I2C1_Init();    // zet de pinnen terug als I2C
  else                        MX_I2C2_Init();
}

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

  // LED2: aan = lancering gedetecteerd.
  //       Vóór de lancering snel knipperen = camera heeft START (nog) niet bevestigd.
  if (launched)                         led_set(GPIO_PIN_2, 1);
  else if (cam_attempts > 0 && !ack_ok) led_set(GPIO_PIN_2, (now / 100) % 2);
  else                                  led_set(GPIO_PIN_2, 0);
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

// Camera starten: START sturen en wachten op "OK START" (max. tries x ~400 ms)
static void camera_start(int tries)
{
  const char msg[] = "START\n";
  for (int i = 0; i < tries && !ack_ok; i++) {
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

#if DUMP_ON_BOOT
  // Herstelmodus: volledige flash naar de XIAO (CSV op de SD), daarna stoppen.
  HAL_Delay(CAM_START_DELAY_MS);               // XIAO laten opstarten
  {
    uint32_t n = 0;
    for (int i = 0; i < 3 && st_dump != HAL_OK; i++)
      st_dump = FlashDump_SendAll(&huart1, &flash_cs, &n);
    dump_records = n;
  }
  while (1) {
    uint32_t t = HAL_GetTick();
    led_set(GPIO_PIN_0, (t / 500) % 2);
    led_set(GPIO_PIN_1, 0);
    led_set(GPIO_PIN_2, st_dump == HAL_OK ? 1 : (t / 100) % 2);
  }
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
	      if (st_acc == HAL_OK) i2c1_fail = 0;
	      else if (i2c1_fail < 255) i2c1_fail++;
	      if (st_acc == HAL_OK) {
	        acc_x = (int16_t)((a[1] << 8) | a[0]);
	        acc_y = (int16_t)((a[3] << 8) | a[2]);
	        acc_z = (int16_t)((a[5] << 8) | a[4]);

	        uint32_t mag2 = (uint32_t)((int32_t)acc_x * acc_x)
	                      + (uint32_t)((int32_t)acc_y * acc_y)
	                      + (uint32_t)((int32_t)acc_z * acc_z);

	        // Lanceerdetectie: |a| > 3 g gedurende 100 ms -> enkel LED2 + launch_time
	        if (!launched) {
	          uint32_t thr2 = (uint32_t)LAUNCH_THR * LAUNCH_THR;

	          if (mag2 > thr2) {
	            if (above_since == 0) above_since = now;
	            if (now - above_since >= LAUNCH_MS) {
	              launched = 1;
	              launch_time = now;
	            }
	          } else {
	            above_since = 0;
	          }
	        }
	      }
	    }

	    // 1b. Camera starten vanaf het inschakelen (los van de lanceerdetectie).
	    //     Eerste keer tot 10 pogingen (max. ~4 s, op het platform).
	    //     Daarna 1 poging per CAM_RETRY_MS, maar niet meer na een lancering
	    //     (blokkeert ~0,4 s en zou dan vluchtdata kosten).
	    if (!ack_ok && now >= CAM_START_DELAY_MS) {
	      if (cam_attempts == 0) {
	        camera_start(10);
	        cam_attempts = 1;
	        t_cam = HAL_GetTick();
	      } else if (!launched && now - t_cam >= CAM_RETRY_MS && cam_attempts < 255) {
	        camera_start(1);
	        cam_attempts++;
	        t_cam = HAL_GetTick();
	      }
	    }

	    // 2. Druk, elke 50 ms. Vanaf het inschakelen naar de flash (+ live naar de XIAO)
	    if (now - t_pres >= 50) {
	      t_pres = now;
	      ms5837_measure(&ps1);
	      ms5837_measure(&ps2);
	      if (ps2.st == HAL_OK) ps2_fail = 0;
	      else if (ps2_fail < 255) ps2_fail++;

	      // 2b. Eén record per druksensor
	      Flash_Payload rec1 = { .acc_x = acc_x, .acc_y = acc_y, .acc_z = acc_z };
	      Flash_Payload rec2 = rec1;
	      rec1.temp = ps1.temp; rec1.pres = ps1.pres;
	      rec2.temp = ps2.temp; rec2.pres = ps2.pres;
	      st_log = FlashLogger_LogPayload(&rec1, 1, now);
	      st_log = FlashLogger_LogPayload(&rec2, 2, now);
	      if (LIVE_TO_XIAO)
	        st_live = FlashDump_SendLive(&huart1, &rec1, &rec2, now);
	    }

	    // 2c. Herstel, max. 1x per seconde:
	    //     - bus vastgelopen (5x na elkaar BNO055 niet gelezen = 50 ms, of 3x MS5837 bus 2)
	    //       -> bus vrijmaken + sensoren op die bus opnieuw initialiseren
	    //     - losse sensor die niet antwoordt -> alleen die sensor opnieuw initialiseren
	    if (now - t_retry >= 1000) {
	      t_retry = now;
	      if (i2c1_fail >= 5) {
	        i2c_bus_recover(&hi2c1, GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_8);    // SDA PB7, SCL PB8
	        i2c1_recoveries++;
	        bno055_init();
	        ms5837_init(&ps1);
	        i2c1_fail = 0;
	      } else {
	        if (bno_id != 0xA0)                   bno055_init();
	        if (!ps1.crc_ok || ps1.st != HAL_OK)  ms5837_init(&ps1);
	      }
	      if (ps2_fail >= 3) {
	        i2c_bus_recover(&hi2c2, GPIOA, GPIO_PIN_10, GPIOA, GPIO_PIN_11);  // SDA PA10, SCL PA11
	        i2c2_recoveries++;
	        ms5837_init(&ps2);
	        ps2_fail = 0;
	      } else if (!ps2.crc_ok) {
	        ms5837_init(&ps2);
	      }
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
  hi2c1.Init.Timing = 0x00402D41;
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
