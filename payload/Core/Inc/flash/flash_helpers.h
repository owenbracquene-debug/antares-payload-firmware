/*
 * flash_helpers.h
 *
 *  Created on: Jun 15, 2026
 *      Author: cedri
 *
 * Payload flash logger (W25Q256JV on SPI1, CS = PA4).
 *
 * One record per pressure sensor per measurement:
 *   [type 1B][sensor 1B][timestamp 4B][temp 4B][pres 4B][acc_x 2B][acc_y 2B][acc_z 2B]
 *   = 20 bytes, little-endian, records never cross a 256-byte page.
 *   Unused bytes at the end of a page stay 0xFF.
 */

#ifndef INC_HELPERS_FLASH_HELPERS_H_
#define INC_HELPERS_FLASH_HELPERS_H_

#include <stdint.h>
#include <stdbool.h>
#include "flash/flash_w25q.h"
#include "flash/cs_struct.h"

typedef struct {
    int32_t temp;          // 0,01 °C
    int32_t pres;          // Pa
    int16_t acc_x;         // 0,01 m/s²
    int16_t acc_y;
    int16_t acc_z;
} Flash_Payload;

typedef struct {
    const CS_Pin_t *cs;
    uint32_t write_addr;   // next page to program
    uint32_t end_addr;     // logging stops here
    uint8_t  page_buf[256];
    uint16_t page_offset;
    uint8_t  full;         // 1 = log area full, new records are dropped
} FlashLogger_t;

#define FLASH_RECORD_PAYLOAD   0x01
#define FLASH_RECORD_SIZE      20
#define FLASH_PAGE_SIZE        256UL

// The driver uses 3-byte addresses, so only the first 16 MB is reachable.
#define FLASH_ADDR_LIMIT       (16UL * 1024UL * 1024UL)
#define FLASH_TEST_SECTOR      (FLASH_ADDR_LIMIT - 4096UL)   // reserved for the readout test
#define FLASH_LOG_END          FLASH_TEST_SECTOR

extern FlashLogger_t flashLogger;

// Checks the JEDEC ID and continues after the last written page.
// Never erases: data from before a reboot is kept.
HAL_StatusTypeDef FlashLogger_Init(const CS_Pin_t *cs);

// sensor: 1 = MS5837 on I2C1, 2 = MS5837 on I2C2
HAL_StatusTypeDef FlashLogger_LogPayload(const Flash_Payload *p,
                                         uint8_t sensor,
                                         uint32_t timestamp);

// Writes the partly filled page (rest stays 0xFF).
HAL_StatusTypeDef FlashLogger_Flush(void);

// Walks the log. Start with *addr = 0; returns false at the end of the log.
bool FlashLogger_ReadNext(const CS_Pin_t *cs,
                          uint32_t *addr,
                          uint8_t *sensor,
                          uint32_t *timestamp,
                          Flash_Payload *out);

// Readout tester: erases the reserved test sector, writes known records,
// reads them back and compares. Does not touch logged flight data.
bool FlashLogger_TestReadBack(const CS_Pin_t *cs);

#endif /* INC_HELPERS_FLASH_HELPERS_H_ */
