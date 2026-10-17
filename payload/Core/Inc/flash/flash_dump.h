/*
 * flash_dump.h
 *
 * After landing: read every record from the flash and send it over UART
 * to the XIAO ESP32-S3, which writes them to CSV files on its SD card.
 *
 * Frame (both directions):
 *   0xA5 0x5A | type (1) | len (1) | payload (len bytes) | crc16 (2, little-endian)
 *   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload.
 *
 * STM32 -> ESP32 (payload always starts with a 16-bit sequence number):
 *   DUMP_BEGIN 0x01 : seq(2) log_bytes(4)
 *   DUMP_DATA  0x02 : seq(2) count(1) count x record(19)
 *                     record = sensor(1) timestamp_ms(4) temp(4) pres(4) acc_x(2) acc_y(2) acc_z(2)
 *   DUMP_END   0x03 : seq(2) total_records(4)
 * ESP32 -> STM32:
 *   ACK        0x10 : seq(2) status(1)   status 0 = written OK, otherwise NAK
 *
 * Every packet is resent until the matching ACK arrives. The ESP32 ignores
 * a sequence number it has already handled (but ACKs it again), so a lost
 * ACK never creates duplicate rows.
 * All integers little-endian.
 */
#ifndef INC_FLASH_FLASH_DUMP_H_
#define INC_FLASH_FLASH_DUMP_H_

#include "stm32c0xx_hal.h"
#include "flash/flash_helpers.h"

#define DUMP_SOF0             0xA5
#define DUMP_SOF1             0x5A
#define DUMP_PKT_BEGIN        0x01
#define DUMP_PKT_DATA         0x02
#define DUMP_PKT_END          0x03
#define DUMP_PKT_ACK          0x10

#define DUMP_REC_SIZE         19
#define DUMP_RECS_PER_PKT     12          // 3 + 12*19 = 231 bytes payload
#define DUMP_ACK_TIMEOUT_MS   500
#define DUMP_MAX_RETRIES      10

uint16_t FlashDump_CRC16(const uint8_t *data, uint16_t len, uint16_t crc);

// Blocks until the whole log is sent (minutes for a big log).
// Returns HAL_OK when every packet was acknowledged.
HAL_StatusTypeDef FlashDump_SendAll(UART_HandleTypeDef *huart,
                                    const CS_Pin_t *cs,
                                    uint32_t *records_sent);

#endif /* INC_FLASH_FLASH_DUMP_H_ */
