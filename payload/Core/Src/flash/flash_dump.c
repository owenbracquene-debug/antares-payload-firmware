/*
 * flash_dump.c
 *
 * Sends the flash log to the ESP32 over UART, see flash_dump.h for the protocol.
 */
#include <string.h>
#include "flash/flash_dump.h"

static uint8_t  tx_buf[4 + 255 + 2];
static uint16_t dump_seq;
static uint8_t  seq_initialised = 0;

uint16_t FlashDump_CRC16(const uint8_t *data, uint16_t len, uint16_t crc)
{
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put_u16(uint8_t *b, uint16_t v) { b[0] = v & 0xFF; b[1] = v >> 8; }
static void put_u32(uint8_t *b, uint32_t v) { for (int i = 0; i < 4; i++) b[i] = (v >> (8 * i)) & 0xFF; }

static void uart_flush_rx(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

// Waits for ACK(seq). Returns 1 = ACK OK, 0 = timeout, -1 = NAK.
static int wait_ack(UART_HandleTypeDef *huart, uint16_t seq, uint32_t timeout_ms)
{
    uint8_t frame[4 + 3 + 2];
    uint8_t n = 0;
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < timeout_ms) {
        uint8_t b;
        if (HAL_UART_Receive(huart, &b, 1, 10) != HAL_OK)
            continue;

        if (n == 0 && b != DUMP_SOF0) continue;
        if (n == 1 && b != DUMP_SOF1) { n = (b == DUMP_SOF0) ? 1 : 0; continue; }
        frame[n++] = b;
        if (n == 4 && (frame[2] != DUMP_PKT_ACK || frame[3] != 3)) { n = 0; continue; }
        if (n < sizeof(frame)) continue;

        n = 0;
        uint16_t crc = FlashDump_CRC16(&frame[2], 2 + 3, 0xFFFF);
        if ((frame[7] | (frame[8] << 8)) != crc) continue;
        uint16_t ack_seq = frame[4] | (frame[5] << 8);
        if (ack_seq != seq) continue;           // stale ACK of an earlier packet
        return frame[6] == 0 ? 1 : -1;
    }
    return 0;
}

// Sends one packet and retries until it is acknowledged.
static HAL_StatusTypeDef send_reliable(UART_HandleTypeDef *huart, uint8_t type,
                                       const uint8_t *payload, uint8_t len)
{
    uint16_t seq = payload[0] | (payload[1] << 8);

    tx_buf[0] = DUMP_SOF0;
    tx_buf[1] = DUMP_SOF1;
    tx_buf[2] = type;
    tx_buf[3] = len;
    memcpy(&tx_buf[4], payload, len);
    put_u16(&tx_buf[4 + len], FlashDump_CRC16(&tx_buf[2], (uint16_t)(2 + len), 0xFFFF));

    for (int attempt = 0; attempt < DUMP_MAX_RETRIES; attempt++) {
        uart_flush_rx(huart);
        if (HAL_UART_Transmit(huart, tx_buf, (uint16_t)(4 + len + 2), 100) != HAL_OK)
            continue;
        if (wait_ack(huart, seq, DUMP_ACK_TIMEOUT_MS) == 1)
            return HAL_OK;
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef FlashDump_SendAll(UART_HandleTypeDef *huart,
                                    const CS_Pin_t *cs,
                                    uint32_t *records_sent)
{
    static uint8_t pl[3 + DUMP_RECS_PER_PKT * DUMP_REC_SIZE];
    uint32_t total = 0;
    HAL_StatusTypeDef st;

    if (records_sent) *records_sent = 0;
    if (!seq_initialised) {                   // keeps counting across dump retries,
        dump_seq = (uint16_t)HAL_GetTick();   // so the ESP never mistakes a new
        seq_initialised = 1;                  // packet for a duplicate
    }

    FlashLogger_Flush();                      // last partly filled page

    // BEGIN
    put_u16(&pl[0], dump_seq);
    put_u32(&pl[2], flashLogger.write_addr);
    if ((st = send_reliable(huart, DUMP_PKT_BEGIN, pl, 6)) != HAL_OK) return st;
    dump_seq++;

    // DATA
    uint32_t addr = 0;
    uint8_t  count = 0;
    bool more = true;
    while (more) {
        Flash_Payload p; uint8_t sensor; uint32_t t;
        more = FlashLogger_ReadNext(cs, &addr, &sensor, &t, &p);
        if (more) {
            uint8_t *r = &pl[3 + count * DUMP_REC_SIZE];
            r[0] = sensor;
            put_u32(&r[1],  t);
            put_u32(&r[5],  (uint32_t)p.temp);
            put_u32(&r[9],  (uint32_t)p.pres);
            put_u16(&r[13], (uint16_t)p.acc_x);
            put_u16(&r[15], (uint16_t)p.acc_y);
            put_u16(&r[17], (uint16_t)p.acc_z);
            count++;
        }
        if (count == DUMP_RECS_PER_PKT || (!more && count > 0)) {
            put_u16(&pl[0], dump_seq);
            pl[2] = count;
            st = send_reliable(huart, DUMP_PKT_DATA, pl, (uint8_t)(3 + count * DUMP_REC_SIZE));
            if (st != HAL_OK) return st;
            dump_seq++;
            total += count;
            if (records_sent) *records_sent = total;
            count = 0;
        }
    }

    // END
    put_u16(&pl[0], dump_seq);
    put_u32(&pl[2], total);
    if ((st = send_reliable(huart, DUMP_PKT_END, pl, 6)) != HAL_OK) return st;
    dump_seq++;

    return HAL_OK;
}
