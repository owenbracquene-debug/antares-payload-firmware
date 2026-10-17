/*
 * flash_helpers.c
 *
 * Payload flash logger, see flash_helpers.h for the record layout.
 */
#include <string.h>
#include "flash/flash_helpers.h"

FlashLogger_t flashLogger;

#define TEST_NUM_RECORDS  100   // 100 x 20 B = 9 pages, fits in the 4 KB test sector

/* ------------------------------------------------------------------------ */
/* Serialisation                                                            */
/* ------------------------------------------------------------------------ */

static void SerializePayload(const Flash_Payload *p, uint8_t sensor,
                             uint32_t timestamp, uint8_t *buf)
{
    uint32_t idx = 0;
    buf[idx++] = FLASH_RECORD_PAYLOAD;
    buf[idx++] = sensor;
    memcpy(&buf[idx], &timestamp, 4); idx += 4;
    memcpy(&buf[idx], &p->temp,   4); idx += 4;
    memcpy(&buf[idx], &p->pres,   4); idx += 4;
    memcpy(&buf[idx], &p->acc_x,  2); idx += 2;
    memcpy(&buf[idx], &p->acc_y,  2); idx += 2;
    memcpy(&buf[idx], &p->acc_z,  2); idx += 2;
}

static bool DeserializePayload(const uint8_t *buf, uint8_t *sensor,
                               uint32_t *timestamp, Flash_Payload *p)
{
    uint32_t idx = 0;
    if (buf[idx++] != FLASH_RECORD_PAYLOAD)
        return false;
    *sensor = buf[idx++];
    memcpy(timestamp, &buf[idx], 4); idx += 4;
    memcpy(&p->temp,  &buf[idx], 4); idx += 4;
    memcpy(&p->pres,  &buf[idx], 4); idx += 4;
    memcpy(&p->acc_x, &buf[idx], 2); idx += 2;
    memcpy(&p->acc_y, &buf[idx], 2); idx += 2;
    memcpy(&p->acc_z, &buf[idx], 2); idx += 2;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Page buffer                                                              */
/* ------------------------------------------------------------------------ */

static void Logger_Reset(FlashLogger_t *lg, const CS_Pin_t *cs,
                         uint32_t start, uint32_t end)
{
    memset(lg, 0, sizeof(*lg));
    lg->cs = cs;
    lg->write_addr = start;
    lg->end_addr = end;
    memset(lg->page_buf, 0xFF, sizeof(lg->page_buf));
}

static HAL_StatusTypeDef Logger_FlushPage(FlashLogger_t *lg)
{
    if (lg->page_offset == 0)
        return HAL_OK;
    if (lg->write_addr + FLASH_PAGE_SIZE > lg->end_addr) {
        lg->full = 1;
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = W25Q_PageProgram(lg->write_addr, lg->page_buf,
                                             FLASH_PAGE_SIZE, lg->cs);
    if (ret != HAL_OK)
        return ret;

    lg->write_addr += FLASH_PAGE_SIZE;
    lg->page_offset = 0;
    memset(lg->page_buf, 0xFF, sizeof(lg->page_buf));
    if (lg->write_addr + FLASH_PAGE_SIZE > lg->end_addr)
        lg->full = 1;   // no room for another page
    return HAL_OK;
}

static HAL_StatusTypeDef Logger_Append(FlashLogger_t *lg, const Flash_Payload *p,
                                       uint8_t sensor, uint32_t timestamp)
{
    if (lg->cs == NULL || lg->full)
        return HAL_ERROR;

    if ((uint32_t)lg->page_offset + FLASH_RECORD_SIZE > FLASH_PAGE_SIZE) {
        HAL_StatusTypeDef ret = Logger_FlushPage(lg);
        if (ret != HAL_OK)
            return ret;
        if (lg->full)
            return HAL_ERROR;
    }

    SerializePayload(p, sensor, timestamp, &lg->page_buf[lg->page_offset]);
    lg->page_offset += FLASH_RECORD_SIZE;
    return HAL_OK;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

static bool PageIsEmpty(uint32_t addr, const CS_Pin_t *cs)
{
    uint8_t b = 0;
    W25Q_ReadData(addr, &b, 1, cs);
    return b == 0xFF;   // a written page always starts with a record type byte
}

HAL_StatusTypeDef FlashLogger_Init(const CS_Pin_t *cs)
{
    Logger_Reset(&flashLogger, NULL, 0, FLASH_LOG_END);

    uint32_t id = W25Q_ReadJEDECID(cs);
    if ((id >> 16) != 0xEF)          // not a Winbond chip -> wiring/CS problem
        return HAL_ERROR;

    // Binary search for the first empty page, so a reboot appends
    // instead of overwriting (programming over data corrupts it).
    uint32_t lo = 0, hi = FLASH_LOG_END / FLASH_PAGE_SIZE;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (PageIsEmpty(mid * FLASH_PAGE_SIZE, cs))
            hi = mid;
        else
            lo = mid + 1;
    }

    flashLogger.cs = cs;
    flashLogger.write_addr = lo * FLASH_PAGE_SIZE;
    if (flashLogger.write_addr >= FLASH_LOG_END)
        flashLogger.full = 1;

    return HAL_OK;
}

HAL_StatusTypeDef FlashLogger_LogPayload(const Flash_Payload *p,
                                         uint8_t sensor,
                                         uint32_t timestamp)
{
    return Logger_Append(&flashLogger, p, sensor, timestamp);
}

HAL_StatusTypeDef FlashLogger_Flush(void)
{
    return Logger_FlushPage(&flashLogger);
}

bool FlashLogger_ReadNext(const CS_Pin_t *cs,
                          uint32_t *addr,
                          uint8_t *sensor,
                          uint32_t *timestamp,
                          Flash_Payload *out)
{
    if (cs == NULL || addr == NULL || sensor == NULL ||
        timestamp == NULL || out == NULL)
        return false;

    while (*addr < FLASH_ADDR_LIMIT) {
        uint32_t offset = *addr % FLASH_PAGE_SIZE;

        // Record would cross the page boundary -> rest of page is padding
        if (offset + FLASH_RECORD_SIZE > FLASH_PAGE_SIZE) {
            *addr += FLASH_PAGE_SIZE - offset;
            continue;
        }

        uint8_t rec[FLASH_RECORD_SIZE];
        W25Q_ReadData(*addr, rec, FLASH_RECORD_SIZE, cs);

        if (rec[0] == 0xFF) {
            if (offset == 0)
                return false;                       // empty page: end of log
            *addr += FLASH_PAGE_SIZE - offset;      // padding: next page
            continue;
        }

        if (!DeserializePayload(rec, sensor, timestamp, out)) {
            *addr += FLASH_PAGE_SIZE - offset;      // corrupt: skip page
            continue;
        }

        *addr += FLASH_RECORD_SIZE;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* Readout tester                                                           */
/* ------------------------------------------------------------------------ */

static void TestRecord(int i, Flash_Payload *p, uint8_t *sensor, uint32_t *t)
{
    *sensor  = (uint8_t)(1 + (i % 2));
    *t       = 10000U + (uint32_t)i * 50U;
    p->temp  = 2150 + i;              // 21,50 °C + i
    p->pres  = 101325 - i * 7;        // Pa
    p->acc_x = (int16_t)(-1500 + i * 3);
    p->acc_y = (int16_t)(i * 11);
    p->acc_z = (int16_t)(981 - i);
}

bool FlashLogger_TestReadBack(const CS_Pin_t *cs)
{
    static FlashLogger_t test;   // static: keeps the 256 B buffer off the stack

    if (W25Q_EraseSector(FLASH_TEST_SECTOR, cs) != HAL_OK)
        return false;

    Logger_Reset(&test, cs, FLASH_TEST_SECTOR, FLASH_TEST_SECTOR + 4096UL);

    for (int i = 0; i < TEST_NUM_RECORDS; i++) {
        Flash_Payload p; uint8_t s; uint32_t t;
        TestRecord(i, &p, &s, &t);
        if (Logger_Append(&test, &p, s, t) != HAL_OK)
            return false;
    }
    if (Logger_FlushPage(&test) != HAL_OK)
        return false;

    uint32_t addr = FLASH_TEST_SECTOR;
    for (int i = 0; i < TEST_NUM_RECORDS; i++) {
        Flash_Payload exp, got; uint8_t es, gs; uint32_t et, gt;
        TestRecord(i, &exp, &es, &et);
        if (!FlashLogger_ReadNext(cs, &addr, &gs, &gt, &got))
            return false;
        if (gs != es || gt != et || got.temp != exp.temp || got.pres != exp.pres ||
            got.acc_x != exp.acc_x || got.acc_y != exp.acc_y || got.acc_z != exp.acc_z)
            return false;
    }

    // Nothing may follow the last record
    Flash_Payload dummy; uint8_t ds; uint32_t dt;
    if (FlashLogger_ReadNext(cs, &addr, &ds, &dt, &dummy))
        return false;

    W25Q_EraseSector(FLASH_TEST_SECTOR, cs);   // leave the test sector clean
    return true;
}
