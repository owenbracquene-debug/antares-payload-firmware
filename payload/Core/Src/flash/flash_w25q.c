/*
 * flash_w25q.c
 *
 *  Created on: Feb 16, 2026
 *      Author: cedri
 */


// w25qxx.c
#include "flash/flash_w25q.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

void W25Q_CS_Low(const CS_Pin_t *Cs_Info) {
    HAL_GPIO_WritePin(Cs_Info->port, Cs_Info->pin, GPIO_PIN_RESET);
}

void W25Q_CS_High(const CS_Pin_t *Cs_Info) {
    HAL_GPIO_WritePin(Cs_Info->port, Cs_Info->pin, GPIO_PIN_SET);
}

uint8_t W25Q_SPI_SendReceive(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

void W25Q_SPI_Transmit(uint8_t *pData, uint16_t Size) {
    HAL_SPI_Transmit(&hspi1, pData, Size, 100);
}

void W25Q_SPI_Receive(uint8_t *pData, uint16_t Size) {
    HAL_SPI_Receive(&hspi1, pData, Size, 100);
}

// Read Manufacturer & Device ID (0xEF 0x40 0x18 for W25Q128 for example)
uint32_t W25Q_ReadJEDECID(const CS_Pin_t *Cs_Info) {
    uint8_t tx[4] = {W25Q_CMD_READ_ID, 0xFF, 0xFF, 0xFF};
    uint8_t rx[4];

    W25Q_CS_Low(Cs_Info);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 100);
    W25Q_CS_High(Cs_Info);

    return (rx[1] << 16) | (rx[2] << 8) | rx[3];
}

uint8_t W25Q_ReadStatusReg1(const CS_Pin_t *Cs_Info) {
    uint8_t tx = W25Q_CMD_READ_SR1;
    uint8_t rx;

    W25Q_CS_Low(Cs_Info);
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 50);   // dummy read
    HAL_SPI_Receive(&hspi1, &rx, 1, 50);
    W25Q_CS_High(Cs_Info);

    return rx;
}

void W25Q_WriteEnable(const CS_Pin_t *Cs_Info) {
    uint8_t cmd = W25Q_CMD_WRITE_EN;
    W25Q_CS_Low(Cs_Info);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 50);
    W25Q_CS_High(Cs_Info);
}

void W25Q_WaitForWriteEnd(const CS_Pin_t *Cs_Info) {
    while (W25Q_ReadStatusReg1(Cs_Info) & 0x01) {          // WIP bit
        HAL_Delay(1);                               // or better HAL_GetTick() timeout
    }
}

// ============================================================================
// New helper: send simple command (no data)
// ============================================================================
static void W25Q_SendCommand(uint8_t cmd, const CS_Pin_t *Cs_Info)
{
    W25Q_CS_Low(Cs_Info);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 50);
    W25Q_CS_High(Cs_Info);
}

// ============================================================================
// Erase 4KB sector at given address (must be sector-aligned: 0, 0x1000, 0x2000...)
// ============================================================================
HAL_StatusTypeDef W25Q_EraseSector(uint32_t addr, const CS_Pin_t *Cs_Info)
{
    if (addr & 0xFFF) {
        return HAL_ERROR;
    }

    uint8_t tx[4];
    tx[0] = W25Q_CMD_SECTOR_ERASE_4K;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >>  8) & 0xFF;
    tx[3] = (addr >>  0) & 0xFF;

    W25Q_WriteEnable(Cs_Info);

    W25Q_CS_Low(Cs_Info);
    HAL_SPI_Transmit(&hspi1, tx, 4, 200);
    W25Q_CS_High(Cs_Info);

    W25Q_WaitForWriteEnd(Cs_Info);

    // Optional: check if WIP cleared and WEL cleared
    uint8_t sr = W25Q_ReadStatusReg1(Cs_Info);
    if (sr & 0x03) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// ============================================================================
// Write up to 256 bytes (one page) – addr must be page-aligned for full page
// ============================================================================
HAL_StatusTypeDef W25Q_PageProgram(uint32_t addr, const uint8_t *data, uint16_t len, const CS_Pin_t *Cs_Info)
{
    if (len == 0 || len > 256) {
        return HAL_ERROR;
    }

    uint8_t tx[260]; // cmd(1) + addr(3) + data(max 256)
    tx[0] = W25Q_CMD_PAGE_PROGRAM;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >>  8) & 0xFF;
    tx[3] = (addr >>  0) & 0xFF;

    memcpy(&tx[4], data, len);

    W25Q_WriteEnable(Cs_Info);

    W25Q_CS_Low(Cs_Info);
    HAL_SPI_Transmit(&hspi1, tx, 4 + len, 500);
    W25Q_CS_High(Cs_Info);

    W25Q_WaitForWriteEnd(Cs_Info);

    return HAL_OK;
}

// ============================================================================
// Read data from flash
// ============================================================================
void W25Q_ReadData(uint32_t addr, uint8_t *buffer, uint16_t len, const CS_Pin_t *Cs_Info) {
    uint8_t tx[4 + len];  // allocate full buffer
    tx[0] = W25Q_CMD_READ_DATA;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >>  8) & 0xFF;
    tx[3] = (addr >>  0) & 0xFF;
    memset(&tx[4], 0xFF, len);  // dummy clocks

    uint8_t rx[4 + len];

    W25Q_CS_Low(Cs_Info);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4 + len, 300);
    W25Q_CS_High(Cs_Info);

    memcpy(buffer, &rx[4], len);
}

HAL_StatusTypeDef W25Q_ChipErase(const CS_Pin_t *Cs_Info)
{
    W25Q_WriteEnable(Cs_Info);

    uint8_t cmd = 0xC7;  // or 0x60

    W25Q_CS_Low(Cs_Info);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    W25Q_CS_High(Cs_Info);

    W25Q_WaitForWriteEnd(Cs_Info);  // This will take 10–50+ seconds!

    // Optional: check status after
    uint8_t sr1 = W25Q_ReadStatusReg1(Cs_Info);
    if (sr1 & 0x03) {  // WIP or WEL still set?
        return HAL_ERROR;
    }

    return HAL_OK;
}

int test_W25Q_FullFlash(const CS_Pin_t *Cs_Info){
    const uint32_t FLASH_SIZE_BYTES = 32UL * 1024UL * 1024UL;
    const uint32_t PAGE_SIZE        = 256UL;
    uint32_t TOTAL_PAGES      = FLASH_SIZE_BYTES / PAGE_SIZE;  // 131072

    // FOR NOW TEST ONLY 10 pages
    TOTAL_PAGES = 10;

    uint8_t write_buf[PAGE_SIZE];
    uint8_t read_buf[PAGE_SIZE];

    for (int i = 0; i < PAGE_SIZE; i++) {
        write_buf[i] = (uint8_t)(0x55 + i * 3);
    }

    uint32_t start_ms = HAL_GetTick();

    // === One-time full chip erase ===
    if (W25Q_ChipErase(Cs_Info) != HAL_OK) {
        return -1;
    }

    uint32_t erase_ms = HAL_GetTick() - start_ms;
    uint32_t total_errors   = 0;
    uint32_t failed_pages   = 0;

    // Now write/read/verify every page (no more erases needed)
    for (uint32_t page = 0; page < TOTAL_PAGES; page++)
    {
        uint32_t addr = page * PAGE_SIZE;

        if (W25Q_PageProgram(addr, write_buf, PAGE_SIZE, Cs_Info) != HAL_OK) {
            continue;
        }

        W25Q_ReadData(addr, read_buf, PAGE_SIZE, Cs_Info);

        uint32_t wrong_bytes = 0;
        for (int j = 0; j < PAGE_SIZE; j++) {
            if (read_buf[j] != write_buf[j]) wrong_bytes++;
        }

        if (wrong_bytes > 0) {
            failed_pages++;
            total_errors += wrong_bytes;
        }
    }

    uint32_t total_ms = HAL_GetTick() - start_ms;
    uint32_t seconds  = total_ms / 1000UL;
    uint32_t millis   = total_ms % 1000UL;

    if (total_ms > 0) {
        uint32_t kb_per_sec = (FLASH_SIZE_BYTES / 1024UL * 1000UL) / total_ms;
    }

    if (failed_pages == 0) {
        return 0;
    } else {
        return -1;
    }
}
