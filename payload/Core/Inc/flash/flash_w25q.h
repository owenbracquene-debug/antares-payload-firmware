/*
 * flash_w25q.h
 *
 *  Created on: Feb 16, 2026
 *      Author: cedri
 */
// flash_w25q.h
#include "stm32c0xx_hal.h"
#include <stdint.h>
#include "cs_struct.h"
#ifndef W25QXX_H_
#define W25QXX_H_

// Commands (most common ones)
#define W25Q_CMD_READ_ID    0x9F
#define W25Q_CMD_READ       0x03
#define W25Q_CMD_WRITE_EN   0x06
#define W25Q_CMD_PAGE_PROG  0x02
#define W25Q_CMD_SECTOR_ER  0x20    // 4KB sector
#define W25Q_CMD_READ_SR1   0x05
#define W25Q_CMD_READ_SR2   0x35
#define W25Q_CMD_WRITE_ENABLE     0x06
#define W25Q_CMD_SECTOR_ERASE_4K  0x20    // 4KB sector erase
#define W25Q_CMD_PAGE_PROGRAM     0x02    // 256-byte page program
#define W25Q_CMD_READ_DATA        0x03    // standard read

void     W25Q_CS_Low(const CS_Pin_t *cs);
void     W25Q_CS_High(const CS_Pin_t *cs);

uint32_t W25Q_ReadJEDECID(const CS_Pin_t *cs);
uint8_t  W25Q_ReadStatusReg1(const CS_Pin_t *cs);

void     W25Q_WriteEnable(const CS_Pin_t *cs);
void     W25Q_WaitForWriteEnd(const CS_Pin_t *cs);

void     W25Q_ReadData(uint32_t addr,
                       uint8_t *buffer,
                       uint16_t len,
                       const CS_Pin_t *cs);

HAL_StatusTypeDef W25Q_PageProgram(uint32_t addr,
                                  const uint8_t *data,
                                  uint16_t len,
                                  const CS_Pin_t *cs);

HAL_StatusTypeDef W25Q_EraseSector(uint32_t addr,
                                  const CS_Pin_t *cs);

HAL_StatusTypeDef W25Q_ChipErase(const CS_Pin_t *cs);

int test_W25Q_FullFlash(const CS_Pin_t *cs);

// Add more functions later: Read, Write_Page, Erase_Sector, Erase_Chip...

#endif
