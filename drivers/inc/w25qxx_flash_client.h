/*
 * w25qxx_flash_client.h
 *
 * Multi-Client W25Q64FV Flash Memory Driver - Client API
 *
 * This header provides a transparent client API for W25Q64FV flash operations.
 * Multiple clients can use these APIs simultaneously without knowledge of resource sharing.
 *
 * The API provides blocking-style functions that internally use the async resource manager
 * for efficient multi-client operation handling.
 *
 *  Created on: 01-Oct-2025
 *      Author: tabrez
 */

#ifndef W25QXX_FLASH_CLIENT_H_
#define W25QXX_FLASH_CLIENT_H_

#include "tm4c123x_ssi_driver.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Flash Client Handle Structure
 *
 * Each client must create and maintain their own handle for flash operations.
 * This provides client isolation and maintains per-client operation context.
 */
typedef struct {
    SSI_Handle_t *pSSIHandle;     /* Pointer to shared SSI flash interface */
    uint8_t clientId;             /* Unique client identifier (assigned during init) */
    uint32_t defaultTimeout;     /* Default operation timeout in milliseconds */
    volatile uint8_t lastOpStatus; /* Status of last operation */
    volatile bool operationComplete; /* Flag set when async operation completes */
} FlashClient_t;

/**
 * @brief Flash Operation Result Codes
 */
typedef enum {
    FLASH_RESULT_SUCCESS = 0,     /* Operation completed successfully */
    FLASH_RESULT_TIMEOUT,         /* Operation timed out */
    FLASH_RESULT_ERROR,           /* Hardware or protocol error */
    FLASH_RESULT_BUSY,            /* Resource temporarily unavailable */
    FLASH_RESULT_INVALID_PARAM,   /* Invalid parameters provided */
    FLASH_RESULT_QUEUE_FULL       /* Operation queue is full */
} FlashResult_t;

/**
 * @brief Flash Device Information Structure
 */
typedef struct {
    uint8_t manufacturerId;       /* JEDEC Manufacturer ID (0xEF for Winbond) */
    uint8_t deviceType;           /* Device Type (0x40 for W25Q series) */
    uint8_t deviceId;             /* Device ID (0x17 for W25Q64FV) */
    uint32_t totalSizeBytes;      /* Total flash size in bytes */
    uint32_t pageSizeBytes;       /* Page size in bytes (256 for W25Q64FV) */
    uint32_t sectorSizeBytes;     /* Sector size in bytes (4096 for W25Q64FV) */
} FlashDeviceInfo_t;

/**
 * Flash Client API Function Prototypes
 */

/* Client lifecycle management */
FlashResult_t FlashClient_Init(FlashClient_t *pClient, SSI_Handle_t *pSSIHandle, uint32_t timeoutMs);
FlashResult_t FlashClient_Deinit(FlashClient_t *pClient);

/* Device identification and status */
FlashResult_t FlashClient_ReadDeviceInfo(FlashClient_t *pClient, FlashDeviceInfo_t *pDeviceInfo);
FlashResult_t FlashClient_GetStatus(FlashClient_t *pClient, uint8_t *pStatus);
FlashResult_t FlashClient_WaitUntilReady(FlashClient_t *pClient, uint32_t timeoutMs);

/* Read operations */
FlashResult_t FlashClient_ReadData(FlashClient_t *pClient, uint32_t address, uint8_t *pData, uint32_t length);
FlashResult_t FlashClient_ReadPage(FlashClient_t *pClient, uint32_t pageNumber, uint8_t *pData);

/* Write operations */
FlashResult_t FlashClient_WritePage(FlashClient_t *pClient, uint32_t pageNumber, const uint8_t *pData);
FlashResult_t FlashClient_WriteData(FlashClient_t *pClient, uint32_t address, const uint8_t *pData, uint32_t length);

/* Erase operations */
FlashResult_t FlashClient_EraseSector(FlashClient_t *pClient, uint32_t sectorNumber);
FlashResult_t FlashClient_ErasePages(FlashClient_t *pClient, uint32_t startPage, uint32_t numPages);

/* Utility functions */
FlashResult_t FlashClient_EnableWrite(FlashClient_t *pClient);
bool FlashClient_IsOperationComplete(FlashClient_t *pClient);
FlashResult_t FlashClient_CancelOperation(FlashClient_t *pClient);

/* Address/page/sector conversion utilities */
uint32_t FlashClient_PageToAddress(uint32_t pageNumber);
uint32_t FlashClient_AddressToPage(uint32_t address);
uint32_t FlashClient_SectorToAddress(uint32_t sectorNumber);
uint32_t FlashClient_AddressToSector(uint32_t address);

/**
 * Flash Memory Layout Constants for W25Q64FV
 */
#define W25Q64FV_TOTAL_SIZE_BYTES     (8 * 1024 * 1024)  /* 8MB total capacity */
#define W25Q64FV_PAGE_SIZE_BYTES      256                 /* 256 bytes per page */
#define W25Q64FV_SECTOR_SIZE_BYTES    (4 * 1024)         /* 4KB per sector */
#define W25Q64FV_PAGES_PER_SECTOR     16                 /* 16 pages per sector */
#define W25Q64FV_TOTAL_PAGES          32768              /* Total number of pages */
#define W25Q64FV_TOTAL_SECTORS        2048               /* Total number of sectors */

/**
 * Expected JEDEC ID for W25Q64FV
 */
#define W25Q64FV_JEDEC_MANUFACTURER   0xEF               /* Winbond manufacturer ID */
#define W25Q64FV_JEDEC_DEVICE_TYPE    0x40               /* W25Q series device type */
#define W25Q64FV_JEDEC_DEVICE_ID      0x17               /* W25Q64FV device ID */

/**
 * Default timeouts for different operations
 */
#define FLASH_DEFAULT_TIMEOUT_MS      1000               /* Default operation timeout */
#define FLASH_READ_TIMEOUT_MS         100                /* Read operation timeout */
#define FLASH_WRITE_TIMEOUT_MS        500                /* Page program timeout */
#define FLASH_ERASE_TIMEOUT_MS        5000               /* Sector erase timeout */

#endif /* W25QXX_FLASH_CLIENT_H_ */
