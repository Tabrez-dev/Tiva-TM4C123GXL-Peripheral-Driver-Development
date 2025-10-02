/*
 * w25qxx_flash_client.c
 *
 * Multi-Client W25Q64FV Flash Memory Driver - Client API Implementation
 *
 * This file implements the transparent client API for W25Q64FV flash operations.
 * Clients use simple blocking-style functions while the driver internally manages
 * resource sharing and asynchronous operations.
 *
 *  Created on: 01-Oct-2025
 *      Author: tabrez
 */

#include "w25qxx_flash_client.h"
#include "tm4c123x_ssi_driver.h"

/* Global client ID counter for assigning unique IDs */
static uint8_t g_nextClientId = 1;

/* Helper function prototypes */
static void FlashClient_OperationCallback(uint8_t clientId, uint8_t status);
static FlashResult_t FlashClient_ExecuteOperation(FlashClient_t *pClient, FlashOperationType_t opType,
                                                 uint32_t address, uint8_t *pData, uint32_t dataLen,
                                                 uint32_t timeoutMs);
static FlashResult_t FlashClient_StatusToResult(uint8_t status);

/***************************************************************************
 * @fn                          - FlashClient_Init
 *
 * @brief                       - Initializes a flash client for multi-client operations
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - pSSIHandle: Pointer to shared SSI flash interface
 * @param[in]                   - timeoutMs: Default timeout for operations
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Each client must call this before using flash operations
 */
FlashResult_t FlashClient_Init(FlashClient_t *pClient, SSI_Handle_t *pSSIHandle, uint32_t timeoutMs)
{
    if (!pClient || !pSSIHandle) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Ensure the SSI handle is configured for flash operations
    if (!pSSIHandle->isFlashInterface) {
        return FLASH_RESULT_ERROR;
    }

    // Assign unique client ID
    pClient->clientId = g_nextClientId++;
    if (g_nextClientId == 0) g_nextClientId = 1; // Prevent ID 0, wrap around

    // Initialize client context
    pClient->pSSIHandle = pSSIHandle;
    pClient->defaultTimeout = timeoutMs;
    pClient->lastOpStatus = 0;
    pClient->operationComplete = true;

    return FLASH_RESULT_SUCCESS;
}

/***************************************************************************
 * @fn                          - FlashClient_Deinit
 *
 * @brief                       - Deinitializes a flash client
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Cancels any pending operations for this client
 */
FlashResult_t FlashClient_Deinit(FlashClient_t *pClient)
{
    if (!pClient) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Cancel any pending operations for this client
    SSI_FlashCancelOperation(pClient->pSSIHandle, pClient->clientId);

    // Clear client context
    pClient->pSSIHandle = 0;
    pClient->clientId = 0;
    pClient->defaultTimeout = 0;
    pClient->lastOpStatus = 0;
    pClient->operationComplete = true;

    return FLASH_RESULT_SUCCESS;
}

/***************************************************************************
 * @fn                          - FlashClient_ReadDeviceInfo
 *
 * @brief                       - Reads JEDEC device identification information
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[out]                  - pDeviceInfo: Pointer to device info structure
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Reads 3-byte JEDEC ID and populates device information
 */
FlashResult_t FlashClient_ReadDeviceInfo(FlashClient_t *pClient, FlashDeviceInfo_t *pDeviceInfo)
{
    if (!pClient || !pDeviceInfo) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    uint8_t jedecId[3] = {0};
    FlashResult_t result = FlashClient_ExecuteOperation(pClient, FLASH_OP_READ_ID,
                                                       0, jedecId, 3, FLASH_READ_TIMEOUT_MS);

    if (result == FLASH_RESULT_SUCCESS) {
        // Populate device information structure
        pDeviceInfo->manufacturerId = jedecId[0];
        pDeviceInfo->deviceType = jedecId[1];
        pDeviceInfo->deviceId = jedecId[2];

        // Set known W25Q64FV characteristics
        pDeviceInfo->totalSizeBytes = W25Q64FV_TOTAL_SIZE_BYTES;
        pDeviceInfo->pageSizeBytes = W25Q64FV_PAGE_SIZE_BYTES;
        pDeviceInfo->sectorSizeBytes = W25Q64FV_SECTOR_SIZE_BYTES;
    }

    return result;
}

/***************************************************************************
 * @fn                          - FlashClient_GetStatus
 *
 * @brief                       - Reads the flash status register
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[out]                  - pStatus: Pointer to status byte
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Returns current status register value
 */
FlashResult_t FlashClient_GetStatus(FlashClient_t *pClient, uint8_t *pStatus)
{
    if (!pClient || !pStatus) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    return FlashClient_ExecuteOperation(pClient, FLASH_OP_READ_STATUS,
                                      0, pStatus, 1, FLASH_READ_TIMEOUT_MS);
}

/***************************************************************************
 * @fn                          - FlashClient_WaitUntilReady
 *
 * @brief                       - Waits until flash device is ready (not busy)
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - timeoutMs: Maximum time to wait in milliseconds
 *
 * @return                      - FlashResult_t: Success or timeout
 *
 * @Note                        - Polls status register until BUSY bit is clear
 */
FlashResult_t FlashClient_WaitUntilReady(FlashClient_t *pClient, uint32_t timeoutMs)
{
    if (!pClient) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    uint32_t startTime = 0; // In real implementation, use system tick timer
    uint8_t status;

    do {
        FlashResult_t result = FlashClient_GetStatus(pClient, &status);
        if (result != FLASH_RESULT_SUCCESS) {
            return result;
        }

        if ((status & W25QXX_STATUS_BUSY) == 0) {
            return FLASH_RESULT_SUCCESS; // Device is ready
        }

        // Simple delay - in production use proper delay function
        for (volatile uint32_t i = 0; i < 1000; i++);
        startTime++; // Increment timeout counter

    } while (startTime < timeoutMs);

    return FLASH_RESULT_TIMEOUT;
}

/***************************************************************************
 * @fn                          - FlashClient_ReadData
 *
 * @brief                       - Reads data from flash memory
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - address: Flash address to read from
 * @param[out]                  - pData: Buffer to store read data
 * @param[in]                   - length: Number of bytes to read
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Can read across page boundaries
 */
FlashResult_t FlashClient_ReadData(FlashClient_t *pClient, uint32_t address, uint8_t *pData, uint32_t length)
{
    if (!pClient || !pData || length == 0) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Validate address range
    if (address + length > W25Q64FV_TOTAL_SIZE_BYTES) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    return FlashClient_ExecuteOperation(pClient, FLASH_OP_READ_DATA,
                                      address, pData, length, FLASH_READ_TIMEOUT_MS);
}

/***************************************************************************
 * @fn                          - FlashClient_ReadPage
 *
 * @brief                       - Reads a complete page from flash memory
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - pageNumber: Page number to read (0 to 32767)
 * @param[out]                  - pData: Buffer to store page data (must be 256 bytes)
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Reads exactly one page (256 bytes)
 */
FlashResult_t FlashClient_ReadPage(FlashClient_t *pClient, uint32_t pageNumber, uint8_t *pData)
{
    if (!pClient || !pData) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Validate page number
    if (pageNumber >= W25Q64FV_TOTAL_PAGES) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    uint32_t address = FlashClient_PageToAddress(pageNumber);
    return FlashClient_ReadData(pClient, address, pData, W25Q64FV_PAGE_SIZE_BYTES);
}

/***************************************************************************
 * @fn                          - FlashClient_EnableWrite
 *
 * @brief                       - Enables write operations by setting Write Enable Latch
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Must be called before any write or erase operation
 */
FlashResult_t FlashClient_EnableWrite(FlashClient_t *pClient)
{
    if (!pClient) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    return FlashClient_ExecuteOperation(pClient, FLASH_OP_WRITE_ENABLE,
                                      0, 0, 0, FLASH_READ_TIMEOUT_MS);
}

/***************************************************************************
 * @fn                          - FlashClient_WritePage
 *
 * @brief                       - Writes a complete page to flash memory
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - pageNumber: Page number to write (0 to 32767)
 * @param[in]                   - pData: Data to write (must be 256 bytes)
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Automatically enables write and waits for completion
 */
FlashResult_t FlashClient_WritePage(FlashClient_t *pClient, uint32_t pageNumber, const uint8_t *pData)
{
    if (!pClient || !pData) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Validate page number
    if (pageNumber >= W25Q64FV_TOTAL_PAGES) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Enable write operations
    FlashResult_t result = FlashClient_EnableWrite(pClient);
    if (result != FLASH_RESULT_SUCCESS) {
        return result;
    }

    // Execute page program operation
    uint32_t address = FlashClient_PageToAddress(pageNumber);
    result = FlashClient_ExecuteOperation(pClient, FLASH_OP_PAGE_PROGRAM,
                                        address, (uint8_t*)pData, W25Q64FV_PAGE_SIZE_BYTES,
                                        FLASH_WRITE_TIMEOUT_MS);

    if (result == FLASH_RESULT_SUCCESS) {
        // Wait for programming to complete
        result = FlashClient_WaitUntilReady(pClient, FLASH_WRITE_TIMEOUT_MS);
    }

    return result;
}

/***************************************************************************
 * @fn                          - FlashClient_EraseSector
 *
 * @brief                       - Erases a 4KB sector of flash memory
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - sectorNumber: Sector number to erase (0 to 2047)
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Automatically enables write and waits for completion
 */
FlashResult_t FlashClient_EraseSector(FlashClient_t *pClient, uint32_t sectorNumber)
{
    if (!pClient) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Validate sector number
    if (sectorNumber >= W25Q64FV_TOTAL_SECTORS) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    // Enable write operations
    FlashResult_t result = FlashClient_EnableWrite(pClient);
    if (result != FLASH_RESULT_SUCCESS) {
        return result;
    }

    // Execute sector erase operation
    uint32_t address = FlashClient_SectorToAddress(sectorNumber);
    result = FlashClient_ExecuteOperation(pClient, FLASH_OP_SECTOR_ERASE,
                                        address, 0, 0, FLASH_ERASE_TIMEOUT_MS);

    if (result == FLASH_RESULT_SUCCESS) {
        // Wait for erase to complete
        result = FlashClient_WaitUntilReady(pClient, FLASH_ERASE_TIMEOUT_MS);
    }

    return result;
}

/***************************************************************************
 * Helper Functions
 ***************************************************************************/

/***************************************************************************
 * @fn                          - FlashClient_ExecuteOperation
 *
 * @brief                       - Executes a flash operation using the resource manager
 *
 * @param[in]                   - pClient: Pointer to client handle structure
 * @param[in]                   - opType: Type of operation to execute
 * @param[in]                   - address: Flash address for operation
 * @param[in/out]               - pData: Data buffer pointer
 * @param[in]                   - dataLen: Data length
 * @param[in]                   - timeoutMs: Operation timeout
 *
 * @return                      - FlashResult_t: Success or error code
 *
 * @Note                        - Internal function that provides blocking interface over async operations
 */
static FlashResult_t FlashClient_ExecuteOperation(FlashClient_t *pClient, FlashOperationType_t opType,
                                                 uint32_t address, uint8_t *pData, uint32_t dataLen,
                                                 uint32_t timeoutMs)
{
    // Create operation request
    FlashRequest_t request = {
        .opType = opType,
        .address = address,
        .pData = pData,
        .dataLen = dataLen,
        .clientId = pClient->clientId,
        .priority = 5,  // Default priority
        .status = FLASH_STATUS_PENDING,
        .callback = FlashClient_OperationCallback
    };

    // Mark operation as in progress
    pClient->operationComplete = false;
    pClient->lastOpStatus = FLASH_STATUS_PENDING;

    // Queue operation in resource manager
    uint8_t queueResult = SSI_FlashQueueOperation(pClient->pSSIHandle, &request);
    if (queueResult != 0) {
        pClient->operationComplete = true;
        return (queueResult == 1) ? FLASH_RESULT_QUEUE_FULL : FLASH_RESULT_INVALID_PARAM;
    }

    // Wait for operation completion with timeout
    uint32_t timeoutCounter = 0;
    while (!pClient->operationComplete && timeoutCounter < timeoutMs) {
        // Simple delay - in production use proper delay function
        for (volatile uint32_t i = 0; i < 1000; i++);
        timeoutCounter++;
    }

    // Check if operation completed
    if (!pClient->operationComplete) {
        // Timeout occurred - try to cancel operation
        SSI_FlashCancelOperation(pClient->pSSIHandle, pClient->clientId);
        pClient->operationComplete = true;
        return FLASH_RESULT_TIMEOUT;
    }

    // Convert operation status to result code
    return FlashClient_StatusToResult(pClient->lastOpStatus);
}

/***************************************************************************
 * @fn                          - FlashClient_OperationCallback
 *
 * @brief                       - Callback function for async operation completion
 *
 * @param[in]                   - clientId: Client ID that initiated the operation
 * @param[in]                   - status: Operation completion status
 *
 * @return                      - none
 *
 * @Note                        - Called from interrupt context when operation completes
 */
static void FlashClient_OperationCallback(uint8_t clientId, uint8_t status)
{
    // Find client by ID and update operation status
    // In a real implementation, maintain a client registry
    // For now, assume single client or use global state

    // This is a simplified implementation - production code would maintain
    // a client registry to map clientId to client handles
    static FlashClient_t *g_currentClient = 0;

    if (g_currentClient && g_currentClient->clientId == clientId) {
        g_currentClient->lastOpStatus = status;
        g_currentClient->operationComplete = true;
    }
}

/***************************************************************************
 * @fn                          - FlashClient_StatusToResult
 *
 * @brief                       - Converts operation status to result code
 *
 * @param[in]                   - status: Operation status flags
 *
 * @return                      - FlashResult_t: Corresponding result code
 *
 * @Note                        - Maps internal status codes to client API result codes
 */
static FlashResult_t FlashClient_StatusToResult(uint8_t status)
{
    if (status & FLASH_STATUS_COMPLETE) {
        return FLASH_RESULT_SUCCESS;
    } else if (status & FLASH_STATUS_TIMEOUT) {
        return FLASH_RESULT_TIMEOUT;
    } else if (status & FLASH_STATUS_ERROR) {
        return FLASH_RESULT_ERROR;
    } else if (status & FLASH_STATUS_ACTIVE) {
        return FLASH_RESULT_BUSY;
    } else {
        return FLASH_RESULT_ERROR;
    }
}

/***************************************************************************
 * Utility Functions for Address/Page/Sector Conversion
 ***************************************************************************/

uint32_t FlashClient_PageToAddress(uint32_t pageNumber)
{
    return pageNumber * W25Q64FV_PAGE_SIZE_BYTES;
}

uint32_t FlashClient_AddressToPage(uint32_t address)
{
    return address / W25Q64FV_PAGE_SIZE_BYTES;
}

uint32_t FlashClient_SectorToAddress(uint32_t sectorNumber)
{
    return sectorNumber * W25Q64FV_SECTOR_SIZE_BYTES;
}

uint32_t FlashClient_AddressToSector(uint32_t address)
{
    return address / W25Q64FV_SECTOR_SIZE_BYTES;
}

bool FlashClient_IsOperationComplete(FlashClient_t *pClient)
{
    return pClient ? pClient->operationComplete : false;
}

FlashResult_t FlashClient_CancelOperation(FlashClient_t *pClient)
{
    if (!pClient) {
        return FLASH_RESULT_INVALID_PARAM;
    }

    uint8_t result = SSI_FlashCancelOperation(pClient->pSSIHandle, pClient->clientId);
    return (result == 0) ? FLASH_RESULT_SUCCESS : FLASH_RESULT_ERROR;
}
