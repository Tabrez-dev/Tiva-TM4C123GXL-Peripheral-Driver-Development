/*
 * tm4c123x_i2c_driver.h
 *
 *  Created on: Jan 2025
 *      Author: Tabrez
 */

#ifndef DRIVERS_INC_TM4C123X_I2C_DRIVER_H_
#define DRIVERS_INC_TM4C123X_I2C_DRIVER_H_

#include "tm4c123x.h"

/*
 * Handle structure for I2Cx peripheral
 * Used for both master and slave modes
 */
typedef struct {
    I2C_RegDef_t *pI2Cx;            /* Peripheral base address pointer */
    uint8_t *pTxBuffer;             /* Tx buffer address */
    uint8_t *pRxBuffer;             /* Rx buffer address */
    uint32_t TxLen;                 /* Tx length */
    uint32_t RxLen;                 /* Rx length */
    uint8_t TxRxState;              /* Communication state */
    uint8_t DevAddr;                /* Slave/device address */
    uint32_t RxSize;                /* Rx size */
    uint8_t Sr;                     /* Repeated start value */
} I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM        100000      /* Standard mode: 100 kHz */
#define I2C_SCL_SPEED_FM        400000      /* Fast mode: 400 kHz */
#define I2C_SCL_SPEED_FMP       1000000     /* Fast mode plus: 1 MHz */

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

/*
 * I2C application states
 */
#define I2C_READY               0
#define I2C_BUSY_IN_RX          1
#define I2C_BUSY_IN_TX          2

/*
 * I2C application events
 */
#define I2C_EV_TX_CMPLT         0
#define I2C_EV_RX_CMPLT         1
#define I2C_EV_STOP             2
#define I2C_ERROR_BERR          3   /* Bus error */
#define I2C_ERROR_ARLO          4   /* Arbitration lost */
#define I2C_ERROR_AF            5   /* ACK failure */
#define I2C_ERROR_OVR           6   /* Overrun */
#define I2C_ERROR_TIMEOUT       7   /* Timeout */
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/*
 * I2C repeated start macros
 */
#define I2C_DISABLE_SR          0
#define I2C_ENABLE_SR           1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_CLKTO          ( 1 << I2C_MCS_CLKTO)
#define I2C_FLAG_BUSBSY         ( 1 << I2C_MCS_BUSBSY)
#define I2C_FLAG_IDLE           ( 1 << I2C_MCS_IDLE)
#define I2C_FLAG_ARBLST         ( 1 << I2C_MCS_ARBLST)
#define I2C_FLAG_DATACK         ( 1 << I2C_MCS_DATACK)
#define I2C_FLAG_ADRACK         ( 1 << I2C_MCS_ADRACK)
#define I2C_FLAG_ERROR          ( 1 << I2C_MCS_ERROR)
#define I2C_FLAG_BUSY           ( 1 << I2C_MCS_BUSY)

/***********************************************************************************
 *                      APIs supported by this driver
 *         For more information about the APIs check the function definitions
 ***********************************************************************************/

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_MasterInit(I2C_Handle_t *pI2CHandle, uint32_t I2C_SCLSpeed);
void I2C_SlaveInit(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive (Blocking mode)
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data Send and Receive (Interrupt mode)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * Slave mode data transfer
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_LoopbackControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C Master Control Sequence Helper Functions
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendSingle(I2C_RegDef_t *pI2Cx);
void I2C_MasterBurstContinue(I2C_RegDef_t *pI2Cx);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* DRIVERS_INC_TM4C123X_I2C_DRIVER_H_ */
