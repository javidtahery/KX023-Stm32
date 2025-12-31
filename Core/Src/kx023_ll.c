/*
 * kx023_ll.c
 *
 * Created on: Dec 29, 2025
 * Author: USER
 */
#include "kx023_ll.h"

// ================================================================
// Private Functions Prototypes (Helper Functions)
// ================================================================
static int KX023_ReadRegister(KX023_Ctx_t *ctx, uint8_t address);
static int KX023_ReadRegisters(KX023_Ctx_t *ctx, uint8_t address, uint8_t *data, size_t length);
static int KX023_WriteRegister(KX023_Ctx_t *ctx, uint8_t address, uint8_t value);
static void KX023_SetBit(KX023_Ctx_t *ctx, uint8_t register_address, uint8_t bit_index);
static void KX023_ClearBit(KX023_Ctx_t *ctx, uint8_t register_address, uint8_t bit_index);

// Internal helper functions that were protected in the original file
static void SetPerformanceLowCurrentMode(KX023_Ctx_t *ctx);
static void SetPerformanceHighResolutionMode(KX023_Ctx_t *ctx);
static void DisableNewAccelerationDataInterrupt(KX023_Ctx_t *ctx);
static void EnableNewAccelerationDataInterrupt(KX023_Ctx_t *ctx);
static void SetAccelerationDataRange(KX023_Ctx_t *ctx, AccelerationRange_t range);
static void DisableTapFunction(KX023_Ctx_t *ctx);
static void EnableTapFunction(KX023_Ctx_t *ctx);
static void DisableWakeUpFunction(KX023_Ctx_t *ctx);
static void EnableWakeUpFunction(KX023_Ctx_t *ctx);
static void DisableTiltPositionFunction(KX023_Ctx_t *ctx);
static void EnableTiltPositionFunction(KX023_Ctx_t *ctx);
static void EnableFiltering(KX023_Ctx_t *ctx);
static void DisableFiltering(KX023_Ctx_t *ctx);
static void SetFilterCornerFrequency(KX023_Ctx_t *ctx, FilterCornerFrequency_t frequency);
static void SetAccelerationDataRate(KX023_Ctx_t *ctx, AccelerationOutputDataRate_t odr);
static void DisablePhysicalInterruptPin(KX023_Ctx_t *ctx, int interrupt_number);
static void EnablePhysicalInterruptPin(KX023_Ctx_t *ctx, int interrupt_number, uint8_t interrupt_polarity_active_high, uint8_t interrupt_by_pulse);
static void ConfigPhysicalInterruptEvent(KX023_Ctx_t *ctx, int interrupt_number, uint8_t buffer_full, uint8_t watermark, uint8_t data_ready, uint8_t tap, uint8_t wake_up, uint8_t tilt);
static void DisableSampleBuffer(KX023_Ctx_t *ctx);
static void EnableSampleBuffer(KX023_Ctx_t *ctx);
static void SetSampleBufferResolution(KX023_Ctx_t *ctx, uint8_t use_16_bit_res);
static void DisableBufferFullInterrupt(KX023_Ctx_t *ctx);
static void EnableBufferFullInterrupt(KX023_Ctx_t *ctx);
static void SetTiltPositionFunctionOutputDataRate(KX023_Ctx_t *ctx, TiltPositionOutputDataRate_t odr);
static void SetDirectionalTapFunctionOutputDataRate(KX023_Ctx_t *ctx, DirectionalTapOutputDataRate_t odr);
static void SetMotionWakeUpFunctionOutputDataRate(KX023_Ctx_t *ctx, MotionWakeUpOutputDataRate_t odr);
static void SetMotionWakeUpInterruptDirection(KX023_Ctx_t *ctx, uint8_t xn, uint8_t xp, uint8_t yn, uint8_t yp, uint8_t zn, uint8_t zp);
static void SetInterruptMotionWakeUpTimer(KX023_Ctx_t *ctx, MotionWakeUpOutputDataRate_t odr, float desired_delay_time_sec);
static void SetInterruptMotionWakeUpThreshold(KX023_Ctx_t *ctx, float desired_threshold);
static void SetTiltPositionInterruptDirection(KX023_Ctx_t *ctx, uint8_t xn, uint8_t xp, uint8_t yn, uint8_t yp, uint8_t zn, uint8_t zp);
static void ConfigTiltPositionTimer(KX023_Ctx_t *ctx, TiltPositionOutputDataRate_t odr, float desired_delay_time_msec);
static void SetTiltPositionAngleLowLimit(KX023_Ctx_t *ctx, uint8_t expected_angle);
static void SetTiltPositionAngleHighLimit(KX023_Ctx_t *ctx, uint8_t expected_angle);
static void SetTiltPositionHysteresisAngle(KX023_Ctx_t *ctx);
static void SetDirectionalTapInterruptDirection(KX023_Ctx_t *ctx, uint8_t xn, uint8_t xp, uint8_t yn, uint8_t yp, uint8_t zn, uint8_t zp);
static void DisableSingleTap(KX023_Ctx_t *ctx);
static void EnableSingleTap(KX023_Ctx_t *ctx);
static void DisableDoubleTap(KX023_Ctx_t *ctx);
static void EnableDoubleTap(KX023_Ctx_t *ctx);
static void SetDoubleTapMinimumTimeSeparation(KX023_Ctx_t *ctx, DirectionalTapOutputDataRate_t odr, float desired_minimum_time_separation_between_taps);
static void SetTapHighThresholdValue(KX023_Ctx_t *ctx, uint8_t value);
static void SetTapLowThresholdValue(KX023_Ctx_t *ctx, uint8_t value);

// ================================================================
// Low Level I/O Functions (The Core of the Port)
// ================================================================

void KX023_Init_I2C(KX023_Ctx_t *ctx, I2C_TypeDef *i2c, uint8_t addr) {
    ctx->is_spi = 0;
    ctx->i2c_instance = i2c;
    ctx->slave_address = (addr << 1); // Left shift for 8-bit address format in STM32
    ctx->spi_instance = NULL;
}

void KX023_Init_SPI(KX023_Ctx_t *ctx, SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint32_t cs_pin) {
    ctx->is_spi = 1;
    ctx->spi_instance = spi;
    ctx->cs_port = cs_port;
    ctx->cs_pin = cs_pin;
    ctx->i2c_instance = NULL;

    // Ensure CS pin is high initially
    LL_GPIO_SetOutputPin(ctx->cs_port, ctx->cs_pin);
}

// Helper function for SPI transmit/receive
static uint8_t SPI_Transfer(SPI_TypeDef *SPIx, uint8_t data) {
//    while (!LL_SPI_IsActiveFlag_TXE(SPIx));
//    LL_SPI_TransmitData8(SPIx, data);
//    while (!LL_SPI_IsActiveFlag_RXNE(SPIx));
//    return LL_SPI_ReceiveData8(SPIx);
}


// Constant value for timeout (adjustable based on micro frequency)
#define I2C_TIMEOUT_COUNT  100000

static int KX023_ReadRegisters(KX023_Ctx_t *ctx, uint8_t address, uint8_t *data, size_t length) {
    if (ctx->is_spi) {
//        // SPI section usually does not hang unless clock is cut, but can be checked for high accuracy
//        LL_GPIO_ResetOutputPin(ctx->cs_port, ctx->cs_pin);
//        SPI_Transfer(ctx->spi_instance, 0x80 | address);
//        for (size_t i = 0; i < length; i++) {
//            data[i] = SPI_Transfer(ctx->spi_instance, 0x00);
//        }
//        LL_GPIO_SetOutputPin(ctx->cs_port, ctx->cs_pin);
//        return 1;
    } else {
        I2C_TypeDef *I2Cx = ctx->i2c_instance;
        uint32_t timeout = I2C_TIMEOUT_COUNT;

        // 1. Send Start
        LL_I2C_GenerateStartCondition(I2Cx);
        while(!LL_I2C_IsActiveFlag_SB(I2Cx)) {
            if (--timeout == 0) return -2; // Timeout Error
        }

        // 2. Send address for register write
        LL_I2C_TransmitData8(I2Cx, ctx->slave_address);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_ADDR(I2Cx)) {
            if (LL_I2C_IsActiveFlag_AF(I2Cx)) {
                LL_I2C_ClearFlag_AF(I2Cx);
                LL_I2C_GenerateStopCondition(I2Cx);
                return -1; // NACK Error
            }
            if (--timeout == 0) return -2;
        }
        LL_I2C_ClearFlag_ADDR(I2Cx);

        // 3. Send target register address

		while(!LL_I2C_IsActiveFlag_TXE(I2Cx))
		{
			if (--timeout == 0) return -2;
		}
        LL_I2C_TransmitData8(I2Cx, address);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_TXE(I2Cx)) {
            if (--timeout == 0) return -2;
        }

        // 4. Generate Re-Start for reading data
        LL_I2C_GenerateStartCondition(I2Cx);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_SB(I2Cx)) {
            if (--timeout == 0) return -2;
        }

        LL_I2C_TransmitData8(I2Cx, ctx->slave_address | 1);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_ADDR(I2Cx)) {
            if (LL_I2C_IsActiveFlag_AF(I2Cx)) {
                LL_I2C_ClearFlag_AF(I2Cx);
                LL_I2C_GenerateStopCondition(I2Cx);
                return -1;
            }
            if (--timeout == 0) return -2;
        }
        LL_I2C_ClearFlag_ADDR(I2Cx);

        // 5. Receive data
        for(size_t i = 0; i < length; i++) {
            if(i < length - 1) {
                LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
            } else {
                LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
            }

            timeout = I2C_TIMEOUT_COUNT;
            while(!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
                if (--timeout == 0) return -2;
            }
            data[i] = LL_I2C_ReceiveData8(I2Cx);
        }

        LL_I2C_GenerateStopCondition(I2Cx);
        return 1;
    }
}

static int KX023_WriteRegister(KX023_Ctx_t *ctx, uint8_t address, uint8_t value) {
    if (ctx->is_spi) {
        LL_GPIO_ResetOutputPin(ctx->cs_port, ctx->cs_pin);
        SPI_Transfer(ctx->spi_instance, address & 0x7F);
        SPI_Transfer(ctx->spi_instance, value);
        LL_GPIO_SetOutputPin(ctx->cs_port, ctx->cs_pin);
        return 1;
    } else {
        I2C_TypeDef *I2Cx = ctx->i2c_instance;
        uint32_t timeout = I2C_TIMEOUT_COUNT;

        LL_I2C_GenerateStartCondition(I2Cx);
        while(!LL_I2C_IsActiveFlag_SB(I2Cx)) if (--timeout == 0) return -2;

        LL_I2C_TransmitData8(I2Cx, ctx->slave_address);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_ADDR(I2Cx)) {
            if (LL_I2C_IsActiveFlag_AF(I2Cx)) {
                LL_I2C_ClearFlag_AF(I2Cx);
                LL_I2C_GenerateStopCondition(I2Cx);
                return -1;
            }
            if (--timeout == 0) return -2;
        }
        LL_I2C_ClearFlag_ADDR(I2Cx);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_TXE(I2Cx))	if (--timeout == 0) return -2;
        LL_I2C_TransmitData8(I2Cx, address);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_TXE(I2Cx)) if (--timeout == 0) return -2;

        LL_I2C_TransmitData8(I2Cx, value);
        timeout = I2C_TIMEOUT_COUNT;
        while(!LL_I2C_IsActiveFlag_TXE(I2Cx)) if (--timeout == 0) return -2;

        LL_I2C_GenerateStopCondition(I2Cx);
        return 1;
    }
}

static int KX023_ReadRegister(KX023_Ctx_t *ctx, uint8_t address) {
    uint8_t value = 0;
    if (KX023_ReadRegisters(ctx, address, &value, 1) != 1) {
        return -1;
    }
    return value;
}

static void KX023_SetBit(KX023_Ctx_t *ctx, uint8_t register_address, uint8_t bit_index) {
    if (bit_index > 7) return;
    uint8_t reg_value = KX023_ReadRegister(ctx, register_address);
    reg_value |= (1 << bit_index);
    KX023_WriteRegister(ctx, register_address, reg_value);
}

static void KX023_ClearBit(KX023_Ctx_t *ctx, uint8_t register_address, uint8_t bit_index) {
    if (bit_index > 7) return;
    uint8_t reg_value = KX023_ReadRegister(ctx, register_address);
    reg_value &= ~(1 << bit_index);
    KX023_WriteRegister(ctx, register_address, reg_value);
}

// ================================================================
// Public API Implementation
// ================================================================

KX023_Status_t KX023_Begin(KX023_Ctx_t *ctx) {
    // Check WHO_AM_I register (0x15)
    int who_am_i = KX023_ReadRegister(ctx, KX023_Who_AM_I_REG);
    if (who_am_i != 0x15) {
        KX023_End(ctx);
        return KX023_STATUS_DEVICE_NOT_RECOGNIZE;
    }

    // Check COTR register
    uint8_t cotr_reg_val = (uint8_t)KX023_ReadRegister(ctx, KX023_COTR_REG);
    if (!((cotr_reg_val == 0x55) || (cotr_reg_val == 0xAA))) {
        KX023_End(ctx);
        return KX023_STATUS_DEVICE_SELF_TEST_FAIL;
    }

    return KX023_STATUS_OK;
}

void KX023_End(KX023_Ctx_t *ctx) {
    if (ctx->is_spi) {
        LL_GPIO_SetOutputPin(ctx->cs_port, ctx->cs_pin);
    } else {
        KX023_WriteRegister(ctx, KX023_CNTL1_REG, 0x18);
    }
}

void KX023_ClearInterrupt(KX023_Ctx_t *ctx) {
    KX023_ReadRegister(ctx, KX023_INT_REL_REG);
}

void KX023_ConfigPhysicalInterruptPin(KX023_Ctx_t *ctx, int interrupt_number, PhysicalInterruptParameter_t params) {
    EnablePhysicalInterruptPin(ctx, interrupt_number,
                               (params.polarity == KX023_INTERRUPT_POLARITY_ACTIVE_HIGH),
                               (params.signal_type == KX023_INTERRUPT_TYPE_PULSE));
    ConfigPhysicalInterruptEvent(ctx, interrupt_number,
                                 params.events.buffer_full_interrupt,
                                 params.events.watermark_interrupt,
                                 params.events.data_ready_interrupt,
                                 params.events.tap_function_interrupt,
                                 params.events.wake_up_function_interrupt,
                                 params.events.tilt_position_function_interrupt);
}

void KX023_SoftwareReset(KX023_Ctx_t *ctx) {
    KX023_SetStandbyMode(ctx);
    KX023_SetBit(ctx, KX023_CNTL2_REG, KX023_CNTL2_SRST_BIT);
}

void KX023_SetStandbyMode(KX023_Ctx_t *ctx) {
    KX023_ClearBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_PC1_BIT);
}

void KX023_SetOperatingMode(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_PC1_BIT);
}

KX023_Interrupt_Type_t KX023_GetInterruptType(KX023_Ctx_t *ctx) {
    uint8_t regValue = KX023_ReadRegister(ctx, KX023_INS2_REG);

    if (regValue & (1 << KX023_INS2_BFI_BIT)) return KX023_INTERRUPT_BUFFER_FULL;
    if (regValue & (1 << KX023_INS2_WMI_BIT)) return KX023_INTERRUPT_WATERMARK;
    if (regValue & (1 << KX023_INS2_DRDY_BIT)) return KX023_INTERRUPT_NEW_ACCELERATION_DATA_AVAILABLE;

    switch (regValue & KX023_INS2_TDTS_MASK) {
    case 0x00: break;
    case 0x04: return KX023_INTERRUPT_SINGLE_TAP;
    case 0x08: return KX023_INTERRUPT_DOUBLE_TAP;
    default: return KX023_INTERRUPT_UNKNOWN_ERROR;
    }

    if (regValue & (1 << KX023_INS2_WUFS_BIT)) return KX023_INTERRUPT_WAKE_UP;
    if (regValue & (1 << KX023_INS2_TPS_BIT)) return KX023_INTERRUPT_TILT_POSITION;

    return KX023_INTERRUPT_NO_INTERRUPT;
}

// ================================================================
// Configuration Implementations
// ================================================================

KX023_Status_t KX023_ConfigAsynchronousReadBack(KX023_Ctx_t *ctx, AccelerationRange_t range, AccelerationOutputDataRate_t odr) {
    KX023_SoftwareReset(ctx);
    LL_mDelay(50);
    KX023_SetStandbyMode(ctx);
    SetPerformanceHighResolutionMode(ctx);
    SetAccelerationDataRange(ctx, range);
    SetAccelerationDataRate(ctx, odr);
    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ReadAsynchronousData(KX023_Ctx_t *ctx, float *x, float *y, float *z) {
    if ((x == NULL) || (y == NULL) || (z == NULL)) return KX023_STATUS_NULL_POINTERS;

    uint8_t buffer[6]; // 3 registers * 2 bytes
    if (KX023_ReadRegisters(ctx, KX023_XOUTL_REG, buffer, 6) != 1) {
        *x = 0; *y = 0; *z = 0;
        return KX023_STATUS_REG_READ_WRITE_ERROR;
    }

    int16_t x_raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t y_raw = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t z_raw = (int16_t)(buffer[5] << 8 | buffer[4]);

    *x = x_raw * ((int)(ctx->_accelerationRange)) / 32768.0f;
    *y = y_raw * ((int)(ctx->_accelerationRange)) / 32768.0f;
    *z = z_raw * ((int)(ctx->_accelerationRange)) / 32768.0f;

    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ConfigSynchronousInterruptReadBack(KX023_Ctx_t *ctx, AccelerationRange_t range, AccelerationOutputDataRate_t odr) {
    KX023_SoftwareReset(ctx);
    LL_mDelay(50);
    KX023_SetStandbyMode(ctx);
    SetPerformanceHighResolutionMode(ctx);
    SetAccelerationDataRange(ctx, range);
    EnableNewAccelerationDataInterrupt(ctx);
    SetAccelerationDataRate(ctx, odr);
    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ConfigBufferFullInterrupt(KX023_Ctx_t *ctx, AccelerationRange_t range, AccelerationOutputDataRate_t odr) {
    KX023_SoftwareReset(ctx);
    LL_mDelay(50);
    KX023_SetStandbyMode(ctx);
    SetPerformanceLowCurrentMode(ctx);
    SetAccelerationDataRange(ctx, range);
    SetAccelerationDataRate(ctx, odr);
    EnableSampleBuffer(ctx);
    SetSampleBufferResolution(ctx, 0);
    EnableBufferFullInterrupt(ctx);
    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ReadBuffer(KX023_Ctx_t *ctx, uint8_t *buffer) {
    KX023_ReadRegisters(ctx, KX023_BUF_READ_REG, buffer, 252);
    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ConfigWakeUpFunction(KX023_Ctx_t *ctx, AccelerationRange_t range, MotionWakeUpOutputDataRate_t odr, float desired_delay_time_sec, float desired_threshold, DirectionInfoParams_t direction) {
    KX023_SoftwareReset(ctx);
    LL_mDelay(50);
    KX023_SetStandbyMode(ctx);
    SetPerformanceHighResolutionMode(ctx);
    EnableWakeUpFunction(ctx);
    SetAccelerationDataRange(ctx, range);
    SetMotionWakeUpFunctionOutputDataRate(ctx, odr);
    SetMotionWakeUpInterruptDirection(ctx, direction.x_negative, direction.x_positive, direction.y_negative, direction.y_positive, direction.z_negative, direction.z_positive);
    SetInterruptMotionWakeUpTimer(ctx, odr, desired_delay_time_sec);
    SetInterruptMotionWakeUpThreshold(ctx, desired_threshold);
    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ConfigActivateTiltPosition(KX023_Ctx_t *ctx, AccelerationRange_t range, TiltPositionOutputDataRate_t odr, float desired_delay_time_msec, DirectionInfoParams_t direction) {
    KX023_SoftwareReset(ctx);
    LL_mDelay(50);
    KX023_SetStandbyMode(ctx);
    SetPerformanceHighResolutionMode(ctx);
    SetAccelerationDataRange(ctx, range);
    EnableTiltPositionFunction(ctx);
    SetTiltPositionInterruptDirection(ctx, direction.x_negative, direction.x_positive, direction.y_negative, direction.y_positive, direction.z_negative, direction.z_positive);
    SetTiltPositionFunctionOutputDataRate(ctx, odr);
    ConfigTiltPositionTimer(ctx, odr, desired_delay_time_msec);
    SetTiltPositionHysteresisAngle(ctx);
    return KX023_STATUS_OK;
}

KX023_Status_t KX023_ConfigActivateTapFunction(KX023_Ctx_t *ctx, AccelerationRange_t range, DirectionalTapOutputDataRate_t odr, DirectionInfoParams_t direction, TapMode_t tap_mode) {
    KX023_SoftwareReset(ctx);
    LL_mDelay(50);
    KX023_SetStandbyMode(ctx);
    SetPerformanceHighResolutionMode(ctx);
    SetAccelerationDataRange(ctx, range);
    EnableTapFunction(ctx);
    SetDirectionalTapInterruptDirection(ctx, direction.x_negative, direction.x_positive, direction.y_negative, direction.y_positive, direction.z_negative, direction.z_positive);
    SetDirectionalTapFunctionOutputDataRate(ctx, odr);

    if ((tap_mode & KX023_TAP_MODE_SINGLE_TAP) == KX023_TAP_MODE_SINGLE_TAP) EnableSingleTap(ctx);
    if ((tap_mode & KX023_TAP_MODE_DOUBLE_TAP) == KX023_TAP_MODE_DOUBLE_TAP) EnableDoubleTap(ctx);

    SetDoubleTapMinimumTimeSeparation(ctx, odr, 0.3);
    SetTapHighThresholdValue(ctx, 0xCB);
    SetTapLowThresholdValue(ctx, 0x1A);

    KX023_WriteRegister(ctx, KX023_FTD_REG, 0xA2);
    KX023_WriteRegister(ctx, KX023_STD_REG, 0xA2);
    KX023_WriteRegister(ctx, KX023_TLT_REG, 0x28);
    KX023_WriteRegister(ctx, KX023_TWS_REG, 0xA0);

    return KX023_STATUS_OK;
}

// ================================================================
// Internal Logic Helpers (Copied from original, adapted to C/LL)
// ================================================================

static void SetPerformanceLowCurrentMode(KX023_Ctx_t *ctx) {
    KX023_ClearBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_RES_BIT);
}

static void SetPerformanceHighResolutionMode(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_RES_BIT);
}

static void DisableNewAccelerationDataInterrupt(KX023_Ctx_t *ctx) {
    KX023_ClearBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_DRDYE_BIT);
}

static void EnableNewAccelerationDataInterrupt(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_DRDYE_BIT);
}

static void SetAccelerationDataRange(KX023_Ctx_t *ctx, AccelerationRange_t range) {
    uint8_t cntl1_reg_value = KX023_ReadRegister(ctx, KX023_CNTL1_REG);
    cntl1_reg_value &= ~KX023_CNTL1_GSEL_MASK;

    switch (range) {
        case KX023_ACCLERATION_RANGE_4G:
            cntl1_reg_value |= (1 << KX023_CNTL1_GSEL0_BIT);
            ctx->_accelerationRange = KX023_ACCLERATION_RANGE_4G;
            break;
        case KX023_ACCLERATION_RANGE_8G:
            cntl1_reg_value |= (1 << KX023_CNTL1_GSEL1_BIT);
            ctx->_accelerationRange = KX023_ACCLERATION_RANGE_8G;
            break;
        default: // 2G
            ctx->_accelerationRange = KX023_ACCLERATION_RANGE_2G;
            break;
    }
    KX023_WriteRegister(ctx, KX023_CNTL1_REG, cntl1_reg_value);
}

static void DisableTapFunction(KX023_Ctx_t *ctx) {
    KX023_ClearBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_TDTE_BIT);
}

static void EnableTapFunction(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_TDTE_BIT);
}

static void DisableWakeUpFunction(KX023_Ctx_t *ctx) {
    KX023_ClearBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_WUFE_BIT);
}

static void EnableWakeUpFunction(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_WUFE_BIT);
}

static void DisableTiltPositionFunction(KX023_Ctx_t *ctx) {
    KX023_ClearBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_TPE_BIT);
}

static void EnableTiltPositionFunction(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_CNTL1_REG, KX023_CNTL1_TPE_BIT);
}

static void SetAccelerationDataRate(KX023_Ctx_t *ctx, AccelerationOutputDataRate_t odr) {
    uint8_t reg_value = KX023_ReadRegister(ctx, KX023_ODCNTL_REG);
    reg_value &= ~KX023_ODCNTL_OSA_MASK;
    reg_value |= (((uint8_t)odr << KX023_ODCNTL_OSA0_BIT) & KX023_ODCNTL_OSA_MASK);
    KX023_WriteRegister(ctx, KX023_ODCNTL_REG, reg_value);
}

static void EnablePhysicalInterruptPin(KX023_Ctx_t *ctx, int interrupt_number, uint8_t interrupt_polarity_active_high, uint8_t interrupt_by_pulse) {
    uint8_t regAddr = (interrupt_number == 1) ? KX023_INC1_REG : KX023_INC5_REG;
    uint8_t ienBit = (interrupt_number == 1) ? KX023_INC1_IEN1_BIT : KX023_INC5_IEN2_BIT;
    uint8_t ieaBit = (interrupt_number == 1) ? KX023_INC1_IEA1_BIT : KX023_INC5_IEA2_BIT;
    uint8_t ielBit = (interrupt_number == 1) ? KX023_INC1_IEL1_BIT : KX023_INC5_IEL2_BIT;

    if (interrupt_number != 1 && interrupt_number != 2) return;

    KX023_SetBit(ctx, regAddr, ienBit);

    if (interrupt_polarity_active_high) KX023_SetBit(ctx, regAddr, ieaBit);
    else KX023_ClearBit(ctx, regAddr, ieaBit);

    if (interrupt_by_pulse) KX023_SetBit(ctx, regAddr, ielBit);
    else KX023_ClearBit(ctx, regAddr, ielBit);
}

static void ConfigPhysicalInterruptEvent(KX023_Ctx_t *ctx, int interrupt_number, uint8_t buffer_full, uint8_t watermark, uint8_t data_ready, uint8_t tap, uint8_t wake_up, uint8_t tilt) {
    uint8_t regAddress;
    if (interrupt_number == 1) regAddress = KX023_INC4_REG;
    else if (interrupt_number == 2) regAddress = KX023_INC6_REG;
    else return;

    uint8_t regValue = 0;
    if (buffer_full) regValue |= (1 << KX023_INC4_6_BFI_BIT);
    if (watermark) regValue |= (1 << KX023_INC4_6_WMI_BIT);
    if (data_ready) regValue |= (1 << KX023_INC4_6_DRDYI_BIT);
    if (tap) regValue |= (1 << KX023_INC4_6_TDTI_BIT);
    if (wake_up) regValue |= (1 << KX023_INC4_6_WUFI_BIT);
    if (tilt) regValue |= (1 << KX023_INC4_6_TPI_BIT);
    KX023_WriteRegister(ctx, regAddress, regValue);
}

static void EnableSampleBuffer(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_BUF_CNTL2_REG, KX023_BUF_CNTL2_BUFE_BIT);
}

static void SetSampleBufferResolution(KX023_Ctx_t *ctx, uint8_t use_16_bit_res) {
    if (use_16_bit_res) KX023_SetBit(ctx, KX023_BUF_CNTL2_REG, KX023_BUF_CNTL2_BRES_BIT);
    else KX023_ClearBit(ctx, KX023_BUF_CNTL2_REG, KX023_BUF_CNTL2_BRES_BIT);
}

static void EnableBufferFullInterrupt(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_BUF_CNTL2_REG, KX023_BUF_CNTL2_BFIE_BIT);
}

static void SetTiltPositionFunctionOutputDataRate(KX023_Ctx_t *ctx, TiltPositionOutputDataRate_t odr) {
    uint8_t reg_value = KX023_ReadRegister(ctx, KX023_CNTL3_REG);
    reg_value &= ~KX023_CNTL3_OTP_MASK;
    reg_value |= (((uint8_t)odr << KX023_CNTL3_OTP0_BIT) & KX023_CNTL3_OTP_MASK);
    KX023_WriteRegister(ctx, KX023_CNTL3_REG, reg_value);
}

static void SetDirectionalTapFunctionOutputDataRate(KX023_Ctx_t *ctx, DirectionalTapOutputDataRate_t odr) {
    uint8_t reg_value = KX023_ReadRegister(ctx, KX023_CNTL3_REG);
    reg_value &= ~KX023_CNTL3_OTDT_MASK;
    reg_value |= (((uint8_t)odr << KX023_CNTL3_OTDT0_BIT) & KX023_CNTL3_OTDT_MASK);
    KX023_WriteRegister(ctx, KX023_CNTL3_REG, reg_value);
}

static void SetMotionWakeUpFunctionOutputDataRate(KX023_Ctx_t *ctx, MotionWakeUpOutputDataRate_t odr) {
    uint8_t reg_value = KX023_ReadRegister(ctx, KX023_CNTL3_REG);
    reg_value &= ~KX023_CNTL3_OWUF_MASK;
    reg_value |= (((uint8_t)odr << KX023_CNTL3_OWUF0_BIT) & KX023_CNTL3_OWUF_MASK);
    KX023_WriteRegister(ctx, KX023_CNTL3_REG, reg_value);
}

static void SetMotionWakeUpInterruptDirection(KX023_Ctx_t *ctx, uint8_t xn, uint8_t xp, uint8_t yn, uint8_t yp, uint8_t zn, uint8_t zp) {
    uint8_t regValue = 0;
    if (xn) regValue |= (1 << KX023_INC2_XNWUE_BIT);
    if (xp) regValue |= (1 << KX023_INC2_XPWUE_BIT);
    if (yn) regValue |= (1 << KX023_INC2_YNWUE_BIT);
    if (yp) regValue |= (1 << KX023_INC2_YPWUE_BIT);
    if (zn) regValue |= (1 << KX023_INC2_ZNWUE_BIT);
    if (zp) regValue |= (1 << KX023_INC2_ZPWUE_BIT);
    KX023_WriteRegister(ctx, KX023_INC2_REG, regValue);
}

static void SetInterruptMotionWakeUpTimer(KX023_Ctx_t *ctx, MotionWakeUpOutputDataRate_t odr, float desired_delay_time_sec) {
    float odr_value = 0;
    switch (odr) {
        case (KX023_MWUODR_0_781HZ): odr_value = 0.781f; break;
        case (KX023_MWUODR_1_563HZ): odr_value = 1.563f; break;
        case (KX023_MWUODR_3_125HZ): odr_value = 3.125f; break;
        case (KX023_MWUODR_6_25HZ): odr_value = 6.25f; break;
        case (KX023_MWUODR_12_5HZ): odr_value = 12.5f; break;
        case (KX023_MWUODR_25HZ): odr_value = 25.0f; break;
        case (KX023_MWUODR_50HZ): odr_value = 50.0f; break;
        case (KX023_MWUODR_100HZ): odr_value = 100.0f; break;
        default: return;
    }
    uint8_t count = (uint8_t)(desired_delay_time_sec * odr_value);
    KX023_WriteRegister(ctx, KX023_WUFC_REG, count);
}

static void SetInterruptMotionWakeUpThreshold(KX023_Ctx_t *ctx, float desired_threshold) {
    uint8_t count = (uint8_t)(desired_threshold * 16.0f);
    KX023_WriteRegister(ctx, KX023_ATH_REG, count);
}

static void SetTiltPositionInterruptDirection(KX023_Ctx_t *ctx, uint8_t xn, uint8_t xp, uint8_t yn, uint8_t yp, uint8_t zn, uint8_t zp) {
    uint8_t regValue = 0;
    if (xn) regValue |= (1 << KX023_CNTL2_LEM_BIT);
    if (xp) regValue |= (1 << KX023_CNTL2_RIM_BIT);
    if (yn) regValue |= (1 << KX023_CNTL2_DOM_BIT);
    if (yp) regValue |= (1 << KX023_CNTL2_UPM_BIT);
    if (zn) regValue |= (1 << KX023_CNTL2_FDM_BIT);
    if (zp) regValue |= (1 << KX023_CNTL2_FUM_BIT);
    KX023_WriteRegister(ctx, KX023_CNTL2_REG, regValue);
}

static void ConfigTiltPositionTimer(KX023_Ctx_t *ctx, TiltPositionOutputDataRate_t odr, float desired_delay_time_msec) {
    float ms_time_per_cycle = 0;
    switch (odr) {
        case KX023_TPODR_1_563HZ: ms_time_per_cycle = 639.795f; break;
        case KX023_TPODR_6_25HZ: ms_time_per_cycle = 160.0f; break;
        case KX023_TPODR_12_5HZ: ms_time_per_cycle = 80.0f; break;
        case KX023_TPODR_50HZ: ms_time_per_cycle = 20.0f; break;
        default: return;
    }
    uint8_t desired_delay_cycle = (uint8_t)(desired_delay_time_msec / ms_time_per_cycle);
    KX023_WriteRegister(ctx, KX023_TILT_TIMER_REG, desired_delay_cycle);
}

static void SetTiltPositionHysteresisAngle(KX023_Ctx_t *ctx) {
    KX023_WriteRegister(ctx, KX023_HYST_SET_REG, 0x14);
}

static void SetDirectionalTapInterruptDirection(KX023_Ctx_t *ctx, uint8_t xn, uint8_t xp, uint8_t yn, uint8_t yp, uint8_t zn, uint8_t zp) {
    uint8_t regValue = 0;
    if (xn) regValue |= (1 << KX023_INC3_TLEM_BIT);
    if (xp) regValue |= (1 << KX023_INC3_TRIM_BIT);
    if (yn) regValue |= (1 << KX023_INC3_TDOM_BIT);
    if (yp) regValue |= (1 << KX023_INC3_TUPM_BIT);
    if (zn) regValue |= (1 << KX023_INC3_TFDM_BIT);
    if (zp) regValue |= (1 << KX023_INC3_TFUM_BIT);
    KX023_WriteRegister(ctx, KX023_INC3_REG, regValue);
}

static void EnableSingleTap(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_TDTRC_REG, KX023_TDTRC_STRE_BIT);
}

static void EnableDoubleTap(KX023_Ctx_t *ctx) {
    KX023_SetBit(ctx, KX023_TDTRC_REG, KX023_TDTRC_DTRE_BIT);
}

static void SetDoubleTapMinimumTimeSeparation(KX023_Ctx_t *ctx, DirectionalTapOutputDataRate_t odr, float desired_minimum_time_separation_between_taps) {
    float num_of_cycle = 0;
    // Calculation based on original cpp
    switch (odr) {
        case KX023_DTODR_12_5HZ: num_of_cycle = desired_minimum_time_separation_between_taps * 12.5f; break;
        case KX023_DTODR_25HZ: num_of_cycle = desired_minimum_time_separation_between_taps * 25.0f; break;
        case KX023_DTODR_50HZ: num_of_cycle = desired_minimum_time_separation_between_taps * 50.0f; break;
        case KX023_DTODR_100HZ: num_of_cycle = desired_minimum_time_separation_between_taps * 100.0f; break;
        case KX023_DTODR_200HZ: num_of_cycle = desired_minimum_time_separation_between_taps * 200.0f; break;
        case KX023_DTODR_400HZ:
        case KX023_DTODR_800HZ:
        case KX023_DTODR_1600HZ: num_of_cycle = desired_minimum_time_separation_between_taps * 400.0f; break;
        default: return;
    }
    KX023_WriteRegister(ctx, KX023_TDTC_REG, (uint8_t)num_of_cycle);
}

static void SetTapHighThresholdValue(KX023_Ctx_t *ctx, uint8_t value) {
    KX023_WriteRegister(ctx, KX023_TTH_REG, value);
}

static void SetTapLowThresholdValue(KX023_Ctx_t *ctx, uint8_t value) {
    KX023_WriteRegister(ctx, KX023_TTL_REG, value);
}
