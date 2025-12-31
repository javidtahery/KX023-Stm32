/*
 * kx023_ll.h
 *
 *  Created on: Dec 29, 2025
 *      Author: USER
 */
#ifndef __KIONIX_KX023_LL_H__
#define __KIONIX_KX023_LL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#include <math.h>

/* These files must exist in your project include path */
#include "kx023_data_types.h"
#include "kx023_reg_map.h"
#include "kx023_status.h"

#define KX023_DEFAULT_ADDR 0x1E
#define I2C_TIMEOUT_COUNT  100000

/**
 * @brief KX023 Context Structure for Interface and State Management
 */
typedef struct {
    uint8_t is_spi;             // 1: SPI, 0: I2C
    I2C_TypeDef *i2c_instance;  // e.g., I2C3
    SPI_TypeDef *spi_instance;  // e.g., SPI1

    // CS Pin configuration for SPI mode
    GPIO_TypeDef *cs_port;
    uint32_t cs_pin;

    uint8_t slave_address;      // 8-bit shifted I2C address

    AccelerationRange_t _accelerationRange;
} KX023_Ctx_t;

// ================================================================
// Initialization Functions
// ================================================================

void KX023_Init_I2C(KX023_Ctx_t *ctx, I2C_TypeDef *i2c, uint8_t addr);
void KX023_Init_SPI(KX023_Ctx_t *ctx, SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint32_t cs_pin);

KX023_Status_t KX023_Begin(KX023_Ctx_t *ctx);
void KX023_End(KX023_Ctx_t *ctx);

KX023_Interrupt_Type_t KX023_GetInterruptType(KX023_Ctx_t *ctx);
void KX023_ClearInterrupt(KX023_Ctx_t *ctx);
void KX023_ConfigPhysicalInterruptPin(KX023_Ctx_t *ctx, int interrupt_number, PhysicalInterruptParameter_t params);

void KX023_SoftwareReset(KX023_Ctx_t *ctx);
void KX023_SetStandbyMode(KX023_Ctx_t *ctx);
void KX023_SetOperatingMode(KX023_Ctx_t *ctx);

// ================================================================
// Core Operational Functions
// ================================================================

KX023_Status_t KX023_ConfigAsynchronousReadBack(KX023_Ctx_t *ctx, AccelerationRange_t range, AccelerationOutputDataRate_t odr);
KX023_Status_t KX023_ReadAsynchronousData(KX023_Ctx_t *ctx, float *x, float *y, float *z);

KX023_Status_t KX023_ConfigSynchronousInterruptReadBack(KX023_Ctx_t *ctx, AccelerationRange_t range, AccelerationOutputDataRate_t odr);

KX023_Status_t KX023_ConfigBufferFullInterrupt(KX023_Ctx_t *ctx, AccelerationRange_t range, AccelerationOutputDataRate_t odr);
KX023_Status_t KX023_ReadBuffer(KX023_Ctx_t *ctx, uint8_t *buffer);

KX023_Status_t KX023_ConfigWakeUpFunction(KX023_Ctx_t *ctx, AccelerationRange_t range, MotionWakeUpOutputDataRate_t odr, float desired_delay_time_sec, float desired_threshold, DirectionInfoParams_t direction);

KX023_Status_t KX023_ConfigActivateTiltPosition(KX023_Ctx_t *ctx, AccelerationRange_t range, TiltPositionOutputDataRate_t odr, float desired_delay_time_msec, DirectionInfoParams_t direction);

KX023_Status_t KX023_ConfigActivateTapFunction(KX023_Ctx_t *ctx, AccelerationRange_t range, DirectionalTapOutputDataRate_t odr, DirectionInfoParams_t direction, TapMode_t tap_mode);

#ifdef __cplusplus
}
#endif

#endif /* __KIONIX_KX023_LL_H__ */
