/*
 * mpu6050.h
 *
 * MPU-6050 IMU over r_i3c (I2C legacy mode, g_i3c_imu, static address 0x68).
 *
 * Reading is driven by the 1 kHz GPT0 tick: mpu6050_on_tick() (called from
 * control_tick_isr) starts a non-blocking 14-byte burst read from register
 * 0x3B. The I3C read-complete callback (ISR context, priority 5 - lower
 * than the tick, higher than nothing it needs to preempt) does the
 * accel-angle/complementary-filter math immediately, right there in the
 * callback, instead of waiting for the next main-loop poll. That trades a
 * bit of I3C-ISR execution time (measured, see
 * mpu6050_get_last_control_step_cycles()) for lower latency: a fresh
 * sample is usable ~0.4 ms after the I3C transfer completes, instead of up
 * to ~1.4 ms later if it had to wait for the next 1 kHz tick's main-loop
 * pass. This is deliberately NOT the same tradeoff made elsewhere in this
 * project (e.g. encoders/battery keep their math in the main loop) -
 * angle freshness matters more here because this is what a balance
 * controller will read from.
 *
 * Axis convention (pitch) comes from the ELEGOO Tumbller stock firmware
 * (KalmanFilter.cpp): accel angle = atan2(accel_y, accel_z), rate = gyro_x.
 * This board uses the same MPU-6050 mounting, so the same pair applies.
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include "bsp_api.h"
#include "r_i3c_api.h"

typedef struct st_mpu6050_angle
{
    float accel_angle_deg;    /* atan2(accel_y, accel_z), degrees. No bias, but noisy. */
    float filtered_angle_deg; /* Complementary filter output, degrees. */
    float gyro_rate_dps;      /* Bias-corrected gyro_x rate, degrees/second. */
    float gyro_z_dps;         /* Bias-corrected gyro_z (yaw) rate, degrees/second, sensor sign. */
} mpu6050_angle_t;

/* mpu6050_get_bus_recovery_result(): 0..9 = SCL clocks needed to free SDA
 * (0 = bus was idle), or one of these. */
#define MPU6050_BUS_RECOVERY_NOT_RUN   (0xFFFFFFFFUL)
#define MPU6050_BUS_RECOVERY_SDA_STUCK (0xFFFFFFFEUL) /* SDA still low after 9 clocks. */
#define MPU6050_BUS_RECOVERY_SCL_STUCK (0xFFFFFFFDUL) /* SCL won't go high: no pull-up, or shorted. */

uint32_t mpu6050_get_bus_recovery_result(void);

/*
 * Runs I2C bus recovery (frees an SDA left held low by a mid-transfer
 * reset), then opens g_i3c_imu, checks WHO_AM_I, wakes the device onto the gyro-X PLL
 * clock, configures DLPF/sample-rate/full-scale, and averages ~1 s of
 * gyro_x/gyro_z samples for the biases (robot must be held still). All I3C
 * transactions here are blocking (polled) since the 1 kHz tick is not
 * running yet. Checks every fsp_err_t and returns the first failure.
 */
fsp_err_t mpu6050_init(void);

/* Call from control_tick_isr (GPT0, 1 kHz). Starts the next burst read if
 * the previous one has completed; otherwise counts a missed read. No
 * blocking FSP calls. */
void mpu6050_on_tick(void);

/* Consistent snapshot, safe to call from the main loop. */
void mpu6050_get_angle(mpu6050_angle_t * p_angle);

uint32_t mpu6050_get_sequence(void);
uint32_t mpu6050_get_missed_read_count(void);
uint32_t mpu6050_get_failed_read_count(void);
uint32_t mpu6050_get_last_read_cycles(void);

/* DWT cycles spent on the accel-angle/filter math for the most recently
 * completed sample, measured inside the I3C callback. */
uint32_t mpu6050_get_last_control_step_cycles(void);

/* FSP callback for g_i3c_imu. Registered via the FSP configurator; not
 * meant to be called directly by application code. */
#ifndef mpu6050_i3c_callback
void mpu6050_i3c_callback(i3c_callback_args_t const * const p_args);
#endif

#endif /* MPU6050_H_ */
