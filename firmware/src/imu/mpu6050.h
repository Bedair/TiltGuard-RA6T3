/*
 * mpu6050.h
 *
 * MPU-6050 IMU over r_i3c (I2C legacy mode, g_i3c_imu, static address 0x68).
 *
 * Reading is driven by the 1 kHz GPT0 tick: mpu6050_on_tick() (called from
 * control_tick_isr) starts a non-blocking 14-byte burst read from register
 * 0x3B; the I3C completion callback (ISR context, lower priority than the
 * tick) stores the raw sample. mpu6050_update() turns the latest raw sample
 * into an accelerometer angle and a complementary-filtered pitch angle; it
 * does the atan2()/float work, so it must be called from the main loop, not
 * from an ISR.
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
} mpu6050_angle_t;

/*
 * Opens g_i3c_imu, checks WHO_AM_I, wakes the device onto the gyro-X PLL
 * clock, configures DLPF/sample-rate/full-scale, and averages ~1 s of
 * gyro_x samples for the bias (robot must be held still). All I3C
 * transactions here are blocking (polled) since the 1 kHz tick is not
 * running yet. Checks every fsp_err_t and returns the first failure.
 */
fsp_err_t mpu6050_init(void);

/* Call from control_tick_isr (GPT0, 1 kHz). Starts the next burst read if
 * the previous one has completed; otherwise counts a missed read. No
 * blocking FSP calls. */
void mpu6050_on_tick(void);

/* Call once per main-loop iteration. If a new raw sample has arrived since
 * the last call, computes the accel angle, gyro rate, and complementary
 * filter update (fixed dt = 1 ms). Does float/atan2 work - main loop only. */
void mpu6050_update(void);

/* Consistent snapshot, safe to call from the main loop. */
void mpu6050_get_angle(mpu6050_angle_t * p_angle);

uint32_t mpu6050_get_sequence(void);
uint32_t mpu6050_get_missed_read_count(void);
uint32_t mpu6050_get_failed_read_count(void);
uint32_t mpu6050_get_last_read_cycles(void);

/* FSP callback for g_i3c_imu. Registered via the FSP configurator; not
 * meant to be called directly by application code. */
#ifndef mpu6050_i3c_callback
void mpu6050_i3c_callback(i3c_callback_args_t const * const p_args);
#endif

#endif /* MPU6050_H_ */
