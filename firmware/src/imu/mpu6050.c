/*
 * mpu6050.c
 */

#include <math.h>
#include <stdbool.h>
#include "mpu6050.h"
#include "hal_data.h"
#include "../control/balance.h"

/* MPU-6050 register map (subset used here). */
#define MPU6050_REG_SMPLRT_DIV     (0x19U)
#define MPU6050_REG_CONFIG         (0x1AU)
#define MPU6050_REG_GYRO_CONFIG    (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG   (0x1CU)
#define MPU6050_REG_GYRO_XOUT_H    (0x43U)
#define MPU6050_REG_PWR_MGMT_1     (0x6BU)
#define MPU6050_REG_WHO_AM_I       (0x75U)
#define MPU6050_REG_ACCEL_XOUT_H   (0x3BU)

#define MPU6050_I2C_ADDRESS        (0x68U)
#define MPU6050_WHO_AM_I_EXPECTED  (0x68U)
#define MPU6050_DEVICE_INDEX       (0U)

/*
 * This MCU is the only master on an I2C-only point-to-point bus (no other
 * I3C device, no dynamic address assignment). R_I3C_DeviceCfgSet still
 * requires a self-identity; the value is never used to address us.
 */
#define MPU6050_MASTER_OWN_ADDRESS (0x0AU)

#define MPU6050_SAMPLE_BYTES              (14U)
#define MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS (65.5f) /* FS_SEL = 1, +/-500 dps. */
#define MPU6050_RAD_TO_DEG                (57.29577951308232f)

/* ~1 s and a generous 10 ms per-transaction timeout, both at the 200 MHz core clock. */
#define MPU6050_GYRO_CAL_DURATION_CYCLES  (200000000U)
#define MPU6050_BLOCKING_TIMEOUT_CYCLES   (2000000U)

#define MPU6050_FILTER_DT_S                (0.001f) /* 1 kHz tick. */

/* Complementary filter time constant: below ~1/(2*pi*tau) the angle follows
 * the accelerometer, above it the integrated gyro. On the floor the wheels
 * accelerate the robot hard, and atan2(ay, az) reads that horizontal
 * acceleration as tilt (~6 deg per 1 m/s^2), so the accel must only correct
 * slow gyro drift. The stock Kalman filter (Q_angle 0.001, R_angle 0.5,
 * dt 5 ms) settles to an accel gain of ~0.003 per step, i.e. tau ~1.5 s.
 * The old alpha 0.98 at 1 kHz was tau = 49 ms: fine lifted, but on the floor
 * the wheel acceleration fed straight into the angle (3-5 Hz oscillation in
 * the first floor test). tau = 1 s: alpha = tau / (tau + dt) = 0.999. A
 * residual gyro bias b gives a steady error of b * tau (0.1 dps -> 0.1 deg). */
#define MPU6050_COMPLEMENTARY_FILTER_TAU_S (1.0f)
#define MPU6050_COMPLEMENTARY_FILTER_ALPHA \
    (MPU6050_COMPLEMENTARY_FILTER_TAU_S / (MPU6050_COMPLEMENTARY_FILTER_TAU_S + MPU6050_FILTER_DT_S))

typedef enum e_mpu6050_phase
{
    MPU6050_PHASE_IDLE = 0,
    MPU6050_PHASE_WRITE_THEN_READ,
    MPU6050_PHASE_READING,
    MPU6050_PHASE_WRITE_ONLY,
} mpu6050_phase_t;

typedef struct st_mpu6050_raw_sample
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_z;
} mpu6050_raw_sample_t;

/* Transfer state machine, shared by the blocking init helpers and the tick-driven path. */
static volatile mpu6050_phase_t s_phase           = MPU6050_PHASE_IDLE;
static uint8_t                  s_reg_addr;
static uint8_t                  s_write_payload[2];
static uint8_t                  s_rx_buffer[MPU6050_SAMPLE_BYTES];
static uint8_t                * s_p_rx_buffer;
static uint32_t                 s_rx_length;
static volatile bool            s_is_tick_transfer;

/* Blocking (init-time) completion signal. */
static volatile bool     s_blocking_done;
static volatile uint32_t s_blocking_event_status;

/* Tick-driven (1 kHz) state. */
static volatile bool     s_read_busy;
static volatile uint32_t s_read_start_cyccnt;
static volatile uint32_t s_last_read_cycles;
static volatile uint32_t s_missed_read_count;
static volatile uint32_t s_failed_read_count;
static volatile uint32_t s_sequence;
static mpu6050_raw_sample_t s_latest_raw;

/* Filter state. Written from the I3C read-complete callback (ISR, priority
 * 5), read via mpu6050_get_angle()'s snapshot from the main loop. */
static float                    s_gyro_x_bias_raw;
static float                    s_gyro_z_bias_raw;
static volatile mpu6050_angle_t s_angle;
static volatile uint32_t        s_last_control_step_cycles;

/*
 * I2C bus recovery (UM10204 section 3.1.16). The MPU-6050 runs from the robot's
 * battery-fed 5 V rail, so it stays powered through an FPB reset. A reset in
 * the middle of a read can leave it holding SDA low, waiting for clocks that
 * never come. The I3C master then cannot issue a START, and every transfer
 * times out. Fix: with the pins temporarily as open-drain GPIO, clock SCL
 * until the slave releases SDA (at most 9 clocks), then send a STOP.
 */
#define MPU6050_SCL_PIN                (BSP_IO_PORT_01_PIN_00) /* P100. */
#define MPU6050_SDA_PIN                (BSP_IO_PORT_01_PIN_01) /* P101. */
#define MPU6050_BUS_RECOVERY_MAX_CLOCKS (9U)
#define MPU6050_BUS_HALF_PERIOD_US     (5U)                    /* ~100 kHz. */

/* Same value as ra_gen/pin_data.c, used to hand the pins back to the peripheral. */
#define MPU6050_PIN_CFG_PERIPHERAL     ((uint32_t) IOPORT_CFG_DRIVE_HIGH | (uint32_t) IOPORT_CFG_PERIPHERAL_PIN | \
                                        (uint32_t) IOPORT_PERIPHERAL_IIC)
#define MPU6050_PIN_CFG_OD_HIGH        ((uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT | \
                                        (uint32_t) IOPORT_CFG_NMOS_ENABLE |           \
                                        (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH)

static uint32_t s_bus_recovery_result = MPU6050_BUS_RECOVERY_NOT_RUN;

static void mpu6050_bus_delay (void)
{
    R_BSP_SoftwareDelay(MPU6050_BUS_HALF_PERIOD_US, BSP_DELAY_UNITS_MICROSECONDS);
}

static fsp_err_t mpu6050_bus_recover (void)
{
    /* Release both lines (open-drain high), SCL first. */
    fsp_err_t err = R_IOPORT_PinCfg(&g_ioport_ctrl, MPU6050_SCL_PIN, MPU6050_PIN_CFG_OD_HIGH);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, MPU6050_SDA_PIN, MPU6050_PIN_CFG_OD_HIGH);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    mpu6050_bus_delay();

    uint32_t clocks = 0U;

    R_BSP_PinAccessEnable();

    while ((BSP_IO_LEVEL_LOW == R_BSP_PinRead(MPU6050_SDA_PIN)) && (clocks < MPU6050_BUS_RECOVERY_MAX_CLOCKS))
    {
        R_BSP_PinWrite(MPU6050_SCL_PIN, BSP_IO_LEVEL_LOW);
        mpu6050_bus_delay();
        R_BSP_PinWrite(MPU6050_SCL_PIN, BSP_IO_LEVEL_HIGH);
        mpu6050_bus_delay();
        clocks++;
    }

    bool const sda_released = (BSP_IO_LEVEL_HIGH == R_BSP_PinRead(MPU6050_SDA_PIN));

    if (sda_released && (clocks > 0U))
    {
        /* STOP: SDA low -> high while SCL is high, so the slave sees a clean bus. */
        R_BSP_PinWrite(MPU6050_SCL_PIN, BSP_IO_LEVEL_LOW);
        mpu6050_bus_delay();
        R_BSP_PinWrite(MPU6050_SDA_PIN, BSP_IO_LEVEL_LOW);
        mpu6050_bus_delay();
        R_BSP_PinWrite(MPU6050_SCL_PIN, BSP_IO_LEVEL_HIGH);
        mpu6050_bus_delay();
        R_BSP_PinWrite(MPU6050_SDA_PIN, BSP_IO_LEVEL_HIGH);
        mpu6050_bus_delay();
    }

    bool const scl_high = (BSP_IO_LEVEL_HIGH == R_BSP_PinRead(MPU6050_SCL_PIN));

    R_BSP_PinAccessDisable();

    if (!sda_released)
    {
        s_bus_recovery_result = MPU6050_BUS_RECOVERY_SDA_STUCK;
    }
    else if (!scl_high)
    {
        s_bus_recovery_result = MPU6050_BUS_RECOVERY_SCL_STUCK;
    }
    else
    {
        s_bus_recovery_result = clocks; /* 0 = bus was already idle. */
    }

    /* Hand the pins back to the I3C peripheral. */
    err = R_IOPORT_PinCfg(&g_ioport_ctrl, MPU6050_SCL_PIN, MPU6050_PIN_CFG_PERIPHERAL);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, MPU6050_SDA_PIN, MPU6050_PIN_CFG_PERIPHERAL);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    return ((MPU6050_BUS_RECOVERY_SDA_STUCK == s_bus_recovery_result) ||
            (MPU6050_BUS_RECOVERY_SCL_STUCK == s_bus_recovery_result)) ? FSP_ERR_INVALID_HW_CONDITION : FSP_SUCCESS;
}

uint32_t mpu6050_get_bus_recovery_result (void)
{
    return s_bus_recovery_result;
}

static void mpu6050_enable_dwt (void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

static fsp_err_t mpu6050_wait_blocking (void)
{
    uint32_t const start_cyccnt = DWT->CYCCNT;

    while (!s_blocking_done)
    {
        if ((DWT->CYCCNT - start_cyccnt) > MPU6050_BLOCKING_TIMEOUT_CYCLES)
        {
            s_phase = MPU6050_PHASE_IDLE;

            return FSP_ERR_TIMEOUT;
        }
    }

    if (I3C_EVENT_STATUS_SUCCESS != s_blocking_event_status)
    {
        return FSP_ERR_ABORTED;
    }

    return FSP_SUCCESS;
}

static fsp_err_t mpu6050_blocking_write_reg (uint8_t reg, uint8_t value)
{
    s_write_payload[0] = reg;
    s_write_payload[1] = value;
    s_is_tick_transfer = false;
    s_blocking_done     = false;
    s_phase             = MPU6050_PHASE_WRITE_ONLY;

    fsp_err_t err = R_I3C_Write(&g_i3c_imu_ctrl, s_write_payload, 2U, false);

    if (FSP_SUCCESS != err)
    {
        s_phase = MPU6050_PHASE_IDLE;

        return err;
    }

    return mpu6050_wait_blocking();
}

static fsp_err_t mpu6050_blocking_read_regs (uint8_t reg, uint8_t * p_buffer, uint32_t length)
{
    s_reg_addr          = reg;
    s_p_rx_buffer        = p_buffer;
    s_rx_length          = length;
    s_is_tick_transfer   = false;
    s_blocking_done      = false;
    s_phase              = MPU6050_PHASE_WRITE_THEN_READ;

    fsp_err_t err = R_I3C_Write(&g_i3c_imu_ctrl, &s_reg_addr, 1U, true);

    if (FSP_SUCCESS != err)
    {
        s_phase = MPU6050_PHASE_IDLE;

        return err;
    }

    return mpu6050_wait_blocking();
}

static fsp_err_t mpu6050_calibrate_gyro_bias (void)
{
    int64_t  sum_x          = 0;
    int64_t  sum_z          = 0;
    uint32_t count          = 0U;
    uint8_t  buffer[6];     /* GYRO_XOUT_H..GYRO_ZOUT_L. */
    uint32_t const start_cyccnt = DWT->CYCCNT;

    while ((DWT->CYCCNT - start_cyccnt) < MPU6050_GYRO_CAL_DURATION_CYCLES)
    {
        fsp_err_t err = mpu6050_blocking_read_regs(MPU6050_REG_GYRO_XOUT_H, buffer, sizeof(buffer));

        if (FSP_SUCCESS != err)
        {
            return err;
        }

        int16_t const gx_raw = (int16_t) ((uint16_t) ((buffer[0] << 8) | buffer[1]));
        int16_t const gz_raw = (int16_t) ((uint16_t) ((buffer[4] << 8) | buffer[5]));
        sum_x += gx_raw;
        sum_z += gz_raw;
        count++;
    }

    if (0U == count)
    {
        return FSP_ERR_TIMEOUT;
    }

    s_gyro_x_bias_raw = (float) sum_x / (float) count;
    s_gyro_z_bias_raw = (float) sum_z / (float) count;

    return FSP_SUCCESS;
}

fsp_err_t mpu6050_init (void)
{
    mpu6050_enable_dwt();

    s_phase              = MPU6050_PHASE_IDLE;
    s_read_busy          = false;
    s_missed_read_count  = 0U;
    s_failed_read_count  = 0U;
    s_sequence           = 0U;
    s_last_control_step_cycles = 0U;
    s_angle.accel_angle_deg    = 0.0f;
    s_angle.filtered_angle_deg = 0.0f;
    s_angle.gyro_rate_dps      = 0.0f;
    s_angle.gyro_z_dps         = 0.0f;

    fsp_err_t err = mpu6050_bus_recover();

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_I3C_Open(&g_i3c_imu_ctrl, &g_i3c_imu_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    i3c_device_cfg_t const master_cfg =
    {
        .static_address  = MPU6050_MASTER_OWN_ADDRESS,
        .dynamic_address = MPU6050_MASTER_OWN_ADDRESS,
    };

    err = R_I3C_DeviceCfgSet(&g_i3c_imu_ctrl, &master_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    i3c_device_table_cfg_t const device_cfg =
    {
        .static_address        = MPU6050_I2C_ADDRESS,
        .dynamic_address       = 0U,
        .device_protocol       = I3C_DEVICE_PROTOCOL_I2C,
        .ibi_accept            = false,
        .master_request_accept = false,
        .ibi_payload           = false,
    };

    err = R_I3C_MasterDeviceTableSet(&g_i3c_imu_ctrl, MPU6050_DEVICE_INDEX, &device_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_I3C_Enable(&g_i3c_imu_ctrl);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_I3C_DeviceSelect(&g_i3c_imu_ctrl, MPU6050_DEVICE_INDEX, I3C_BITRATE_MODE_I2C_STDBR);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    uint8_t who_am_i = 0U;
    err = mpu6050_blocking_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1U);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    if (MPU6050_WHO_AM_I_EXPECTED != who_am_i)
    {
        return FSP_ERR_INVALID_HW_CONDITION;
    }

    /* Wake, gyro-X PLL as clock reference (CLKSEL = 1; PWR_MGMT_1 = 0x00 would leave it on
     * the internal 8 MHz oscillator instead). */
    err = mpu6050_blocking_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01U);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* DLPF_CFG = 0: accel 260 Hz / gyro 256 Hz bandwidth, the power-on
     * default the stock ELEGOO firmware runs with (its mpu.initialize()
     * never writes CONFIG). With DLPF_CFG = 0 the gyro output rate is
     * 8 kHz instead of 1 kHz, so SMPLRT_DIV below changes with it. */
    err = mpu6050_blocking_write_reg(MPU6050_REG_CONFIG, 0x00U);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Sample rate = Gyro_Output_Rate / (1 + SMPLRT_DIV) = 8 kHz / (1 + 7) = 1 kHz,
     * matching the 1 kHz read tick. (The accelerometer outputs 1 kHz regardless.) */
    err = mpu6050_blocking_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07U);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Gyro full scale +/-500 dps (FS_SEL = 1). */
    err = mpu6050_blocking_write_reg(MPU6050_REG_GYRO_CONFIG, 0x08U);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Accel full scale +/-4 g (AFS_SEL = 1). */
    err = mpu6050_blocking_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x08U);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = mpu6050_calibrate_gyro_bias();

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Seed the filter from one fresh accel reading so it starts near the true angle
     * instead of ramping up from 0 if the robot wasn't held perfectly upright. */
    uint8_t seed_buffer[MPU6050_SAMPLE_BYTES];
    err = mpu6050_blocking_read_regs(MPU6050_REG_ACCEL_XOUT_H, seed_buffer, sizeof(seed_buffer));

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    int16_t const accel_y = (int16_t) ((uint16_t) ((seed_buffer[2] << 8) | seed_buffer[3]));
    int16_t const accel_z = (int16_t) ((uint16_t) ((seed_buffer[4] << 8) | seed_buffer[5]));

    s_angle.accel_angle_deg    = atan2f((float) accel_y, (float) accel_z) * MPU6050_RAD_TO_DEG;
    s_angle.filtered_angle_deg = s_angle.accel_angle_deg;

    return FSP_SUCCESS;
}

void mpu6050_on_tick (void)
{
    if (s_read_busy)
    {
        s_missed_read_count++;

        return;
    }

    s_read_busy          = true;
    s_is_tick_transfer   = true;
    s_phase              = MPU6050_PHASE_WRITE_THEN_READ;
    s_p_rx_buffer        = s_rx_buffer;
    s_rx_length          = MPU6050_SAMPLE_BYTES;
    s_reg_addr           = MPU6050_REG_ACCEL_XOUT_H;
    s_read_start_cyccnt  = DWT->CYCCNT;

    fsp_err_t err = R_I3C_Write(&g_i3c_imu_ctrl, &s_reg_addr, 1U, true);

    if (FSP_SUCCESS != err)
    {
        s_failed_read_count++;
        s_read_busy = false;
        s_phase     = MPU6050_PHASE_IDLE;
    }
}

static void mpu6050_tick_transfer_succeeded (uint8_t const * p_data, uint32_t length)
{
    uint32_t const now = DWT->CYCCNT;
    s_last_read_cycles = now - s_read_start_cyccnt;

    if (MPU6050_SAMPLE_BYTES == length)
    {
        mpu6050_raw_sample_t raw;
        raw.accel_x = (int16_t) ((uint16_t) ((p_data[0] << 8) | p_data[1]));
        raw.accel_y = (int16_t) ((uint16_t) ((p_data[2] << 8) | p_data[3]));
        raw.accel_z = (int16_t) ((uint16_t) ((p_data[4] << 8) | p_data[5]));
        raw.gyro_x  = (int16_t) ((uint16_t) ((p_data[8] << 8) | p_data[9]));
        raw.gyro_z  = (int16_t) ((uint16_t) ((p_data[12] << 8) | p_data[13]));

        s_latest_raw = raw;
        s_sequence++;

        /* "Control step": the accel-angle/complementary-filter math, done
         * here (ISR context) rather than the main loop, so a fresh sample
         * is usable ~0.4 ms after the transfer completes instead of up to
         * ~1.4 ms later. Timed so this tradeoff can be checked, not assumed. */
        uint32_t const control_step_start = DWT->CYCCNT;

        float const accel_angle_deg = atan2f((float) raw.accel_y, (float) raw.accel_z) * MPU6050_RAD_TO_DEG;
        float const gyro_rate_dps   = ((float) raw.gyro_x - s_gyro_x_bias_raw) / MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS;
        float const gyro_z_dps      = ((float) raw.gyro_z - s_gyro_z_bias_raw) / MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS;

        float const filtered_angle_deg =
            (MPU6050_COMPLEMENTARY_FILTER_ALPHA * (s_angle.filtered_angle_deg + (gyro_rate_dps * MPU6050_FILTER_DT_S)))
            + ((1.0f - MPU6050_COMPLEMENTARY_FILTER_ALPHA) * accel_angle_deg);

        s_angle.accel_angle_deg    = accel_angle_deg;
        s_angle.filtered_angle_deg = filtered_angle_deg;
        s_angle.gyro_rate_dps      = gyro_rate_dps;
        s_angle.gyro_z_dps         = gyro_z_dps;

        /* Balance control law on this fresh sample (src/control/balance.c).
         * It gates itself on the safety state; it doesn't touch the motors
         * unless balancing is enabled and the robot is ACTIVE. */
        balance_step(filtered_angle_deg, gyro_rate_dps, gyro_z_dps);

        s_last_control_step_cycles = DWT->CYCCNT - control_step_start;
    }
    else
    {
        s_failed_read_count++;
    }

    s_read_busy = false;
}

void mpu6050_i3c_callback (i3c_callback_args_t const * const p_args)
{
    switch (p_args->event)
    {
        case I3C_EVENT_WRITE_COMPLETE:
        {
            if (I3C_EVENT_STATUS_SUCCESS != p_args->event_status)
            {
                s_phase = MPU6050_PHASE_IDLE;

                if (s_is_tick_transfer)
                {
                    s_failed_read_count++;
                    s_read_busy = false;
                }
                else
                {
                    s_blocking_event_status = p_args->event_status;
                    s_blocking_done         = true;
                }

                break;
            }

            if (MPU6050_PHASE_WRITE_THEN_READ == s_phase)
            {
                s_phase = MPU6050_PHASE_READING;

                fsp_err_t const err = R_I3C_Read(&g_i3c_imu_ctrl, s_p_rx_buffer, s_rx_length, false);

                if (FSP_SUCCESS != err)
                {
                    s_phase = MPU6050_PHASE_IDLE;

                    if (s_is_tick_transfer)
                    {
                        s_failed_read_count++;
                        s_read_busy = false;
                    }
                    else
                    {
                        s_blocking_event_status = I3C_EVENT_STATUS_ABORTED;
                        s_blocking_done         = true;
                    }
                }
            }
            else /* MPU6050_PHASE_WRITE_ONLY */
            {
                s_phase = MPU6050_PHASE_IDLE;

                if (s_is_tick_transfer)
                {
                    s_read_busy = false;
                }
                else
                {
                    s_blocking_event_status = I3C_EVENT_STATUS_SUCCESS;
                    s_blocking_done         = true;
                }
            }

            break;
        }

        case I3C_EVENT_READ_COMPLETE:
        {
            s_phase = MPU6050_PHASE_IDLE;

            if (I3C_EVENT_STATUS_SUCCESS != p_args->event_status)
            {
                if (s_is_tick_transfer)
                {
                    s_failed_read_count++;
                    s_read_busy = false;
                }
                else
                {
                    s_blocking_event_status = p_args->event_status;
                    s_blocking_done         = true;
                }

                break;
            }

            if (s_is_tick_transfer)
            {
                mpu6050_tick_transfer_succeeded(s_p_rx_buffer, p_args->transfer_size);
            }
            else
            {
                s_blocking_event_status = I3C_EVENT_STATUS_SUCCESS;
                s_blocking_done         = true;
            }

            break;
        }

        default:
        {
            break;
        }
    }
}

void mpu6050_get_angle (mpu6050_angle_t * p_angle)
{
    uint32_t const primask = __get_PRIMASK();

    __disable_irq();
    *p_angle = s_angle;
    __set_PRIMASK(primask);
}

uint32_t mpu6050_get_sequence (void)
{
    return s_sequence;
}

uint32_t mpu6050_get_missed_read_count (void)
{
    return s_missed_read_count;
}

uint32_t mpu6050_get_failed_read_count (void)
{
    return s_failed_read_count;
}

uint32_t mpu6050_get_last_read_cycles (void)
{
    return s_last_read_cycles;
}

uint32_t mpu6050_get_last_control_step_cycles (void)
{
    return s_last_control_step_cycles;
}
