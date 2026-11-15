/*
 * blackbox.h
 *
 * Black-box recorder for untethered runs: the last 10 s of every armed run,
 * kept in RAM across a reset (S2) and read back later with the "dump"
 * console command (tools/telemetry/dump.py does that and plots it).
 *
 *   - While armed, balance_step() (1 kHz ISR) stores one packed record every
 *     BLACKBOX_PERIOD_STEPS steps (100 Hz) into a ring of BLACKBOX_RECORDS
 *     (10 s). Each arm starts a new recording, overwriting the previous one.
 *   - When the run ends (S1 disarm, tilt cut-off, any other FAULT), the main
 *     loop freezes it: the reason and fault bits are stored and a CRC-32 is
 *     computed over the header and all records.
 *   - Everything lives in .ram_noinit, which the startup code doesn't zero
 *     or initialise, and SRAM keeps its contents through a pin reset (S2),
 *     a fault-handler reset or a debugger attach. It is lost on power loss
 *     and may be overwritten by flashing a new image (the J-Link flash loader
 *     runs from RAM) - so read it before reflashing, and keep the board
 *     powered while plugging the laptop in.
 *   - At boot, blackbox_init() checks the magic word, the layout and the
 *     CRC. A run that was still recording when the reset hit (S2 or a fault
 *     while armed) is frozen then, with reason "reset".
 *
 * Record (18 bytes, all int16, little endian):
 *   t        10 ms steps since arm (uint16, wraps after 655 s - unwrapped in the dump)
 *   ang      angle x100, deg
 *   gyro     pitch rate x10, deg/s
 *   pwm_l/r  driver command, signed permille (+ = FWD) - planned only in dry-run
 *   bal      kp*e + kd*gyro, stock 8-bit units, before clamping
 *   spd      speed-loop term actually applied (after the soft-start ramp), 8-bit units
 *   enc_l/r  signed encoder position since arm, counts (660 per wheel rev; wraps at +/-32768)
 */

#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include <stdbool.h>
#include <stdint.h>

#define BLACKBOX_PERIOD_STEPS (10U)   /* 1 kHz / 10 = 100 Hz. */
#define BLACKBOX_RECORDS      (1000U) /* 10 s at 100 Hz: 1000 x 18 B = 18000 B. */

typedef struct st_blackbox_record
{
    uint16_t t_10ms;
    int16_t  ang_x100;
    int16_t  gyro_x10;
    int16_t  pwm_left;
    int16_t  pwm_right;
    int16_t  bal;
    int16_t  spd;
    int16_t  enc_left;
    int16_t  enc_right;
} blackbox_record_t;

typedef enum e_blackbox_reason
{
    BLACKBOX_REASON_NONE = 0,  /* Still recording. */
    BLACKBOX_REASON_S1,        /* Disarmed by S1. */
    BLACKBOX_REASON_TILT,      /* Tilt cut-off (FAULT with SAFETY_FAULT_TILT). */
    BLACKBOX_REASON_FAULT,     /* Any other FAULT - see the fault bits. */
    BLACKBOX_REASON_RESET,     /* Reset (S2, fault handler) while still armed. */
} blackbox_reason_t;

/* What was running when the recording started; stored in its header. */
typedef struct st_blackbox_run_info
{
    float kp;
    float kd;
    float setpoint_deg;
    float speed_kp;
    float speed_ki;
    float turn_kd;
    float arm_angle_deg;
    bool  dry_run;
    bool  speed_loop_on;
} blackbox_run_info_t;

/* Boot: validates what survived the reset; recovers a run cut off by a reset.
 * Prints one "blackbox: ..." line on channel 0. Call before the control tick starts. */
void blackbox_init(void);

/* ISR (balance_step()), first step of an armed run: starts a new recording. */
void blackbox_start(blackbox_run_info_t const * p_info);

/* ISR (balance_step()), every BLACKBOX_PERIOD_STEPS armed steps. No-op unless recording. */
void blackbox_record(blackbox_record_t const * p_record);

/* Main loop, when the safety state leaves ACTIVE: stops recording, stores the
 * reason, computes the CRC (~1 ms), and prints a "blackbox: ..." line. */
void blackbox_freeze(blackbox_reason_t reason, uint32_t fault_reasons);

/* Main loop ("dump" console command): starts printing the frozen recording as
 * CSV on RTT channel 1. Refused (with a message) while armed or if nothing
 * valid is stored. */
void blackbox_dump_start(void);

/* Main loop, every tick: sends the next dump lines while channel 1 has room.
 * Never blocks and never drops a line. */
void blackbox_dump_poll(void);

#endif /* BLACKBOX_H_ */
