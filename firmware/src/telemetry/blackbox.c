/*
 * blackbox.c
 *
 * See blackbox.h. Concurrency: blackbox_start()/blackbox_record() run in the
 * balance ISR, everything else in the main loop. The ISR only writes while
 * s_recording is set; the main loop clears it (blackbox_freeze()) before it
 * touches the store, and the ISR can't preempt itself, so the store is never
 * written by both at once. A new arm while a dump is running bumps
 * s_generation, and the dump stops.
 */

#include <stddef.h>
#include <string.h>
#include "blackbox.h"
#include "telemetry.h"
#include "bsp_api.h"
#include "../build_info.h"
#include "../safety/safety.h"

#define BLACKBOX_MAGIC_RECORDING (0x52454342UL) /* "BCER" */
#define BLACKBOX_MAGIC_FROZEN    (0x5A4F5246UL) /* "FROZ" */

/* Changes whenever the record or header layout does, so a recording made by
 * an older build with another layout is ignored instead of misread. */
#define BLACKBOX_LAYOUT ((1UL << 24) | ((uint32_t) sizeof(blackbox_record_t) << 16) | (uint32_t) BLACKBOX_RECORDS)

#define BLACKBOX_BUILD_ID_MAX      (40U)
#define BLACKBOX_DUMP_LINES_PER_POLL (8U) /* Bounds the main-loop time per tick. */

typedef struct st_blackbox_header
{
    uint32_t            magic;
    uint32_t            layout;
    uint32_t            head;          /* Next write index. */
    uint32_t            count;         /* Valid records, <= BLACKBOX_RECORDS. */
    uint32_t            total;         /* Records written since arm (the run may be longer than the ring). */
    uint32_t            reason;        /* blackbox_reason_t. */
    uint32_t            fault_reasons; /* SAFETY_FAULT_* at the freeze. */
    blackbox_run_info_t info;
    char                build_id[BLACKBOX_BUILD_ID_MAX];
    uint32_t            crc;           /* CRC-32 of everything above, then all records. Must stay last. */
} blackbox_header_t;

typedef struct st_blackbox_store
{
    blackbox_header_t header;
    blackbox_record_t records[BLACKBOX_RECORDS];
} blackbox_store_t;

/* Not zeroed by the startup code - survives a reset (see blackbox.h). */
static blackbox_store_t s_store BSP_PLACE_IN_SECTION(BSP_SECTION_NOINIT);

static volatile bool     s_recording;  /* ISR writes records only while set. */
static volatile uint32_t s_generation; /* Bumped by every blackbox_start(). */
static bool              s_valid;      /* Store holds a frozen, CRC-checked recording. */

typedef enum e_blackbox_dump_stage
{
    BLACKBOX_DUMP_IDLE = 0,
    BLACKBOX_DUMP_BUILD,
    BLACKBOX_DUMP_RUN,
    BLACKBOX_DUMP_PARAMS,
    BLACKBOX_DUMP_COLUMNS,
    BLACKBOX_DUMP_RECORDS,
    BLACKBOX_DUMP_END,
} blackbox_dump_stage_t;

typedef struct st_blackbox_dump
{
    blackbox_dump_stage_t stage;
    uint32_t              generation; /* s_generation when the dump started. */
    uint32_t              sent;       /* Records sent so far. */
    uint32_t              first;      /* Ring index of the oldest record. */
    uint32_t              t_wraps;    /* 65536 x 10 ms wraps seen so far. */
    uint16_t              t_prev;
} blackbox_dump_t;

static blackbox_dump_t  s_dump;
static telemetry_line_t s_dump_line; /* Static: off the 1 KB main stack. */

/* CRC-32 (IEEE, reflected 0xEDB88320), 4 bits at a time: 16-entry table. */
static uint32_t const s_crc_nibble[16] =
{
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL, 0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL, 0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL,
};

static uint32_t blackbox_crc_update (uint32_t crc, uint8_t const * p_data, size_t length)
{
    for (size_t i = 0U; i < length; i++)
    {
        crc ^= p_data[i];
        crc  = (crc >> 4) ^ s_crc_nibble[crc & 0x0FU];
        crc  = (crc >> 4) ^ s_crc_nibble[crc & 0x0FU];
    }

    return crc;
}

static uint32_t blackbox_crc (void)
{
    uint32_t crc = 0xFFFFFFFFUL;

    crc = blackbox_crc_update(crc, (uint8_t const *) &s_store.header, offsetof(blackbox_header_t, crc));
    crc = blackbox_crc_update(crc, (uint8_t const *) s_store.records, sizeof(s_store.records));

    return ~crc;
}

static bool blackbox_indices_sane (void)
{
    return (BLACKBOX_LAYOUT == s_store.header.layout) &&
           (s_store.header.head < BLACKBOX_RECORDS) &&
           (s_store.header.count <= BLACKBOX_RECORDS);
}

static const char * blackbox_reason_name (uint32_t reason)
{
    switch (reason)
    {
        case BLACKBOX_REASON_S1:
        {
            return "S1";
        }

        case BLACKBOX_REASON_TILT:
        {
            return "tilt";
        }

        case BLACKBOX_REASON_FAULT:
        {
            return "fault";
        }

        case BLACKBOX_REASON_RESET:
        {
            return "reset";
        }

        case BLACKBOX_REASON_NONE:
        default:
        {
            return "running";
        }
    }
}

static void blackbox_seal (void)
{
    s_store.header.build_id[BLACKBOX_BUILD_ID_MAX - 1U] = '\0';
    s_store.header.magic = BLACKBOX_MAGIC_FROZEN;
    s_store.header.crc   = blackbox_crc();
    s_valid              = true;
}

static __attribute__((noinline)) void blackbox_print_summary (const char * p_prefix)
{
    blackbox_header_t const * const p = &s_store.header;

    telemetry_printf("%sreason=%s faults=0x%x, %u records (%u.%02u s of %u.%02u s armed), build %s\r\n",
                     p_prefix,
                     blackbox_reason_name(p->reason),
                     (unsigned) p->fault_reasons,
                     (unsigned) p->count,
                     (unsigned) (p->count / 100U), (unsigned) (p->count % 100U),
                     (unsigned) (p->total / 100U), (unsigned) (p->total % 100U),
                     p->build_id);
}

void blackbox_init (void)
{
    s_recording = false;
    s_valid     = false;
    s_dump.stage = BLACKBOX_DUMP_IDLE;

    if ((BLACKBOX_MAGIC_FROZEN == s_store.header.magic) && blackbox_indices_sane() &&
        (blackbox_crc() == s_store.header.crc))
    {
        s_valid = true;
        blackbox_print_summary("blackbox: kept from before the reset - ");
        telemetry_printf("blackbox: type 'dump' (or run tools/telemetry/dump.py) to read it; the next arm overwrites it\r\n");
    }
    else if ((BLACKBOX_MAGIC_RECORDING == s_store.header.magic) && blackbox_indices_sane())
    {
        /* Reset hit while armed (S2, or a fault handler): keep what was recorded. */
        s_store.header.reason        = BLACKBOX_REASON_RESET;
        s_store.header.fault_reasons = 0U;
        blackbox_seal();
        blackbox_print_summary("blackbox: run cut off by a reset while armed - ");
        telemetry_printf("blackbox: type 'dump' (or run tools/telemetry/dump.py) to read it; the next arm overwrites it\r\n");
    }
    else
    {
        s_store.header.magic = 0U;
        telemetry_printf("blackbox: empty (power-on, or nothing valid survived the reset)\r\n");
    }
}

void blackbox_start (blackbox_run_info_t const * p_info)
{
    s_recording = false;
    s_valid     = false;
    s_generation++;

    (void) memset(&s_store.header, 0, sizeof(s_store.header));
    s_store.header.layout = BLACKBOX_LAYOUT;
    s_store.header.info   = *p_info;

    for (uint32_t i = 0U; (i < (BLACKBOX_BUILD_ID_MAX - 1U)) && ('\0' != g_build_id[i]); i++)
    {
        s_store.header.build_id[i] = g_build_id[i];
    }

    s_store.header.magic = BLACKBOX_MAGIC_RECORDING;
    s_recording          = true;
}

void blackbox_record (blackbox_record_t const * p_record)
{
    if (!s_recording)
    {
        return;
    }

    blackbox_header_t * const p = &s_store.header;

    s_store.records[p->head] = *p_record;
    p->head = (p->head + 1U) % BLACKBOX_RECORDS;

    if (p->count < BLACKBOX_RECORDS)
    {
        p->count++;
    }

    p->total++;
}

void blackbox_freeze (blackbox_reason_t reason, uint32_t fault_reasons)
{
    if (!s_recording)
    {
        return; /* Nothing was recorded for this run (e.g. the bench build). */
    }

    s_recording = false; /* From here on the ISR leaves the store alone. */
    __DMB();

    s_store.header.reason        = (uint32_t) reason;
    s_store.header.fault_reasons = fault_reasons;
    blackbox_seal();
    blackbox_print_summary("blackbox: froze ");
}

void blackbox_dump_start (void)
{
    if (SAFETY_STATE_ACTIVE == safety_get_state())
    {
        telemetry_printf("dump refused: armed - disarm (S1) first\r\n");

        return;
    }

    if (!s_valid || (blackbox_crc() != s_store.header.crc))
    {
        telemetry_printf("dump refused: no valid recording (nothing armed since power-on, or the CRC doesn't match)\r\n");

        return;
    }

    s_dump.stage      = BLACKBOX_DUMP_BUILD;
    s_dump.generation = s_generation;
    s_dump.sent       = 0U;
    s_dump.first      = (s_store.header.count < BLACKBOX_RECORDS) ? 0U : s_store.header.head;
    /* Records are taken every 10th step from the arm on, so the oldest kept
     * record is number (total - count) since arm: that gives the 65536 x 10 ms
     * wraps of t_10ms before it, for runs longer than 655 s. */
    s_dump.t_wraps    = (s_store.header.total - s_store.header.count) >> 16;
    s_dump.t_prev     = 0U;

    blackbox_print_summary("dump: sending as CSV on RTT channel 1 - ");
}

static void blackbox_line_hex (telemetry_line_t * p_line, uint32_t value)
{
    static char const s_hex[] = "0123456789abcdef";
    bool              started = false;

    telemetry_line_str(p_line, "0x");

    for (int32_t shift = 28; shift >= 0; shift -= 4)
    {
        uint32_t const nibble = (value >> (uint32_t) shift) & 0x0FU;

        if ((0U != nibble) || started || (0 == shift))
        {
            started = true;
            telemetry_line_char(p_line, s_hex[nibble]);
        }
    }
}

static void blackbox_line_param (telemetry_line_t * p_line, const char * p_name, float value)
{
    telemetry_line_char(p_line, ' ');
    telemetry_line_str(p_line, p_name);
    telemetry_line_char(p_line, '=');
    telemetry_line_fixed(p_line, value, 3U);
}

static void blackbox_build_record_line (telemetry_line_t * p_line)
{
    blackbox_record_t const * const p_rec = &s_store.records[(s_dump.first + s_dump.sent) % BLACKBOX_RECORDS];

    if ((0U != s_dump.sent) && (p_rec->t_10ms < s_dump.t_prev))
    {
        s_dump.t_wraps++;
    }

    s_dump.t_prev = p_rec->t_10ms;

    uint32_t const t_ms = ((s_dump.t_wraps * 65536UL) + p_rec->t_10ms) * 10UL;

    telemetry_line_u32(p_line, t_ms);
    telemetry_line_char(p_line, ',');
    telemetry_line_fixed(p_line, (float) p_rec->ang_x100 / 100.0f, 2U);
    telemetry_line_char(p_line, ',');
    telemetry_line_fixed(p_line, (float) p_rec->gyro_x10 / 10.0f, 1U);
    telemetry_line_char(p_line, ',');
    telemetry_line_i32(p_line, p_rec->pwm_left);
    telemetry_line_char(p_line, ',');
    telemetry_line_i32(p_line, p_rec->pwm_right);
    telemetry_line_char(p_line, ',');
    telemetry_line_i32(p_line, p_rec->bal);
    telemetry_line_char(p_line, ',');
    telemetry_line_i32(p_line, p_rec->spd);
    telemetry_line_char(p_line, ',');
    telemetry_line_i32(p_line, p_rec->enc_left);
    telemetry_line_char(p_line, ',');
    telemetry_line_i32(p_line, p_rec->enc_right);

    s_dump.sent++;
}

/* Builds the line for the current stage and advances the stage. */
static void blackbox_build_dump_line (telemetry_line_t * p_line)
{
    blackbox_header_t const * const p = &s_store.header;

    telemetry_line_begin(p_line);

    switch (s_dump.stage)
    {
        case BLACKBOX_DUMP_BUILD:
        {
            telemetry_line_str(p_line, "# blackbox build=");
            telemetry_line_str(p_line, p->build_id);
            s_dump.stage = BLACKBOX_DUMP_RUN;
            break;
        }

        case BLACKBOX_DUMP_RUN:
        {
            telemetry_line_str(p_line, "# run reason=");
            telemetry_line_str(p_line, blackbox_reason_name(p->reason));
            telemetry_line_str(p_line, " faults=");
            blackbox_line_hex(p_line, p->fault_reasons);
            telemetry_line_str(p_line, " records=");
            telemetry_line_u32(p_line, p->count);
            telemetry_line_str(p_line, " armed_ms=");
            telemetry_line_u32(p_line, p->total * 10UL);
            telemetry_line_str(p_line, " period_ms=10 mode=");
            telemetry_line_str(p_line, p->info.dry_run ? "DRY-RUN" : "LIVE");
            telemetry_line_str(p_line, " speed_loop=");
            telemetry_line_str(p_line, p->info.speed_loop_on ? "ON" : "OFF");
            telemetry_line_str(p_line, " arm_angle=");
            telemetry_line_fixed_signed(p_line, p->info.arm_angle_deg, 2U);
            s_dump.stage = BLACKBOX_DUMP_PARAMS;
            break;
        }

        case BLACKBOX_DUMP_PARAMS:
        {
            telemetry_line_str(p_line, "# params");
            blackbox_line_param(p_line, "kp", p->info.kp);
            blackbox_line_param(p_line, "kd", p->info.kd);
            blackbox_line_param(p_line, "sp", p->info.setpoint_deg);
            blackbox_line_param(p_line, "skp", p->info.speed_kp);
            blackbox_line_param(p_line, "ski", p->info.speed_ki);
            blackbox_line_param(p_line, "tkd", p->info.turn_kd);
            s_dump.stage = BLACKBOX_DUMP_COLUMNS;
            break;
        }

        case BLACKBOX_DUMP_COLUMNS:
        {
            telemetry_line_str(p_line, "t_ms,ang,gyro,pwmL,pwmR,bal,spd,encL,encR");
            s_dump.stage = (0U == p->count) ? BLACKBOX_DUMP_END : BLACKBOX_DUMP_RECORDS;
            break;
        }

        case BLACKBOX_DUMP_RECORDS:
        {
            blackbox_build_record_line(p_line);

            if (s_dump.sent >= p->count)
            {
                s_dump.stage = BLACKBOX_DUMP_END;
            }

            break;
        }

        case BLACKBOX_DUMP_END:
        default:
        {
            telemetry_line_str(p_line, "# end records=");
            telemetry_line_u32(p_line, s_dump.sent);
            s_dump.stage = BLACKBOX_DUMP_IDLE;
            break;
        }
    }
}

void blackbox_dump_poll (void)
{
    if (BLACKBOX_DUMP_IDLE == s_dump.stage)
    {
        return;
    }

    if ((s_dump.generation != s_generation) || (SAFETY_STATE_ACTIVE == safety_get_state()))
    {
        /* Armed mid-dump: the store is being overwritten. */
        s_dump.stage = BLACKBOX_DUMP_IDLE;
        telemetry_line_begin(&s_dump_line);
        telemetry_line_str(&s_dump_line, "# dump aborted: armed");
        (void) telemetry_line_send(&s_dump_line, 1U);
        telemetry_printf("dump aborted: armed during the dump (the recording is being overwritten)\r\n");

        return;
    }

    for (uint32_t i = 0U; (i < BLACKBOX_DUMP_LINES_PER_POLL) && (BLACKBOX_DUMP_IDLE != s_dump.stage); i++)
    {
        if (telemetry_write_space(1U) < TELEMETRY_LINE_MAX)
        {
            return; /* Host hasn't caught up yet - try again next tick, nothing dropped. */
        }

        bool const last = (BLACKBOX_DUMP_END == s_dump.stage);

        blackbox_build_dump_line(&s_dump_line);
        (void) telemetry_line_send(&s_dump_line, 1U); /* Fits: room was checked above. */

        if (last)
        {
            telemetry_printf("dump: done, %u records\r\n", (unsigned) s_dump.sent);
        }
    }
}
