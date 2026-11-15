/*
 * console.c
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "console.h"
#include "telemetry.h"
#include "../control/balance.h"
#include "../control/jog.h"
#include "../safety/safety.h"
#include "blackbox.h"

#define CONSOLE_LINE_MAX       (48U)
#define CONSOLE_READ_CHUNK     (16U)
#define CONSOLE_MAX_FRAC_DIGITS (6U)

typedef enum e_console_param
{
    CONSOLE_PARAM_KP = 0,
    CONSOLE_PARAM_KD,
    CONSOLE_PARAM_SP,
    CONSOLE_PARAM_SKP,
    CONSOLE_PARAM_SKI,
    CONSOLE_PARAM_TKD,
} console_param_t;

typedef struct st_console_param_def
{
    const char    * p_name;
    console_param_t id;
    float           min;
    float           max;
} console_param_def_t;

static const console_param_def_t s_param_defs[] =
{
    { "kp",  CONSOLE_PARAM_KP,    0.0f, 200.0f },
    { "kd",  CONSOLE_PARAM_KD,    0.0f,  20.0f },
    { "sp",  CONSOLE_PARAM_SP,  -20.0f,  20.0f },
    { "skp", CONSOLE_PARAM_SKP,   0.0f,  50.0f },
    { "ski", CONSOLE_PARAM_SKI,   0.0f,   5.0f },
    { "tkd", CONSOLE_PARAM_TKD,   0.0f,  10.0f },
};

#define CONSOLE_PARAM_COUNT (sizeof(s_param_defs) / sizeof(s_param_defs[0]))

static char     s_line[CONSOLE_LINE_MAX];
static uint32_t s_line_length;
static bool     s_line_overflow;
static int32_t  s_pending_jog_percent; /* Handed from console_execute() to console_poll(). */

/* "-1.234" style: sign, integer part, 3-digit fraction (SEGGER printf has no %f). */
typedef struct st_decimal3
{
    const char * p_sign;
    int          int_part;
    unsigned     frac_part;
} decimal3_t;

static decimal3_t console_split3 (float value)
{
    decimal3_t   out;
    float const  abs_value = fabsf(value);

    out.p_sign    = (value < 0.0f) ? "-" : "";
    out.int_part  = (int) abs_value;
    out.frac_part = (unsigned) (((abs_value - (float) out.int_part) * 1000.0f) + 0.5f);

    if (out.frac_part >= 1000U)
    {
        out.frac_part = 0U;
        out.int_part += 1;
    }

    return out;
}

/* Parses [+-]digits[.digits]; the whole token must be consumed. */
static bool console_parse_float (const char * p_text, float * p_value)
{
    bool     negative     = false;
    bool     any_digit    = false;
    uint32_t int_part     = 0U;
    uint32_t frac_part    = 0U;
    uint32_t frac_scale   = 1U;
    uint32_t frac_digits  = 0U;

    if (('-' == *p_text) || ('+' == *p_text))
    {
        negative = ('-' == *p_text);
        p_text++;
    }

    while ((*p_text >= '0') && (*p_text <= '9'))
    {
        if (int_part > 100000U)
        {
            return false; /* Far outside every parameter range anyway. */
        }

        int_part  = (int_part * 10U) + (uint32_t) (*p_text - '0');
        any_digit = true;
        p_text++;
    }

    if ('.' == *p_text)
    {
        p_text++;

        while ((*p_text >= '0') && (*p_text <= '9'))
        {
            if (frac_digits < CONSOLE_MAX_FRAC_DIGITS)
            {
                frac_part   = (frac_part * 10U) + (uint32_t) (*p_text - '0');
                frac_scale *= 10U;
                frac_digits++;
            }

            any_digit = true;
            p_text++;
        }
    }

    if ((!any_digit) || ('\0' != *p_text))
    {
        return false;
    }

    float const magnitude = (float) int_part + ((float) frac_part / (float) frac_scale);

    *p_value = negative ? -magnitude : magnitude;

    return true;
}

/* Parses a whole number token ("12"); false on anything else. */
static bool console_parse_int (const char * p_text, int32_t * p_value)
{
    float value = 0.0f;

    if ((!console_parse_float(p_text, &value)) || ((value - floorf(value)) > 0.0f) || (fabsf(value) > 100000.0f))
    {
        return false;
    }

    *p_value = (int32_t) value;

    return true;
}

/*
 * Prints one "<prefix><name>=<value>" field. Lines are built from several
 * small printf calls rather than one call with ~20 arguments: the main stack
 * is 1 KB, and each wide call costs its whole argument block plus one
 * decimal3_t per value on the stack at once.
 */
static __attribute__((noinline)) void console_print_field (const char * p_prefix, const char * p_name, float value)
{
    decimal3_t const d = console_split3(value);

    telemetry_printf("%s%s=%s%d.%03u", p_prefix, p_name, d.p_sign, d.int_part, d.frac_part);
}

void console_print_params (void)
{
    balance_params_t p;
    balance_get_params(&p);

    console_print_field("params: ", "kp", p.kp);
    console_print_field(" ", "kd", p.kd);
    console_print_field(" ", "sp", p.setpoint_deg);
    console_print_field(" ", "skp", p.speed_kp);
    console_print_field(" ", "ski", p.speed_ki);
    console_print_field(" ", "tkd", p.turn_kd);
    telemetry_printf("\r\n");
}

void console_print_armed (float angle_deg, balance_params_t const * p_params)
{
    console_print_field("armed: ", "angle", angle_deg);
    console_print_field(", ", "setpoint", p_params->setpoint_deg);
    console_print_field(", ", "kp", p_params->kp);
    console_print_field(", ", "kd", p_params->kd);
    console_print_field(", ", "skp", p_params->speed_kp);
    console_print_field(", ", "ski", p_params->speed_ki);
    console_print_field(", ", "tkd", p_params->turn_kd);
    telemetry_printf("\r\n");
}

void console_print_mode (void)
{
    if (balance_get_dry_run())
    {
        telemetry_printf("mode: DRY-RUN - S1 arms and the law runs and logs, motors stay off (STBY low, PWM 0). "
                         "'dry 0' to drive.\r\n");
    }
    else
    {
        telemetry_printf("mode: LIVE - the motors are driven when armed. 'dry 1' for dry-run.\r\n");
    }
}

void console_print_speed_loop (void)
{
    balance_params_t p;
    balance_get_params(&p);

    if ((p.speed_kp <= 0.0f) && (p.speed_ki <= 0.0f)) /* Same test as balance_step(); both are >= 0. */
    {
        telemetry_printf("speed loop: OFF (skp=0 ski=0; filter and integrator held at 0)\r\n");
    }
    else
    {
        console_print_field("speed loop: ON (", "skp", p.speed_kp);
        console_print_field(" ", "ski", p.speed_ki);
        telemetry_printf(")\r\n");
    }
}

static void console_print_help (void)
{
    telemetry_printf("commands: kp <v> | kd <v> | sp <deg> | skp <v> | ski <v> | tkd <v> | "
                     "dry <0|1> | jog <1..%d> | dump | show | help  (space optional: 'skp0')\r\n", JOG_MAX_PERCENT);
}

static __attribute__((noinline)) void console_reject_range (console_param_def_t const * p_def)
{
    telemetry_printf("rejected: %s out of range ", p_def->p_name);
    console_print_field("[", "min", p_def->min);
    console_print_field(", ", "max", p_def->max);
    telemetry_printf("]\r\n");
}

/* What console_poll() does after a command has executed - done there,
 * after console_execute() has returned, so the printf chain never sits on
 * top of the parsing frames. */
typedef enum e_console_reply
{
    CONSOLE_REPLY_NONE = 0,
    CONSOLE_REPLY_PARAMS,
    CONSOLE_REPLY_HELP,
    CONSOLE_REPLY_MODE,
    CONSOLE_REPLY_JOG,
    CONSOLE_REPLY_DUMP,
} console_reply_t;

static void console_apply (console_param_def_t const * p_def, float value)
{
    balance_params_t p;
    balance_get_params(&p);

    switch (p_def->id)
    {
        case CONSOLE_PARAM_KP:
        {
            p.kp = value;
            break;
        }

        case CONSOLE_PARAM_KD:
        {
            p.kd = value;
            break;
        }

        case CONSOLE_PARAM_SP:
        {
            p.setpoint_deg = value;
            break;
        }

        case CONSOLE_PARAM_SKP:
        {
            p.speed_kp = value;
            break;
        }

        case CONSOLE_PARAM_SKI:
        {
            p.speed_ki = value;
            break;
        }

        case CONSOLE_PARAM_TKD:
        default:
        {
            p.turn_kd = value;
            break;
        }
    }

    balance_set_params(&p); /* One masked copy: applied between two control steps. */
}

/* True if the command name (p_line, name_length characters) is p_name. */
static bool console_name_is (const char * p_line, size_t name_length, const char * p_name)
{
    return (strlen(p_name) == name_length) && (0 == strncmp(p_line, p_name, name_length));
}

static console_reply_t console_execute_dry (const char * p_value)
{
    bool dry_run;

    if ((NULL != p_value) && (0 == strcmp(p_value, "1")))
    {
        dry_run = true;
    }
    else if ((NULL != p_value) && (0 == strcmp(p_value, "0")))
    {
        dry_run = false;
    }
    else
    {
        telemetry_printf("rejected: 'dry' needs 0 (drive the motors) or 1 (dry-run)\r\n");

        return CONSOLE_REPLY_NONE;
    }

    if (!balance_set_dry_run(dry_run))
    {
        telemetry_printf("rejected: can't change dry-run while armed - disarm (S1) first\r\n");

        return CONSOLE_REPLY_NONE;
    }

    return CONSOLE_REPLY_MODE;
}

static __attribute__((noinline)) console_reply_t console_execute (char * p_line)
{
    /*
     * "<name>[ ]<value>": the name is the leading run of letters, so the
     * space is optional ("skp0" = "skp 0", "sp-1.5" = "sp -1.5"). The value
     * is everything after it, with surrounding spaces trimmed; NULL if empty.
     */
    size_t name_length = 0U;

    while ((p_line[name_length] >= 'a') && (p_line[name_length] <= 'z'))
    {
        name_length++;
    }

    char * p_value = &p_line[name_length];

    while (' ' == *p_value)
    {
        p_value++;
    }

    size_t length = strlen(p_value);

    while ((length > 0U) && (' ' == p_value[length - 1U]))
    {
        p_value[length - 1U] = '\0';
        length--;
    }

    if (0U == length)
    {
        p_value = NULL;
    }

    if ((NULL == p_value) && console_name_is(p_line, name_length, "show"))
    {
        return CONSOLE_REPLY_PARAMS;
    }

    if ((NULL == p_value) && console_name_is(p_line, name_length, "help"))
    {
        return CONSOLE_REPLY_HELP;
    }

    if ((NULL == p_value) && console_name_is(p_line, name_length, "dump"))
    {
        return CONSOLE_REPLY_DUMP;
    }

    if (console_name_is(p_line, name_length, "dry"))
    {
        return console_execute_dry(p_value);
    }

    if (console_name_is(p_line, name_length, "jog"))
    {
        if ((NULL == p_value) || (!console_parse_int(p_value, &s_pending_jog_percent)))
        {
            telemetry_printf("rejected: 'jog' needs a whole percent, e.g. 'jog 10' (max %d)\r\n", JOG_MAX_PERCENT);

            return CONSOLE_REPLY_NONE;
        }

        return CONSOLE_REPLY_JOG; /* jog_start() checks the state and range, and prints. */
    }

    for (uint32_t i = 0U; i < CONSOLE_PARAM_COUNT; i++)
    {
        console_param_def_t const * const p_def = &s_param_defs[i];

        if (!console_name_is(p_line, name_length, p_def->p_name))
        {
            continue;
        }

        float value = 0.0f;

        if ((NULL == p_value) || (!console_parse_float(p_value, &value)))
        {
            telemetry_printf("rejected: '%s' needs a number, e.g. '%s 1.5'\r\n", p_def->p_name, p_def->p_name);

            return CONSOLE_REPLY_NONE;
        }

        if ((value < p_def->min) || (value > p_def->max))
        {
            console_reject_range(p_def);

            return CONSOLE_REPLY_NONE;
        }

        console_apply(p_def, value);

        return CONSOLE_REPLY_PARAMS; /* Echo the full, now-active parameter set. */
    }

    telemetry_printf("unknown command '%s'\r\n", p_line);

    return CONSOLE_REPLY_HELP;
}

static __attribute__((noinline)) void console_reply (console_reply_t reply)
{
    switch (reply)
    {
        case CONSOLE_REPLY_PARAMS:
        {
            console_print_params();
            console_print_speed_loop();
            break;
        }

        case CONSOLE_REPLY_HELP:
        {
            console_print_help();
            break;
        }

        case CONSOLE_REPLY_MODE:
        {
            console_print_mode();
            break;
        }

        case CONSOLE_REPLY_JOG:
        {
            jog_start(s_pending_jog_percent);
            break;
        }

        case CONSOLE_REPLY_DUMP:
        {
            blackbox_dump_start(); /* Checks the state and the CRC, and prints. */
            break;
        }

        case CONSOLE_REPLY_NONE:
        default:
        {
            break; /* Already reported (rejection) - nothing more to print. */
        }
    }
}

void console_poll (void)
{
    char     chunk[CONSOLE_READ_CHUNK];
    unsigned read_count = telemetry_read(chunk, sizeof(chunk));

    while (read_count > 0U)
    {
        for (unsigned i = 0U; i < read_count; i++)
        {
            char const c = chunk[i];

            if (('\r' == c) || ('\n' == c))
            {
                if (s_line_overflow)
                {
                    telemetry_printf("rejected: line longer than %u characters\r\n", (unsigned) (CONSOLE_LINE_MAX - 1U));
                }
                else if (s_line_length > 0U)
                {
                    s_line[s_line_length] = '\0';
                    console_reply(console_execute(s_line));
                }
                else
                {
                    /* Empty line (e.g. the '\n' of a "\r\n" pair): ignore. */
                }

                s_line_length   = 0U;
                s_line_overflow = false;
            }
            else if (((' ' == c) || ('\t' == c)) && (0U == s_line_length))
            {
                /* Skip leading whitespace. */
            }
            else if (s_line_length < (CONSOLE_LINE_MAX - 1U))
            {
                /* Lowercase, tabs as spaces, so "KP\t55" works too. */
                char stored = ('\t' == c) ? ' ' : c;

                if ((stored >= 'A') && (stored <= 'Z'))
                {
                    stored = (char) (stored - 'A' + 'a');
                }

                s_line[s_line_length] = stored;
                s_line_length++;
            }
            else
            {
                s_line_overflow = true;
            }
        }

        read_count = telemetry_read(chunk, sizeof(chunk));
    }
}
