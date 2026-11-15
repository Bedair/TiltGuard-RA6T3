/*
 * bt_remote.c
 *
 * See bt_remote.h. The UART callback (ISR) only queues each byte with its
 * tick; bt_remote_poll() (main loop) logs and acts on it. Single producer,
 * single consumer, so the queue needs no lock.
 */

#include "bt_remote.h"
#include "hal_data.h"
#include "../control/control_tick.h"
#include "../control/drive.h"
#include "../telemetry/telemetry.h"

#define BT_REMOTE_QUEUE_SIZE (32U) /* Power of two. 9600 baud = ~1 byte/ms max, polled every ms. */

typedef struct st_bt_remote_rx
{
    uint32_t tick;
    uint8_t  byte;
} bt_remote_rx_t;

static bt_remote_rx_t    s_queue[BT_REMOTE_QUEUE_SIZE];
static volatile uint32_t s_head;          /* Written by the ISR. */
static volatile uint32_t s_tail;          /* Written by the main loop. */
static volatile uint32_t s_dropped;       /* Queue full. */
static volatile uint32_t s_rx_errors;     /* Framing / parity / overrun / break. */
static uint32_t          s_reported_dropped;
static uint32_t          s_reported_errors;
static uint32_t          s_last_rx_tick;
static bool              s_open;

/* Drive keys of the joystick update being received (main loop only). */
#define BT_KEY_FORWARD  (1U << 0)
#define BT_KEY_BACKWARD (1U << 1)
#define BT_KEY_LEFT     (1U << 2)
#define BT_KEY_RIGHT    (1U << 3)
#define BT_KEY_STOP     (1U << 4)

static uint32_t          s_frame_keys;       /* 0 = no update open. */
static uint32_t          s_frame_last_tick;  /* Tick of its last key. */

void bt_uart_callback (uart_callback_args_t * p_args)
{
    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        uint32_t const head = s_head;

        if ((head - s_tail) >= BT_REMOTE_QUEUE_SIZE)
        {
            s_dropped++;

            return;
        }

        s_queue[head & (BT_REMOTE_QUEUE_SIZE - 1U)].tick = control_tick_count();
        s_queue[head & (BT_REMOTE_QUEUE_SIZE - 1U)].byte = (uint8_t) p_args->data;
        s_head = head + 1U;
    }
    else if (0U != (p_args->event & (UART_EVENT_ERR_PARITY | UART_EVENT_ERR_FRAMING |
                                     UART_EVENT_ERR_OVERFLOW | UART_EVENT_BREAK_DETECT)))
    {
        s_rx_errors++;
    }
    else
    {
        /* TX events: nothing is ever sent. */
    }
}

fsp_err_t bt_remote_init (void)
{
    fsp_err_t const err = R_SCI_UART_Open(&g_uart_bt_ctrl, &g_uart_bt_cfg);

    s_open = (FSP_SUCCESS == err);

    if (s_open)
    {
        telemetry_printf("bt: remote on SCI0 RX (P410/D0), 9600 8N1, receive only. "
                         "Keys f/b/l/i/s while armed: drive %d, turn %d, kept until the next key or 's' (%u ms max)\r\n",
                         (int) BT_REMOTE_SPEED, (int) BT_REMOTE_TURN, (unsigned) BT_REMOTE_HOLD_MS);
    }
    else
    {
        telemetry_printf("bt: UART open failed (err=%d) - Bluetooth remote off, balancing unaffected\r\n", (int) err);
    }

    return err;
}

/* Adds a key to the open frame. Returns false for keys that aren't drive keys. */
static bool bt_remote_collect (uint8_t byte, uint32_t tick)
{
    uint32_t key;

    switch (byte)
    {
        case 'f': key = BT_KEY_FORWARD;  break;
        case 'b': key = BT_KEY_BACKWARD; break;
        case 'l': key = BT_KEY_LEFT;     break;
        case 'i': key = BT_KEY_RIGHT;    break;
        case 's': key = BT_KEY_STOP;     break;
        default:  key = 0U;              break;
    }

    if (0U == key)
    {
        return false;
    }

    s_frame_keys      |= key;
    s_frame_last_tick  = tick;

    return true;
}

static int32_t bt_remote_axis (uint32_t keys, uint32_t plus_key, uint32_t minus_key, int32_t magnitude)
{
    bool const plus  = (0U != (keys & plus_key));
    bool const minus = (0U != (keys & minus_key));

    if (plus && !minus)
    {
        return magnitude;
    }

    if (minus && !plus)
    {
        return -magnitude;
    }

    return 0;
}

/* One joystick update = the drive keys that arrive within BT_REMOTE_FRAME_GAP_MS
 * of each other (the app sends a diagonal as two keys ~1 ms apart, either
 * order). The update replaces both targets: a key that is absent means 0. */
static void bt_remote_apply_frame (void)
{
    uint32_t const keys = s_frame_keys;

    s_frame_keys = 0U;

    if (0U != (keys & BT_KEY_STOP))
    {
        drive_stop();

        return;
    }

    int32_t const speed = bt_remote_axis(keys, BT_KEY_FORWARD, BT_KEY_BACKWARD,
                                         BT_REMOTE_FORWARD_SIGN * BT_REMOTE_SPEED);
    int32_t const turn  = bt_remote_axis(keys, BT_KEY_LEFT, BT_KEY_RIGHT, BT_REMOTE_TURN);

    (void) drive_command(speed, turn, BT_REMOTE_HOLD_MS, "bt");
}

void bt_remote_poll (void)
{
    if (!s_open)
    {
        return;
    }

    while (s_tail != s_head)
    {
        bt_remote_rx_t const rx = s_queue[s_tail & (BT_REMOTE_QUEUE_SIZE - 1U)];
        s_tail = s_tail + 1U;

        if (('\r' == rx.byte) || ('\n' == rx.byte) || ('\0' == rx.byte))
        {
            continue; /* Stock ignores these too. */
        }

        uint32_t const since_ms = rx.tick - s_last_rx_tick;
        s_last_rx_tick = rx.tick;

        if ((rx.byte >= 0x20U) && (rx.byte < 0x7FU))
        {
            telemetry_printf("bt: rx '%c' +%u ms\r\n", (int) rx.byte, (unsigned) since_ms);
        }
        else
        {
            telemetry_printf("bt: rx 0x%x +%u ms\r\n", (unsigned) rx.byte, (unsigned) since_ms);
        }

        if (!bt_remote_collect(rx.byte, rx.tick))
        {
            telemetry_printf("bt: ignored (not a drive key)\r\n");
        }
    }

    if ((0U != s_frame_keys) && ((control_tick_count() - s_frame_last_tick) >= BT_REMOTE_FRAME_GAP_MS))
    {
        bt_remote_apply_frame();
    }

    uint32_t const dropped = s_dropped;
    uint32_t const errors  = s_rx_errors;

    if ((dropped != s_reported_dropped) || (errors != s_reported_errors))
    {
        s_reported_dropped = dropped;
        s_reported_errors  = errors;
        telemetry_printf("bt: rx errors=%u (framing/parity/overrun) dropped=%u\r\n", (unsigned) errors, (unsigned) dropped);
    }
}
