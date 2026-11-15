/*
 * bt_remote.h
 *
 * Phone remote through the robot board's own Bluetooth (BLE) module, the way
 * the stock ELEGOO firmware uses it (Command.h getBluetoothData(), 9600 baud,
 * one ASCII character per command). Receive only: the stock protocol never
 * answers, so only the module's TX -> our RX is wired:
 *
 *   BT module UART_TXD -> Q1 (pulls low only) -> robot Nano socket D0 (GERUI D0)
 *   -> FPB D0 = P410 = SCI0 RXD0, internal pull-up (gives the high level the
 *   Nano used to). P410 is 5 V-tolerant. P410 was the ultrasonic Echo
 *   (ULTRASONIC_ENABLED = 0 now). SCI0's TX pin (P411) stays the right encoder.
 *
 * FSP: stack "g_uart_bt" (r_sci_uart), channel 0, 9600 8N1, callback
 * bt_uart_callback, interrupt priority below the control path. Pins tab:
 * SCI0 asynchronous, RXD0 = P410 (input pull-up), TXD0 = P213 - the
 * configurator requires a TX pin; P213 is the board's PMOD2_TXD0 (Pmod 2 is
 * unused and unreachable; the clock runs from HOCO, not the crystal). Nothing
 * is ever transmitted, so P213 just idles high.
 *
 * Keys (stock BLUETOOTH mode): 'f' forward, 'b' backward, 'l' left,
 * 'i' right, 's' stop. Everything else ('0'..'9', '*', '#': follow/obstacle/
 * LED modes) is logged and ignored.
 *
 * App behaviour (measured 2026-11-15, ELEGOO BLE app joystick): one update per
 * change, no repeats while held; a diagonal is two keys ~1 ms apart in either
 * order ('f'+'i'); updates on ~50 ms steps; 's' when the stick returns to the
 * centre. So:
 *   - The keys arriving within BT_REMOTE_FRAME_GAP_MS of each other are one
 *     update; it replaces BOTH targets (a missing key means 0): 'f' alone =
 *     straight, 'f'+'i' = forward-right curve, 'i' alone = spin right.
 *   - An update holds until the next one or 's', with BT_REMOTE_HOLD_MS as a
 *     backstop for a lost link (the module's connection status isn't wired to
 *     us). Holding the stick longer than that stops the robot: move it again.
 * The setpoints go through control/drive.c: refused unless armed, cleared on
 * disarm or fault.
 *
 * Every received byte is logged on RTT channel 0 with the time since the
 * previous one ("bt: rx 'f' +120 ms").
 */

#ifndef BT_REMOTE_H_
#define BT_REMOTE_H_

#include <stdint.h>
#include "bsp_api.h"

/* Stock non-Bluetooth defaults for a first try (stock Bluetooth uses 80/80). */
#define BT_REMOTE_SPEED        (40)  /* Counts per 40 ms. */
#define BT_REMOTE_TURN         (50)  /* 8-bit PWM units. */

/* -1: 'f' drives against firmware FWD (the way "jog" rolls), because on this
 * robot firmware FWD is the side the user calls the back. +1 for stock. */
#define BT_REMOTE_FORWARD_SIGN (-1)

/* Backstop for a lost link: an update returns to 0 after this long. */
#define BT_REMOTE_HOLD_MS      (5000U)

/* Keys closer than this belong to one joystick update (a byte at 9600 baud is
 * ~1.04 ms; updates come >= ~50 ms apart). */
#define BT_REMOTE_FRAME_GAP_MS (5U)

/* Opens the UART. On error the remote is just off (prints why); it is not a
 * safety function, so balancing goes on without it. */
fsp_err_t bt_remote_init(void);

/* Main loop, every tick: handles the received characters and prints. */
void bt_remote_poll(void);

#endif /* BT_REMOTE_H_ */
