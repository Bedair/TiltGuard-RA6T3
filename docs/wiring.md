# TiltGuard wiring (terminal-block architecture, Phase 2)

Referenced from `CLAUDE.md`. This is the detailed, as-built wiring reference;
`CLAUDE.md`'s pin map is the quick summary.

Terminal-block boards (push-in/screw terminals) replaced the original jumper
wires between the FPB and the robot board. Everything in this document
reflects that architecture, not the earlier ad-hoc jumper setup.

## Hardware overview

- **RA6T3 side**: ElectroCookie Uno Terminal Block Shield (push-in terminals)
  on the FPB's Arduino headers. It has **dedicated SDA/SCL terminals,
  separate from A4/A5** - A4/A5 are plain analog pins on this shield, not
  the I2C bus. Don't confuse them.
- **Robot side**: GERUI Nano terminal adapter (30 screw terminals, one per
  Nano pin) plugged into the Tumbller's empty Nano socket, through male
  header link strips.
- **Between them**, as modules taped to the chassis:
  - **Buck converter**, set to 5.0 V. IN+ = GERUI `VIN` (battery `VM`,
    7-8.4 V), IN- = GND. OUT+ = GERUI `5V` (robot 5 V rail) and both
    shifters' HV side. OUT- = GND.
  - **Shifter 1** (BSS138, 4-channel): LV1-4 <- FPB D5, D6, D7, D12;
    HV1-4 -> robot D5, D6, D7, D12. Carries PWMA, PWMB, AIN1, BIN1.
  - **Shifter 2**: LV1 <- FPB D8; HV1 -> robot D8. Carries STBY.
    Channels 2-4 are spare.
  - Both shifters: LV = FPB 3V3, HV = buck OUT+, GND = FPB GND.
  - **Reset pull-downs (added 2026-11-13):** 1 kOhm from the **LV side** of
    STBY (Shifter 2 LV1 / FPB D8), PWMA (Shifter 1 LV1 / D5) and PWMB
    (Shifter 1 LV2 / D6) to GND. Without them, the shifter's 10 k pull-ups
    drive all five motor lines high while the RA6T3 is in reset, and the
    motors spin. Never fit them on the HV side. Sizing, and the
    measurement still to do, are in `docs/hardware-findings.md`.
- **The FPB stays USB-powered for now**, electrically separate from the
  robot's 5 V rail. Later, for untethered running: buck OUT+ -> 1N5819 ->
  ElectroCookie 5V (diode-OR, not done yet).

## Pin map (robot GERUI terminal -> FPB ElectroCookie terminal)

| Signal | Robot | Path | FPB | MCU | Notes |
|---|---|---|---|---|---|
| PWMA | D5 | Shifter 1 ch1 | D5 | P409 | GPT1 A, 20 kHz, 1 k LV pull-down |
| PWMB | D6 | Shifter 1 ch2 | D6 | P113 | GPT2 A, 20 kHz, 1 k LV pull-down |
| AIN1 | D7 | Shifter 1 ch3 | D7 | P008 | GPIO |
| BIN1 | D12 | Shifter 1 ch4 | D12 | P110 | GPIO |
| STBY | D8 | Shifter 2 ch1 | D8 | P006 | GPIO, low = motors off, 1 k LV pull-down |
| ENC_L | D2 | direct | D3 | P408 | IRQ7, both edges, 5 V-tolerant |
| ENC_R | D4 | direct | D1 | P411 | IRQ4, both edges, 5 V-tolerant |
| ECHO | A3 | direct | D0 | P410 | IRQ5, both edges, 5 V-tolerant |
| TRIG | D11 | direct | D11 | P109 | GPIO out, 3.3 V |
| SDA | A4 | direct | SDA (dedicated terminal) | P101 | I3C0 legacy I2C, 5 V-tolerant |
| SCL | A5 | direct | SCL (dedicated terminal) | P100 | I3C0 legacy I2C, 5 V-tolerant |
| VOL | A2 | direct | D4 | P500 | ADC AN016, max 1.1 V |
| GND | GND x2 | direct | GND x2 | - | two wires, to avoid the ground offset (see Grounding) |

**Not used:** Pmod 2, J6 (A0-A5), FPB A4/A5.

## Power

Two separate power domains right now:

1. **FPB logic**: USB, as always during development.
2. **Robot 5 V rail + shifter HV side**: buck converter, set to 5.0 V,
   fed from the battery (`VM`, 7-8.4 V on a 2S pack). This is the first
   time in the project the real battery has been in the loop at all -
   previously (Phase 1) the robot's 5 V rail was bench-powered straight
   from the FPB's own 5 V output with the battery switch OFF.

These two domains share a common GND (two wires - see Grounding) but are
otherwise independent; the FPB does not depend on the battery to run.

**Planned, not yet done:** for untethered operation, diode-OR the buck
converter's OUT+ into the ElectroCookie's 5V rail through a 1N5819
Schottky diode, so the FPB can eventually run from the battery too without
needing USB connected. Until that's wired in, the FPB must stay on USB
whenever the robot is powered.

## Grounding

**Finding (2026-11):** with only a single GND jumper between the robot
board and the FPB (before the terminal blocks existed), the two grounds
differed by **~0.15 V** at the current draw present during testing. That's
a candidate explanation for the battery-sense ADC channel reading ~9-10%
high, but **not confirmed as the (sole) cause** - see the open issue in
`docs/hardware-findings.md`.

**Current wiring**: two GND wires between the robot and FPB terminal
blocks (see the pin map above), carried over from the mitigation applied
before the terminal-block boards existed. This has not been re-verified
against the battery-sense reading since the terminal-block changeover -
still a partial mitigation, not a confirmed fix. The direct FPB-GND/3.3V
reference test described in `docs/hardware-findings.md`'s open issue is
still the next step, deferred to Phase 4.

## Never connect

- **`VM` (the battery net) to any FPB pin.** `VM` only ever goes into the
  buck converter's IN+. The FPB's own supply is USB, kept electrically
  separate.
- **The FPB's A4/A5 to SDA/SCL.** On the ElectroCookie shield, SDA/SCL are
  separate, dedicated terminals - A4/A5 are plain analog pins on this
  board, not an alternate I2C location. Wiring them together would short
  two unrelated signals.

## Pin history (why things ended up where they are)

### Right encoder: P205 -> P105 -> P411

Originally **P205** (Pmod2-7, IRQ1), direct, 5 V-tolerant - this was fine
on the original jumper-wire wiring, where Pmod2 was reachable. It's not
reachable on the terminal-block boards: **Pmod 2 isn't broken out on
either the ElectroCookie or GERUI boards**, only the Arduino-header-shaped
pins are.

Interim fix was **P105** (Arduino D2, IRQ0) through a BSS138 shifter
channel - P105 isn't 5 V-tolerant, hence the shifter.

Final: **P411** (Arduino D1, IRQ4), direct, no shifter - P411 *is*
5 V-tolerant, so once the terminal-block layout required an
Arduino-header pin anyway, there was no reason to keep spending a shifter
channel on this signal.

IRQ4 is shared by three candidate pins on this MCU: P111, P402, and P411.
Checked all three against `configuration.xml` before making this change:
P111 was clean (plain input, no IRQ enabled), but **P402 had a stray
IRQ-enable bit set from the original board template**, unused by
anything - the same kind of leftover that made us pick P410 over P411 for
Echo, months earlier. This time P411 was dictated by the wiring, so P402's
stray enable was disabled directly instead.

### Ultrasonic Echo: P407 -> P410

The original plan put Echo on P407 (Pmod2-8) - also a Pmod2 pin, also
unreachable on the terminal-block boards, and (separately) checked against
the RA6T3 pin-function database and found to have **no IRQ capability at
all**, so it couldn't have done edge-timestamped capture even if it were
reachable.

Candidates with IRQ capability that are also 5 V-tolerant: P400 (IRQ0),
P401 (IRQ5-DS), P410 (IRQ5), P411 (IRQ4). P400/P401 were ruled out - they're
the `I2C_I3C_SEL_SCL`/`SDA` lines on the I3C bus mux, actively driven;
repurposing either breaks the IMU link. Between P410 and P411, P401 (IRQ5's
other candidate) is safely a GPIO *output* (can't also have its IRQ-enable
bit set), so P410 had no dual-pin-conflict risk, unlike P411 at the time
(which still had P402's stray enable active) - that's why Echo landed on
P410 and the right encoder didn't take P411 until P402 was cleaned up.

P410 and P109 (Trig) also both needed their pins freed from unused
peripheral groups left over from the board template (P410 from an `SCI0`
"Simple SPI" group, P109 from an `SCI9` "Simple I2C" group) before the
Pins tab would let them be set to GPIO/IRQ.

### Battery sense (VOL): P002 -> P500

Moved from A2/P002 (J6-3) to D4/P500 (AN016) to free up J6 entirely - J6
isn't used by anything now. Checked five free Arduino-header candidates
(P403/D9, P301/D10, P111/D13, P411/D1, P500/D4) against the pin-function
database first: only P500 has any ADC function in silicon (`ADC0: AN016`)
- the other four aren't wired to the SAR ADC at all, not just unavailable
in the configurator.
