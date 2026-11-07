# TiltGuard wiring (Phase 1)

Referenced from `CLAUDE.md`. This is the detailed, as-built wiring reference;
`CLAUDE.md`'s pin map is the quick summary.

## Current pin map (confirmed on hardware, 2026-11)

| Signal | FPB pin / header label | MCU pin | Notes |
|---|---|---|---|
| IMU SDA | J1-9 (Arduino SDA) | P101 | I3C0 in I2C-legacy mode, 400 kHz. 5 V-tolerant, FPB has pull-ups. |
| IMU SCL | J1-10 (Arduino SCL) | P100 | Same bus. |
| Left encoder (M2A) | D3 | P408 | IRQ7, both edges. 5 V-tolerant. Confirmed working. |
| Right encoder (M1A) | Pmod2-7 | P205 | IRQ1, both edges. 5 V-tolerant. Confirmed working. |
| **Ultrasonic Echo** | **Arduino D0** (also Pmod2 MISO position) | **P410** | IRQ5, both edges. 5 V-tolerant. Moved here from Pmod2-8/P407 - see below. Confirmed working: stable 22.00 +/- 0.02 cm, tracked a hand to ~3 cm. |
| Ultrasonic Trig | D11 | P109 | GPIO output, 3.3 V (HC-SR04 accepts 3.3 V logic on Trig). |
| Battery sense | A2 | P002 (AN002) | Analog in, 10 k / 1.5 k divider on the robot board. Max 1.10 V at the pin at full battery - do not exceed. **Open issue, deferred to Phase 4** - reads ~9-10% high; see Grounding below. |

## Changes from the original plan

### Ultrasonic Echo: P407 -> P410 (Arduino D0)

The original pin map put Echo on P407 (Pmod2-8). Checked against the RA6T3
pin-function database (`PinCfgR7FA6T3xxxxFM.xml`, not just the FSP
configurator UI): P407's capability list is `ADC0(Digital), AGT0, GPT,
GPT_OPS, I3C/IIC, USB FS` - **no IRQ function at all**. It cannot do
edge-timestamped capture, which the Echo pulse-width measurement needs.

Candidates with IRQ capability that are also 5 V-tolerant: P400 (IRQ0),
P401 (IRQ5-DS), P410 (IRQ5), P411 (IRQ4). P400/P401 were ruled out
immediately - they're the `I2C_I3C_SEL_SCL`/`SDA` lines actively driving the
IMU bus's electrical mode; repurposing either breaks the I3C link.

Between P410 and P411: RA6T3's ICU shares IRQ4 and IRQ5 across two
candidate pins each (IRQ5: P401 or P410; IRQ4: P402 or P411), and the FSP
manual for `r_icu` explicitly warns that *"pin configuration does not show
conflicts when the same IRQ is used by multiple pins"* - the tool won't
catch it if both pins sharing a channel are left IRQ-enabled at once. P402
(IRQ4's other candidate) already had its IRQ-enable bit set from the board
template, unused by anything - picking P411 would have required also going
to disable P402 first. P410's pin-pair partner, P401, is safely a GPIO
*output* (can't have its IRQ-enable bit set at the same time), so P410 was
the cleaner choice with no such trap.

P410 and P109 (Trig) also both needed a second fix beyond the stack/pin
assignment itself: both pins were still claimed by unused peripheral
groups left over from the board template (P410 by an `SCI0` "Simple SPI"
group spanning P102/P103/P410/P411; P109 by an `SCI9` "Simple I2C" group
spanning P109/P110). In the Pins tab, a pin's Mode dropdown stays locked to
"Peripheral" as long as any such group still claims it - the fix was to
find the `SCI0`/`SCI9` entries themselves (not the individual pin rows) and
set their Mode to Disabled, which released the pins back to plain GPIO.

## Grounding

**Finding (2026-11):** with only a single GND jumper between the robot
board and the FPB, the two grounds differed by **~0.15 V** at the current
draw present during testing. That's a candidate explanation for the
battery-sense ADC channel reading ~9-10% high (see the open issue below),
but it is **not yet confirmed as the (sole) cause** - the ADC input has not
been tested directly against the FPB's own GND/3.3 V rail with the robot
board out of the loop, which would isolate a pure ground-offset error from
anything else (resistor tolerance, wiring fault on the divider itself,
etc.).

**Mitigation in place:** extra GND wires added between the robot board and
the FPB, in parallel with the existing single jumper, to reduce the
IR-drop-induced offset. This has not been re-measured/re-verified against
the ADC reading - treat as a partial mitigation, not a confirmed fix.

**Recommendation going forward:** tie grounds at more than one physical
point if practical (e.g., one near the IMU/logic side, one near the
motor/power side), rather than a single point far from the sensing
circuitry. The direct FPB-GND/3.3V-reference test described in
`docs/hardware-findings.md`'s open issue is the next step before trusting
the battery-sense reading - this is deferred to Phase 4.
