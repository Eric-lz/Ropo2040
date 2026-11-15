# Ropo2040 logic board: build guide (50 x 70 mm perfboard)

This board carries only logic: the RP2040-Zero, the 74HC14, and connectors. Everything that touches 12 V (battery, INA219 shunt, buck-boost input, HW-039 drivers, motors) goes on a separate power board; section 9 covers how to wire that one.

| File | What it is |
|---|---|
| `layout_top.svg` / `.png` | Component side: where every part goes. Pin rings are coloured by net. |
| `layout_bottom.svg` / `.png` | Solder side, **flipped left to right** the way you see it when soldering. |
| `wiring_list.md` | Tick-box list of every bare link, plus the holes for every part. |
| `make_layout.py` | Generates all of the above and checks every net (no shorts, no breaks). Edit its tables and run `python make_layout.py` to change anything. |

Holes are named **row letter – column number**. With the component side up, rows A–R run top to bottom and columns 1–24 run left to right; `C-4` is row C, column 4. Write a few row letters and column numbers on both sides of the board with a fine marker before you start. The board must have **24 x 18 holes**; count yours first.

**There are no wire crossings.** All 65 connections are bare 22 AWG wire on the solder side, running straight along rows and columns with 90° turns. Nothing is soldered diagonally, and no insulated jumpers are needed. The one place a signal has to cross a rail, R5 does it: it lies flat with its insulated body over the 3V3 trunk.

---

## 1. What goes where

```
             POWER BOARD (12 V)                                   LOGIC BOARD (this one)
battery ─XT60─► INA219 screw terminals ─► 12 V ─┬─► HW-039 L  B+/B− ─► left motor
                                               ├─► HW-039 R  B+/B− ─► right motor
                                               └─► buck-boost IN ─► OUT 5 V ──► J1 ─► D1 ─► RP2040 5V
INA219 header (VCC GND SCL SDA)  ◄─────────────────────────────────────────────── J9
HW-039 control pins (RPWM LPWM R_EN L_EN VCC) ◄────────────────────────────────── J2, J3, J4
motor harness: encoder (pins 2-4) ───────────────────────────────────────────────► J5, J6
motor harness: WOW switch (pins 6-7) ────────────────────────────────────────────► J7, J8
FS-iA10B i-BUS port (servo plug) ──────────────────────────────────────────────► J12
```

| Ref | Header | Pins (left to right, or top to bottom) |
|---|---|---|
| J1 | 5V IN, screw terminal | + (L-2), − (N-2) |
| J2 | DRIVER L | RPWM, LPWM, R_EN (column 17, rows C–E) + L_EN (E-18) |
| J3 | DRIVER R | RPWM, LPWM, R_EN (column 17, rows G–I) + L_EN (I-18) |
| J4 | DRIVER VCC | VCC L (C-20), VCC R (D-20): 3.3 V for each HW-039's VCC pin |
| J5 | ENCODER L | LED+, GND, ENC |
| J6 | ENCODER R | ENC, GND, LED+ (mirror of J5) |
| J7 / J8 | WOW L / WOW R | SW (top), 3V3 (bottom) |
| J9 | EXPANSION + INA219, 2 x 9 | 5V, GND, 3V3, GP29/A3, GP28/A2, GP27/A1, GP26/A0, SCL, SDA. Both columns are identical. |
| J11 | SPARE | GP7, 3V3 (e.g. the future bump switch) |
| J12 | RC iBUS, servo order | GND (E-2), 5V (E-3), S (E-4). A standard servo plug fits straight on. |

## 2. Design decisions (and why)

- **USB protection: D1.** The RP2040-Zero's 5V pin is wired straight to the USB connector's VBUS (assume no diode on the module). D1, a 1N5819 Schottky, sits between the buck output and that pin:
  - *Battery only:* the buck's 5.0 V goes through D1, so the RP2040 sees about 4.6–4.7 V. Its 3.3 V regulator is fine with that.
  - *USB only:* the PC powers the logic. D1 stops USB current from flowing back into the buck.
  - *Both:* USB (about 5.0 V) is higher than the 4.6 V after D1, so D1 stays off and the PC powers the logic. No current flows between the PC and the buck in either direction.
  - **Keep the buck at 5.0 V, not higher**, so the board side never rises above the PC's USB voltage. CB1 sits before D1, so it doesn't add inrush current when you plug USB in.
- **Ground routing.** GND leaves the RP2040's GND pin under the module, runs out over the top edge (row A) and down the left edge (column 1), then along the bottom edge (row R). That puts GND outside the 5V run in column 3, which is what lets J12 have 5V in the middle pin, like a servo plug.
- **One ground connection between the boards: J1−.** The HW-039 GND pins are **left unconnected** on purpose. On the HW-039, the GND pin is the same copper as B−. Wiring it to this board would create a second ground path, and part of the motor current would return through the logic board. That is the "motor current shifting the ground" problem from `config.h`. The drivers' logic still gets its ground through B−. The INA219, encoders and receiver have no other ground path, so their GND wires to this board are fine.
- **74HC14 on 3.3 V**, so its outputs are safe for the RP2040. The encoder chain is: 5.1 kΩ pull-up (R1/R2), then a 1 nF filter (C2/C3, about a 5 µs time constant, far below the ~160 µs between edges at full speed), then the 74HC14. Its outputs go straight up to GP9 (left) and GP8 (right). The input links run under the socket, between its two pin rows.
- **WOW switches switch to 3.3 V**, using the RP2040's **internal pull-down**: closed = high. C4/C5 (100 nF from the switch line to 3V3) filter bounce and noise. A capacitor to 3V3 filters exactly as well as one to GND, and this way the routing stays crossing-free. Ignore the WOW inputs for the first ~20 ms after boot, while C4/C5 discharge.
- **Encoder and WOW have separate headers.** The motor harness already groups the pins that way: M+, [LED, GND, ENC], M−, [WOW, WOW]. Each motor cable uses a 3-pin plug for the encoder, a 2-pin plug for WOW, and two wires to the driver.
- **Driver enable.** R_EN and L_EN are joined on the board (one GPIO per driver), and both have their own header pin, so each wire runs straight to its pin on the HW-039.
- **Receiver.** iBUS is a UART at 115200 baud and the receiver only transmits, so it needs a single RX pin: GP13 (UART0 RX). The FS-iA10B can drive its signal line at 5 V, which is too much for the RP2040, so R6 (1 kΩ) and R7 (2.2 kΩ) divide it down:
  - **5 V receiver:** GP13 sees 5 × 2.2 / 3.2 = 3.44 V, under the RP2040's 3.6 V limit.
  - **3.3 V receiver:** GP13 sees 2.27 V, still above the RP2040's 2.0 V high threshold.

  The signal runs from E-4 down column 4 through R6 to the divider node (K-4 to M-4), where R7 goes to GND. From the node it passes through R5 (1 kΩ, lying flat from L-4 to L-8, bridging the 3V3 trunk), then along row L to GP13. R5 is only a series resistor and doesn't change the divided level. J12's 5V is after D1, so the receiver also runs on USB power while you debug.
- **J9 is two identical columns.** The INA219 plugs into one column with 4 jumper wires; the future sensor board uses the other. It carries 5V, GND, 3V3, the four ADC pins (one per cliff sensor) and I2C.
- **Unused:** GP10 and GP11 are left unconnected; there is no crossing-free way out from between the other pins. GP7 is free on J11.

## 3. New pin map

| GPIO | Function | Header |
|---|---|---|
| GP0 / GP1 | Left driver RPWM / LPWM (PWM slice 0 A/B) | J2 |
| GP2 | Left driver R_EN + L_EN | J2 |
| GP3 | Right WOW switch (pull-down, high = closed) | J8 |
| GP4 / GP5 | Right driver RPWM / LPWM (PWM slice 2 A/B) | J3 |
| GP6 | Right driver R_EN + L_EN | J3 |
| GP7 | Spare | J11 |
| GP8 | Right encoder (from 74HC14 pin 8) | J6 via U3 |
| GP9 | Left encoder (from 74HC14 pin 10) | J5 via U3 |
| GP10, GP11 | Not connected | — |
| GP12 | Left WOW switch (pull-down, high = closed) | J7 |
| GP13 | iBUS in (UART0 RX, 115200 8N1) | J12 |
| GP14 / GP15 | I2C1 SDA / SCL (unchanged) | J9 |
| GP16 | Onboard WS2812 (unchanged) | — |
| GP26–GP29 | ADC0–ADC3, free for the cliff sensors | J9 |

When you move from the breadboard to this board, the `inc/config.h` values for the current single-motor setup become:

- `ENCODER_PIN 9`
- `MOTOR_PWM_FWD_PIN 0`
- `MOTOR_PWM_REV_PIN 1`
- `MOTOR_EN_PIN 2`

The right motor is encoder 8, PWM 4/5, enable 6. For the WOW switches, call `gpio_pull_down()` on GP3 and GP12. For iBUS, use `uart_init(uart0, 115200)` with `gpio_set_function(13, GPIO_FUNC_UART)`. The 74HC14 inverts the encoder signal, which doesn't matter because both edges are counted.

## 4. Parts list

### Logic board

| Qty | Part | Ref |
|---|---|---|
| 1 | DIP-14 IC socket | U3 |
| 1 | 74HC14N | U3 |
| 2 + 1 | 1x8 + 1x7 female header (2.54 mm) | U1 socket: column 9, column 15, row K |
| 1 | 1x5 male header, soldered **pointing down** on the RP2040-Zero's bottom edge (GP9–GP13) | U1 |
| 2 | 1x9 male header | J9 (side by side) |
| 2 + 2 | 1x3 male header + single male pin | J2, J3 |
| 2 | 1x3 male header | J5, J6 |
| 4 | 1x2 male header | J4, J7, J8, J11 |
| 1 | 1x3 male header | J12 |
| 1 | 2-pin screw terminal, 5.08 mm (KF301) | J1 |
| 1 | 1N5819 Schottky diode (1N5817 also works) | D1 |
| 1 | 100 µF electrolytic, ≥ 10 V, **5 mm diameter** (6.3 mm won't fit next to J1) | CB1 |
| 2 | 5.1 kΩ ¼ W | R1, R2 |
| 2 | 100 Ω ¼ W | R3, R4 |
| 1 | 1 kΩ ¼ W (lies flat, 10.16 mm lead spacing) | R5 |
| 1 | 1 kΩ ¼ W | R6 |
| 1 | 2.2 kΩ ¼ W | R7 |
| 3 | 100 nF ceramic, 2.54 mm | C1, C4, C5 |
| 2 | 1 nF ceramic, 2.54 mm | C2, C3 |
| — | 22 AWG solid wire | all links |

### Power board

Female XT60, 2 x 220–470 µF ≥ 25 V electrolytics (one per driver), wire of 18 AWG or heavier, and optionally an inline blade fuse after the XT60.

## 5. How to make the connections

- **Bare links** (everything on this board): strip 22 AWG completely and tin it. Lay it flat along the holes it runs over, and solder it at **every hole it passes**. For 90° corners, bend it with pliers before laying it down, or use two straight pieces that share the corner hole. For two-hole links, a clipped resistor lead is easier to handle.
- Every hole a link passes is part of that net. Never put another part's lead in those holes.
- 143 places have two different nets on neighbouring pads. Use little solder, then check each row with a magnifier and a light behind the board. The busiest spot is column 9, rows L–O, which carries four different nets.

## 6. Build order

1. **Sockets and headers.** Plug the RP2040-Zero into its three female headers (1x8, 1x8, 1x7) and solder them with the module in place so the U stays square. Then solder the DIP socket (notch on the left, toward column 9), the male headers and J1 (wire entry facing the left edge).
2. **Power links** (wiring list: GND, 3V3, VBUS, 5V_IN). Then check with the meter that GND, 3V3, VBUS and 5V_IN are not shorted to each other.
3. **D1 and CB1.** D1 lies flat along column 3, cathode in F-3 with the **stripe toward row F**, anode in I-3. CB1's **+ lead goes in J-3**, − in J-2.
4. **Resistors and capacitors.** R1–R4, R6 and R7 stand upright; the beige circle on the top view shows which hole the body sits over. **R5 lies flat** from L-4 to L-8: bend its leads at the body so it sits on the board, with its insulated body over L-5 (the 3V3 trunk). Don't let a bare lead touch L-5.
5. **Signal lanes**, then **tick everything off** in `wiring_list.md` with the meter in continuity mode.

## 7. First power-up

1. **Nothing plugged in.** Check that GND, 3V3, VBUS and 5V_IN don't beep against each other. On diode test, VBUS→5V_IN shows a diode drop in one direction only; that's D1.
2. **Buck-boost:** set it to **5.0 V** with nothing connected, then wire OUT+ / OUT− to J1 + / −.
3. **Power on, RP2040 and 74HC14 out.** J9's 5V pin should read about 4.6–4.7 V (5 V minus D1).
4. **Power off, plug in the RP2040 and the 74HC14** (notch left). Power on: J9's 3V3 pin should read 3.3 V.
5. **USB check.** With the battery off, plug in USB. The board runs, and J1 + reads about 0 V, because D1 is blocking. Then switch the battery on with USB still connected: J1 + reads 5.0 V and J9's 5V pin reads the USB voltage.
6. Connect the INA219, drivers, encoders and receiver.

## 8. Off-board wiring

| Board | Cable | Other end |
|---|---|---|
| J1 + / − | 2 wires | buck-boost OUT+ / OUT− |
| J2 RPWM, LPWM, R_EN, L_EN | 4 jumper wires | left HW-039 R_PWM, L_PWM, R_EN, L_EN |
| J3 (same) | 4 jumper wires | right HW-039 |
| J4 VCC L / VCC R | 1 wire each | VCC pin of the left / right HW-039 |
| — | — | HW-039 **GND, R_IS, L_IS: leave unconnected** |
| J5 LED+, GND, ENC | 3-pin plug | left motor harness pins 2, 3, 4 (LED anode, common ground, phototransistor collector) |
| J6 ENC, GND, LED+ | 3-pin plug | right motor harness pins 4, 3, 2 (mirrored motor, so the same cable fits) |
| J7 / J8 SW, 3V3 | 2-pin plug | WOW switch, motor harness pins 6 and 7 (either way round) |
| J9 3V3, GND, SCL, SDA | 4 jumper wires | INA219 header VCC, GND, SCL, SDA. Its VIN+/VIN− header pins stay empty; current goes through its screw terminals. |
| J12 GND, 5V, S | servo extension (female–female) | FS-iA10B i-BUS servo port: −, +, S |
| — | — | motor harness pins 1 and 5 (M+ / M−) go to the HW-039 M+ / M− terminals |

On J5/J6, GND is the middle pin. A plug turned the wrong way swaps LED+ and ENC, so check before powering up.

## 9. The power board (12 V side)

- **12 V path:** female XT60 (battery) → INA219 screw terminal **VIN+**. Its **VIN−** goes to a 12 V distribution point, which feeds the left HW-039 B+, the right HW-039 B+ and the buck-boost IN+.
- **Ground: one star point at the battery −.** Run separate wires from it to the left B−, the right B− and the buck IN−. Don't chain them from one terminal to the next.
- **At each HW-039:** solder a 220–470 µF ≥ 25 V capacitor across the B+/B− terminals.
- **At each motor:** solder 100 nF across the motor terminals and twist the motor wires.
- **Cable routing:** keep the encoder/WOW cables away from the motor wires. If they have to run side by side, cross them at right angles.
- **Wire sizes:** 18 AWG or heavier for B+/B− and the motor wires.
- **Buck-boost OUT+ / OUT−** go to the logic board's J1. This is the **only** ground wire between the two boards (the INA219 GND wire to J9 doesn't count: the INA219 has no other ground path).
- **INA219 range:** its 0.1 Ω shunt reads up to ±3.2 A, and all motor current flows through it. If both motors together can draw more than that (stall, hard starts), the reading clips. At 5 A the shunt also dissipates 2.5 W. Check your motors' stall current. If it's too high, swap in a smaller shunt or move the INA219 to measure one branch.

## 10. Check these before soldering

- **Board:** 24 x 18 holes.
- **RP2040-Zero, USB at the top:** the left row is 5V, GND, 3V3, GP29, GP28, GP27, GP26, GP15, GP14; the right row is GP0–GP8. The bottom edge reads GP13, GP12, GP11, GP10, GP9 from left to right, in line with GP14 and GP8. (A published KiCad footprint confirms the bottom-edge positions.) Those 5 pins need a male header pointing **down**, like the sides.
- **FS-iA10B iBUS level:** the R6/R7 divider handles either 3.3 V or 5 V. After building, you can measure GP13 against GND with the receiver bound and the transmitter on. The meter averages the data, so it reads a bit below the high level, but it should stay under 3.5 V.
- **Motor harness order** (pins 1–7: M+, LED anode, common GND, collector, M−, WOW, WOW) and **HW-039 header** (top row VCC, R_IS, R_EN, R_PWM; bottom row GND, L_IS, L_EN, L_PWM): taken as you described them.
