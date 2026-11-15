# Ropo2040 logic board: build guide (50 x 70 mm perfboard)

This board carries the logic: the RP2040-Zero, the 74HC14, the INA219 and the connectors. The INA219 plugs into column 1 and hangs off the left edge. Battery current runs only through the module's own screw terminals, never through the perfboard. Everything else that touches 12 V (buck-boost input, HW-039 drivers, motors) goes on a separate power board, described in section 9.

| File | What it is |
|---|---|
| `layout_top.svg` / `.png` | Component side: where every part, flat resistor and jumper goes. Pin rings are coloured by net. |
| `layout_bottom.svg` / `.png` | Solder side, **flipped left to right** the way you see it when soldering. |
| `wiring_list.md` | Tick-box list of every jumper and bare link, plus the holes for every part. |
| `make_layout.py` | Generates all of the above and checks every net (no shorts, no breaks). Edit its tables and run `python make_layout.py` to change anything. |

Holes are named **row letter – column number**. With the component side up, rows A–R run top to bottom and columns 1–24 run left to right; `C-4` is row C, column 4. Write a few row letters and column numbers on both sides of the board with a fine marker before you start. The board must have **24 x 18 holes**; count yours first.

**The solder side is bare traces only.** All 68 bare links are 22 AWG wire running straight along rows and columns with 90° turns, and none of them cross. The few places where one connection must pass another are made on the component side:

- **Three resistors that have to be there anyway**, lying flat with their insulated bodies over the traces: R6, R7 and R5 in the iBUS circuit.
- **Three short straight insulated jumpers**, JP1 to JP3.

---

## 1. What goes where

```
battery + ─XT60─► INA219 screw terminal VIN+ ─► VIN− ─► power board 12 V ─┬─► HW-039 L ─► left motor
               (module hanging off this board's left edge)                 ├─► HW-039 R ─► right motor
                                                                           └─► buck-boost ─► 5 V ─► J1
HW-039 control pins (RPWM LPWM R_EN L_EN VCC) ◄──────────────────────── J2, J3, J4
motor harness: encoder (pins 2-4) ─────────────────────────────────────► J5, J6
motor harness: WOW switch (pins 6-7) ──────────────────────────────────► J7, J8
FS-iA10B i-BUS port (servo plug) ──────────────────────────────────────► J12
future sensor board ◄──────────────────────────────────────────────────► J9
```

| Ref | Part / header | Pins |
|---|---|---|
| J1 | 5V IN, screw terminal | + (B-1), − (D-1), wire entry facing the left edge |
| U2 | INA219, **upside down**, 1x6 female header in column 1 | VCC (H-1), GND (I-1), SCL (J-1), SDA (K-1), VIN− (L-1), VIN+ (M-1). The two VIN holes have **nothing** connected: they carry battery voltage on the module. |
| J2 | DRIVER L | RPWM, LPWM, R_EN (column 17, rows C–E) + L_EN (E-18) |
| J3 | DRIVER R | RPWM, LPWM, R_EN (column 17, rows G–I) + L_EN (I-18) |
| J4 | DRIVER VCC | VCC L (C-20), VCC R (D-20): 3.3 V for each HW-039's VCC pin |
| J5 | ENCODER L | LED+, GND, ENC |
| J6 | ENCODER R | ENC, GND, LED+ (mirror of J5) |
| J7 / J8 | WOW L / WOW R | SW (top), 3V3 (bottom) |
| J9 | EXPANSION, 1x9 in column 7 | 5V, GND, 3V3, GP29/A3, GP28/A2, GP27/A1, GP26/A0, SCL, SDA (rows C–K) |
| J11 | SPARE | GP7, 3V3 (e.g. the future bump switch) |
| J12 | RC iBUS | S (B-4), 5V (C-4), GND (D-4). The 5V pin is in the middle, so a standard servo plug fits straight on. |

## 2. Design decisions (and why)

- **USB protection: D1.** The RP2040-Zero's 5V pin is wired straight to its USB connector (assume no diode on the module). D1, a 1N5819 Schottky, sits between the buck's 5 V and that pin, right next to it:
  - *Battery only:* the buck's 5.0 V goes through D1, so the RP2040 sees about 4.6–4.7 V. Its 3.3 V regulator is fine with that.
  - *USB only:* the PC powers the RP2040 and everything on 3.3 V (74HC14, encoders, INA219). D1 stops USB current from flowing back into the buck. J9's 5V pin and the receiver stay **off**, since they hang off the buck's 5 V. To test the receiver, switch the battery on.
  - *Both:* USB (about 5.0 V) is higher than the 4.6 V after D1, so D1 stays off and the PC powers the RP2040. No current flows between the PC and the buck in either direction.
  - **Keep the buck at 5.0 V, not higher**, so the RP2040 side never rises above the PC's USB voltage.
- **INA219 on this board, upside down.** Its pin order (VCC, GND, SCL, SDA, VIN−, VIN+) is the mirror image of the RP2040's left row. Mounted the normal way up it would need 4–5 crossings. Flipped, its SCL and SDA line up with the RP2040's I2C rows (J and K) and run straight across, and only three short jumpers are left:
  - **JP1:** 3.3 V to the INA219's VCC.
  - **JP2 and JP3:** the GND and 3V3 trunks hopping over the two I2C lines.

  Battery current runs only through the module's screw terminals. Its VIN−/VIN+ header pins sit in empty holes.
- **Ground.** GND runs from the RP2040 along row D, down column 3 (hopping the I2C lines with JP2), then along the bottom edge (row R).
  - **J1− is the only ground connection to the power board.**
  - **The HW-039 GND pins stay unconnected** on purpose. On the HW-039, the GND pin is the same copper as B−. Wiring it here would create a second ground path, and part of the motor current would return through this board. That's the "motor current shifting the ground" problem from `config.h`. The drivers' logic still gets its ground through B−.
  - The INA219, encoders and receiver have no other ground path, so their GND connections here are fine.
- **74HC14 on 3.3 V**, so its outputs are safe for the RP2040. The encoder chain is: 5.1 kΩ pull-up (R1/R2), then a 1 nF filter (C2/C3, about 5 µs, far below the ~160 µs between edges at full speed), then the 74HC14. Its outputs go straight up to GP9 (left) and GP8 (right). The input links run under the socket, between its two pin rows.
- **WOW switches switch to 3.3 V**, using the RP2040's **internal pull-down**: closed = high. C4/C5 (100 nF from the switch line to 3V3) filter bounce and noise. Ignore the WOW inputs for the first ~20 ms after boot, while C4/C5 discharge.
- **Separate encoder and WOW headers.** The motor harness already groups the pins that way: M+, [LED, GND, ENC], M−, [WOW, WOW]. Each motor cable gets a 3-pin encoder plug, a 2-pin WOW plug, and two wires to the driver.
- **Driver enable.** R_EN and L_EN are joined on the board (one GPIO per driver). Both have their own header pin, so each wire runs straight to its pin on the HW-039.
- **Receiver: R6, R7 and R5 do double duty.** iBUS is a UART at 115200 baud and the receiver only transmits, so it needs one RX pin: GP13 (UART0 RX). The FS-iA10B can drive its signal line at 5 V, so R6 (1 kΩ) and R7 (2.2 kΩ) divide it down:
  - **5 V receiver:** GP13 sees 3.44 V, under the RP2040's 3.6 V limit.
  - **3.3 V receiver:** GP13 sees 2.27 V, still above the RP2040's 2.0 V high threshold.
  - **R5 (1 kΩ)** is a series resistor that doesn't change the level.

  All three lie flat, each bridging traces on the way from J12 (top left) to GP13:
  - **R6** (B-6 to F-6) crosses the 5V, GND and 3V3 lanes.
  - **R7** (G-6 to G-2) crosses the 3V3 and GND trunks to reach GND.
  - **R5** (H-6 to L-6) crosses the SCL and SDA lanes.
- **J9** is now a single 1x9 for the future sensor board: 5V, GND, 3V3, the four ADC pins (one per cliff sensor) and I2C, shared with the INA219.
- **No CB1 on this board.** There's no room for a bulk capacitor next to J1. Put a 100 µF (≥ 10 V) across the buck-boost's output terminals instead.
- **Unused:** GP10 and GP11 have no clean way out from between the other pins. GP7 is free on J11.

## 3. Pin map

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
| GP14 / GP15 | I2C1 SDA / SCL (unchanged): INA219 and J9 | U2, J9 |
| GP16 | Onboard WS2812 (unchanged) | — |
| GP26–GP29 | ADC0–ADC3, free for the cliff sensors | J9 |

When you move from the breadboard to this board, the `inc/config.h` values for the current single-motor setup become:

- `ENCODER_PIN 9`
- `MOTOR_PWM_FWD_PIN 0`
- `MOTOR_PWM_REV_PIN 1`
- `MOTOR_EN_PIN 2`

The right motor is encoder 8, PWM 4/5, enable 6. I2C and the INA219 address stay as they are.

- **WOW switches:** call `gpio_pull_down()` on GP3 and GP12.
- **iBUS:** use `uart_init(uart0, 115200)` with `gpio_set_function(13, GPIO_FUNC_UART)`.
- **Encoders:** the 74HC14 inverts the signal, which doesn't matter because both edges are counted.

## 4. Parts list

### Logic board

| Qty | Part | Ref |
|---|---|---|
| 1 | DIP-14 IC socket | U3 |
| 1 | 74HC14N | U3 |
| 2 + 1 | 1x8 + 1x7 female header (2.54 mm) | U1 socket: column 9, column 15, row K |
| 1 | 1x5 male header, soldered **pointing down** on the RP2040-Zero's bottom edge (GP9–GP13) | U1 |
| 1 | 1x6 female header | U2 (INA219) socket, column 1 rows H–M |
| 1 | 1x6 male header, soldered on the INA219's **component side** | U2 |
| 1 | 1x9 male header | J9 |
| 2 + 2 | 1x3 male header + single male pin | J2, J3 |
| 3 | 1x3 male header | J5, J6, J12 |
| 4 | 1x2 male header | J4, J7, J8, J11 |
| 1 | 2-pin screw terminal, 5.08 mm (KF301) | J1 |
| 1 | 1N5819 Schottky diode (1N5817 also works) | D1 |
| 2 | 5.1 kΩ ¼ W | R1, R2 |
| 2 | 100 Ω ¼ W | R3, R4 |
| 2 | 1 kΩ ¼ W (lie flat, 10.16 mm lead spacing) | R5, R6 |
| 1 | 2.2 kΩ ¼ W (lies flat, 10.16 mm) | R7 |
| 3 | 100 nF ceramic, 2.54 mm | C1, C4, C5 |
| 2 | 1 nF ceramic, 2.54 mm | C2, C3 |
| — | 22 AWG solid wire, bare and insulated | links and JP1–JP3 |

### Power board

Female XT60, 2 x 220–470 µF ≥ 25 V electrolytics (one per driver), 100 µF ≥ 10 V for the buck output, wire of 18 AWG or heavier, and optionally an inline blade fuse after the XT60.

## 5. How to make the connections

- **Bare links (solder side):** strip 22 AWG completely and tin it. Lay it flat along the holes it runs over, and solder it at **every hole it passes**. For 90° corners, bend it with pliers before laying it down, or use two straight pieces that share the corner hole. For two-hole links, a clipped resistor lead is easier.
- **Jumpers JP1–JP3 (component side):** straight pieces of insulated 22 AWG. Strip only the ends, push them through the two end holes, and lay the wire flat on the board. Solder the ends on the solder side; the holes in between stay untouched.
- **Flat resistors R5, R6, R7:** bend the leads right at the body so it lies on the board over the traces it bridges. Only the end holes get soldered.
- Every hole a bare link passes is part of that net. Never put another part's lead in those holes.
- 145 places have two different nets on neighbouring pads. Use little solder, then check each row with a magnifier and a light behind the board. The busiest spot is column 9, rows L–O, which carries four different nets.

## 6. Build order

1. **Sockets and headers.**
   - Plug the RP2040-Zero into its three female headers (1x8, 1x8, 1x7) and solder them with the module in place, so the U stays square.
   - Then the DIP socket (notch on the left, toward column 9).
   - Then the INA219's 1x6 female header in column 1.
   - Then the male headers and J1 (wire entry facing the left edge).
2. **Power links** (wiring list: GND, 3V3, VBUS, 5V_IN). Then check with the meter that GND, 3V3, VBUS and 5V_IN are not shorted to each other.
3. **Jumpers JP1–JP3.**
4. **D1:** standing, body over B-8 (anode), **stripe end up**, top lead bent over and down into C-8 (cathode).
5. **Resistors and capacitors.** R1–R4 stand upright; the beige circle on the top view shows which hole the body sits over. R5, R6 and R7 lie flat.
6. **Signal lanes**, then **tick everything off** in `wiring_list.md` with the meter in continuity mode. Finish with the two VIN holes (L-1, M-1): they must not beep to anything.

## 7. First power-up

1. **Nothing plugged in.** Check that GND, 3V3, VBUS and 5V_IN don't beep against each other. On diode test, VBUS→5V_IN shows a diode drop in one direction only; that's D1.
2. **Buck-boost:** set it to **5.0 V** with nothing connected, then wire OUT+ / OUT− to J1 + / −.
3. **Power on, RP2040, 74HC14 and INA219 out.** J9's 5V pin reads 5.0 V. The RP2040's 5V socket hole (C-9) reads about 4.6–4.7 V (5 V minus D1).
4. **Power off, plug in the RP2040 and the 74HC14** (notch left). Power on: J9's 3V3 pin reads 3.3 V.
5. **Power off, plug in the INA219 upside down.** Its VCC pin goes in H-1; check against the printed labels before powering. Power on: it should answer at I2C address 0x40.
6. **USB check.** With the battery off, plug in USB. The RP2040 runs, and J9's 5V pin reads about 0 V because D1 is blocking. Switch the battery on with USB still connected: J9's 5V reads 5.0 V.
7. Connect the drivers, encoders and receiver, then the battery wires to the INA219's screw terminals.

## 8. Off-board wiring

| Board | Cable | Other end |
|---|---|---|
| J1 + / − | 2 wires | buck-boost OUT+ / OUT− |
| INA219 screw terminal VIN+ / VIN− | twisted pair, ≥ 18 AWG | VIN+ from the battery (+ via XT60), VIN− to the power board's 12 V point |
| J2 RPWM, LPWM, R_EN, L_EN | 4 jumper wires | left HW-039 R_PWM, L_PWM, R_EN, L_EN |
| J3 (same) | 4 jumper wires | right HW-039 |
| J4 VCC L / VCC R | 1 wire each | VCC pin of the left / right HW-039 |
| — | — | HW-039 **GND, R_IS, L_IS: leave unconnected** |
| J5 LED+, GND, ENC | 3-pin plug | left motor harness pins 2, 3, 4 (LED anode, common ground, phototransistor collector) |
| J6 ENC, GND, LED+ | 3-pin plug | right motor harness pins 4, 3, 2 (mirrored motor, so the same cable fits) |
| J7 / J8 SW, 3V3 | 2-pin plug | WOW switch, motor harness pins 6 and 7 (either way round) |
| J9 | your future sensor cable | 5V, GND, 3V3, ADC0–3, I2C |
| J12 S, 5V, GND | servo extension (female–female) | FS-iA10B i-BUS servo port: S, +, − |
| — | — | motor harness pins 1 and 5 (M+ / M−) go to the HW-039 M+ / M− terminals |

On J5/J6, GND is the middle pin. A plug turned the wrong way swaps LED+ and ENC, so check before powering up.

## 9. The power board (12 V side)

- **12 V path:** battery + (XT60) → INA219 **VIN+** (on this board's edge) → INA219 **VIN−** → the power board's 12 V point. That point feeds the left HW-039 B+, the right HW-039 B+ and the buck-boost IN+. Run the two INA219 wires as a twisted pair and keep them away from the encoder and WOW cables.
- **Ground: one star point at the battery −.** Run separate wires from it to the left B−, the right B− and the buck IN−. Don't chain them from one terminal to the next.
- **At each HW-039:** solder a 220–470 µF ≥ 25 V capacitor across the B+/B− terminals.
- **At each motor:** solder 100 nF across the motor terminals and twist the motor wires.
- **At the buck-boost:** 100 µF across its output.
- **Cable routing:** keep the encoder/WOW cables away from the motor wires. If they have to run side by side, cross them at right angles.
- **Wire sizes:** 18 AWG or heavier for the battery, B+/B− and motor wires.
- **Buck-boost OUT+ / OUT−** go to this board's J1. That is the **only** ground wire between the two boards.
- **INA219 range:** its 0.1 Ω shunt reads up to ±3.2 A, and all motor current flows through it. If both motors together can draw more than that (stall, hard starts), the reading clips. At 5 A the shunt also dissipates 2.5 W. Check your motors' stall current.

## 10. Check these before soldering

- **Board:** 24 x 18 holes.
- **INA219 orientation:**
  - **Header:** solder it on the **component side**, so with the module upside down the pins point down into the board.
  - **Pin order:** plugged in, the pins read VCC, GND, SCL, SDA, VIN−, VIN+ from row H down to row M.
  - **Clearance:** the outline on the drawing is approximate (about 22 x 25 mm). Check that the module clears J1, and that J1's wires and the INA219's down-facing screw terminal don't fight for space at the left edge.
- **RP2040-Zero, USB at the top:**
  - **Left row:** 5V, GND, 3V3, GP29, GP28, GP27, GP26, GP15, GP14.
  - **Right row:** GP0–GP8.
  - **Bottom edge:** GP13, GP12, GP11, GP10, GP9 from left to right, in line with GP14 and GP8. Those 5 pins need a male header pointing **down**, like the sides.
- **FS-iA10B iBUS level:** the R6/R7 divider handles either 3.3 V or 5 V. After building, measure GP13 against GND with the receiver bound and the transmitter on. It should stay under 3.5 V.
- **Motor harness order** (pins 1–7: M+, LED anode, common GND, collector, M−, WOW, WOW) and **HW-039 header** (top row VCC, R_IS, R_EN, R_PWM; bottom row GND, L_IS, L_EN, L_PWM): taken as you described them.
