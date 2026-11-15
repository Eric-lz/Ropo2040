#!/usr/bin/env python3
"""Ropo2040 logic board: perfboard layout generator (50 x 70 mm board, 24 x 18 holes).

Writes:
  layout_top.svg     component side, as you look at it
  layout_bottom.svg  solder side, flipped left<->right (what you see while soldering)
  wiring_list.md     tick-box list of every bare link, plus the holes for every part

Everything on the solder side is bare tinned wire laid along the rows and columns.
Where a connection must cross another, it does so on the component side: either a
resistor lying flat over the traces, or a short straight insulated jumper.

The script also checks the wiring: every bare link is merged into connected
groups of holes, and each group must carry exactly one net from the pin list.
Run it after any edit:  python make_layout.py
"""
import os
import sys
from html import escape

OUT = os.path.dirname(os.path.abspath(__file__))

COLS, ROWS = 24, 18
ROWL = "ABCDEFGHIJKLMNOPQR"


def hn(c, r):
    """Hole name, e.g. (4, 3) -> 'C-4' (row letter, dash, column number)."""
    return f"{ROWL[r - 1]}-{c}"


# Board edges in hole units (hole 1 is at 1.0). 70 mm / 50 mm board, grid centred.
EDGE_L, EDGE_R = -1.28, 26.28
EDGE_T, EDGE_B = -0.34, 19.34

# --------------------------------------------------------------------------
# Net colours (pin rings, bare-link centre stripes)
# --------------------------------------------------------------------------
NETC = {
    'GND': '#1a1a1a', '3V3': '#e67e00', 'VBUS': '#d62728', '5V_IN': '#ff5fb0',
    'SDA': '#1f77b4', 'SCL': '#17a2b8',
    'A0': '#8c8c8c', 'A1': '#8c8c8c', 'A2': '#8c8c8c', 'A3': '#8c8c8c',
    'ENC_L_RAW': '#2ca02c', 'ENC_R_RAW': '#6b8e23', 'ENC_L': '#00a676', 'ENC_R': '#4caf50',
    'WOW_L': '#9467bd', 'WOW_R': '#b07cc6', 'LED_L': '#c49a00', 'LED_R': '#c49a00',
    'DRV_L_RPWM': '#0057b8', 'DRV_L_LPWM': '#3f8efc', 'DRV_L_EN': '#e377c2',
    'DRV_R_RPWM': '#8a3ffc', 'DRV_R_LPWM': '#c27bff', 'DRV_R_EN': '#f06bb5',
    'IBUS': '#008b8b', 'IBUS_RX': '#008b8b', 'IBUS_DIV': '#20b2aa', 'SPARE': '#7f7f00', 'NC': '#bbbbbb',
}
NET_DESC = {
    'GND': 'ground', '3V3': '3.3 V from the RP2040-Zero regulator',
    'VBUS': 'RP2040 5V pin (USB VBUS), after the diode',
    '5V_IN': '5 V from the buck-boost, before the diode',
}

# --------------------------------------------------------------------------
# Components. pins: (col, row, pin label, net). body: rect (c0, r0, c1, r1) in hole units.
# --------------------------------------------------------------------------
C = []


def comp(ref, title, kind, pins, body=None, fill='#555', plabel='R', note='', extra=None):
    C.append(dict(ref=ref, title=title, kind=kind, pins=pins, body=body, fill=fill,
                  plabel=plabel, note=note, extra=extra or []))


# RP2040-Zero, USB-C toward the top edge, on a U of female headers:
# 1x8 (column 9, rows C-J) + 1x8 (column 15, rows C-J) + 1x7 (row K, columns 9-15).
LNAMES = ['5V', 'GND', '3V3', 'GP29', 'GP28', 'GP27', 'GP26', 'GP15', 'GP14']
LNETS = ['VBUS', 'GND', '3V3', 'A3', 'A2', 'A1', 'A0', 'SCL', 'SDA']
RNETS = ['DRV_L_RPWM', 'DRV_L_LPWM', 'DRV_L_EN', 'WOW_R', 'DRV_R_RPWM', 'DRV_R_LPWM', 'DRV_R_EN', 'SPARE', 'ENC_R']
BNAMES = ['GP13', 'GP12', 'GP11', 'GP10', 'GP9']
BNETS = ['IBUS', 'WOW_L', 'NC', 'NC', 'ENC_L']
pins = [(9, 3 + i, LNAMES[i], LNETS[i]) for i in range(9)]
pins += [(15, 3 + i, f'GP{i}', RNETS[i]) for i in range(9)]
pins += [(10 + i, 11, BNAMES[i], BNETS[i]) for i in range(5)]
comp('U1', 'RP2040-Zero', 'mcu', pins, body=(8.46, 2.8, 15.54, 12.05), fill='#24324a')

# Expansion header: 1 x 9 male, a 1:1 copy of the RP2040 left row (its 5V pin is 5V_IN).
XL = ['5V', 'GND', '3V3', 'A3', 'A2', 'A1', 'A0', 'SCL', 'SDA']
XN = ['5V_IN', 'GND', '3V3', 'A3', 'A2', 'A1', 'A0', 'SCL', 'SDA']
comp('J9', 'EXPANSION', 'header', [(7, 3 + i, XL[i], XN[i]) for i in range(9)],
     body=(6.5, 2.5, 7.5, 11.5), plabel='R')

# INA219 module, mounted upside down (header soldered on the component side) on a 1x6 female
# header in column 1, body hanging off the left edge. Upside down, its pins read VCC, GND, SCL,
# SDA, VIN-, VIN+ from top to bottom, which lines SCL/SDA up with the RP2040's I2C rows.
# VIN-/VIN+ carry battery voltage on the module: their holes here stay unconnected.
comp('U2', 'INA219 (upside down)', 'ina',
     [(1, 8, 'VCC', '3V3'), (1, 9, 'GND', 'GND'), (1, 10, 'SCL', 'SCL'), (1, 11, 'SDA', 'SDA'),
      (1, 12, 'VIN−', 'NC'), (1, 13, 'VIN+', 'NC')],
     body=(-7.0, 5.6, 1.7, 15.4), fill='#6a3d9a', plabel='R',
     note='1x6 female header; module hangs off the left edge, components facing down')

# Power entry: buck 5 V in on J1, D1 into the RP2040 5V pin
comp('J1', '5V IN', 'screw', [(1, 2, '+', '5V_IN'), (1, 4, '−', 'GND')],
     body=(-0.7, 1.03, 2.4, 4.97), fill='#2a7ab0', note='from the buck-boost output')
comp('D1', '1N5819', 'DS', [(8, 3, 'K', 'VBUS'), (8, 2, 'A', '5V_IN')],
     note='Schottky, standing: body over B-8, stripe (cathode) end up, top lead bent down into C-8')
# RC receiver: S, 5V, GND from top to bottom (5V in the middle, so a servo plug fits)
comp('J12', 'RC iBUS', 'header', [(4, 2, 'S', 'IBUS_RX'), (4, 3, '5V', '5V_IN'), (4, 4, 'GND', 'GND')],
     body=(3.5, 1.5, 4.5, 4.5), plabel='R')

# Motor drivers (signals only; HW-039 GND stays unconnected, see BUILD.md)
comp('J2', 'DRIVER L', 'header',
     [(17, 3, 'RPWM', 'DRV_L_RPWM'), (17, 4, 'LPWM', 'DRV_L_LPWM'), (17, 5, 'R_EN', 'DRV_L_EN'),
      (18, 5, 'L_EN', 'DRV_L_EN')], plabel='R',
     extra=[(16.5, 2.5, 17.5, 5.5), (17.5, 4.5, 18.5, 5.5)])
comp('J3', 'DRIVER R', 'header',
     [(17, 7, 'RPWM', 'DRV_R_RPWM'), (17, 8, 'LPWM', 'DRV_R_LPWM'), (17, 9, 'R_EN', 'DRV_R_EN'),
      (18, 9, 'L_EN', 'DRV_R_EN')], plabel='R',
     extra=[(16.5, 6.5, 17.5, 9.5), (17.5, 8.5, 18.5, 9.5)])
comp('J4', 'DRIVER VCC', 'header', [(20, 3, 'VCC L', '3V3'), (20, 4, 'VCC R', '3V3')],
     body=(19.5, 2.5, 20.5, 4.5), plabel='R')

# 74HC14 in a DIP-14 socket, notch (pin 1 end) on the left.
U3TOP = [('14', 'VCC', '3V3'), ('13', '6A', '3V3'), ('12', '6Y', 'NC'), ('11', '5A', 'ENC_L_RAW'),
         ('10', '5Y', 'ENC_L'), ('9', '4A', 'ENC_R_RAW'), ('8', '4Y', 'ENC_R')]
U3BOT = [('1', '1A', 'GND'), ('2', '1Y', 'NC'), ('3', '2A', 'GND'), ('4', '2Y', 'NC'),
         ('5', '3A', 'GND'), ('6', '3Y', 'NC'), ('7', 'GND', 'GND')]
comp('U3', '74HC14', 'dip',
     [(10 + i, 14, f'{n} {f}', net) for i, (n, f, net) in enumerate(U3TOP)] +
     [(10 + i, 17, f'{n} {f}', net) for i, (n, f, net) in enumerate(U3BOT)],
     body=(9.26, 13.4, 16.74, 17.6), fill='#1d1d1d')

# Motor sensor headers: encoder (3 pins, same order as harness pins 2-4) and WOW switch (2 pins)
comp('J5', 'ENCODER L', 'header', [(6, 16, 'LED+', 'LED_L'), (7, 16, 'GND', 'GND'),
                                   (8, 16, 'ENC', 'ENC_L_RAW')],
     body=(5.5, 15.5, 8.5, 16.5), plabel='D')
comp('J6', 'ENCODER R', 'header', [(18, 16, 'ENC', 'ENC_R_RAW'), (19, 16, 'GND', 'GND'),
                                   (20, 16, 'LED+', 'LED_R')],
     body=(17.5, 15.5, 20.5, 16.5), plabel='D')
comp('J7', 'WOW L', 'header', [(7, 13, 'SW', 'WOW_L'), (7, 14, '3V3', '3V3')],
     body=(6.5, 12.5, 7.5, 14.5), plabel='R')
comp('J8', 'WOW R', 'header', [(22, 13, 'SW', 'WOW_R'), (22, 14, '3V3', '3V3')],
     body=(21.5, 12.5, 22.5, 14.5), plabel='U')
comp('J11', 'SPARE GP7', 'header', [(21, 13, 'GP7', 'SPARE'), (21, 14, '3V3', '3V3')],
     body=(20.5, 12.5, 21.5, 14.5), plabel='U')

# Pin-label side overrides for crowded spots: L/R = left/right of the pin, U/D = above/below.
LBL_SIDE = {(17, 5): 'D', (18, 5): 'D', (17, 9): 'D', (18, 9): 'D',
            (21, 13): 'U', (21, 14): 'D', (22, 13): 'U', (22, 14): 'D'}

# Passives: (ref, value, pin1, pin2, kind, body-over-hole or mounting note)
PASSIVES = [
    ('R1', '5k1', (8, 14, '3V3'), (8, 15, 'ENC_L_RAW'), 'R', 2),
    ('R2', '5k1', (18, 14, '3V3'), (18, 15, 'ENC_R_RAW'), 'R', 1),
    ('R3', '100R', (5, 15, '3V3'), (6, 15, 'LED_L'), 'R', 1),
    ('R4', '100R', (20, 14, '3V3'), (20, 15, 'LED_R'), 'R', 2),
    # iBUS divider and bridge. All three lie flat (10.16 mm) and each body bridges bare traces:
    # R6 over the 5V/GND/3V3 lanes, R7 over the 3V3 and GND trunks, R5 over the I2C lanes.
    ('R6', '1k', (6, 2, 'IBUS_RX'), (6, 6, 'IBUS_DIV'), 'R', 0),
    ('R7', '2k2', (6, 7, 'IBUS_DIV'), (2, 7, 'GND'), 'R', 0),
    ('R5', '1k', (6, 8, 'IBUS_DIV'), (6, 12, 'IBUS'), 'R', 0),
    ('C1', '100n', (5, 16, '3V3'), (4, 16, 'GND'), 'C', 0),
    ('C2', '1n', (8, 17, 'ENC_L_RAW'), (7, 17, 'GND'), 'C', 0),
    ('C3', '1n', (18, 17, 'ENC_R_RAW'), (19, 17, 'GND'), 'C', 0),
    ('C4', '100n', (6, 13, 'WOW_L'), (6, 14, '3V3'), 'C', 0),
    ('C5', '100n', (23, 13, 'WOW_R'), (23, 14, '3V3'), 'C', 0),
]
for ref, val, a, b, k, bo in PASSIVES:
    comp(ref, val, k, [(a[0], a[1], '', a[2]), (b[0], b[1], '', b[2])],
         note=('lying flat, 10.16 mm' if bo == 0 else 'standing') if k == 'R' else 'ceramic, 2.54 mm')

# --------------------------------------------------------------------------
# Bare links on the solder side (straight runs; every hole they pass is joined).
# --------------------------------------------------------------------------
BARE = []
# RP2040 left row -> J9 (rows D-K; the 5V pin goes to D1 instead)
for i, n in enumerate(LNETS[1:], 1):
    BARE.append((n, (9, 3 + i), (7, 3 + i)))
BARE += [
    # 5V: RP2040 5V pin -> D1 cathode; D1 anode -> J9 5V -> row C lane -> J12 5V -> J1 +
    ('VBUS', (9, 3), (8, 3)),
    ('5V_IN', (8, 2), (7, 2)), ('5V_IN', (7, 2), (7, 3)), ('5V_IN', (7, 3), (1, 3)),
    ('5V_IN', (1, 3), (1, 2)),
    # GND: J9 -> row D -> trunk down column 3 -> bottom rail (row R)
    ('GND', (7, 4), (1, 4)), ('GND', (3, 4), (3, 9)), ('GND', (3, 9), (1, 9)),
    ('GND', (3, 12), (3, 18)), ('GND', (3, 18), (19, 18)),
    ('GND', (2, 7), (3, 7)), ('GND', (4, 16), (3, 16)),
    ('GND', (7, 16), (7, 18)), ('GND', (19, 16), (19, 18)),
    ('GND', (10, 17), (10, 18)), ('GND', (12, 17), (12, 18)), ('GND', (14, 17), (14, 18)),
    ('GND', (16, 17), (16, 18)),
    # 3V3, left: J9 -> trunk (column 5) -> INA219 VCC (via a jumper) -> row N -> 74HC14
    ('3V3', (7, 5), (5, 5)), ('3V3', (5, 5), (5, 9)), ('3V3', (5, 8), (4, 8)),
    ('3V3', (2, 8), (1, 8)),
    ('3V3', (5, 12), (5, 16)), ('3V3', (5, 14), (11, 14)),
    # 3V3, right: under the RP2040, out over the top edge, down column 24, back along row N
    ('3V3', (9, 5), (11, 5)), ('3V3', (11, 5), (11, 2)), ('3V3', (11, 2), (24, 2)),
    ('3V3', (24, 2), (24, 14)), ('3V3', (24, 14), (18, 14)), ('3V3', (20, 2), (20, 4)),
    # I2C: J9 straight on to the INA219
    ('SCL', (7, 10), (1, 10)), ('SDA', (7, 11), (1, 11)),
    # motor drivers (R_EN and L_EN joined on the board)
    ('DRV_L_RPWM', (15, 3), (17, 3)), ('DRV_L_LPWM', (15, 4), (17, 4)), ('DRV_L_EN', (15, 5), (18, 5)),
    ('DRV_R_RPWM', (15, 7), (17, 7)), ('DRV_R_LPWM', (15, 8), (17, 8)), ('DRV_R_EN', (15, 9), (18, 9)),
    # GP3 -> WOW R, GP7 -> spare header
    ('WOW_R', (15, 6), (23, 6)), ('WOW_R', (23, 6), (23, 13)), ('WOW_R', (23, 13), (22, 13)),
    ('SPARE', (15, 10), (21, 10)), ('SPARE', (21, 10), (21, 13)),
    # 74HC14 outputs straight up to GP8 / GP9
    ('ENC_R', (15, 11), (16, 11)), ('ENC_R', (16, 11), (16, 14)),
    ('ENC_L', (14, 11), (14, 14)),
    # GP12 -> WOW L
    ('WOW_L', (11, 11), (11, 13)), ('WOW_L', (11, 13), (6, 13)),
    # iBUS: J12 S -> R6 -> divider node (column 6) -> R7 to GND, R5 -> row L -> GP13
    ('IBUS_RX', (4, 2), (6, 2)), ('IBUS_DIV', (6, 6), (6, 8)),
    ('IBUS', (6, 12), (10, 12)), ('IBUS', (10, 12), (10, 11)),
    # encoder inputs: out from under the 74HC14 socket (between its pin rows)
    ('ENC_L_RAW', (13, 14), (13, 15)), ('ENC_L_RAW', (13, 15), (8, 15)),
    ('ENC_L_RAW', (8, 15), (8, 17)),
    ('ENC_R_RAW', (15, 14), (15, 15)), ('ENC_R_RAW', (15, 15), (18, 15)),
    ('ENC_R_RAW', (18, 15), (18, 17)),
    ('LED_L', (6, 15), (6, 16)), ('LED_R', (20, 15), (20, 16)),
]

# Straight insulated jumpers on the component side.
JUMPERS = [
    ('3V3', (4, 8), (2, 8)),    # INA219 VCC, over the GND trunk (H-3)
    ('GND', (3, 9), (3, 12)),   # GND trunk, over the SCL/SDA lanes (J-3, K-3)
    ('3V3', (5, 9), (5, 12)),   # 3V3 trunk, over the SCL/SDA lanes (J-5, K-5)
]


# --------------------------------------------------------------------------
# Connectivity check
# --------------------------------------------------------------------------
def seg_holes(a, b):
    (c1, r1), (c2, r2) = a, b
    if c1 != c2 and r1 != r2:
        raise ValueError(f'link {hn(*a)}-{hn(*b)} is not straight')
    if c1 == c2:
        return [(c1, r) for r in range(min(r1, r2), max(r1, r2) + 1)]
    return [(c, r1) for c in range(min(c1, c2), max(c1, c2) + 1)]


def check():
    errors = []
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        parent[find(a)] = find(b)

    lead = {}  # hole -> (ref, pin, net)
    for cp in C:
        for c, r, lab, net in cp['pins']:
            if not (1 <= c <= COLS and 1 <= r <= ROWS):
                errors.append(f'{cp["ref"]} pin off board at {c},{r}')
            if (c, r) in lead:
                errors.append(f'hole {hn(c, r)} used by {lead[(c, r)][0]} and {cp["ref"]}')
            lead[(c, r)] = (cp['ref'], lab, net)
            find((c, r))
    bare_net = {}
    for net, a, b in BARE:
        hs = seg_holes(a, b)
        for h in hs:
            if not (1 <= h[0] <= COLS and 1 <= h[1] <= ROWS):
                errors.append(f'bare {net} off board {h}')
            if h in bare_net and bare_net[h] != net:
                errors.append(f'hole {hn(*h)} on bare links of {bare_net[h]} and {net}')
            bare_net[h] = net
        for h in hs[1:]:
            union(hs[0], h)
    for net, a, b in JUMPERS:
        seg_holes(a, b)
        union(a, b)
    groups = {}
    for h in parent:
        groups.setdefault(find(h), set()).add(h)
    netgroups = {}
    for g, hs in groups.items():
        nets = {lead[h][2] for h in hs if h in lead}
        if 'NC' in nets and len(hs) > 1:
            errors.append(f'NC pin connected: {[hn(*h) for h in hs]}')
        real = nets - {'NC'}
        if len(real) > 1:
            errors.append(f'SHORT between {sorted(real)} via {sorted(hn(*h) for h in hs)}')
        for n in real:
            netgroups.setdefault(n, []).append(g)
        for h in hs:
            if h in bare_net and h in lead and lead[h][2] != bare_net[h]:
                errors.append(f'bare {bare_net[h]} touches {lead[h][0]} {lead[h][1]} '
                              f'({lead[h][2]}) at {hn(*h)}')
    for net, a, b in JUMPERS:
        for h in (a, b):
            if h in lead:
                errors.append(f'jumper {net} ends on a component pin at {hn(*h)}')
    for n, gs in netgroups.items():
        if len(gs) > 1:
            parts = []
            for g in gs:
                parts.append(sorted(f'{lead[h][0]}.{lead[h][1] or hn(*h)}' for h in groups[g] if h in lead))
            errors.append(f'net {n} is split into {len(gs)} pieces: {parts}')
    used = {}
    for h, (_, _, net) in lead.items():
        used[h] = net
    for h, net in bare_net.items():
        used.setdefault(h, net)
    close = 0
    for (c, r), net in used.items():
        for d in ((1, 0), (0, 1)):
            o = (c + d[0], r + d[1])
            if o in used and used[o] != net and 'NC' not in (net, used[o]):
                close += 1
    return errors, close, lead, bare_net


# --------------------------------------------------------------------------
# SVG drawing
# --------------------------------------------------------------------------
S = 40                      # px per 2.54 mm
MX, TOP = 70, 110           # side margin, space for title
BOT = 64                    # space below the board for labels
PAD = 6.4                   # extra canvas (hole units) for the INA219 hanging off the left edge
W = int(MX * 2 + (EDGE_R - EDGE_L + PAD) * S)
BOARD_H = (EDGE_B - EDGE_T) * S
FONT = "font-family='Segoe UI, Arial, sans-serif'"


class V:
    def __init__(self, mirror):
        self.m = mirror

    def x(self, c):
        return MX + ((EDGE_R - c) if self.m else (c - EDGE_L + PAD)) * S

    def y(self, r):
        return TOP + (r - EDGE_T) * S

    def rect(self, c0, r0, c1, r1):
        xa, xb = sorted((self.x(c0), self.x(c1)))
        return xa, self.y(r0), xb - xa, self.y(r1) - self.y(r0)


def txt(x, y, s, size=10, fill='#111', anchor='middle', weight='normal', halo=True,
        rotate=None, italic=False):
    tr = f" transform='rotate({rotate} {x:.1f} {y:.1f})'" if rotate else ''
    h = (" stroke='#fff' stroke-width='3' paint-order='stroke' stroke-linejoin='round'"
         if halo else '')
    st = " font-style='italic'" if italic else ''
    return (f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' fill='{fill}' "
            f"text-anchor='{anchor}' dominant-baseline='central' font-weight='{weight}' "
            f"{FONT}{h}{st}{tr}>{escape(s)}</text>")


def board_and_grid(v, out):
    bx, by, bw, bh = v.rect(EDGE_L, EDGE_T, EDGE_R, EDGE_B)
    out.append(f"<rect x='{bx}' y='{by}' width='{bw}' height='{bh}' rx='10' "
               f"fill='#d8bd85' stroke='#8a6d3b' stroke-width='2'/>")
    for c, r in ((-0.55, 0.4), (25.55, 0.4), (-0.55, 18.6), (25.55, 18.6)):
        out.append(f"<circle cx='{v.x(c)}' cy='{v.y(r)}' r='{S*0.2}' fill='#fff' stroke='#8a6d3b'/>")
    for c in range(1, COLS + 1):
        for r in range(1, ROWS + 1):
            out.append(f"<circle cx='{v.x(c):.1f}' cy='{v.y(r):.1f}' r='{S*0.30:.1f}' "
                       f"fill='#c98a3c' stroke='#9c6a2c' stroke-width='0.8'/>")
            out.append(f"<circle cx='{v.x(c):.1f}' cy='{v.y(r):.1f}' r='{S*0.12:.1f}' fill='#4a3418'/>")
    for c in range(1, COLS + 1):
        out.append(txt(v.x(c), v.y(EDGE_T) - 14, str(c), 11, '#444', halo=False, weight='bold'))
        out.append(txt(v.x(c), v.y(EDGE_B) + 14, str(c), 11, '#444', halo=False, weight='bold'))
    for r in range(1, ROWS + 1):
        xl, xr = sorted((v.x(EDGE_L), v.x(EDGE_R)))
        out.append(txt(xl - 16, v.y(r), ROWL[r - 1], 11, '#444', halo=False, weight='bold'))
        out.append(txt(xr + 16, v.y(r), ROWL[r - 1], 11, '#444', halo=False, weight='bold'))


def header_outline(v, out, rects):
    for rc in rects:
        x, y, w, h = v.rect(*rc)
        out.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='3' "
                   f"fill='#2b2b2b' fill-opacity='0.88' stroke='#000' stroke-width='1.2'/>")


def draw_top(lead):
    v = V(False)
    H = int(TOP + BOARD_H + BOT + 190)
    out = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}'>",
           f"<rect width='{W}' height='{H}' fill='#ffffff'/>"]
    out.append(txt(MX, 30, 'Ropo2040 logic board: COMPONENT SIDE (top view)', 20, '#111', 'start',
                   'bold', halo=False))
    out.append(txt(MX, 56, '50 x 70 mm, 24 x 18 holes, 2.54 mm pitch. Holes are named row letter + '
                   'column number (e.g. C-4). Pin rings are coloured by net.', 12, '#444', 'start',
                   halo=False))
    out.append(txt(MX, 76, 'No 12 V on this board. Bare links on the solder side (layout_bottom.svg); '
                   'crossings are made up here by flat resistors (R5-R7) and 3 straight insulated jumpers.',
                   12, '#a33', 'start', halo=False))
    board_and_grid(v, out)

    # RP2040 module
    for cp in C:
        if cp['kind'] != 'mcu':
            continue
        x, y, w, h = v.rect(*cp['body'])
        out.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='6' "
                   f"fill='{cp['fill']}' fill-opacity='0.92' stroke='#111' stroke-width='1.5'/>")
        ux, uy, uw, uh = v.rect(10.6, 2.25, 13.4, 3.35)
        out.append(f"<rect x='{ux}' y='{uy}' width='{uw}' height='{uh}' rx='4' fill='#aab3bf' stroke='#333'/>")
        out.append(txt(ux + uw / 2, uy + uh / 2, 'USB-C', 10, '#111', halo=False, weight='bold'))
        out.append(txt(v.x(12), v.y(5.0), 'U1', 18, '#fff', weight='bold', halo=False))
        out.append(txt(v.x(12), v.y(5.8), 'RP2040-Zero', 13, '#fff', halo=False))
        out.append(txt(v.x(12), v.y(6.6), 'in female headers:', 10, '#cfd8e3', halo=False))
        out.append(txt(v.x(12), v.y(7.1), '1x8 + 1x8 (sides)', 10, '#cfd8e3', halo=False))
        out.append(txt(v.x(12), v.y(7.6), '+ 1x7 (row K)', 10, '#cfd8e3', halo=False))
        out.append(txt(v.x(12), v.y(8.5), 'GP10, GP11: unused', 9, '#cfd8e3', halo=False))

    # INA219 module hanging off the left edge (upside down)
    for cp in C:
        if cp['kind'] != 'ina':
            continue
        x, y, w, h = v.rect(*cp['body'])
        out.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='6' "
                   f"fill='{cp['fill']}' fill-opacity='0.55' stroke='#111' stroke-width='1.5' "
                   f"stroke-dasharray='7 4'/>")
        tx, ty, tw, th = v.rect(-6.6, 8.2, -4.4, 12.8)
        out.append(f"<rect x='{tx}' y='{ty}' width='{tw}' height='{th}' rx='3' fill='#2a7ab0' "
                   f"fill-opacity='0.5' stroke='#1d5b85' stroke-dasharray='4 3'/>")
        out.append(txt(tx + tw / 2, ty + th / 2, 'screw terminal (underneath)', 9, '#fff',
                       halo=False, rotate=-90, weight='bold'))
        cx = v.x(-1.6)
        out.append(txt(cx, v.y(6.4), 'U2 INA219', 14, '#fff', weight='bold', halo=False))
        out.append(txt(cx, v.y(7.0), 'UPSIDE DOWN', 11, '#ffe08a', weight='bold', halo=False))
        out.append(txt(cx, v.y(13.8), 'hangs off the edge', 10, '#f0e6ff', halo=False))
        out.append(txt(cx, v.y(14.4), '(outline approx.)', 10, '#f0e6ff', halo=False))
        header_outline(v, out, [(0.5, 7.5, 1.5, 13.5)])

    # sockets / headers / screw terminal
    for cp in C:
        k = cp['kind']
        if k == 'dip':
            x, y, w, h = v.rect(*cp['body'])
            out.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='4' fill='#6b6b6b' "
                       f"stroke='#000' stroke-width='1.5'/>")
            cx, cy, cw, ch = v.rect(9.6, 14.45, 16.4, 16.55)
            out.append(f"<rect x='{cx}' y='{cy}' width='{cw}' height='{ch}' rx='3' fill='#222' stroke='#000'/>")
            out.append(f"<path d='M {cx} {cy+ch/2-9} a 9 9 0 0 1 0 18' fill='#6b6b6b'/>")
            out.append(txt(cx + cw / 2, cy + ch / 2, 'U3 74HC14 (DIP-14 socket)', 13, '#fff',
                           weight='bold', halo=False))
        elif k in ('header', 'header2'):
            rects = cp['extra'] or [cp['body']]
            header_outline(v, out, rects)
        elif k == 'screw':
            x, y, w, h = v.rect(*cp['body'])
            out.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='4' fill='{cp['fill']}' "
                       f"fill-opacity='0.92' stroke='#111' stroke-width='1.5'/>")
            rs = [p[1] for p in cp['pins']]
            pc = cp['pins'][0][0]
            for r in rs:
                ex, ey, ew, eh = v.rect(pc - 1.55, r - 0.32, pc - 0.95, r + 0.32)
                out.append(f"<rect x='{ex}' y='{ey}' width='{ew}' height='{eh}' fill='#0d2f45'/>")
            rm = sum(rs) / len(rs)
            out.append(txt(v.x(pc + 0.95), v.y(rm), 'J1', 12, '#fff', weight='bold', halo=False,
                           rotate=-90))
            out.append(txt(v.x(pc - 2.2), v.y(rm), '5V IN (buck)', 10, '#1d5b85', weight='bold',
                           halo=False, rotate=-90))

    # diode, electrolytic
    for cp in C:
        if cp['kind'] == 'DS':
            (c1, r1, *_), (c2, r2, *_) = cp['pins']
            out.append(f"<line x1='{v.x(c1)}' y1='{v.y(r1)}' x2='{v.x(c2)}' y2='{v.y(r2)}' "
                       f"stroke='#777' stroke-width='3'/>")
            bx, by = v.x(c2), v.y(r2)
            out.append(f"<circle cx='{bx}' cy='{by}' r='{S*0.36}' fill='#222' stroke='#000'/>")
            out.append(f"<circle cx='{bx}' cy='{by}' r='{S*0.2}' fill='none' stroke='#ddd' stroke-width='3'/>")
            out.append(txt(bx - 17, by - 6, 'D1', 10, '#000', 'end', 'bold'))
            out.append(txt(bx - 17, by + 6, '1N5819', 8, '#5a3d00', 'end', 'bold'))
        if cp['kind'] == 'D':
            (c1, r1, *_), (c2, r2, *_) = cp['pins']
            x, y, w, h = v.rect(c1 - 0.28, r1 + 0.55, c1 + 0.28, r2 - 0.55)
            out.append(f"<line x1='{v.x(c1)}' y1='{v.y(r1)}' x2='{v.x(c2)}' y2='{v.y(r2)}' "
                       f"stroke='#777' stroke-width='3'/>")
            out.append(f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='3' fill='#222' stroke='#000'/>")
            bx, by, bw, bh = v.rect(c1 - 0.28, r1 + 0.62, c1 + 0.28, r1 + 0.85)
            out.append(f"<rect x='{bx}' y='{by}' width='{bw}' height='{bh}' fill='#ddd'/>")
            out.append(txt(v.x(c1) + 18, v.y((r1 + r2) / 2) - 7, 'D1', 10, '#000', 'start', 'bold'))
            out.append(txt(v.x(c1) + 18, v.y((r1 + r2) / 2) + 6, '1N5819', 8, '#5a3d00', 'start', 'bold'))
        if cp['kind'] == 'CE':
            (c1, r1, *_), (c2, r2, *_) = cp['pins']
            cx, cy = v.x((c1 + c2) / 2), v.y((r1 + r2) / 2)
            out.append(f"<circle cx='{cx}' cy='{cy}' r='{S*0.98}' fill='#1f3d7a' fill-opacity='0.85' "
                       f"stroke='#0b1a38' stroke-width='1.5'/>")
            neg = [p for p in cp['pins'] if p[2] == '−'][0]
            sgn = 1 if v.x(neg[0]) > cx else -1
            out.append(f"<path d='M {cx+sgn*S*0.55} {cy-S*0.8} A {S*0.98} {S*0.98} 0 0 {1 if sgn > 0 else 0} "
                       f"{cx+sgn*S*0.55} {cy+S*0.8}' fill='none' stroke='#cfd8e3' stroke-width='5'/>")
            out.append(txt(cx, cy - S * 0.62, 'CB1', 9, '#fff', weight='bold', halo=False))

    # small passives
    for ref, val, a, b, k, bo in PASSIVES:
        x1, y1, x2, y2 = v.x(a[0]), v.y(a[1]), v.x(b[0]), v.y(b[1])
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='#777' stroke-width='3'/>")
        if k == 'R' and bo == 0:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            horiz = abs(x2 - x1) > abs(y2 - y1)
            ln = max(abs(x2 - x1), abs(y2 - y1)) - S * 1.25
            bw, bh = (ln, S * 0.5) if horiz else (S * 0.5, ln)
            out.append(f"<rect x='{mx-bw/2}' y='{my-bh/2}' width='{bw}' height='{bh}' rx='{S*0.25}' "
                       f"fill='#e9d3a8' stroke='#6b4f1d' stroke-width='1.5'/>")
            bands = {'1k': ('#8b4513', '#000', '#e53935'), '2k2': ('#e53935', '#e53935', '#e53935')}
            for i, bc in enumerate(bands.get(val, ())):
                if horiz:
                    out.append(f"<rect x='{mx-bw/2+8+i*9}' y='{my-bh/2}' width='4' height='{bh}' fill='{bc}'/>")
                else:
                    out.append(f"<rect x='{mx-bw/2}' y='{my-bh/2+8+i*9}' width='{bw}' height='4' fill='{bc}'/>")
        elif k == 'R':
            bx, by = (x1, y1) if bo == 1 else (x2, y2)
            out.append(f"<circle cx='{bx}' cy='{by}' r='{S*0.36}' fill='#e9d3a8' stroke='#6b4f1d' "
                       f"stroke-width='1.5'/>")
        else:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            horiz = abs(x2 - x1) > abs(y2 - y1)
            bw, bh = (S * 0.75, S * 0.42) if horiz else (S * 0.42, S * 0.75)
            out.append(f"<rect x='{mx-bw/2}' y='{my-bh/2}' width='{bw}' height='{bh}' rx='4' "
                       f"fill='#f2c14e' stroke='#8a6d00' stroke-width='1.5'/>")

    # pins
    for cp in C:
        for c, r, lab, net in cp['pins']:
            col = NETC.get(net, '#000')
            if cp['kind'] in ('header', 'header2', 'screw', 'ina'):
                s = S * 0.62
                out.append(f"<rect x='{v.x(c)-s/2:.1f}' y='{v.y(r)-s/2:.1f}' width='{s}' height='{s}' "
                           f"rx='3' fill='#c9a227' stroke='{col}' stroke-width='4'/>")
            else:
                out.append(f"<circle cx='{v.x(c):.1f}' cy='{v.y(r):.1f}' r='{S*0.3}' fill='none' "
                           f"stroke='{col}' stroke-width='4'/>")
            out.append(f"<circle cx='{v.x(c):.1f}' cy='{v.y(r):.1f}' r='{S*0.12}' fill='#4a3418'/>")

    # pin labels
    for cp in C:
        k, side = cp['kind'], cp['plabel']
        if k in ('R', 'C', 'CE', 'D', 'DS', 'screw'):
            continue
        for c, r, lab, net in cp['pins']:
            x, y = v.x(c), v.y(r)
            if k == 'ina':
                col = '#ffb3b3' if 'VIN' in lab else '#fff'
                out.append(txt(x - 16, y, lab + (' (12 V, no trace)' if 'VIN' in lab else ''), 9, col,
                               'end', halo=False, weight='bold'))
            elif k == 'mcu':
                if c == 9:
                    out.append(txt(x + 15, y, lab, 9, '#fff', 'start', halo=False, weight='bold'))
                elif c == 15 and r <= 10:
                    out.append(txt(x - 15, y, lab, 9, '#fff', 'end', halo=False, weight='bold'))
                elif c == 15:
                    out.append(txt(x - 15, y - 4, lab, 8, '#fff', 'end', halo=False, weight='bold'))
                else:
                    out.append(txt(x, y - 22, lab, 8, '#fff', halo=False, weight='bold', rotate=-90))
            elif k == 'dip':
                n, f = lab.split()
                if r == 14:
                    out.append(txt(x, y + 14, n, 8, '#eee', halo=False))
                    out.append(txt(x, y - 17, f, 8, '#000', weight='bold'))
                else:
                    out.append(txt(x, y - 14, n, 8, '#eee', halo=False))
                    out.append(txt(x, y + 17, f, 8, '#000', weight='bold'))
            elif k == 'header2':
                if c == 6:
                    out.append(txt(x - 15, y, lab, 9, '#111', 'end', weight='bold'))
            elif LBL_SIDE.get((c, r), side) == 'D' and side != 'D':
                out.append(txt(x, y + 17, lab, 8, '#111', weight='bold'))
            elif LBL_SIDE.get((c, r)) == 'U':
                out.append(txt(x, y - 17, lab, 8, '#111', weight='bold'))
            elif side == 'L':
                out.append(txt(x - 15, y, lab, 9, '#111', 'end', weight='bold'))
            elif side == 'R':
                out.append(txt(x + 15, y, lab, 9, '#111', 'start', weight='bold'))
            elif side == 'D':
                out.append(txt(x, y + 18, lab, 8, '#111', weight='bold'))
    for cp in C:
        if cp['kind'] == 'screw':
            for c, r, lab, net in cp['pins']:
                out.append(txt(v.x(c) + 15, v.y(r), lab, 15, '#fff', 'start', halo=False, weight='bold'))
        if cp['kind'] == 'CE':
            pos = [p for p in cp['pins'] if p[2] == '+'][0]
            out.append(txt(v.x(pos[0]), v.y(pos[1]) + 15, '+', 13, '#fff', halo=False, weight='bold'))

    # insulated jumpers (component side)
    for i, (net, a, b) in enumerate(JUMPERS, 1):
        x1, y1, x2, y2 = v.x(a[0]), v.y(a[1]), v.x(b[0]), v.y(b[1])
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='#000' stroke-width='{S*0.26}' "
                   f"stroke-linecap='round'/>")
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{NETC[net]}' "
                   f"stroke-width='{S*0.17}' stroke-linecap='round'/>")
        for h in (a, b):
            out.append(f"<circle cx='{v.x(h[0])}' cy='{v.y(h[1])}' r='{S*0.12}' fill='#4a3418'/>")
        tx, ty = (x1 + x2) / 2, (y1 + y2) / 2
        if abs(x2 - x1) > abs(y2 - y1):
            ty += 15
        else:
            tx -= 20
        out.append(txt(tx, ty, f'JP{i}', 9, '#000', weight='bold'))

    # reference labels
    lbl = {
        'J9': (7.3, 0.95, 'J9 EXPANSION', 'middle'),
        'J12': (4.0, 0.95, 'J12 RC iBUS', 'middle'),
        'J2': (17.5, 2.05, 'J2 DRIVER L', 'middle'), 'J3': (17.5, 6.1, 'J3 DRIVER R', 'middle'),
        'J4': (20.7, 2.05, 'J4 DRIVER VCC', 'middle'),
        'J5': (7.0, 17.75, 'J5 ENCODER L', 'middle'), 'J6': (19.0, 17.75, 'J6 ENCODER R', 'middle'),
        'J7': (7.38, 12.6, 'J7 WOW L', 'start'),
        'J11': (20.3, 11.85, 'J11 SPARE', 'middle'), 'J8': (22.8, 11.85, 'J8 WOW R', 'middle'),
    }
    for ref, (c, r, s, a) in lbl.items():
        if s:
            out.append(txt(v.x(c), v.y(r), s, 11, '#000', a, 'bold'))
    for ref, val, a, b, k, bo in PASSIVES:
        x1, y1, x2, y2 = v.x(a[0]), v.y(a[1]), v.x(b[0]), v.y(b[1])
        horiz = abs(x2 - x1) > abs(y2 - y1)
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        if k == 'R' and bo == 0:
            if horiz:
                out.append(txt(mx, my + 17, f'{ref} {val} (flat)', 8, '#000', weight='bold'))
            else:
                out.append(txt(mx - 15, my, f'{ref} {val} (flat)', 8, '#000', weight='bold', rotate=-90))
        elif k == 'R':
            bx, by = (x1, y1) if bo == 1 else (x2, y2)
            if ref in ('R3', 'R4'):
                out.append(txt(bx - 17, by, ref, 9, '#000', 'end', 'bold'))
                out.append(txt(bx - 17, by + 11, val, 8, '#5a3d00', 'end', 'bold'))
            else:
                out.append(txt(bx + 17, by - 5, ref, 9, '#000', 'start', 'bold'))
                out.append(txt(bx + 17, by + 6, val, 8, '#5a3d00', 'start', 'bold'))
        elif horiz:
            out.append(txt(mx, my + 17, f'{ref} {val}', 8, '#000', weight='bold'))
        else:
            sx = mx - 14 if ref == 'C4' else mx + 14
            out.append(txt(sx, my - 6, ref, 9, '#000', 'end' if ref == 'C4' else 'start', 'bold'))
            out.append(txt(sx, my + 6, val, 8, '#5a3d00', 'end' if ref == 'C4' else 'start', 'bold'))

    # legend
    ly = TOP + BOARD_H + BOT + 10
    out.append(txt(MX, ly, 'Notes', 14, '#111', 'start', 'bold', halo=False))
    notes = [
        'Off this board: HW-039 drivers, buck-boost, battery, motors. The INA219 plugs in at column 1 and hangs off the left edge.',
        'INA219: header soldered on the COMPONENT side, module plugged in upside down. VIN-/VIN+ (battery voltage) holes stay empty of traces.',
        'J1: 5.08 mm 2-pin screw terminal, wire entry facing the left edge. All other J are 2.54 mm male pin headers.',
        'R1-R4 stand upright (body over the beige circle). R5, R6, R7 lie flat (10.16 mm) with their bodies over bare traces.',
        'JP1-JP3: straight insulated 22 AWG jumpers on this side. D1 stands over B-8, stripe end up, lead bent into C-8.',
        'J12 takes a standard servo plug: signal (B-4), 5V (C-4), GND (D-4). C2/C3 1 nF, C1/C4/C5 100 nF.',
        'Pin rings: black = GND, orange = 3V3, red = 5V after D1 (VBUS), pink = 5V from the buck before D1.',
        'Keep rows A-B in columns 9-15 clear on the top side: the USB plug comes in there.',
    ]
    for i, n in enumerate(notes):
        out.append(txt(MX, ly + 24 + i * 20, '• ' + n, 12, '#333', 'start', halo=False))
    out.append('</svg>')
    return '\n'.join(out)


def pin_desc(l):
    if not l:
        return ''
    ref, lab, net = l
    if not lab:
        return ref
    if ref == 'U3':
        n, f = lab.split()
        return f'U3 pin {n} {f}'
    if ref == 'U1':
        return f'RP2040 {lab}'
    return f'{ref} {lab}'


def draw_bottom(lead, bare_net):
    v = V(True)
    H = int(TOP + BOARD_H + BOT + 120)
    out = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}'>",
           f"<rect width='{W}' height='{H}' fill='#ffffff'/>"]
    out.append(txt(MX, 30, 'Ropo2040 logic board: SOLDER SIDE (board flipped over left to right)', 20,
                   '#111', 'start', 'bold', halo=False))
    out.append(txt(MX, 56, 'Column 1 is now on the RIGHT. Rows are unchanged (A at the top). '
                   'This is exactly what you see when you turn the board over to solder.', 12,
                   '#a33', 'start', halo=False))
    out.append(txt(MX, 76, 'Silver = bare tinned wire (22 AWG, insulation stripped), laid along the '
                   'holes and soldered at every hole it crosses. Nothing crosses on this side.', 12, '#444',
                   'start', halo=False))
    board_and_grid(v, out)

    for cp in C:
        if cp['body'] and cp['kind'] in ('mcu', 'dip', 'screw', 'ina'):
            x, y, w, h = v.rect(*cp['body'])
            out.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='5' "
                       f"fill='none' stroke='#6b5a3a' stroke-width='1.2' stroke-dasharray='5 4'/>")
    gl = {'U1': (12, 2.15, 'U1 RP2040-Zero (above)'), 'U3': (13, 15.5, 'U3 74HC14 (above)'),
          'J1': (0.6, 5.5, 'J1 (above)'), 'U2': (-3.0, 10.5, 'U2 INA219 (above)')}
    for ref, (c, r, s) in gl.items():
        out.append(txt(v.x(c), v.y(r), s, 10, '#6b5a3a', italic=True))

    for net, a, b in BARE:
        x1, y1, x2, y2 = v.x(a[0]), v.y(a[1]), v.x(b[0]), v.y(b[1])
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='#7d8288' "
                   f"stroke-width='{S*0.34}' stroke-linecap='round'/>")
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='#d9dde2' "
                   f"stroke-width='{S*0.24}' stroke-linecap='round'/>")
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{NETC[net]}' "
                   f"stroke-width='{S*0.07}' stroke-linecap='round'/>")
    for (c, r), (ref, lab, net) in lead.items():
        out.append(f"<circle cx='{v.x(c):.1f}' cy='{v.y(r):.1f}' r='{S*0.2}' fill='#c4c8cd' "
                   f"stroke='#5f6368' stroke-width='1.2'/>")
    for i, (net, a, b) in enumerate(JUMPERS, 1):
        x1, y1, x2, y2 = v.x(a[0]), v.y(a[1]), v.x(b[0]), v.y(b[1])
        out.append(f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{NETC[net]}' "
                   f"stroke-width='3' stroke-dasharray='6 4' stroke-opacity='0.8'/>")
        for h in (a, b):
            out.append(f"<circle cx='{v.x(h[0])}' cy='{v.y(h[1])}' r='{S*0.2}' fill='#c4c8cd' "
                       f"stroke='{NETC[net]}' stroke-width='3'/>")
        tx, ty = (x1 + x2) / 2, (y1 + y2) / 2
        out.append(txt(tx + (0 if abs(x2 - x1) > abs(y2 - y1) else 18), ty + (14 if abs(x2 - x1) > abs(y2 - y1) else 0),
                       f'JP{i}', 8, '#000', weight='bold'))
    short = {}
    for cp in C:
        for c, r, lab, net in cp['pins']:
            if cp['kind'] in ('R', 'C', 'CE', 'D', 'DS'):
                short[(c, r)] = cp['ref']
            elif cp['kind'] == 'dip':
                short[(c, r)] = 'p' + lab.split()[0]
            elif cp['kind'] == 'mcu':
                short[(c, r)] = lab
            else:
                short[(c, r)] = f"{cp['ref']}.{lab}" if lab else cp['ref']
    for (c, r), s in short.items():
        out.append(txt(v.x(c), v.y(r) - S * 0.36, s, 7, '#000', weight='bold'))

    ly = TOP + BOARD_H + BOT + 6
    out.append(txt(MX, ly, f'{len(BARE)} bare links on this side. Dashed = the {len(JUMPERS)} insulated jumpers '
                   'on the component side (solder their ends here). Tick everything off in wiring_list.md.',
                   13, '#111', 'start', 'bold', halo=False))
    out.append(txt(MX, ly + 24, 'Centre stripe colour = net (same colours as the pin rings on the top view).',
                   12, '#444', 'start', halo=False))
    out.append('</svg>')
    return '\n'.join(out)


NET_ORDER = ['GND', '3V3', 'VBUS', '5V_IN', 'SDA', 'SCL', 'A0', 'A1', 'A2', 'A3',
             'DRV_L_RPWM', 'DRV_L_LPWM', 'DRV_L_EN', 'DRV_R_RPWM', 'DRV_R_LPWM', 'DRV_R_EN',
             'ENC_L', 'ENC_R', 'ENC_L_RAW', 'ENC_R_RAW', 'LED_L', 'LED_R',
             'WOW_L', 'WOW_R', 'IBUS_RX', 'IBUS_DIV', 'IBUS', 'SPARE']


def wiring_list(lead):
    def at(h):
        d = pin_desc(lead.get(h))
        return f'{hn(*h)}' + (f' ({d})' if d else '')

    L = ['# Ropo2040 logic board: wiring checklist', '',
         'Generated by `make_layout.py`; do not edit by hand. Hole names are row letter + column '
         'number. Everything below is on the solder side, as bare 22 AWG wire (insulation '
         'stripped) soldered at every hole it passes, plus a few straight insulated jumpers on the '
         'component side.', '']
    L.append('## Insulated jumpers (component side)'), L.append('')
    for i, (net, a, b) in enumerate(JUMPERS, 1):
        L.append(f'- [ ] **JP{i}** `{net}`: {at(a)} → {at(b)}  ({len(seg_holes(a, b))} holes, '
                 f'lying flat over the bare traces in between)')
    L.append('')
    L.append('## Bare links, by net (solder side)'), L.append('')
    for net in NET_ORDER:
        segs = [(a, b) for n, a, b in BARE if n == net]
        if not segs:
            continue
        d = NET_DESC.get(net)
        L.append(f'### {net}' + (f' ({d})' if d else ''))
        for a, b in segs:
            span = len(seg_holes(a, b))
            L.append(f'- [ ] {at(a)} → {at(b)}  ({span} holes)')
        L.append('')
    L.append('## Parts and the holes they go in'), L.append('')
    L.append('| Ref | Part | Holes |'), L.append('|---|---|---|')
    for cp in C:
        holes = ', '.join(f'{hn(c, r)}' + (f' {lab}' if lab else '') for c, r, lab, net in cp['pins'])
        name = cp['title'] + (f' ({cp["note"]})' if cp['note'] else '')
        L.append(f'| {cp["ref"]} | {name} | {holes} |')
    L.append('')
    return '\n'.join(L)


def main():
    errors, close, lead, bare_net = check()
    if errors:
        print('CONNECTIVITY ERRORS:')
        for e in errors:
            print('  -', e)
    else:
        print('Connectivity check: OK (every net is one piece, no shorts, no NC pin connected)')
    print(f'Adjacent pads on different nets (solder carefully): {close}')
    print(f'Bare links: {len(BARE)}   insulated jumpers: {len(JUMPERS)}')
    with open(os.path.join(OUT, 'layout_top.svg'), 'w', encoding='utf-8') as f:
        f.write(draw_top(lead))
    with open(os.path.join(OUT, 'layout_bottom.svg'), 'w', encoding='utf-8') as f:
        f.write(draw_bottom(lead, bare_net))
    with open(os.path.join(OUT, 'wiring_list.md'), 'w', encoding='utf-8') as f:
        f.write(wiring_list(lead))
    print('Wrote layout_top.svg, layout_bottom.svg and wiring_list.md')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
