"""
wifi_radar — Single-badge directional WiFi radar for Tildagon.

Rotate the badge physically to sweep. The IMU tracks heading via
integrated gyro Z-axis. Each WiFi scan captures APs with RSSI;
the current heading is stamped onto each result. APs accumulate
on a polar radar display as persistent blips that fade over time.

Controls:
  A (UP)   — cycle colour theme
  D (DOWN) — reset heading to 0
  F (CANCEL) — exit
"""

import app
import asyncio
import math
import imu
from events.input import Buttons, BUTTON_TYPES

# ── Tuning ─────────────────────────────────────────────────────────────────────
SCAN_PERIOD   = 1.0
TX_POWER      = -40
PATH_LOSS_N   = 2.8
MAX_RANGE_M   = 30.0
BLIP_LIFE     = 15.0
RADAR_R       = 95
TWO_PI        = 2 * math.pi
DEG2RAD       = 0.01745329
GYRO_DEAD     = 3.0
TURN_INVERT   = -1
LOG_MAX       = math.log(1 + MAX_RANGE_M)
DRAW_OFFSET   = -math.pi * 0.5

THEMES = [
    {"bg": (0.0, 0.02, 0.0), "grid": (0.0, 0.15, 0.0),
     "blip": (0.0, 1.0, 0.3), "txt": (0.0, 0.8, 0.2), "dim": (0.0, 0.3, 0.1)},
    {"bg": (0.0, 0.0, 0.03), "grid": (0.0, 0.08, 0.2),
     "blip": (0.3, 0.7, 1.0), "txt": (0.2, 0.6, 1.0), "dim": (0.1, 0.2, 0.4)},
    {"bg": (0.03, 0.01, 0.0), "grid": (0.15, 0.08, 0.0),
     "blip": (1.0, 0.6, 0.1), "txt": (0.9, 0.5, 0.1), "dim": (0.3, 0.15, 0.05)},
]


def _rssi_to_dist(rssi):
    if rssi >= 0:
        return 0.5
    return min(MAX_RANGE_M, 10.0 ** ((TX_POWER - rssi) / (10.0 * PATH_LOSS_N)))


def _dist_to_radar(dist):
    return min(RADAR_R, (math.log(1 + dist) / LOG_MAX) * RADAR_R)


def _lerp_angle(a, b, t):
    """Shortest-path lerp between two angles in radians."""
    diff = (b - a + math.pi) % TWO_PI - math.pi
    return (a + diff * t) % TWO_PI


class WifiRadarApp(app.App):

    def __init__(self):
        self.button_states = Buttons(self)
        self._heading   = 0.0
        self._blips     = []
        self._pending   = []
        self._dev_count = 0
        self._scan_age  = 0.0
        self._theme_idx = 0
        self._net_ok    = False
        self._credits   = False
        self._tilt_x    = 0.0
        self._tilt_y    = 0.0

        self._init_network()

    def _init_network(self):
        try:
            import network
            self._wlan = network.WLAN(network.STA_IF)
            self._wlan.active(True)
            self._net_ok = True
            asyncio.create_task(self._scan_loop())
        except Exception:
            self._wlan = None
            self._net_ok = False

    async def _scan_loop(self):
        while True:
            await asyncio.sleep(0.1)
            heading_before = self._heading
            try:
                results = self._wlan.scan()
            except Exception:
                results = []
            heading_after = self._heading
            heading = (heading_before + heading_after) * 0.5

            self._dev_count = len(results)
            self._scan_age = 0.0

            batch = []
            for e in results:
                bssid = bytes(e[1][:6])
                rssi  = e[3]
                dist  = _rssi_to_dist(rssi)
                r     = _dist_to_radar(dist)
                batch.append((bssid, rssi, r, heading))
            self._pending = batch

    def _ingest_one(self):
        """Merge one pending scan result into blips. Called each frame."""
        if not self._pending:
            return
        bssid, rssi, r, heading = self._pending.pop(0)
        for ob in self._blips:
            if ob["mac"] == bssid:
                ob["r"] = ob["r"] * 0.4 + r * 0.6
                ob["ang"] = _lerp_angle(ob["ang"], heading, 0.6)
                ob["rssi"] = int(ob["rssi"] * 0.3 + rssi * 0.7)
                ob["life"] = BLIP_LIFE
                return
        self._blips.append({
            "mac": bssid, "rssi": rssi,
            "r": r, "ang": heading,
            "life": BLIP_LIFE,
        })
        if len(self._blips) > 64:
            self._blips.pop(0)

    # ── App lifecycle ──────────────────────────────────────────────────────

    def update(self, delta):
        if self._credits:
            self._up_credits(delta)
            return

        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            self.minimise()
            return

        if self.button_states.get(BUTTON_TYPES["UP"]):
            self.button_states.clear()
            self._theme_idx = (self._theme_idx + 1) % len(THEMES)

        if self.button_states.get(BUTTON_TYPES["DOWN"]):
            self.button_states.clear()
            self._credits = True
            return

        try:
            gyro = imu.gyro_read()
            gz = gyro[2] * TURN_INVERT
            if abs(gz) > GYRO_DEAD:
                dt = delta * 0.001
                gz -= GYRO_DEAD if gz > 0 else -GYRO_DEAD
                self._heading += gz * DEG2RAD * dt
                self._heading = self._heading % TWO_PI
        except Exception:
            pass

        self._ingest_one()

        dt_s = delta * 0.001
        self._scan_age += dt_s
        expired = []
        for i, b in enumerate(self._blips):
            b["life"] -= dt_s
            if b["life"] <= 0:
                expired.append(i)
        for i in reversed(expired):
            self._blips.pop(i)

    def _up_credits(self, delta):
        try:
            acc = imu.acc_read()
            self._tilt_x = acc[0] * 0.5
            self._tilt_y = acc[1] * 0.5
        except Exception:
            pass
        for btn in BUTTON_TYPES.values():
            if self.button_states.get(btn):
                self.button_states.clear()
                self._credits = False
                return

    # ── Drawing ────────────────────────────────────────────────────────────

    def draw(self, ctx):
        if self._credits:
            self._draw_credits(ctx)
            return
        T = THEMES[self._theme_idx]
        ctx.save()

        ctx.rgb(*T["bg"]).rectangle(-120, -120, 240, 240).fill()

        self._draw_grid(ctx, T)
        self._draw_blips(ctx, T)
        self._draw_heading_tick(ctx, T)
        self._draw_hud(ctx, T)

        ctx.restore()

    def _draw_grid(self, ctx, T):
        ctx.line_width = 0.5
        gr, gg, gb = T["grid"]
        for ring in (1, 2, 3, 4):
            frac = ring / 4.0
            radius = int(RADAR_R * frac)
            ctx.rgb(gr, gg, gb)
            ctx.begin_path()
            ctx.arc(0, 0, radius, 0, TWO_PI, False)
            ctx.stroke()

        for i in range(12):
            a = i * TWO_PI / 12
            ctx.rgb(gr * 0.6, gg * 0.6, gb * 0.6)
            ctx.begin_path()
            ctx.move_to(0, 0)
            ctx.line_to(math.cos(a) * RADAR_R, math.sin(a) * RADAR_R)
            ctx.stroke()

    def _draw_blips(self, ctx, T):
        br, bg, bb = T["blip"]
        for b in self._blips:
            alpha = min(1.0, b["life"] / (BLIP_LIFE * 0.5))
            rel_ang = b["ang"] - self._heading + DRAW_OFFSET
            px = math.cos(rel_ang) * b["r"]
            py = math.sin(rel_ang) * b["r"]

            size = 2 + int(3 * alpha)
            ctx.rgb(br * alpha, bg * alpha, bb * alpha)
            ctx.begin_path()
            ctx.arc(px, py, size, 0, TWO_PI, False)
            ctx.fill()

            if alpha > 0.6:
                ctx.rgb(br, bg, bb)
                ctx.begin_path()
                ctx.arc(px, py, size + 2, 0, TWO_PI, False)
                ctx.stroke()

    def _draw_heading_tick(self, ctx, T):
        ctx.rgb(*T["txt"])
        ctx.line_width = 2
        ctx.begin_path()
        ctx.move_to(0, -(RADAR_R + 2))
        ctx.line_to(-4, -(RADAR_R + 8))
        ctx.line_to(4, -(RADAR_R + 8))
        ctx.fill()

    def _draw_hud(self, ctx, T):
        tr, tg, tb = T["txt"]
        dr, dg, db = T["dim"]

        deg = int(math.degrees(self._heading)) % 360
        ctx.font_size = 11
        t = f"{deg:03d}"
        ctx.rgb(tr, tg, tb)
        ctx.move_to(-ctx.text_width(t) / 2, -(RADAR_R + 10))
        ctx.text(t)

        ctx.font_size = 9
        t = f"{self._dev_count} AP"
        ctx.rgb(dr, dg, db)
        ctx.move_to(-ctx.text_width(t) / 2, RADAR_R + 14)
        ctx.text(t)

        if not self._net_ok:
            ctx.font_size = 9
            ctx.rgb(1.0, 0.3, 0.2)
            t = "NO NET"
            ctx.move_to(-ctx.text_width(t) / 2, RADAR_R + 6)
            ctx.text(t)

        age_norm = min(1.0, self._scan_age / SCAN_PERIOD)
        bar_w = int(24 * (1.0 - age_norm))
        ctx.rgb(dr, dg, db).rectangle(-12, RADAR_R + 2, 24, 3).fill()
        ctx.rgb(tr, tg, tb).rectangle(-12, RADAR_R + 2, bar_w, 3).fill()

        ctx.font_size = 7
        ctx.rgb(dr, dg, db)
        ctx.move_to(-3, -(RADAR_R + 18))
        ctx.text("A")
        ctx.move_to(-3, RADAR_R + 24)
        ctx.text("D")

    def _draw_credits(self, ctx):
        ctx.save()
        ctx.rgb(0.01, 0.02, 0.04).rectangle(-120, -120, 240, 240).fill()
        tx = self._tilt_x
        ty = self._tilt_y

        ctx.font_size = 18
        t = "WiFi Radar"
        ctx.rgb(0.0, 0.83, 1.0)
        ctx.move_to(-ctx.text_width(t) / 2 + tx * 1.2, -50 + ty * 1.2)
        ctx.text(t)

        ctx.font_size = 11
        lines = [
            ("@webboggles", 0.0, 0.83, 1.0),
            ("weborder.uk", 0.0, 0.65, 0.8),
            ("", 0, 0, 0),
            ("IMU Directional Radar", 0.5, 0.4, 0.2),
            ("WiFi AP Scanner", 0.5, 0.4, 0.2),
        ]
        y = -15
        for txt, r, g, b in lines:
            if not txt:
                y += 8
                continue
            w = ctx.text_width(txt)
            ctx.rgb(r, g, b).move_to(-w / 2 + tx * 0.5, y + ty * 0.5)
            ctx.text(txt)
            y += 15

        ctx.font_size = 9
        t = "ANY BTN: BACK"
        ctx.rgb(0.3, 0.3, 0.3)
        ctx.move_to(-ctx.text_width(t) / 2, 80)
        ctx.text(t)
        ctx.restore()


__app_export__ = WifiRadarApp
