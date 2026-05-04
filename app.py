"""
wifi_radar — Tildagon directional WiFi radar + ESP-NOW mesh map.

Two modes, toggled with C (CONFIRM):

  AP MODE    Directional WiFi AP scanner. Rotate the badge to sweep;
             RSSI -> distance, heading -> bearing. APs accumulate as
             fading blips.
  MESH MODE  ESP-NOW mesh of other badges running this app. Each node
             broadcasts its direct-neighbour RSSI list. A small force
             solver relaxes known pairwise distances into a consistent
             2D relational map with self pinned at centre.

Controls:
  A (UP)     cycle colour theme
  D (DOWN)   show credits
  C (CONFIRM) toggle AP / MESH mode
  F (CANCEL) exit
"""

import app
import asyncio
import math
import imu
from events.input import Buttons, BUTTON_TYPES

try:
    import apps.tildagon_wifi_radar.mesh as mesh
except ImportError:
    import mesh

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
ASSET_PATH    = "/apps/tildagon_wifi_radar/"
LOGO_W        = 140
LOGO_H        = 80

MESH_BCAST_MS   = 500
MESH_RELAX_ITER = 4
MESH_SPRING_K   = 0.8
MESH_CENTER_K   = 0.02
MESH_DAMP       = 0.80
MESH_MAX_NODES  = 24

MODE_AP   = 0
MODE_MESH = 1

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


def _mac_short(m):
    """Display label for a 6-byte MAC: last 2 bytes hex."""
    if not m or len(m) < 6:
        return "----"
    return "{:02X}{:02X}".format(m[4], m[5])


def _fmt_metres(d):
    """Human-readable distance label: cm under 1 m, metres above."""
    if d < 1.0:
        return "{:d}cm".format(int(d * 100))
    if d < 10.0:
        return "{:.1f}m".format(d)
    return "{:d}m".format(int(d))


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

        self._mode          = MODE_AP
        self._mesh          = mesh.MeshManager(name="radar")
        self._mesh_bcast_ms = 0
        # mesh_nodes[mac] = {x, y, vx, vy}
        self._mesh_nodes = {}

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
            if self._mode != MODE_AP:
                await asyncio.sleep(0.3)
                continue
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

    # ── Mesh tick ──────────────────────────────────────────────────────────

    def _update_mesh(self, delta):
        self._mesh.receive()
        self._mesh_bcast_ms += delta
        if self._mesh_bcast_ms >= MESH_BCAST_MS:
            self._mesh.broadcast()
            self._mesh_bcast_ms = 0
        self._relax_mesh(delta)

    def _relax_mesh(self, delta):
        """Force-directed layout. Self pinned at (0,0); nodes drift toward
        target RSSI-derived edge distances."""
        known = self._mesh.known_nodes()
        self_mac = self._mesh.self_mac

        # Garbage-collect disappeared nodes.
        for m in list(self._mesh_nodes.keys()):
            if m not in known:
                del self._mesh_nodes[m]

        # Seed new nodes at a small random-ish offset derived from the MAC,
        # so every ego gets a deterministic but different starting pose.
        for m in known:
            if m not in self._mesh_nodes:
                seed = (m[4] << 8) | m[5]
                ang = (seed / 65535.0) * TWO_PI
                r0 = 20.0 + (m[3] % 16)
                self._mesh_nodes[m] = {
                    'x': math.cos(ang) * r0,
                    'y': math.sin(ang) * r0,
                    'vx': 0.0, 'vy': 0.0,
                }
            if len(self._mesh_nodes) >= MESH_MAX_NODES:
                break

        # Build a quick lookup for target distances (in metres).
        targets = []
        for a, b, rssi in self._mesh.iter_edges():
            if a not in known and a != self_mac:
                continue
            if b not in known and b != self_mac:
                continue
            targets.append((a, b, _rssi_to_dist(rssi)))

        dt = min(0.05, max(0.001, delta * 0.001))

        for _ in range(MESH_RELAX_ITER):
            # Accumulate per-node forces.
            fx = {m: 0.0 for m in self._mesh_nodes}
            fy = {m: 0.0 for m in self._mesh_nodes}

            for a, b, d_t in targets:
                ax = 0.0 if a == self_mac else self._mesh_nodes[a]['x']
                ay = 0.0 if a == self_mac else self._mesh_nodes[a]['y']
                bx = 0.0 if b == self_mac else self._mesh_nodes[b]['x']
                by = 0.0 if b == self_mac else self._mesh_nodes[b]['y']
                dx = bx - ax
                dy = by - ay
                d = math.sqrt(dx * dx + dy * dy) + 1e-3
                err = (d - d_t)
                # Unit vector scaled to spring force.
                ux = dx / d
                uy = dy / d
                f  = MESH_SPRING_K * err
                if a != self_mac:
                    fx[a] +=  ux * f
                    fy[a] +=  uy * f
                if b != self_mac:
                    fx[b] += -ux * f
                    fy[b] += -uy * f

            # Weak centring pull prevents drift, keeps layout on screen.
            for m, n in self._mesh_nodes.items():
                fx[m] += -MESH_CENTER_K * n['x']
                fy[m] += -MESH_CENTER_K * n['y']

            for m, n in self._mesh_nodes.items():
                n['vx'] = (n['vx'] + fx[m] * dt) * MESH_DAMP
                n['vy'] = (n['vy'] + fy[m] * dt) * MESH_DAMP
                n['x'] += n['vx'] * dt
                n['y'] += n['vy'] * dt

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

        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            self._mode = MODE_MESH if self._mode == MODE_AP else MODE_AP

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

        if self._mode == MODE_AP:
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
        else:
            self._update_mesh(delta)

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
        if self._mode == MODE_AP:
            self._draw_blips(ctx, T)
        else:
            self._draw_mesh(ctx, T)
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
        tr, tg, tb = T["txt"]
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

            if alpha > 0.35:
                dist = _rssi_to_dist(b["rssi"])
                label = _fmt_metres(dist)
                ctx.font_size = 9
                ctx.rgb(tr * alpha, tg * alpha, tb * alpha)
                ctx.move_to(px + size + 2, py + 3).text(label)

    def _draw_mesh(self, ctx, T):
        """Render relational mesh map. Model coords are metres; we log-scale
        them into the radar ring. The display rotates with heading so the
        user can physically point at a peer."""
        br, bg, bb = T["blip"]
        dr, dg, db = T["dim"]
        self_mac = self._mesh.self_mac

        cosh = math.cos(-self._heading + DRAW_OFFSET)
        sinh = math.sin(-self._heading + DRAW_OFFSET)

        def project(mx, my):
            d = math.sqrt(mx * mx + my * my)
            if d < 1e-3:
                return 0.0, 0.0
            r = _dist_to_radar(d)
            ux = mx / d
            uy = my / d
            rx = ux * cosh - uy * sinh
            ry = ux * sinh + uy * cosh
            return rx * r, ry * r

        # Edges first, so nodes render on top.
        ctx.line_width = 1
        for a, b, _rssi in self._mesh.iter_edges():
            if a == self_mac:
                ax, ay = 0.0, 0.0
            elif a in self._mesh_nodes:
                n = self._mesh_nodes[a]
                ax, ay = project(n['x'], n['y'])
            else:
                continue
            if b == self_mac:
                bx, by = 0.0, 0.0
            elif b in self._mesh_nodes:
                n = self._mesh_nodes[b]
                bx, by = project(n['x'], n['y'])
            else:
                continue
            ctx.rgb(dr, dg, db)
            ctx.begin_path()
            ctx.move_to(ax, ay)
            ctx.line_to(bx, by)
            ctx.stroke()

        # Self dot.
        ctx.rgb(br, bg, bb)
        ctx.begin_path()
        ctx.arc(0, 0, 4, 0, TWO_PI, False)
        ctx.fill()

        # Peer blips + labels.
        ctx.font_size = 10
        for m, n in self._mesh_nodes.items():
            px, py = project(n['x'], n['y'])
            direct = self._mesh.direct.get(m)
            strong = direct is not None
            size = 4 if strong else 3
            r, g, b = (br, bg, bb) if strong else (dr, dg, db)
            ctx.rgb(r, g, b)
            ctx.begin_path()
            ctx.arc(px, py, size, 0, TWO_PI, False)
            ctx.fill()
            if strong:
                ctx.rgb(br, bg, bb)
                ctx.begin_path()
                ctx.arc(px, py, size + 3, 0, TWO_PI, False)
                ctx.stroke()

            label = _mac_short(m)
            ctx.rgb(*T["txt"])
            ctx.move_to(px + 5, py - 4).text(label)

            if strong and direct is not None:
                dist = _rssi_to_dist(direct['rssi'])
            else:
                dist = math.sqrt(n['x'] * n['x'] + n['y'] * n['y'])
            dlabel = _fmt_metres(dist)
            ctx.font_size = 9
            ctx.rgb(dr * 1.8, dg * 1.8, db * 1.8)
            ctx.move_to(px + 5, py + 6).text(dlabel)

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
        ctx.font_size = 13
        t = f"{deg:03d}"
        ctx.rgb(tr, tg, tb)
        ctx.move_to(-ctx.text_width(t) / 2, -(RADAR_R + 10))
        ctx.text(t)

        ctx.font_size = 11
        if self._mode == MODE_AP:
            t = f"{self._dev_count} AP"
        else:
            t = f"{len(self._mesh.direct)}/{len(self._mesh_nodes)} mesh"
        ctx.rgb(dr, dg, db)
        ctx.move_to(-ctx.text_width(t) / 2, RADAR_R + 14)
        ctx.text(t)

        if self._mode == MODE_AP and not self._net_ok:
            ctx.font_size = 11
            ctx.rgb(1.0, 0.3, 0.2)
            t = "NO NET"
            ctx.move_to(-ctx.text_width(t) / 2, RADAR_R + 6)
            ctx.text(t)

        if self._mode == MODE_MESH and self._mesh._e is None:
            ctx.font_size = 11
            ctx.rgb(1.0, 0.3, 0.2)
            t = "NO ESPNOW"
            ctx.move_to(-ctx.text_width(t) / 2, RADAR_R + 6)
            ctx.text(t)

        if self._mode == MODE_AP:
            age_norm = min(1.0, self._scan_age / SCAN_PERIOD)
            bar_w = int(24 * (1.0 - age_norm))
            ctx.rgb(dr, dg, db).rectangle(-12, RADAR_R + 2, 24, 3).fill()
            ctx.rgb(tr, tg, tb).rectangle(-12, RADAR_R + 2, bar_w, 3).fill()
        else:
            t = f"tx{self._mesh.tx_count} rx{self._mesh.rx_count}"
            ctx.font_size = 9
            ctx.rgb(dr, dg, db)
            ctx.move_to(-ctx.text_width(t) / 2, RADAR_R + 24)
            ctx.text(t)

        ctx.font_size = 9
        ctx.rgb(dr, dg, db)
        ctx.move_to(-3, -(RADAR_R + 18))
        ctx.text("A")
        mode_lbl = "AP" if self._mode == MODE_AP else "MESH"
        ctx.move_to(-ctx.text_width(mode_lbl) / 2, -(RADAR_R + 28))
        ctx.rgb(tr, tg, tb)
        ctx.text(mode_lbl)

    def _draw_credits(self, ctx):
        ctx.save()
        ctx.rgb(0.01, 0.02, 0.04).rectangle(-120, -120, 240, 240).fill()
        tx = self._tilt_x
        ty = self._tilt_y

        lx = -LOGO_W / 2 + tx * 1.0
        ly = -LOGO_H / 2 - 20 + ty * 1.0
        ctx.image(ASSET_PATH + "logo.png", lx, ly, LOGO_W, LOGO_H)

        ctx.font_size = 13
        lines = [
            ("@webboggles", 0.0, 0.83, 1.0),
            ("weborder.uk", 0.0, 0.65, 0.8),
            ("", 0, 0, 0),
            ("IMU Directional Radar", 0.5, 0.4, 0.2),
            ("WiFi AP Scanner", 0.5, 0.4, 0.2),
            ("ESP-NOW Mesh Map", 0.5, 0.4, 0.2),
        ]
        y = 35
        for txt, r, g, b in lines:
            if not txt:
                y += 8
                continue
            w = ctx.text_width(txt)
            ctx.rgb(r, g, b).move_to(-w / 2 + tx * 0.5, y + ty * 0.5)
            ctx.text(txt)
            y += 15

        ctx.font_size = 11
        t = "ANY BTN: BACK"
        ctx.rgb(0.3, 0.3, 0.3)
        ctx.move_to(-ctx.text_width(t) / 2, 95)
        ctx.text(t)
        ctx.restore()


__app_export__ = WifiRadarApp
