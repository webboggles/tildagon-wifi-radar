"""
mesh.py — ESP-NOW mesh neighbour discovery + pairwise RSSI exchange.

Each badge periodically broadcasts a packet containing the RSSI it
observes for every direct neighbour it knows about. Receivers combine
their own direct-link RSSI (from espnow peers_table) with the second-hop
RSSI reported inside every packet, yielding a partial graph of
edge-weighted distances good enough for a 2D relational layout.

Packet layout (little-endian):
    magic 4 bytes  b'MESH'
    version u8     0x01
    count   u8     n of neighbour entries (0..16)
    name    6s     ascii, zero-padded (informational)
    entries [count] of:
        mac 6s
        rssi i8    (RSSI in dBm, signed)
"""

import struct

try:
    from time import ticks_ms, ticks_diff
except ImportError:
    from time import time as _t
    def ticks_ms():
        return int(_t() * 1000)
    def ticks_diff(a, b):
        return a - b

try:
    import espnow
    import network
    _HAS_NET = True
    _IMP_ERR = None
except Exception as _e:
    _HAS_NET = False
    _IMP_ERR = repr(_e)

BCAST          = b'\xff\xff\xff\xff\xff\xff'
MAGIC          = b'MESH'
VERSION        = 0x01
HDR_FMT        = '<4sBB6s'
HDR_LEN        = struct.calcsize(HDR_FMT)
ENT_FMT        = '<6sb'
ENT_LEN        = struct.calcsize(ENT_FMT)

PEER_TIMEOUT   = 8000
EDGE_TIMEOUT   = 10000
MAX_NEIGHBOURS = 16


class MeshManager:
    """ESP-NOW neighbour discovery and RSSI graph keeper."""

    def __init__(self, name="badge"):
        self.name = (name.encode()[:6] + b'\x00' * 6)[:6]
        self._e = None
        self._err = None
        self.rx_count = 0
        self.tx_count = 0
        self.self_mac = b'\x00' * 6

        # direct[mac] = {'rssi':int, 'name':str, 'time':ms}
        self.direct = {}
        # edges[(a,b)] where a<b (bytes cmp) = {'rssi':int, 'time':ms}
        self.edges = {}

        if not _HAS_NET:
            self._err = "no espnow: " + str(_IMP_ERR)
            return

        try:
            sta = network.WLAN(network.STA_IF)
            sta.active(True)
            try:
                sta.disconnect()
            except Exception:
                pass
            try:
                sta.config(channel=1)
            except Exception:
                pass
            try:
                self.self_mac = bytes(sta.config('mac'))
            except Exception:
                try:
                    self.self_mac = bytes(sta.config(mac=None))
                except Exception:
                    pass
            self._e = espnow.ESPNow()
            self._e.active(True)
            try:
                self._e.add_peer(BCAST)
            except Exception:
                pass
        except Exception as e:
            self._e = None
            self._err = repr(e)

    # ── Broadcast ─────────────────────────────────────────────────

    def broadcast(self):
        """Send our direct neighbour list as an ESP-NOW broadcast."""
        if not self._e:
            return False

        items = sorted(
            self.direct.items(),
            key=lambda kv: kv[1]['rssi'],
            reverse=True,
        )[:MAX_NEIGHBOURS]

        count = len(items)
        hdr = struct.pack(HDR_FMT, MAGIC, VERSION, count, self.name)
        body = bytearray()
        for mac, info in items:
            rssi = max(-128, min(0, int(info['rssi'])))
            body += struct.pack(ENT_FMT, mac, rssi)

        try:
            self._e.send(BCAST, hdr + bytes(body), False)
            self.tx_count += 1
            return True
        except Exception:
            return False

    # ── Receive ───────────────────────────────────────────────────

    def receive(self):
        """Drain the RX queue; update direct + edges."""
        if not self._e:
            return
        now = ticks_ms()

        # Pull RSSI for the last-seen peer from ESPNow peers_table if avail.
        try:
            ptable = dict(self._e.peers_table)
        except Exception:
            ptable = {}

        for _ in range(12):
            try:
                if not self._e.any():
                    break
                mac, data = self._e.irecv(0)
            except Exception:
                break
            if mac is None or data is None:
                continue
            if len(data) < HDR_LEN or data[:4] != MAGIC:
                continue

            mac = bytes(mac)
            if mac == BCAST or mac == self.self_mac:
                continue

            self.rx_count += 1

            try:
                _, ver, count, nm = struct.unpack(HDR_FMT, data[:HDR_LEN])
            except Exception:
                continue
            if ver != VERSION:
                continue

            try:
                name = nm.rstrip(b'\x00').decode('utf-8', 'ignore')
            except Exception:
                name = ''

            rssi_direct = -90
            pe = ptable.get(mac)
            if pe:
                try:
                    rssi_direct = int(pe[0])
                except Exception:
                    pass

            prev = self.direct.get(mac)
            if prev:
                rssi_direct = int(prev['rssi'] * 0.5 + rssi_direct * 0.5)
            self.direct[mac] = {
                'rssi': rssi_direct,
                'name': name,
                'time': now,
            }
            self._set_edge(self.self_mac, mac, rssi_direct, now)

            off = HDR_LEN
            need = count * ENT_LEN
            if len(data) < off + need:
                continue
            for i in range(count):
                try:
                    pmac, prssi = struct.unpack(
                        ENT_FMT, data[off:off + ENT_LEN])
                except Exception:
                    break
                off += ENT_LEN
                pmac = bytes(pmac)
                if pmac == self.self_mac or pmac == BCAST or pmac == mac:
                    continue
                self._set_edge(mac, pmac, int(prssi), now)

        self._expire(now)

    # ── Graph helpers ────────────────────────────────────────────

    @staticmethod
    def _key(a, b):
        return (a, b) if a < b else (b, a)

    def _set_edge(self, a, b, rssi, now):
        if a == b:
            return
        k = self._key(bytes(a), bytes(b))
        prev = self.edges.get(k)
        if prev:
            rssi = int(prev['rssi'] * 0.4 + rssi * 0.6)
        self.edges[k] = {'rssi': rssi, 'time': now}

    def _expire(self, now):
        gone = [m for m, v in self.direct.items()
                if ticks_diff(now, v['time']) > PEER_TIMEOUT]
        for m in gone:
            del self.direct[m]
        egone = [k for k, v in self.edges.items()
                 if ticks_diff(now, v['time']) > EDGE_TIMEOUT]
        for k in egone:
            del self.edges[k]

    # ── Inspection ───────────────────────────────────────────────

    def known_nodes(self):
        """All MACs we have heard of (direct or via second-hop)."""
        s = set()
        for k in self.edges.keys():
            s.add(k[0])
            s.add(k[1])
        s.discard(self.self_mac)
        return s

    def iter_edges(self):
        """Yield (mac_a, mac_b, rssi)."""
        for (a, b), v in self.edges.items():
            yield a, b, v['rssi']

    def close(self):
        if self._e:
            try:
                self._e.active(False)
            except Exception:
                pass
