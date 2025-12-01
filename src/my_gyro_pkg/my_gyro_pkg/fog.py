#!/usr/bin/env python3
import time, math, threading
import serial

# ----------------- Config -----------------
PORT = "/dev/ttyUSB0"
BAUD = 912600
FS_HZ        = 8000.0
DT_FIXED     = 1.0 / FS_HZ

V_PER_COUNT  = 2.5 / (2**23)      # volts per LSB (24-bit signed)
SF_V_PER_DPS = 0.0041             # 4.1 mV / (deg/s) per cal sheet

CALIB_SECS   = 15.0               # one-time still bias (volts)
FC_HZ        = None               # e.g., 5.0 to enable gentle rate LPF; None = off
ORIENT_SIGN  = -1.0               # +1: current sign; -1: flip so CW -> +angle

SCALE_CORR   = 1.000              # optional fine trim after a 360° test

# ----------------- Helpers -----------------
def sign_extend_24(v):
    if v & 0x800000:
        v -= 0x1000000
    return v

def normalize_pm180(h):
    h = (h + 180.0) % 360.0
    if h < 0.0: h += 360.0
    return h - 180.0

def lpf_alpha(fc_hz, fs_hz):
    w = 2.0 * math.pi * fc_hz
    return w / (w + fs_hz)

# -------------- Parser ---------------------
class Parser:
    def __init__(self):
        self.st = "SYNC"
        self.buf = bytearray()
    @staticmethod
    def sum16(b):
        return (b[1] + b[2] + b[3] + b[4] + b[5]) & 0xFFFF
    def feed(self, byte_in):
        out = None
        if self.st == "SYNC":
            if byte_in == 0xDD:
                self.buf = bytearray([0xDD])
                self.st  = "BODY"
            return None
        if byte_in == 0xDD and len(self.buf) != 0:
            self.buf = bytearray([0xDD])
            self.st  = "BODY"
            return None
        self.buf.append(byte_in)
        if len(self.buf) == 8:
            b = self.buf; self.st = "SYNC"
            if ((b[6] << 8) | b[7]) == self.sum16(b):
                out = bytes(b)
            if b[7] == 0xDD:
                self.buf = bytearray([0xDD]); self.st = "BODY"
            else:
                self.buf.clear()
        return out

def decode(pkt):
    L, H, M = pkt[1], pkt[2], pkt[3]
    raw24 = (H << 16) | (M << 8) | L
    raw   = sign_extend_24(raw24)
    ctr   = pkt[4] & 0x0F
    xbyte = pkt[5]
    return raw, ctr, xbyte

# -------- Aux (Temp/Vsup from counter cadence) --------
class Aux:
    def __init__(self):
        self.temp_c = float("nan"); self.v_sup = float("nan")
        self._tH = None; self._vH = None
    def update(self, ctr, xbyte):
        c = ctr & 0x0F
        if c == 0x00: self._tH = xbyte
        elif c == 0x01 and self._tH is not None:
            HL = (self._tH << 8) | xbyte
            self.temp_c = (HL * (250.0/32768.0)) - 50.0
            self._tH = None
        elif c == 0x02: self._vH = xbyte
        elif c == 0x03 and self._vH is not None:
            HL = (self._vH << 8) | xbyte
            self.v_sup = HL * (10.0/32768.0)
            self._vH = None

# -------------- Shared state ----------------
class State:
    def __init__(self):
        self.bias_v = 0.0
        self.cal_started = False
        self.cal_done = False
        self.cal_start_t = 0.0
        self.sum_v = 0.0
        self.n_v = 0

        self.heading = 0.0
        self.prev_ctr = None
        self.rate_f = 0.0
        self.prev_rate_f = 0.0

        self.alpha = lpf_alpha(FC_HZ, FS_HZ) if FC_HZ else None

        self.pkts_recv = 0
        self.pkts_drop = 0
        self.max_gap   = 1
        self.dc_hist   = [0]*17

        self.temp_c = float("nan")
        self.v_sup  = float("nan")

        self.lock = threading.Lock()

# -------------- Reader Thread ----------------
def reader_thread(port, baud, st: State):
    ser = serial.Serial(port, baud, timeout=0.02, exclusive=True)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    p = Parser()
    aux = Aux()

    try:
        while True:
            chunk = ser.read(32768)
            if not chunk:
                continue
            for byte in chunk:
                pkt = p.feed(byte)
                if pkt is None:
                    continue

                raw, ctr, xb = decode(pkt)
                volts = raw * V_PER_COUNT

                with st.lock:
                    st.pkts_recv += 1
                    aux.update(ctr, xb)
                    st.temp_c, st.v_sup = aux.temp_c, aux.v_sup

                    if not st.cal_started:
                        st.cal_started = True
                        st.cal_start_t = time.monotonic()

                    if not st.cal_done:
                        st.sum_v += volts; st.n_v += 1
                        if (time.monotonic() - st.cal_start_t) >= CALIB_SECS and st.n_v > 0:
                            st.bias_v = st.sum_v / st.n_v
                            st.cal_done = True
                            st.heading = 0.0
                            st.prev_ctr = None
                            st.rate_f = 0.0
                            st.prev_rate_f = 0.0
                        continue

                    rate = (volts - st.bias_v) / SF_V_PER_DPS
                    rate = ORIENT_SIGN * SCALE_CORR * rate

                    if st.alpha is not None:
                        a = st.alpha
                        st.rate_f = a * rate + (1.0 - a) * st.rate_f
                    else:
                        st.rate_f = rate

                    if st.prev_ctr is None:
                        st.prev_ctr = ctr
                        st.prev_rate_f = st.rate_f
                        continue

                    dc = (ctr - st.prev_ctr) & 0x0F
                    if dc == 0: dc = 16
                    st.prev_ctr = ctr
                    st.dc_hist[min(max(dc,1),16)] += 1
                    if dc > st.max_gap: st.max_gap = dc
                    if dc > 1: st.pkts_drop += (dc - 1)

                    if dc > 1:
                        st.heading += (dc - 1) * st.prev_rate_f * DT_FIXED
                        st.heading += 0.5 * (st.prev_rate_f + st.rate_f) * DT_FIXED
                    else:
                        st.heading += 0.5 * (st.prev_rate_f + st.rate_f) * DT_FIXED

                    st.prev_rate_f = st.rate_f
                    st.heading = normalize_pm180(st.heading)
    finally:
        try: ser.close()
        except: pass
