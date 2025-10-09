#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
piplc.py — Raspberry Pi mini-PLC for ABB IRC5 (IRB120)
Flow (theo yêu cầu mới):
  STARTING:  Nhận BTN_START + target (HMI). Pi pulse Start_signal cho robot về Ready.
  PICK_IN:   Vòng lặp: chờ Ready_signal từ robot → nếu chưa đủ 10 (hoặc target tổng),
             Pi pulse Continue_signal để robot gắp 2 vật; Pi đếm xung Object_signal
             (mỗi vật rời magazine = 1 xung). Đủ 10 (hoặc đủ target tổng) → Pi pulse Over_signal.
  MARKING:   Robot tự mark (Mark_signal→laser; đợi Done…) rồi Pick-out 10 vật, xong báo Next_signal.
  AFTER_MARK: Pi nhận Next_signal → nếu chưa đủ target tổng, quay lại PICK_IN; đủ thì count_ok và IDLE.

HMI (hmi.py) gửi lệnh:
  - {"auto": true, "target": N}  : set target_n và vào STARTING
  - {"reset": true}               : về IDLE, xóa đếm
  - {"calib": true}               : xung r_calib_req (nếu dùng), rồi về IDLE

I/O mapping: xem cuối file (gợi ý cho config.yaml).

Yêu cầu:
  - Đã cài driver HAT 4rel4in (Sequent)
  - PyYAML
"""

import os
import time
import json
import signal
import subprocess
import shutil
from pathlib import Path
import yaml

# ---------- tiện ích log ----------
def log(msg: str):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

# ---------- Paths ----------
BASE = Path(__file__).resolve().parent.parent  # ~/piplc
CFG_FILE = BASE / "config.yaml"
STATE_FILE = BASE / "runtime_state.json"
CMD_FILE = BASE / "hmi_cmd.json"
COUNT_FILE = BASE / "pick_counter.json"

# ---------- Robust wrapper for Sequent 4rel4in CLI ----------
class FourRel4In:
    """Hỗ trợ các biến thể CLI: relwr/relrd, rel/r, in/din/readin/inrd."""
    def __init__(self, use_sudo=False, cmd_path=None):
        self.cmd = cmd_path or shutil.which("4rel4in") or "/usr/local/bin/4rel4in"
        self.use_sudo = bool(use_sudo)
        self._variants_read_in = [
            ["in", "{stack}", "{ch}"],
            ["din", "{stack}", "{ch}"],
            ["readin", "{stack}", "{ch}"],
            ["inrd", "{stack}", "{ch}"],
        ]
        self._variants_set_relay = [
            ["relwr", "{stack}", "{ch}", "{state}"],
            ["rel",   "{stack}", "{ch}", "{state}"],
            ["r",     "{stack}", "{ch}", "{state}"],
            ["write", "{stack}", "{ch}", "{state}"],
        ]

    def _run(self, args_fmt, **kw) -> str:
        args = [s.format(**kw) for s in args_fmt]
        cmd = [self.cmd] + args
        if self.use_sudo:
            cmd = ["sudo", "-n"] + cmd
        env = os.environ.copy()
        env["PATH"] = env.get("PATH", "") + ":/usr/local/bin"
        return subprocess.check_output(cmd, text=True, env=env).strip()

    def _try(self, variants, **kw):
        last = None
        for fmt in variants:
            try:
                return self._run(fmt, **kw)
            except Exception as e:
                last = e
        raise RuntimeError(f"4rel4in CLI not responding. Last: {last}")

    def read_in(self, stack:int, ch:int) -> int:
        out = self._try(self._variants_read_in, stack=str(stack), ch=str(ch)).strip()
        if out.endswith(("0","1")):
            return int(out[-1])
        return int(out)

    def set_relay(self, stack:int, ch:int, on:bool):
        state = "on" if on else "off"
        _ = self._try(self._variants_set_relay, stack=str(stack), ch=str(ch), state=state)

# ---------- helpers ----------
def _stack_ch(spec, default_stack):
    if isinstance(spec, int):
        return default_stack, int(spec)
    if isinstance(spec, dict):
        return int(spec.get("stack", default_stack)), int(spec["ch"])
    raise ValueError(f"Bad channel spec: {spec}")

class OutputsLatch:
    def __init__(self, driver: FourRel4In, default_stack:int):
        self.drv = driver
        self.default_stack = default_stack
        self.state = {}  # (stack,ch)->bool

    def set(self, spec, val:bool):
        stack, ch = _stack_ch(spec, self.default_stack)
        key = (stack, ch)
        if self.state.get(key) == bool(val):
            return
        self.drv.set_relay(stack, ch, bool(val))
        self.state[key] = bool(val)

def load_cfg():
    return yaml.safe_load(CFG_FILE.read_text())

def save_state(d: dict):
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(d, indent=2))
    os.replace(tmp, STATE_FILE)

def load_count() -> int:
    try:
        return int(json.loads(COUNT_FILE.read_text()).get("count", 0))
    except Exception:
        return 0

def save_count(c: int):
    tmp = COUNT_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps({"count": c, "ts": time.time()}))
    os.replace(tmp, COUNT_FILE)

def read_cmd():
    try:
        d = json.loads(CMD_FILE.read_text())
        try: os.remove(CMD_FILE)
        except: pass
        return d
    except Exception:
        return None

def now_ms() -> int:
    return int(time.time()*1000)

# ---------- Main controller ----------
class PiPLC:
    def __init__(self):
        self.cfg = load_cfg()

        # Timings & params
        self.loop_ms = int(self.cfg.get("loop_ms", 20))
        self.debounce_obj = int(self.cfg.get("debounce_obj_ms", 80))
        self.post_batch_timeout = int(self.cfg.get("post_batch_timeout_s", 120))
        self.obj_timeout = int(self.cfg.get("obj_timeout_s", 20))
        self.batch_size = int(self.cfg.get("batch_size", 10))
        self.target_n = int(self.cfg.get("target_default", 100))
        self.pulse_ms = int(self.cfg.get("pulse_ms", 150))

        # I/O map
        self.stack = int(self.cfg.get("stack_level", 0))
        self.in_map = self.cfg["inputs"]
        self.out_map = self.cfg["outputs"]

        # Driver
        use_sudo = (os.getenv("PIPLC_USE_SUDO","0") == "1")
        self.drv = FourRel4In(use_sudo=use_sudo)
        self.rel = OutputsLatch(self.drv, self.stack)

        # Runtime
        self.mode = "IDLE"   # IDLE / STARTING / PICK_IN / MARKING / AFTER_MARK / ERROR
        self.total_done = 0
        self.batch_count = 0
        self.target_set = False

        # Edge & pulse
        self.obj_last = 0
        self.obj_last_edge_ms = 0
        self._pulse_until = {}

        # logging trackers
        self.last_mode = self.mode
        self.last_in = {}
        self.last_out = {}

        self.hb_ms = 0
        self.shutdown = False

    # --- log state change ---
    def _set_mode(self, new_mode: str, reason: str = ""):
        if new_mode != self.mode:
            log(f"STATE: {self.mode} → {new_mode}" + (f" | {reason}" if reason else ""))
            self.last_mode = self.mode
            self.mode = new_mode

    # --- low-level I/O ---
    def _in(self, name) -> int:
        spec = self.in_map[name]
        s,ch = _stack_ch(spec, self.stack)
        val = self.drv.read_in(s,ch)
        # log on change
        old = self.last_in.get(name, None)
        if old is None or old != val:
            log(f"IN  {name} = {val}")
            self.last_in[name] = val
        return val

    def _out(self, name, val:bool):
        if name in self.out_map:
            prev = self.last_out.get(name, None)
            if prev is None or prev != bool(val):
                log(f"OUT {name} = {'ON' if val else 'OFF'}")
                self.last_out[name] = bool(val)
            self.rel.set(self.out_map[name], val)

    def _pulse(self, name, dur_ms=None):
        if name not in self.out_map:
            return
        d = int(self.pulse_ms if dur_ms is None else dur_ms)
        log(f"PULSE {name} ({d} ms)")
        self._out(name, True)
        self._pulse_until[name] = now_ms() + d

    def _service_pulses(self):
        t = now_ms()
        to_low = [k for k,until in self._pulse_until.items() if t >= until]
        for k in to_low:
            self._out(k, False)
            del self._pulse_until[k]

    def safe_outputs(self):
        for k in list(self.out_map.keys()):
            self._out(k, False)
        self._pulse_until.clear()

    # --- helpers ---
    def save_runtime(self, err:str=""):
        save_state({
            "mode": self.mode,
            "total_done": self.total_done,
            "batch_count": self.batch_count,
            "target_n": self.target_n,
            "error": err,
            "ts": time.time()
        })

    def edge_obj(self, val:int)->bool:
        t = now_ms()
        if val==1 and self.obj_last==0 and (t - self.obj_last_edge_ms) >= self.debounce_obj:
            self.obj_last_edge_ms = t
            self.obj_last = val
            return True
        self.obj_last = val
        return False

    def handle_cmd(self, cmd):
        if not cmd:
            return
        if cmd.get("reset"):
            log("CMD reset")
            self._set_mode("IDLE", "reset")
            self.total_done = 0
            self.batch_count = 0
            self.target_set = False
            self.safe_outputs()
        if cmd.get("calib"):
            log("CMD calib")
            if "r_calib_req" in self.out_map:
                self._pulse("r_calib_req", self.pulse_ms)
            self._set_mode("IDLE", "calib")
        if cmd.get("auto"):
            t = int(cmd.get("target",0) or 0)
            log(f"CMD auto target={t}")
            if t > 0:
                self.target_n = t
                self.total_done = 0
                self.batch_count = 0
                self.target_set = True
                self._set_mode("STARTING", "auto command")

    # --- main loop ---
    def loop(self):
        self.safe_outputs()
        self.save_runtime()
        log("PiPLC started. Mode=IDLE")

        while not self.shutdown:
            # heartbeat (optional)
            if "pi_alive" in self.out_map and now_ms() - self.hb_ms > 500:
                self.hb_ms = now_ms()
                spec = self.out_map["pi_alive"]
                s,ch = _stack_ch(spec, self.stack)
                key = (s,ch)
                cur = self.rel.state.get(key, False)
                self.rel.set(spec, not cur)

            # maintain pulses
            self._service_pulses()

            # read HMI command
            cmd = read_cmd()
            self.handle_cmd(cmd)

            # read inputs (log done inside _in)
            try:
                btn_start = self._in("btn_start")
                ready_sig = self._in("ready_signal")
                next_sig  = self._in("next_signal")
                obj_sig   = self._in("object_signal")
                # nếu bạn có r_error trong inputs, thêm vào config.yaml rồi uncomment:
                # err_sig   = self._in("r_error")
                err_sig   = 0
            except Exception as e:
                self.save_runtime(err=f"IO read error: {e}")
                time.sleep(self.loop_ms/1000.0)
                continue

            # robot error -> ERROR
            if err_sig == 1:
                self.safe_outputs()
                self._set_mode("ERROR", "robot error input=1")
                self.save_runtime(err="Robot error")
                time.sleep(self.loop_ms/1000.0)
                continue

            # ---- STATE MACHINE ----
            if self.mode == "IDLE":
                # chờ HMI auto(target) + BTN_START
                self.batch_count = 0
                if self.target_set and btn_start == 1:
                    self._set_mode("STARTING", "target set + BTN_START=1")
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "STARTING":
                # Pulse Start_signal để robot move tới Ready
                if "start_signal" in self.out_map:
                    self._pulse("start_signal")
                self.batch_count = 0
                self._set_mode("PICK_IN", "start_signal pulsed")
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "PICK_IN":
                # Đếm xung vật
                if self.edge_obj(obj_sig):
                    self.batch_count += 1
                    self.total_done += 1
                    save_count(self.total_done)
                    log(f"OBJ_PULSE → batch_count={self.batch_count}, total_done={self.total_done}")

                # Đủ target tổng?
                if self.total_done >= self.target_n:
                    if self.batch_count == 0:
                        if "count_ok" in self.out_map:
                            self._pulse("count_ok", self.pulse_ms)
                        self._set_mode("IDLE", "target total reached (no open batch)")
                    else:
                        if "over_signal" in self.out_map:
                            self._pulse("over_signal")
                        self._set_mode("MARKING", "target total reached; closing current batch")
                    self.save_runtime()
                    time.sleep(self.loop_ms/1000.0)
                    continue

                # Đủ 10 cho một lần marking?
                if self.batch_count >= self.batch_size:
                    if "over_signal" in self.out_map:
                        self._pulse("over_signal")
                    self._set_mode("MARKING", "batch_count reached batch_size")
                    self.save_runtime()
                    time.sleep(self.loop_ms/1000.0)
                    continue

                # Chưa đủ 10 & chưa đủ target -> chờ ready rồi continue
                if ready_sig == 1:
                    if "continue_signal" in self.out_map:
                        self._pulse("continue_signal")
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "MARKING":
                # Robot tự mark + pick-out; Pi chuyển sang AFTER_MARK để chờ next_signal
                self._set_mode("AFTER_MARK", "waiting Next_signal")
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "AFTER_MARK":
                if next_sig == 1:
                    if self.total_done >= self.target_n:
                        if "count_ok" in self.out_map:
                            self._pulse("count_ok", self.pulse_ms)
                        self.batch_count = 0
                        self._set_mode("IDLE", "job completed")
                    else:
                        self.batch_count = 0
                        self._set_mode("PICK_IN", "continue next batch")
                    self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "ERROR":
                self.save_runtime(err="ERROR state")
                time.sleep(self.loop_ms/1000.0)
                continue

# ---------- Entrypoint ----------
def main():
    plc = PiPLC()
    def _sig(*_): plc.shutdown = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    plc.loop()

if __name__ == "__main__":
    main()

