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

# ---------- Helpers ----------
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
        self.batch_size = int(self.cfg.get("batch_size", 10))    # 10 cho mỗi lần mark
        self.target_n = int(self.cfg.get("target_default", 100)) # tổng số vật cần mark
        self.pulse_ms = int(self.cfg.get("pulse_ms", 150))       # độ rộng xung cho tín hiệu điều khiển

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
        self.total_done = 0  # tổng đã nhặt (đếm qua xung Object_signal)
        self.batch_count = 0 # số đã nhặt trong batch hiện tại (<=10)
        self.target_set = False

        # Edge detection
        self.obj_last = 0
        self.obj_last_edge_ms = 0
        self.hb_ms = 0

        # pulse bookkeeping
        self._pulse_until = {}  # name -> ms

        self.shutdown = False

    # --- low-level I/O ---
    def _in(self, name) -> int:
        spec = self.in_map[name]
        s,ch = _stack_ch(spec, self.stack)
        return self.drv.read_in(s,ch)

    def _out(self, name, val:bool):
        if name in self.out_map:
            self.rel.set(self.out_map[name], val)

    def _pulse(self, name, dur_ms=None):
        """Đưa xung HIGH trong dur_ms (mặc định dùng pulse_ms) rồi auto-hạ."""
        if name not in self.out_map:
            return
        d = int(self.pulse_ms if dur_ms is None else dur_ms)
        self._out(name, True)
        self._pulse_until[name] = now_ms() + d

    def _service_pulses(self):
        t = now_ms()
        to_low = [k for k,until in self._pulse_until.items() if t >= until]
        for k in to_low:
            self._out(k, False)
            del self._pulse_until[k]

    def safe_outputs(self):
        for k in self.out_map.keys():
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
        """True khi có cạnh lên hợp lệ từ Object_signal (mỗi vật = 1 xung)."""
        t = now_ms()
        if val==1 and self.obj_last==0 and (t - self.obj_last_edge_ms) >= self.debounce_obj:
            self.obj_last_edge_ms = t
            self.obj_last = val
            return True
        self.obj_last = val
        return False

    def wait_input(self, name, expect=1, timeout_s=10.0)->bool:
        """Poll input trong timeout (s)."""
        end = time.time() + timeout_s
        while time.time() < end:
            try:
                if self._in(name) == expect:
                    return True
            except Exception:
                pass
            time.sleep(0.02)
        return False

    # --- command handling ---
    def handle_cmd(self, cmd):
        if not cmd:
            return
        if cmd.get("reset"):
            self.mode = "IDLE"
            self.total_done = 0
            self.batch_count = 0
            self.target_set = False
            self.safe_outputs()
        if cmd.get("calib"):
            if "r_calib_req" in self.out_map:
                self._pulse("r_calib_req", self.pulse_ms)
            self.mode = "IDLE"
        if cmd.get("auto"):
            t = int(cmd.get("target",0) or 0)
            if t > 0:
                self.target_n = t
                self.total_done = 0
                self.batch_count = 0
                self.target_set = True
                self.mode = "STARTING"

    # --- main loop ---
    def loop(self):
        self.safe_outputs()
        self.save_runtime()
        print("[PiPLC] started. Mode=IDLE")

        while not self.shutdown:
            # Heartbeat (optional)
            if "pi_alive" in self.out_map and now_ms() - self.hb_ms > 500:
                self.hb_ms = now_ms()
                spec = self.out_map["pi_alive"]
                s,ch = _stack_ch(spec, self.stack)
                key = (s,ch)
                cur = self.rel.state.get(key, False)
                self.rel.set(spec, not cur)

            # maintain pulse outputs
            self._service_pulses()

            # get HMI command
            cmd = read_cmd()
            self.handle_cmd(cmd)

            # read inputs (try; continue on failure)
            try:
                btn_start = self._in("btn_start")           # nút vật lý
                obj_sig   = self._in("object_signal")       # xung mỗi vật
                ready_sig = self._in("ready_signal")        # robot báo sẵn sàng lần gắp kế tiếp
                next_sig  = self._in("next_signal")         # robot báo chu trình mark+pickout xong
                err_sig   = self._in("r_error") if "r_error" in self.in_map else 0
            except Exception as e:
                self.save_runtime(err=f"IO read error: {e}")
                time.sleep(self.loop_ms/1000.0)
                continue

            # Robot error -> ERROR
            if err_sig == 1:
                self.safe_outputs()
                self.mode = "ERROR"
                self.save_runtime(err="Robot error")
                time.sleep(self.loop_ms/1000.0)
                continue

            # --- STATE MACHINE ---
            if self.mode == "IDLE":
                # chờ HMI auto(target) + BTN_START
                self.batch_count = 0
                if self.target_set and btn_start == 1:
                    self.mode = "STARTING"
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "STARTING":
                # Pulse Start_signal để robot move tới Ready
                if "start_signal" in self.out_map:
                    self._pulse("start_signal")
                # Reset batch cho vòng mới
                self.batch_count = 0
                self.mode = "PICK_IN"
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "PICK_IN":
                # Đếm xung vật
                if self.edge_obj(obj_sig):
                    self.batch_count += 1
                    self.total_done += 1
                    save_count(self.total_done)

                # Kiểm tra đạt target tổng?
                if self.total_done >= self.target_n:
                    # Không cần lôi robot chạy thêm — nếu đang còn đồ trong máy, phần Marking sẽ kết thúc theo batch cũ.
                    # Ở đây vì batch chưa đủ 10 nhưng target tổng đã đạt → không phát Continue nữa.
                    # Nếu batch_count == 0: kết thúc luôn
                    if self.batch_count == 0:
                        if "count_ok" in self.out_map:
                            self._pulse("count_ok", self.pulse_ms)
                        self.mode = "IDLE"
                    else:
                        # Nếu đang có batch dở nhưng đủ target: kết thúc batch này bằng Over (đưa vào Marking luôn)
                        if "over_signal" in self.out_map:
                            self._pulse("over_signal")
                        self.mode = "MARKING"
                    self.save_runtime()
                    time.sleep(self.loop_ms/1000.0)
                    continue

                # Đủ 10 cho một lần marking?
                if self.batch_count >= self.batch_size:
                    # Gọi Marking process bằng Over_signal
                    if "over_signal" in self.out_map:
                        self._pulse("over_signal")
                    self.mode = "MARKING"
                    self.save_runtime()
                    time.sleep(self.loop_ms/1000.0)
                    continue

                # Nếu chưa đủ batch_size và chưa đủ target tổng:
                # Chờ robot báo Ready_signal rồi phát Continue_signal cho lần gắp tiếp (2 vật)
                if ready_sig == 1:
                    if "continue_signal" in self.out_map:
                        self._pulse("continue_signal")
                    # Chờ ready_sig hạ (tránh bắn liên tục)
                    # (Không blocking lâu; chỉ nhả CPU)
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "MARKING":
                # Robot tự chạy: về home, reset drop index, Mark_signal→laser, đợi Done,
                # Pick-out 10 vật rồi về Ready và phát Next_signal.
                # Pi chỉ chờ Next_signal để biết đã xong chu trình.
                self.mode = "AFTER_MARK"
                self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "AFTER_MARK":
                # Đợi robot báo Next_signal (chu trình mark + pickout đã xong)
                if next_sig == 1:
                    # Kết thúc chu trình; quyết định tiếp
                    if self.total_done >= self.target_n:
                        if "count_ok" in self.out_map:
                            self._pulse("count_ok", self.pulse_ms)
                        self.mode = "IDLE"
                        self.batch_count = 0
                    else:
                        # Chu trình tiếp theo: quay lại PICK_IN (batch mới)
                        self.batch_count = 0
                        self.mode = "PICK_IN"
                    self.save_runtime()
                time.sleep(self.loop_ms/1000.0)
                continue

            if self.mode == "ERROR":
                # chờ Reset từ HMI
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
