from flask import Flask, request, redirect, jsonify, render_template_string
import json, os, time
from pathlib import Path

APP = Flask(__name__)

# Thư mục chung với piplc.py
BASE = Path.home() / "~/piplc_template/piplc"
BASE.mkdir(parents=True, exist_ok=True)

CMD_FILE   = BASE / "hmi_cmd.json"
STATE_FILE = BASE / "runtime_state.json"

TEMPLATE = """
<!doctype html>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pi HMI</title>
<style>
  body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:18px;max-width:720px}
  .row{display:flex;gap:8px;align-items:center;margin:8px 0}
  .card{border:1px solid #ddd;border-radius:10px;padding:12px;margin:10px 0}
  button{padding:8px 14px;border-radius:8px;border:1px solid #999;background:#f5f5f5;cursor:pointer}
  input[type=number]{padding:6px 8px;border-radius:6px;border:1px solid #aaa;width:120px}
  .stat{font-size:15px;line-height:1.4}
  .big{font-size:28px;font-weight:700}
  .ok{color:#0a7}
  .warn{color:#d70}
</style>

<h2>Laser Marking HMI</h2>

<div class="card">
  <div class="row">
    <form method="post">
      <label>Target:&nbsp;</label>
      <input type="number" name="target" min="1" step="1" required>
      <button name="action" value="auto">Start AUTO</button>
      <button name="action" value="reset" style="margin-left:8px">Reset</button>
      <button name="action" value="calib" style="margin-left:8px">Calib</button>
    </form>
  </div>
</div>

<div class="card">
  <div class="stat">Trạng thái: <span id="mode" class="big">—</span></div>
  <div class="stat">Đã gắp: <span id="total" class="big">0</span></div>
  <div class="stat">Trong batch hiện tại: <span id="batch" class="big">0</span> / <span id="batchsize">10</span></div>
  <div class="stat">Target tổng: <span id="target" class="big">0</span></div>
  <div class="stat" id="error" class="warn"></div>
</div>

<script>
async function pull(){
  try{
    const r = await fetch('/state');
    const js = await r.json();
    document.getElementById('mode').textContent   = js.mode ?? '—';
    document.getElementById('total').textContent  = js.total_done ?? 0;
    document.getElementById('batch').textContent  = js.batch_count ?? 0;
    document.getElementById('target').textContent = js.target_n ?? 0;
    document.getElementById('batchsize').textContent = js.batch_size ?? 10;

    const errEl = document.getElementById('error');
    if(js.error){ errEl.textContent = 'Error: ' + js.error; errEl.className='stat warn'; }
    else { errEl.textContent = ''; errEl.className='stat'; }
  }catch(e){
    console.log('pull state failed', e);
  }finally{
    setTimeout(pull, 1000); // refresh mỗi 1s
  }
}
pull();
</script>
"""

def write_cmd(d):
    d["ts"] = time.time()
    with open(CMD_FILE, "w") as f:
        json.dump(d, f)

@APP.route("/", methods=["GET","POST"])
def home():
    if request.method == "POST":
        act = request.form.get("action","")
        cmd = {"auto":False, "target":0, "reset":False, "calib":False}
        if act=="auto":
            try:
                cmd["target"] = int(request.form.get("target","0"))
            except:
                cmd["target"] = 0
            cmd["auto"] = True
        elif act=="reset":
            cmd["reset"] = True
        elif act=="calib":
            cmd["calib"] = True
        write_cmd(cmd)
        return redirect("/")
    return render_template_string(TEMPLATE)

@APP.route("/state")
def state():
    # Trả JSON cho JS đọc định kỳ
    data = {
        "mode":"IDLE","total_done":0,"batch_count":0,"target_n":0,
        "error":"","ts":time.time(),"batch_size":10
    }
    try:
        if STATE_FILE.exists():
            data.update(json.loads(STATE_FILE.read_text()))
    except Exception as e:
        data["error"] = f"state read error: {e}"
    # Gửi kèm batch_size để hiển thị (đọc từ config nếu cần)
    try:
        import yaml
        cfg = yaml.safe_load((BASE/"config.yaml").read_text())
        if "batch_size" in cfg: data["batch_size"] = cfg["batch_size"]
    except Exception:
        pass
    return jsonify(data)

if __name__ == "__main__":
    # Chạy dev server (debug), production dùng gunicorn
    APP.run(host="0.0.0.0", port=5000)
