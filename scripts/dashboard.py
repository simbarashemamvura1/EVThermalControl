import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go
import threading
import time
from flask import jsonify
from collections import deque
import socket as sock_module
import json as json_module

# ============================================================
#  EV THERMAL CONTROL SYSTEM — LIVE DASHBOARD
#  Data port 9000: C++ streams JSON every tick
#  Command port 9001: Python sends scenario name on button click
# ============================================================

MAX_POINTS = 100

times        = deque(maxlen=MAX_POINTS)
batt_temps   = deque(maxlen=MAX_POINTS)
fan_speeds   = deque(maxlen=MAX_POINTS)
soc_vals     = deque(maxlen=MAX_POINTS)

state = {
    "bT":       25.0,
    "tgt_bT":   25.0,
    "spd":      0.0,
    "tgt_spd":  0.0,
    "fan":      0.0,
    "soc":      100.0,
    "coolant":  22.0,
    "amb":      20.0,
    "pump":     False,
    "heater":   False,
    "sys":      "NORMAL",
    "dtc":      "",
    "dtcs":     [],       # all active DTCs from C++
    "scenario": "Select a scenario to begin",
    "tick":     0,
    "fault":    False,
    "cpp_connected": False,
}
lock = threading.Lock()
fault_log = []

# ============================================================
#  SEND SCENARIO COMMAND TO C++ ENGINE (port 9001)
# ============================================================
def send_scenario_command(name):
    try:
        s = sock_module.socket(sock_module.AF_INET, sock_module.SOCK_STREAM)
        s.settimeout(2)
        s.connect(('127.0.0.1', 9001))
        s.send((name + "\n").encode('utf-8'))
        s.close()
        print(f"[CMD] Sent scenario '{name}' to C++ engine")
    except Exception as e:
        print(f"[CMD] Could not send to C++ engine: {e}")
        # Fall back to Python scenario
        with lock:
            if not state["cpp_connected"]:
                run_python_scenario(name)

# ============================================================
#  SOCKET READER — reads data from C++ on port 9000
# ============================================================
def socket_reader():
    while True:
        try:
            s = sock_module.socket(sock_module.AF_INET, sock_module.SOCK_STREAM)
            s.settimeout(3)
            s.connect(('127.0.0.1', 9000))
            s.settimeout(None)
            print("[SOCKET] Connected to C++ simulation engine")
            with lock:
                state["cpp_connected"] = True
                state["scenario"] = "C++ engine connected"

            buf = ""
            while True:
                data = s.recv(4096).decode('utf-8', errors='ignore')
                if not data:
                    break
                buf += data
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        d = json_module.loads(line)
                        with lock:
                            state["spd"]     = float(d.get("spd",     0))
                            state["bT"]      = float(d.get("bT",      25))
                            state["tgt_bT"]  = float(d.get("bT",      25))
                            state["tgt_spd"] = float(d.get("spd",     0))
                            state["fan"]     = float(d.get("fan",     0))
                            state["soc"]     = float(d.get("soc",     100))
                            state["coolant"] = float(d.get("coolant", 22))
                            state["pump"]    = bool(d.get("pump",     False))
                            state["heater"]  = bool(d.get("heater",   False))
                            state["sys"]     = str(d.get("sys",       "NORMAL"))
                            state["fault"]   = bool(d.get("fault",    False))
                            state["scenario"]= str(d.get("scenario",  ""))

                            # All active DTCs as list
                            dtcs = d.get("dtcs", [])
                            state["dtcs"] = dtcs

                            # Primary DTC for alert box
                            dtc = str(d.get("dtc", ""))
                            state["dtc"] = dtc

                            # Log each unique DTC
                            for dtc_item in dtcs:
                                if not fault_log or fault_log[-1]["msg"] != dtc_item:
                                    fault_log.append({
                                        "t":   int(d.get("tick", state["tick"])),
                                        "msg": dtc_item
                                    })

                            state["tick"] += 1
                            times.append(state["tick"])
                            batt_temps.append(round(state["bT"], 1))
                            fan_speeds.append(round(state["fan"], 1))
                            soc_vals.append(round(state["soc"], 1))
                    except Exception:
                        pass

        except Exception as e:
            with lock:
                state["cpp_connected"] = False
            print(f"[SOCKET] C++ not available ({e}) — using Python fallback")
            time.sleep(2)

threading.Thread(target=socket_reader, daemon=True).start()

# ============================================================
#  PYTHON FALLBACK PHYSICS
# ============================================================
def lerp(a, b, t):
    return a + (b - a) * t

def physics_loop():
    last_time = time.time()
    while True:
        now = time.time()
        dt  = min(now - last_time, 0.05)
        last_time = now

        with lock:
            if state["cpp_connected"]:
                pass
            else:
                spd_diff = state["tgt_spd"] - state["spd"]
                abs_gap  = abs(spd_diff)
                ease     = min(1.0, abs_gap / 80.0)
                rate     = (0.0022*(0.3+ease*0.7) if spd_diff>0
                            else 0.006*(0.5+ease*0.5))
                state["spd"] = lerp(state["spd"], state["tgt_spd"],
                                    min(1.0, rate*dt*60))
                if abs(state["spd"]) < 0.3 and state["tgt_spd"] == 0:
                    state["spd"] = 0.0

                if not state["fault"]:
                    state["bT"] = lerp(state["bT"], state["tgt_bT"],
                                       min(1.0, 0.012*dt*60))
                    state["bT"] = max(-30.0, min(90.0, state["bT"]))

                bT  = state["bT"]
                spd = state["spd"]

                sys = "NORMAL"
                if state["fault"]:   sys = "FAULT"
                elif bT > 80:        sys = "SAFE SHUTDOWN"
                elif bT > 70:        sys = "FAULT"
                elif bT > 45:        sys = "COOLING"
                elif bT < 10:        sys = "HEATING"
                state["sys"] = sys

                target_fan = 0.0
                if sys == "COOLING":
                    target_fan = min(100.0, (bT-45)*5 + spd*0.08)
                elif sys == "FAULT":
                    target_fan = 100.0
                fan_diff = target_fan - state["fan"]
                fan_rate = 0.06 if fan_diff > 0 else 0.035
                state["fan"] = lerp(state["fan"], target_fan,
                                    min(1.0, fan_rate*dt*60))

                state["heater"] = bT < 10
                state["pump"]   = bT > 40

                base_drain   = 0.0008
                heater_drain = 0.003 if state["heater"] else 0.0
                drive_drain  = spd * 0.000055
                state["soc"] = max(5.0, state["soc"] -
                                   (base_drain+heater_drain+drive_drain)*dt*60)
                state["coolant"] = lerp(state["coolant"],
                    state["amb"]+(bT-state["amb"])*0.4, min(1.0,0.006*dt*60))

                dtc = ""
                if state["fault"]:  dtc = "DTC 0x0005 — Sensor dropout"
                elif bT > 75:       dtc = "DTC 0x0002 — Critical overtemp"
                elif bT > 60:       dtc = "DTC 0x0001 — Overtemp warning"
                elif bT < 5:        dtc = "DTC 0x0003 — Undertemp warning"
                state["dtc"]  = dtc
                state["dtcs"] = [dtc] if dtc else []

                if dtc and (not fault_log or fault_log[-1]["msg"] != dtc):
                    fault_log.append({"t": state["tick"], "msg": dtc})

                state["tick"] += 1
                times.append(state["tick"])
                batt_temps.append(round(bT, 1))
                fan_speeds.append(round(state["fan"], 1))
                soc_vals.append(round(state["soc"], 1))

        time.sleep(0.18)

threading.Thread(target=physics_loop, daemon=True).start()

# ============================================================
#  PYTHON FALLBACK SCENARIO ENGINE
# ============================================================
scenarios = {
    "cold": [
        {"dur":5, "spd":0,   "bT":-10,    "label":"Cold Start — Battery at -10°C"},
        {"dur":6, "spd":40,  "bT":-4,     "label":"Cold Start — Heater active, warming"},
        {"dur":6, "spd":70,  "bT":8,      "label":"Cold Start — Approaching normal range"},
        {"dur":5, "spd":90,  "bT":19,     "label":"Cold Start — Normal temperature reached"},
    ],
    "highway": [
        {"dur":5, "spd":60,  "bT":24,     "label":"Highway — Cruise at 60 kph"},
        {"dur":6, "spd":120, "bT":36,     "label":"Highway — Accelerating to 120 kph"},
        {"dur":6, "spd":180, "bT":47,     "label":"Highway — Cooling system activated"},
        {"dur":4, "spd":180, "bT":64,     "label":"Highway — Overtemp warning"},
        {"dur":4, "spd":180, "bT":74,     "label":"Highway — Fault state, full cooling"},
        {"dur":4, "spd":0,   "bT":82,     "label":"Highway — Safe shutdown triggered"},
    ],
    "fault": [
        {"dur":5, "spd":100, "bT":30,     "label":"Fault Injection — Normal driving"},
        {"dur":5, "spd":100, "bT":"FAULT","label":"Fault Injection — Sensor dropout active"},
        {"dur":5, "spd":100, "bT":33,     "label":"Fault Injection — System recovering"},
        {"dur":4, "spd":80,  "bT":27,     "label":"Fault Injection — Recovery confirmed"},
    ],
    "full": [
        {"dur":5, "spd":0,   "bT":-8,     "label":"Full Run [1/3] — Cold Start: -8°C"},
        {"dur":6, "spd":50,  "bT":2,      "label":"Full Run [1/3] — Heater active"},
        {"dur":5, "spd":90,  "bT":18,     "label":"Full Run [1/3] — Normal reached"},
        {"dur":5, "spd":130, "bT":30,     "label":"Full Run [2/3] — Accelerating"},
        {"dur":5, "spd":180, "bT":46,     "label":"Full Run [2/3] — Cooling activated"},
        {"dur":4, "spd":180, "bT":62,     "label":"Full Run [2/3] — Overtemp warning"},
        {"dur":4, "spd":50,  "bT":38,     "label":"Full Run [2/3] — Decelerating"},
        {"dur":4, "spd":100, "bT":28,     "label":"Full Run [3/3] — Pre-fault baseline"},
        {"dur":5, "spd":100, "bT":"FAULT","label":"Full Run [3/3] — Sensor dropout"},
        {"dur":5, "spd":80,  "bT":27,     "label":"Full Run [3/3] — Full recovery"},
    ],
}

py_timer = None

def run_python_scenario(name):
    global py_timer
    if py_timer: py_timer.cancel()
    steps = scenarios.get(name, [])
    def run_step(idx):
        global py_timer
        if idx >= len(steps):
            with lock: state["scenario"] = "Simulation complete"
            return
        s = steps[idx]
        with lock:
            state["scenario"] = s["label"]
            state["fault"]    = (s["bT"] == "FAULT")
            if not state["fault"]:
                state["tgt_bT"]  = float(s["bT"])
            state["tgt_spd"] = float(s["spd"])
        py_timer = threading.Timer(s["dur"], run_step, args=[idx+1])
        py_timer.start()
    run_step(0)

# ============================================================
#  DASH APP
# ============================================================
app = dash.Dash(__name__, title="EV Thermal Control")

STATE_COLORS = {
    "NORMAL":       {"bg":"#E1F5EE","border":"#1D9E75","text":"#0F6E56"},
    "COOLING":      {"bg":"#FAEEDA","border":"#EF9F27","text":"#854F0B"},
    "HEATING":      {"bg":"#E6F1FB","border":"#378ADD","text":"#185FA5"},
    "FAULT":        {"bg":"#FAECE7","border":"#D85A30","text":"#993C1D"},
    "SAFE SHUTDOWN":{"bg":"#FAECE7","border":"#A32D2D","text":"#6B1515"},
}

def scenario_btn(label, btn_id, accent, bg):
    return html.Button(label, id=btn_id, n_clicks=0, style={
        "flex":"1","padding":"10px 8px","borderRadius":"8px",
        "border":f"1.5px solid {accent}","background":bg,"color":accent,
        "cursor":"pointer","fontWeight":"700","fontSize":"11px",
        "letterSpacing":".04em","textTransform":"uppercase","transition":"opacity .2s",
        "fontFamily":"-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif",
    })

def stat_card(label, el_id):
    return html.Div([
        html.Div(label, style={"fontSize":"10px","color":"#aaa",
            "textTransform":"uppercase","letterSpacing":".05em","marginBottom":"3px"}),
        html.Div("—", id=el_id,
            style={"fontSize":"14px","fontWeight":"700","color":"#1a1a1a"}),
    ], style={"background":"#f5f5f3","borderRadius":"8px","padding":"8px 11px"})

def sensor_dot(dot_id, label, dc="#1D9E75", dg="0 0 4px #1D9E75"):
    return html.Div([
        html.Div(id=dot_id, style={"width":"8px","height":"8px","borderRadius":"50%",
            "background":dc,"boxShadow":dg,"transition":"all .4s","flexShrink":"0"}),
        html.Span(label, style={"fontSize":"10px","fontWeight":"600",
            "color":"#555","letterSpacing":".03em"}),
    ], style={"display":"flex","alignItems":"center","gap":"6px",
              "background":"#f5f5f3","borderRadius":"7px","padding":"5px 10px"})

app.layout = html.Div([

    # Header
    html.Div([
        html.Div([
            html.H1("EV Thermal Control System",
                style={"margin":0,"fontSize":"18px","fontWeight":"700",
                       "color":"#1a1a1a","letterSpacing":"-.01em"}),
            html.P("Software-Defined Vehicle Simulation",
                style={"margin":"2px 0 0","fontSize":"10px","color":"#aaa",
                       "letterSpacing":".04em","textTransform":"uppercase"}),
        ]),
        html.Div([
            html.Div(id="cpp-indicator", style={
                "fontSize":"9px","fontWeight":"700","letterSpacing":".05em",
                "textTransform":"uppercase","marginRight":"12px",
                "padding":"3px 10px","borderRadius":"99px",
                "background":"#f5f5f3","color":"#aaa","border":"1px solid #e8e8e6"
            }),
            html.Span(id="state-badge", children="NORMAL", style={
                "padding":"5px 16px","borderRadius":"99px","fontSize":"11px",
                "fontWeight":"700","background":"#E1F5EE","color":"#0F6E56",
                "border":"1.5px solid #1D9E75","transition":"all .5s","letterSpacing":".05em"
            }),
        ], style={"display":"flex","alignItems":"center"}),
    ], style={"display":"flex","justifyContent":"space-between","alignItems":"center",
              "padding":"14px 20px","background":"#fff",
              "borderBottom":"1px solid #e8e8e6","marginBottom":"12px","borderRadius":"12px"}),

    html.Div([

        # LEFT
        html.Div([
            html.Div(id="scenario-label", style={
                "fontSize":"12px","fontWeight":"600","color":"#534AB7",
                "marginBottom":"8px","minHeight":"18px","letterSpacing":".01em"}),

            html.Div([
                html.Iframe(src="/assets/ev_scene.html",
                    style={"width":"100%","height":"260px","border":"none",
                           "borderRadius":"12px","display":"block"})
            ], style={"borderRadius":"12px","overflow":"hidden",
                      "border":"1px solid #e8e8e6","marginBottom":"10px"}),

            html.Div([
                html.Div(id="scenario-mode-label", style={
                    "fontSize":"9px","fontWeight":"700","color":"#bbb",
                    "textTransform":"uppercase","letterSpacing":".08em","marginBottom":"8px"}),
                html.Div([
                    scenario_btn("Cold Start",     "btn-cold",  "#185FA5","#EBF4FD"),
                    scenario_btn("Highway Heat",   "btn-heat",  "#854F0B","#FDF3E3"),
                    scenario_btn("Fault Injection","btn-fault", "#993C1D","#FDF0ED"),
                    scenario_btn("Full Auto Run",  "btn-full",  "#534AB7","#EEEDFE"),
                ], style={"display":"flex","gap":"8px"}),
            ], style={"background":"#fff","border":"1px solid #e8e8e6",
                      "borderRadius":"12px","padding":"12px 14px","marginBottom":"10px"}),

            html.Div([
                stat_card("Battery Temp","stat-bt"),
                stat_card("Fan Speed",  "stat-fan"),
                stat_card("SoC",        "stat-soc"),
                stat_card("Speed",      "stat-spd"),
                stat_card("Pump",       "stat-pump"),
                stat_card("Heater",     "stat-heat"),
            ], style={"display":"grid","gridTemplateColumns":"1fr 1fr 1fr",
                      "gap":"7px","marginBottom":"10px"}),

            html.Div([
                html.Div("Sensor Status", style={
                    "fontSize":"9px","fontWeight":"700","color":"#bbb",
                    "textTransform":"uppercase","letterSpacing":".08em","marginBottom":"8px"}),
                html.Div([
                    sensor_dot("dot-temp","Temp Sensor"),
                    sensor_dot("dot-soc2","SoC Sensor"),
                    sensor_dot("dot-volt","Voltage"),
                    sensor_dot("dot-can", "CAN Bus"),
                    sensor_dot("dot-pump","Coolant Pump","#888","none"),
                    sensor_dot("dot-fan2","Cooling Fan", "#888","none"),
                ], style={"display":"flex","gap":"6px","flexWrap":"wrap"}),
            ], style={"background":"#fff","border":"1px solid #e8e8e6",
                      "borderRadius":"12px","padding":"12px 14px"}),

            html.Div(id="dtc-alert", style={"marginTop":"8px","minHeight":"32px"}),

        ], style={"flex":"1","minWidth":"0"}),

        # RIGHT
        html.Div([
            html.Div([
                html.Div("Battery Temperature °C", style={
                    "fontSize":"10px","fontWeight":"700","color":"#aaa",
                    "textTransform":"uppercase","letterSpacing":".06em","marginBottom":"8px"}),
                dcc.Graph(id="graph-temp",config={"displayModeBar":False},style={"height":"155px"}),
            ], style={"background":"#fff","border":"1px solid #e8e8e6",
                      "borderRadius":"12px","padding":"12px 14px","marginBottom":"9px"}),

            html.Div([
                html.Div("Fan Speed %", style={
                    "fontSize":"10px","fontWeight":"700","color":"#aaa",
                    "textTransform":"uppercase","letterSpacing":".06em","marginBottom":"8px"}),
                dcc.Graph(id="graph-fan",config={"displayModeBar":False},style={"height":"130px"}),
            ], style={"background":"#fff","border":"1px solid #e8e8e6",
                      "borderRadius":"12px","padding":"12px 14px","marginBottom":"9px"}),

            html.Div([
                html.Div("State of Charge %", style={
                    "fontSize":"10px","fontWeight":"700","color":"#aaa",
                    "textTransform":"uppercase","letterSpacing":".06em","marginBottom":"8px"}),
                dcc.Graph(id="graph-soc",config={"displayModeBar":False},style={"height":"130px"}),
            ], style={"background":"#fff","border":"1px solid #e8e8e6",
                      "borderRadius":"12px","padding":"12px 14px","marginBottom":"9px"}),

            html.Div([
                html.Div("Fault Event Log", style={
                    "fontSize":"10px","fontWeight":"700","color":"#aaa",
                    "textTransform":"uppercase","letterSpacing":".06em","marginBottom":"8px"}),
                html.Div(id="fault-timeline"),
            ], style={"background":"#fff","border":"1px solid #e8e8e6",
                      "borderRadius":"12px","padding":"12px 14px"}),

        ], style={"flex":"1","minWidth":"0"}),

    ], style={"display":"flex","gap":"14px","padding":"0 14px 14px"}),

    dcc.Interval(id="interval", interval=250, n_intervals=0),
    dcc.Store(id="s-cold"), dcc.Store(id="s-heat"),
    dcc.Store(id="s-fault"), dcc.Store(id="s-full"),

], style={"fontFamily":"-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif",
          "background":"#f0f0ee","minHeight":"100vh"})

# ============================================================
#  FLASK LIVE-STATE ENDPOINT
# ============================================================
@app.server.route('/live-state')
def live_state():
    with lock:
        return jsonify({
            "spd":      round(state["spd"], 1),
            "bT":       round(state["bT"], 1),
            "tgt_spd":  round(state["tgt_spd"], 1),
            "tgt_bT":   round(state["tgt_bT"], 1),
            "fan":      round(state["fan"], 1),
            "soc":      round(state["soc"], 1),
            "sys":      state["sys"],
            "fault":    state["fault"],
            "scenario": state["scenario"],
            "pump":     state["pump"],
            "heater":   state["heater"],
            "dtc":      state["dtc"],
        })

# ============================================================
#  SCENARIO CALLBACKS — always try C++ first, fallback to Python
# ============================================================
@app.callback(Output("s-cold","data"), Input("btn-cold","n_clicks"), prevent_initial_call=True)
def run_cold(n):
    threading.Thread(target=send_scenario_command, args=("cold",), daemon=True).start()
    return n

@app.callback(Output("s-heat","data"), Input("btn-heat","n_clicks"), prevent_initial_call=True)
def run_heat(n):
    threading.Thread(target=send_scenario_command, args=("highway",), daemon=True).start()
    return n

@app.callback(Output("s-fault","data"), Input("btn-fault","n_clicks"), prevent_initial_call=True)
def run_fault(n):
    threading.Thread(target=send_scenario_command, args=("fault",), daemon=True).start()
    return n

@app.callback(Output("s-full","data"), Input("btn-full","n_clicks"), prevent_initial_call=True)
def run_full(n):
    threading.Thread(target=send_scenario_command, args=("full",), daemon=True).start()
    return n

# ============================================================
#  MAIN UPDATE CALLBACK
# ============================================================
@app.callback(
    Output("state-badge",       "children"),
    Output("state-badge",       "style"),
    Output("scenario-label",    "children"),
    Output("cpp-indicator",     "children"),
    Output("cpp-indicator",     "style"),
    Output("scenario-mode-label","children"),
    Output("stat-bt",   "children"), Output("stat-fan",  "children"),
    Output("stat-soc",  "children"), Output("stat-spd",  "children"),
    Output("stat-pump", "children"), Output("stat-heat", "children"),
    Output("dtc-alert", "children"),
    Output("graph-temp","figure"),
    Output("graph-fan", "figure"),
    Output("graph-soc", "figure"),
    Output("fault-timeline","children"),
    Output("dot-temp","style"), Output("dot-soc2","style"),
    Output("dot-volt","style"), Output("dot-can", "style"),
    Output("dot-pump","style"), Output("dot-fan2","style"),
    Input("interval","n_intervals"),
)
def update(n):
    with lock:
        d  = dict(state)
        tt = list(times)
        bt = list(batt_temps)
        fs = list(fan_speeds)
        sv = list(soc_vals)
        fl = list(fault_log[-20:])
        active_dtcs = list(d.get("dtcs", []))

    sc = STATE_COLORS.get(d["sys"], STATE_COLORS["NORMAL"])

    badge_style = {
        "padding":"5px 16px","borderRadius":"99px","fontSize":"11px",
        "fontWeight":"700","background":sc["bg"],"color":sc["text"],
        "border":f"1.5px solid {sc['border']}","transition":"all .5s","letterSpacing":".05em"
    }

    cpp_on = d["cpp_connected"]
    cpp_text  = "C++ Engine Live" if cpp_on else "Python Fallback"
    cpp_style = {
        "fontSize":"9px","fontWeight":"700","letterSpacing":".05em",
        "textTransform":"uppercase","marginRight":"12px",
        "padding":"3px 10px","borderRadius":"99px",
        "background": "#E1F5EE" if cpp_on else "#f5f5f3",
        "color":      "#0F6E56" if cpp_on else "#aaa",
        "border":     "1px solid #1D9E75" if cpp_on else "1px solid #e8e8e6",
        "transition": "all .5s"
    }
    mode_label = ("Simulation Scenarios — click to run on C++ engine"
                  if cpp_on else
                  "Simulation Scenarios — Python fallback active")

    ok    = not d["fault"]
    ok_s  = {"width":"8px","height":"8px","borderRadius":"50%",
             "background":"#1D9E75","boxShadow":"0 0 5px #1D9E75",
             "transition":"all .4s","flexShrink":"0"}
    bad_s = {"width":"8px","height":"8px","borderRadius":"50%",
             "background":"#D85A30","boxShadow":"0 0 8px #D85A30",
             "transition":"all .4s","flexShrink":"0"}
    off_s = {"width":"8px","height":"8px","borderRadius":"50%",
             "background":"#ccc","boxShadow":"none",
             "transition":"all .4s","flexShrink":"0"}

    sens_s = ok_s if ok         else bad_s
    pump_s = ok_s if d["pump"]  else off_s
    fan_s  = ok_s if d["fan"]>5 else off_s

    # DTC alert — show ALL active DTCs
    if active_dtcs:
        dtc_el = html.Div([
            html.Span("Active Diagnostic Codes", style={
                "fontSize":"9px","fontWeight":"700","textTransform":"uppercase",
                "letterSpacing":".06em","color":"#993C1D","display":"block","marginBottom":"4px"
            }),
            *[html.Div(dtc_item, style={
                "fontWeight":"600","fontSize":"11px","padding":"2px 0",
                "borderBottom":"1px solid rgba(216,90,48,0.15)"
              }) for dtc_item in active_dtcs]
        ], style={"background":"#FDF0ED","border":"1.5px solid #D85A30",
                  "borderRadius":"8px","padding":"8px 12px","color":"#993C1D"})
    else:
        dtc_el = html.Div()

    p_style = {"fontSize":"14px","fontWeight":"700",
               "color":"#1D9E75" if d["pump"] else "#1a1a1a"}
    h_style = {"fontSize":"14px","fontWeight":"700",
               "color":"#378ADD" if d["heater"] else "#1a1a1a"}

    def base_layout(h):
        return dict(
            paper_bgcolor="white", plot_bgcolor="white",
            margin=dict(l=35,r=10,t=5,b=20),
            xaxis=dict(showgrid=False,color="#ccc",tickfont=dict(size=8)),
            yaxis=dict(showgrid=True,gridcolor="#f5f5f3",color="#aaa",tickfont=dict(size=8)),
            showlegend=False, height=h
        )

    temp_fig = go.Figure()
    if bt:
        temp_fig.add_trace(go.Scatter(x=list(tt),y=list(bt),mode="lines",
            line=dict(color="#D85A30",width=2),
            fill="tozeroy",fillcolor="rgba(216,90,48,0.07)"))
    temp_fig.add_hline(y=45,line=dict(color="rgba(239,159,39,0.5)",width=1,dash="dash"),
        annotation_text="Cool 45°C",annotation_font_size=9,annotation_font_color="#854F0B")
    temp_fig.add_hline(y=10,line=dict(color="rgba(55,138,221,0.5)",width=1,dash="dash"),
        annotation_text="Heat 10°C",annotation_font_size=9,annotation_font_color="#185FA5")
    for f in fl:
        if f["t"] in tt:
            temp_fig.add_vline(x=f["t"],
                line=dict(color="rgba(216,90,48,0.25)",width=1,dash="dot"))
    temp_fig.update_layout(**base_layout(155))
    temp_fig.update_layout(yaxis_range=[-15,90])

    fan_fig = go.Figure()
    if fs:
        fan_fig.add_trace(go.Scatter(x=list(tt),y=list(fs),mode="lines",
            line=dict(color="#1D9E75",width=2),
            fill="tozeroy",fillcolor="rgba(29,158,117,0.07)"))
    fan_fig.update_layout(**base_layout(130))
    fan_fig.update_layout(yaxis_range=[0,105])

    soc_fig = go.Figure()
    if sv:
        soc_fig.add_trace(go.Scatter(x=list(tt),y=list(sv),mode="lines",
            line=dict(color="#534AB7",width=2),
            fill="tozeroy",fillcolor="rgba(83,74,183,0.07)"))
    soc_fig.add_hline(y=15,line=dict(color="rgba(239,159,39,0.4)",width=1,dash="dash"),
        annotation_text="Low SoC",annotation_font_size=9,annotation_font_color="#854F0B")
    soc_fig.update_layout(**base_layout(130))
    soc_fig.update_layout(yaxis_range=[0,100])

    # Fault log — show all logged DTCs
    if fl:
        fault_els = [
            html.Div([
                html.Span(f"t={f['t']}",style={
                    "fontSize":"10px","color":"#aaa","minWidth":"48px","fontFamily":"monospace"}),
                html.Span(f["msg"],style={
                    "fontSize":"11px","color":"#993C1D","fontWeight":"600"}),
            ], style={"display":"flex","alignItems":"center","gap":"8px",
                      "padding":"5px 0","borderBottom":"1px solid #f5f5f3"})
            for f in reversed(fl)
        ]
    else:
        fault_els = [html.Div("No faults recorded.",
            style={"fontSize":"11px","color":"#bbb","padding":"4px 0","fontStyle":"italic"})]

    bT_disp = "— Sensor fault" if d["fault"] else f"{round(d['bT'])}°C"

    return (
        d["sys"], badge_style, d["scenario"],
        cpp_text, cpp_style, mode_label,
        bT_disp, f"{round(d['fan'])}%",
        f"{round(d['soc'])}%", f"{round(d['spd'])} kph",
        html.Span("ON",style=p_style) if d["pump"]   else html.Span("OFF",style=p_style),
        html.Span("ON",style=h_style) if d["heater"] else html.Span("OFF",style=h_style),
        dtc_el,
        temp_fig, fan_fig, soc_fig, fault_els,
        sens_s, sens_s, sens_s, sens_s, pump_s, fan_s,
    )

# ============================================================
#  RUN
# ============================================================
if __name__ == "__main__":
    print("=" * 52)
    print("  EV Thermal Control System — Dashboard")
    print("  Open: http://127.0.0.1:8050")
    print("")
    print("  Mode 1 — Python only:")
    print("    python scripts/dashboard.py")
    print("")
    print("  Mode 2 — C++ engine wired:")
    print("    Terminal 1: .\\cmake-build\\Debug\\EVThermalControl.exe")
    print("    Terminal 2: python scripts/dashboard.py")
    print("=" * 52)
    app.run(debug=False, port=8050)