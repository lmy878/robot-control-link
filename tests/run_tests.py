#!/usr/bin/env python3
"""RCLP automated test orchestrator.

Builds the test binaries, runs the five suites, collects latency / jitter / PER
/ FEC-recovery metrics and writes tests/results/report.{json,md}.

Suites:
  T1  protocol conformance & robustness        (unit, no network)
  T2  best-effort downlink control under loss  (lossy channel)
  T3  uplink best-effort latency/jitter/PER    (lossy channel)
  T4  block FEC recovery rate                  (lossy channel + RS FEC)
  T5  stream priority/scheduling & failsafe    (unit + integration)

Usage:  python3 tests/run_tests.py [--quick] [--duration SEC] [--outdir DIR]
"""
import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"
BUILD = ROOT / "build" / "tests"
TOOLS = TESTS / "tools"
PY = sys.executable
CHANNEL = str(TOOLS / "lossy_channel.py")

sys.path.insert(0, str(TOOLS))
import report  # noqa: E402

TELE = str(BUILD / "telemetry_probe")
CTL = str(BUILD / "control_probe")
TEST_CODEC = str(BUILD / "test_codec")
TEST_SCHED = str(BUILD / "test_sched")
ROBOT = str(ROOT / "build" / "robot_node")
CONTROLLER = str(ROOT / "build" / "controller")


# --------------------------------------------------------------------- helpers
def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def spawn(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def finish(p, timeout):
    """Wait for a process; on timeout interrupt gracefully so buffered stdout
    (the JSON summary) is flushed, then hard-kill as a last resort."""
    try:
        out, err = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.send_signal(signal.SIGINT)
        try:
            out, err = p.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
            out, err = p.communicate()
    return out, err


def last_json(text):
    for line in reversed(text.splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return None


def fail_summary(detail=""):
    return {"summary": detail}


# ------------------------------------------------------------------- scenarios
def link_scenario(kind, loss, duration, fec=None, rate=None):
    p_tx, p_rx, p_e0, p_e1 = free_port(), free_port(), free_port(), free_port()
    statsfile = str(TESTS / "results" / f".stats_{kind}_{loss}_{fec}.json")
    Path(statsfile).parent.mkdir(parents=True, exist_ok=True)

    proxy_cmd = [PY, CHANNEL, "--edge0", str(p_e0), "--edge1", str(p_e1),
                 "--peer1", f"127.0.0.1:{p_rx}", "--loss0", str(loss), "--loss1", "0",
                 "--duration", str(duration + 1.0), "--stats", statsfile, "--seed", "1"]
    if fec:
        proxy_cmd += ["--fec-k", str(fec[0]), "--fec-m", str(fec[1])]

    prog = TELE if kind == "telemetry" else CTL
    r = str(rate or (200 if kind == "telemetry" else 100))
    rx_cmd = [prog, "rx", str(p_rx), "127.0.0.1", str(p_e1), str(duration + 1.5), r]
    tx_cmd = [prog, "tx", str(p_tx), "127.0.0.1", str(p_e0), str(duration), r]

    rx = spawn(rx_cmd)
    proxy = spawn(proxy_cmd)
    time.sleep(0.4)
    tx = spawn(tx_cmd)

    tout, _ = finish(tx, duration + 6)
    rout, _ = finish(rx, duration + 6)
    pout, _ = finish(proxy, duration + 6)

    if os.path.exists(statsfile):
        with open(statsfile) as f:
            proxy_res = json.load(f)
        os.remove(statsfile)
    else:
        proxy_res = last_json(pout)

    return last_json(tout) or {}, last_json(rout) or {}, proxy_res or {}


def scenario_t2(loss, duration):
    """Downlink control is best-effort / FEC-only: a lost command is simply
    superseded by the next one, so PER should track the raw channel loss."""
    tx, rx, px = link_scenario("control", loss, duration)
    sent, received = tx.get("sent", 0), rx.get("received", 0)
    per = ((sent - received) / sent) if sent else 1.0
    sc = {
        "loss": loss,
        "sent": sent,
        "received": received,
        "unique": rx.get("unique", 0),
        "duplicates": rx.get("duplicates", 0),
        "per": per,
        "latency_ms": rx.get("latency_ms", {}),
        "jitter_ms": rx.get("jitter_ms", {}),
        "wire_dropped": px.get("directions", {}).get("0->1", {}).get("dropped", 0),
    }
    # Best-effort downlink: PER should track the raw channel loss. The margin
    # covers small-sample Bernoulli variance (only a few hundred frames/run).
    sc["ok"] = per <= max(0.02, loss + 0.06)
    return sc


def scenario_t3(loss, duration):
    tx, rx, px = link_scenario("telemetry", loss, duration)
    sent, received = tx.get("sent", 0), rx.get("received", 0)
    per = ((sent - received) / sent) if sent else 1.0
    sc = {
        "loss": loss,
        "sent": sent,
        "received": received,
        "per": per,
        "latency_ms": rx.get("latency_ms", {}),
        "jitter_ms": rx.get("jitter_ms", {}),
        "wire_dropped": px.get("directions", {}).get("0->1", {}).get("dropped", 0),
    }
    sc["ok"] = per <= max(0.01, loss + 0.02)
    return sc


def scenario_t4(loss, k, m, duration):
    tx, rx, px = link_scenario("telemetry", loss, duration, fec=(k, m))
    sent, received = tx.get("sent", 0), rx.get("received", 0)
    fec = px.get("fec", {})
    per = ((sent - received) / sent) if sent else 1.0
    sc = {
        "loss": loss,
        "k": k,
        "m": m,
        "overhead": fec.get("overhead", (k + m) / k if k else 0),
        "data_dropped": px.get("directions", {}).get("0->1", {}).get("fec_data_dropped", 0),
        "data_recovered": px.get("directions", {}).get("0->1", {}).get("fec_data_recovered", 0),
        "recovery_rate": fec.get("recovery_rate", 0.0),
        "residual_per": per,
        "latency_ms": rx.get("latency_ms", {}),
        "jitter_ms": rx.get("jitter_ms", {}),
    }
    sc["ok"] = sc["recovery_rate"] >= 0.50 and per <= loss * 0.6
    return sc


def scenario_t5_failsafe(duration):
    """Block the downlink mid-run and confirm the robot fails safe."""
    p_e0, p_e1, p_robot, p_ctl = free_port(), free_port(), free_port(), free_port()
    statsfile = str(TESTS / "results" / ".stats_failsafe.json")
    drop_after = duration * 0.4
    proxy_cmd = [PY, CHANNEL, "--edge0", str(p_e0), "--edge1", str(p_e1),
                 "--loss0", "0", "--loss1", "0", "--duration", str(duration + 1.0),
                 "--drop-after", str(drop_after), "--drop-edge", "1", "--stats", statsfile]
    proxy = spawn(proxy_cmd)
    robot = spawn([ROBOT, "127.0.0.1", str(p_e0), str(p_robot)])
    time.sleep(0.3)
    controller = spawn([CONTROLLER, "127.0.0.1", str(p_e1), str(p_ctl)])

    time.sleep(duration)
    for proc in (controller, robot):
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
    cout, _ = finish(controller, 4)
    finish(robot, 4)
    finish(proxy, 4)
    if os.path.exists(statsfile):
        os.remove(statsfile)

    zs = []
    for line in cout.splitlines():
        mq = re.search(r"quat0=\(([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\)", line)
        if mq:
            zs.append(float(mq.group(4)))
    moved = any(z > 0.03 for z in zs)
    settled = len(zs) >= 2 and all(abs(z) < 0.01 for z in zs[-2:])
    detail = {
        "quat0_z_samples": zs,
        "was_moving_before_drop": moved,
        "settled_to_zero_after_drop": settled,
    }
    return {"ok": moved and settled, "detail": detail}


# ------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="shorter/smaller run")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--outdir", default=str(TESTS / "results"))
    args = ap.parse_args()

    duration = args.duration or (2.0 if args.quick else 4.0)
    t5_duration = 5.0 if not args.quick else 4.0

    print("== building test binaries ==")
    b = subprocess.run(["make", "-C", str(ROOT), "test"])
    if b.returncode != 0:
        print("build failed", file=sys.stderr)
        return 1

    results = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "suites": {}, "metrics": {}}
    try:
        results["commit"] = subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:
        results["commit"] = "n/a"

    # ---- T1 / T5 unit ----
    print("== T1 protocol conformance ==")
    c_out = subprocess.run([TEST_CODEC], capture_output=True, text=True)
    c_json = last_json(c_out.stdout) or {}
    hdr_rate = 0.0
    mh = re.search(r"header_corruption_detection_rate=([\d.]+)", c_out.stdout)
    if mh:
        hdr_rate = float(mh.group(1))
    t1_ok = c_json.get("fail", 1) == 0 and hdr_rate >= 0.999
    results["suites"]["T1_protocol_conformance"] = {
        "ok": t1_ok,
        "detail": {"pass": c_json.get("pass"), "fail": c_json.get("fail"),
                   "corruption_detection_rate": hdr_rate},
        "summary": f"{c_json.get('pass', 0)}/{c_json.get('total', 0)} 通过, "
                   f"帧头损坏检测率 {hdr_rate * 100:.1f}%",
    }

    print("== T5 stream priority / scheduling ==")
    s_out = subprocess.run([TEST_SCHED], capture_output=True, text=True)
    s_json = last_json(s_out.stdout) or {}
    t5_unit_ok = s_json.get("fail", 1) == 0

    print("== T2 best-effort downlink ==")
    t2_losses = [0.0, 0.05] if args.quick else [0.0, 0.05, 0.10, 0.20]
    t2_scen = [scenario_t2(l, duration) for l in t2_losses]
    base2 = next((s for s in t2_scen if s["loss"] == 0.0), t2_scen[0])
    t2_ok = all(sc["ok"] for sc in t2_scen)
    results["suites"]["T2_downlink_besteffort"] = {
        "ok": t2_ok,
        "scenarios": t2_scen,
        "summary": f"无损 PER {base2['per'] * 100:.2f}%，PER 随丢包率上升（FEC-only）"
                   if t2_ok else "下行 PER 异常",
    }
    results["metrics"]["downlink_per"] = base2["per"]

    print("== T3 uplink performance ==")
    t3_losses = [0.0, 0.05] if args.quick else [0.0, 0.01, 0.05, 0.10]
    t3_scen = [scenario_t3(l, duration) for l in t3_losses]
    base = next((s for s in t3_scen if s["loss"] == 0.0), t3_scen[0])
    t3_ok = base["per"] <= 0.01 and base["latency_ms"].get("p99", 999) < 20.0
    results["suites"]["T3_uplink_perf"] = {
        "ok": t3_ok,
        "scenarios": t3_scen,
        "summary": f"无损 PER {base['per'] * 100:.2f}%, p99 时延 "
                   f"{base['latency_ms'].get('p99', 0):.2f} ms",
    }
    results["metrics"]["latency_ms"] = base["latency_ms"]
    results["metrics"]["jitter_ms"] = base["jitter_ms"]
    results["metrics"]["per"] = base["per"]

    print("== T4 FEC recovery ==")
    t4_configs = [(10, 4, 0.10)] if args.quick else [(4, 1, 0.10), (10, 2, 0.10),
                                                     (10, 4, 0.10), (20, 4, 0.10)]
    t4_scen = [scenario_t4(loss, k, m, duration) for (k, m, loss) in t4_configs]
    t4_ok = all(sc["ok"] for sc in t4_scen)
    recovery_rates = [sc["recovery_rate"] for sc in t4_scen]
    results["suites"]["T4_fec_recovery"] = {
        "ok": t4_ok,
        "scenarios": t4_scen,
        "summary": f"平均恢复率 {sum(recovery_rates) / len(recovery_rates) * 100:.1f}%",
    }
    results["metrics"]["fec_recovery_rate"] = (
        sum(recovery_rates) / len(recovery_rates) if recovery_rates else 0.0)

    print("== T5 failsafe ==")
    failsafe = scenario_t5_failsafe(t5_duration)
    results["suites"]["T5_scheduling_safety"] = {
        "ok": bool(t5_unit_ok and failsafe["ok"]),
        "detail": {
            "scheduling_unit_tests": "通过" if t5_unit_ok else "失败",
            "dispatch_order": "control -> telemetry -> debug",
            "duplicate_suppression": "通过" if t5_unit_ok else "失败",
            "failsafe_was_moving": failsafe["detail"]["was_moving_before_drop"],
            "failsafe_settled_to_zero": failsafe["detail"]["settled_to_zero_after_drop"],
            "quat0_z_samples": failsafe["detail"]["quat0_z_samples"],
        },
        "summary": "优先级/去重通过, 失效保护触发" if (t5_unit_ok and failsafe["ok"])
                   else "存在问题",
    }

    results["ok"] = all(s.get("ok") for s in results["suites"].values())

    jpath, mpath = report.write(results, args.outdir)
    print(f"\n== report: {jpath}")
    print(f"== report: {mpath}")
    print("\n" + report.markdown(results))
    return 0 if results["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
