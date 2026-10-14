#!/usr/bin/env python3
"""Render the aggregated test results as JSON and Markdown."""
import json
import os


def _fmt(x, nd=3):
    if x is None:
        return "n/a"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return str(x)


def markdown(results):
    r = results
    out = []
    out.append("# RCLP 自动化测试报告")
    out.append("")
    out.append(f"- 生成时间: {r.get('generated', '')}")
    out.append(f"- 提交: `{r.get('commit', 'n/a')}`")
    out.append(f"- 总体结果: **{'通过' if r.get('ok') else '失败'}**")
    out.append("")

    # ---- headline metrics ----
    m = r.get("metrics", {})
    out.append("## 关键指标 (latency / jitter / PER / FEC)")
    out.append("")
    out.append("| 指标 | 值 |")
    out.append("|------|----|")
    if "latency_ms" in m:
        lat = m["latency_ms"]
        out.append(f"| 时延 mean / p95 / p99 / max (ms) | "
                   f"{_fmt(lat.get('mean'))} / {_fmt(lat.get('p95'))} / "
                   f"{_fmt(lat.get('p99'))} / {_fmt(lat.get('max'))} |")
    if "jitter_ms" in m:
        j = m["jitter_ms"]
        out.append(f"| 抖动 RFC3550 / stddev (ms) | "
                   f"{_fmt(j.get('rfc3550'))} / {_fmt(j.get('stddev'))} |")
    if "per" in m:
        out.append(f"| 丢包率 PER (无损链路需为 0) | {_fmt(m['per'] * 100, 2)}% |")
    if "fec_recovery_rate" in m:
        out.append(f"| FEC 恢复率 (数据包) | {_fmt(m['fec_recovery_rate'] * 100, 1)}% |")
    out.append("")

    # ---- per-suite summary ----
    out.append("## 测试套件总览")
    out.append("")
    out.append("| 套件 | 结果 | 说明 |")
    out.append("|------|------|------|")
    for name, s in r.get("suites", {}).items():
        out.append(f"| {name} | {'PASS' if s.get('ok') else 'FAIL'} | {s.get('summary', '')} |")
    out.append("")

    # ---- T2 ----
    s = r["suites"].get("T2_downlink_besteffort")
    if s and s.get("scenarios"):
        out.append("## T2 下行控制 (best-effort / FEC-only)")
        out.append("")
        out.append("| 丢包率 | 发送 | 接收 | PER | 时延均值(ms) | p99(ms) | 重复帧 |")
        out.append("|-------:|-----:|-----:|----:|------------:|--------:|-------:|")
        for sc in s["scenarios"]:
            out.append(f"| {_fmt(sc['loss'] * 100, 0)}% | {sc['sent']} | {sc['received']} | "
                       f"{_fmt(sc['per'] * 100, 1)}% | {_fmt(sc['latency_ms'].get('mean'))} | "
                       f"{_fmt(sc['latency_ms'].get('p99'))} | {sc['duplicates']} |")
        out.append("")

    # ---- T3 ----
    s = r["suites"].get("T3_uplink_perf")
    if s and s.get("scenarios"):
        out.append("## T3 上行遥测性能 (best-effort)")
        out.append("")
        out.append("| 丢包率 | 发送 | 接收 | PER | 时延均值 | p99 | 抖动RFC3550 |")
        out.append("|-------:|-----:|-----:|----:|---------:|----:|------------:|")
        for sc in s["scenarios"]:
            out.append(f"| {_fmt(sc['loss'] * 100, 0)}% | {sc['sent']} | {sc['received']} | "
                       f"{_fmt(sc['per'] * 100, 1)}% | {_fmt(sc['latency_ms'].get('mean'))} | "
                       f"{_fmt(sc['latency_ms'].get('p99'))} | {_fmt(sc['jitter_ms'].get('rfc3550'))} |")
        out.append("")

    # ---- T4 ----
    s = r["suites"].get("T4_fec_recovery")
    if s and s.get("scenarios"):
        out.append("## T4 FEC 恢复率")
        out.append("")
        out.append("| 丢包率 | (k,m) | 开销 | 丢失数据包 | 恢复 | 恢复率 | 剩余PER | 时延均值 |")
        out.append("|-------:|------:|-----:|-----------:|-----:|-------:|--------:|---------:|")
        for sc in s["scenarios"]:
            out.append(f"| {_fmt(sc['loss'] * 100, 0)}% | ({sc['k']},{sc['m']}) | "
                       f"{_fmt(sc['overhead'], 2)}x | {sc['data_dropped']} | "
                       f"{sc['data_recovered']} | {_fmt(sc['recovery_rate'] * 100, 1)}% | "
                       f"{_fmt(sc['residual_per'] * 100, 1)}% | {_fmt(sc['latency_ms'].get('mean'))} |")
        out.append("")

    # ---- T5 ----
    s = r["suites"].get("T5_scheduling_safety")
    if s:
        out.append("## T5 流优先级与安全")
        out.append("")
        for k, v in s.get("detail", {}).items():
            out.append(f"- {k}: {v}")
        out.append("")

    return "\n".join(out) + "\n"


def write(results, outdir):
    os.makedirs(outdir, exist_ok=True)
    jpath = os.path.join(outdir, "report.json")
    mpath = os.path.join(outdir, "report.md")
    with open(jpath, "w") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    with open(mpath, "w") as f:
        f.write(markdown(results))
    return jpath, mpath
