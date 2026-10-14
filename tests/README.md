# RCLP 测试方案与自动化测试

本目录包含 `robot-control-link`（RCLP 协议 + 上层收发代码）的**全套 5 项测试方案**、
自动化测试脚本与指标采集工具。测试在本地回环（loopback）上运行，通过一个可编程的
“有损无线信道”模拟器注入丢包、并可叠加块 FEC，从而在无电台的环境下量化链路行为。

```
tests/
  run_tests.py            测试编排入口（构建 + 运行 5 个套件 + 汇总报告）
  README.md               本文件（测试方案）
  probe/
    telemetry_probe.cpp   上行探针：时延 / 抖动 / PER
    control_probe.cpp     下行可靠探针：ACK + 重传（ARQ）
  unit/
    test_codec.cpp        T1 协议一致性 / 鲁棒性
    test_sched.cpp        T5 流优先级 / 去重（mock transport）
  tools/
    lossy_channel.py      有损信道 + FEC + 线路统计
    fec.py                GF(256) Cauchy MDS 纠删码（模拟 WFB-ng FEC）
    report.py             结果汇总（JSON + Markdown）
  results/                运行产物：report.json / report.md
```

## 运行

```sh
make test                     # 仅编译测试所需的二进制
python3 tests/run_tests.py    # 运行全部 5 个套件（约 1–2 分钟）
python3 tests/run_tests.py --quick          # 快速模式（场景更少/更短）
python3 tests/run_tests.py --duration 6     # 自定义每个场景的时长
```

产物：`tests/results/report.json`（机器可读）与 `tests/results/report.md`（人类可读）。
退出码 0 表示全部套件通过。

## 测试拓扑

```
                  ┌────────────────────────────────────────────┐
  tx / robot ─────┤  lossy_channel.py                          ├───── rx / controller
   (edge0)        │   0->1 :  丢包(Bernoulli) [+ 块 FEC]       │      (edge1)
                  │   1->0 :  丢包(Bernoulli)                  │
                  └────────────────────────────────────────────┘
```

`lossy_channel.py` 双向转发 UDP 数据报，按设定的概率独立丢包；上行方向可叠加
`(k, m)` 系统化 Reed–Solomon 纠删码（GF(256) Cauchy 矩阵，MDS），用于模拟 WFB-ng
在 RCLP 之下提供的 FEC。信道同时解析 RCLP 帧头，统计每流/重传/重复帧数量。
可通过 `--drop-after` 在指定时刻起对某方向 100% 丢包，用于失效保护测试。

---

## 5 项测试套件

### T1 协议一致性与鲁棒性（`unit/test_codec.cpp`，无网络）

- **目的**：验证线上协议编解码正确、损坏可检测。
- **内容**：CRC 已知向量（CRC-16/CCITT-FALSE `0x29B1`、CRC-32/ISO-HDLC
  `0xCBF43926`）；16 字节帧头往返；**逐位翻转**帧头 0..13 字节必须全部被拒
  （统计“帧头损坏检测率”，要求 100%）；短帧 / 错 magic / 错版本 / 超长 payload
  被拒；payload CRC 校验；所有 TLV 消息往返；未知 tag 跳过；`ControlPacket=20 B`、
  `TelemetryPacket=340 B`；流方向与优先级分类。
- **判据**：全部用例通过 **且** 帧头损坏检测率 = 100%。

### T2 可靠下行链路（ACK + 重传 / ARQ）

- **目的**：验证下行指令在丢包下最终可靠投递、去重正确。
- **方法**：`control_probe tx` 以 100 Hz 通过 `Endpoint::sendReliable` 下发
  `ControlPacket`；`control_probe rx` 接收并自动 ACK；信道丢包率取
  `{0, 5%, 10%, 20%}`。tx 侧统计下发 / 确认 / 未确认，rx 侧统计唯一指令与重复，
  信道侧统计重传帧与重复帧。
- **指标**：投递率 = acked/sent；重传帧数；重复帧数；ACK 时延（RTT）。
- **判据**：丢包 ≤10% 时投递率 ≥99.5% 且无未确认指令；重传帧数应随丢包率上升。

### T3 上行遥测性能（best-effort）

- **目的**：量化上行遥测的**时延、抖动、PER**。
- **方法**：`telemetry_probe tx` 以 200 Hz 发送 340 B `TelemetryPacket`，在
  payload 的 `states[0..1]` 内嵌 `CLOCK_MONOTONIC` 双精度时间戳；rx 侧据此计算
  单向时延。丢包率取 `{0, 1%, 5%, 10%}`。
- **指标**：时延 mean/p50/p95/p99/max/stddev；抖动（RFC 3550 + 标准差）；
  PER = (sent−received)/sent。
- **判据**：无损链路 PER ≤1% 且 p99 时延 <20 ms（协议目标）。

### T4 FEC 恢复率

- **目的**：量化块 FEC 在丢包下的**恢复率**及其时延代价。
- **方法**：在 T3 基础上对上行方向启用 `(k, m)` 纠删码，配置
  `(4,1)/(10,2)/(10,4)/(20,4)`（丢包 10%）。信道丢弃数据/校验包后，用任意 ≥k 个
  存活包重建数据包；统计被恢复的丢失数据包。
- **指标**：
  - 恢复率 = 已恢复数据包 / 线路丢失数据包；
  - 剩余 PER = 纠错后仍丢失的数据包 / 总数据包；
  - 开销 = (k+m)/k；FEC 引入的块缓冲时延。
- **判据**：恢复率 ≥50%，且剩余 PER 相比原始丢包率至少降低 40%
  （即 FEC 显著改善可靠性）；m 越大恢复率越高。

### T5 流优先级、调度与失效保护

- **目的**：验证三条逻辑流的优先级调度、去重，以及 500 ms 失控失效保护。
- **方法**：
  - **单元（`unit/test_sched.cpp`）**：用内存 mock transport 预置被打乱的
    debug/telemetry/control 帧，断言 `Endpoint::poll` 按 **control → telemetry →
    debug** 分发；同流保持到达顺序；重复序号只分发一次但会重发 ACK。
  - **集成（失效保护）**：`robot_node ↔ 信道 ↔ controller`，运行中途用
    `--drop-after` 阻断下行（控制器→机器人）。观察控制器打印的遥测 `quat0`：
    阻断前偏航分量非零（机器人在动），阻断 ≥500 ms 后归零（失效保护生效），
    同时 `acked` 计数冻结（不再有新指令被确认）。
- **判据**：单元用例全通过；集成中断前有运动、断后归零、确认计数冻结。

---

## 指标定义与采集

| 指标 | 定义 | 采集位置 |
|------|------|----------|
| 时延 latency | 接收单调时钟 − 发送端内嵌单调时间戳（单向） | `telemetry_probe` rx |
| 抖动 jitter | RFC 3550 平滑插值抖动 `J += (|D| − J)/16`，另报告时延标准差 | `telemetry_probe` rx |
| PER | (发送数 − 接收数) / 发送数 | tx 计数 + rx 计数 |
| FEC 恢复率 | 已恢复的丢失数据包 / 线路丢失数据包 | `lossy_channel.py` |

> 说明：真实部署中 FEC 与时延/PER 由 WFB-ng 提供；本方案用同一数学模型（MDS 纠删码
> + 独立丢包）在回环上可复现地测出这些指标，并把 RCLP 协议栈放在其上层一起验证。
> 后续可将 `lossy_channel.py` 替换为真实的 `wfb_tx`/`wfb_rx` 服务端口，探针与
> 报告逻辑无需改动。

## 局限

- 时延为**单向**且两端运行于同一主机，依赖 `CLOCK_MONOTONIC` 一致；跨机测试需做时钟同步。
- FEC 采用纠删（erasure）模型（丢包位置已知），未模拟误码纠错（error correction）。
- 回环网络无真实无线噪声/乱序；如需乱序/突发丢包可在 `lossy_channel.py` 中扩展
  丢包模型（如 Gilbert–Elliott）。
