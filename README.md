# omni-arm-link

面向 **全向底盘 + 4 机械臂** 机器人的实时控制链路，基于 **WFB-ng** 无线电台：
下行指令与上行遥测均为尽力而为（仅 FEC，无 ACK/重传），自研二进制帧格式，
使用 Linux C++17。

完整的协议规范见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)。

## 快速开始

只需 `g++`（C++17）和 `make`，无需电台即可跑通。下面的命令通过第 4 个参数
指定收发所用的网卡 IP（示例为 `192.168.1.10`，请替换为本机实际网卡地址）：

```sh
cd robot-control-link
make                              # 构建全部可执行文件

# 终端 A：机器人节点，绑定 5600，用 192.168.1.10 这块网卡收发
./build/robot_node 192.168.1.10 5601 5600 192.168.1.10

# 终端 B：控制器，绑定 5601，用 192.168.1.10 这块网卡收发
./build/controller 192.168.1.10 5600 5601 192.168.1.10

#测试结束后，生成最近一次测试的测试报告
python3 tests/run_tests.py
```

正常运行时控制器会打印 `telem=200Hz`。按 `Ctrl-C` 停止，
或在后台启动时执行 `kill %1`。参数说明与更多示例见 [运行](#运行)。

## 目录结构

```
include/link/   协议头文件（common、crc、frame、tlv、messages、
                transport、endpoint、kinematics、control、telemetry）
src/            实现
apps/           robot_node（Jetson）、controller（RPi），以及 sender/receiver 演示程序
docs/           PROTOCOL.md
```

## 构建

依赖：`g++`（支持 C++17）和 `make`。

```sh
cd robot-control-link
make            # 生成 build/robot_node、build/controller、build/sender、build/receiver
```

清理构建产物：

```sh
make clean
```

## 测试

完整的 5 项测试方案、自动化脚make clean本与指标（时延 / 抖动 / PER / FEC 恢复率）
采集见 [`tests/README.md`](tests/README.md)：

生成最近一次测试的测试报告
```sh
python3 tests/run_tests.py          # 结果输出到 tests/results/
```

## 运行

先在项目根目录完成 `make`。默认使用本地回环地址（`127.0.0.1`），无需电台即可运行。
每个程序都接受相同的四个可选参数：

```
<程序> [peer_ip] [peer_port] [bind_port] [src_ip]
```

`src_ip` 用于指定本机发送所用的网卡 IP（默认 `0.0.0.0`，即所有网卡）。绑定到某块
网卡自己的 IP 后，内核会用该网卡作为源地址发送，并只在该网卡上接收。例如让双方
都走 `192.168.1.10` 这块网卡：

```sh
./build/robot_node 192.168.1.50 5601 5600 192.168.1.10 &
./build/controller 192.168.1.50 5600 5601 192.168.1.10
```

### 应用模式（主任务）—— robot_node + controller

终端 A（先启动机器人节点）：sh
kill %1 %2  

```sh
./build/robot_node
```

终端 B（再启动控制器）：

```sh
./build/controller
```

以上为默认配置：robot_node 绑定 `5600` 并以 200 Hz 发送 340 B 遥测，controller
绑定 `5601` 并以 100 Hz 发送 20 B 控制包。

若需后台运行或自定义地址/端口：

```sh
./build/robot_node &                 # 默认：绑定 5600，向 127.0.0.1:5601 发送
./build/controller                   # 默认：绑定 5601，向 127.0.0.1:5600 发送

# 自定义示例
./build/robot_node 127.0.0.1 5601 5600 &
./build/controller 127.0.0.1 5600 5601
```

停止：

```sh
kill %1 %2          # 若使用 & 在后台启动
# 或在前台终端按 Ctrl-C
```

正常运行时，控制器应报告 `telem=200Hz`，同时能看到解析出的
`quat0`/`laser0`/`state0` 数值，以及偶尔出现的 `robot log[...]` 调试输出。

### 演示模式 —— sender + receiver

终端 A（先启动接收端）：

```sh
./build/receiver
```

终端 B（再启动发送端）：

```sh
./build/sender
```

receiver 绑定 `5600`，sender 绑定 `5601`。同样支持自定义地址/端口，例如：

```sh
./build/receiver 127.0.0.1 5601 5600 &
./build/sender   127.0.0.1 5600 5601
```

## 应用层数据包（主任务）

- `ControlPacket` —— 下行，**20 B**：`joystick[4]` + `estop` + `mode` +
  `buttons`。控制流（最高优先级），尽力而为（仅 FEC），由控制器以 100 Hz 发送。
- `TelemetryPacket` —— 上行，**340 B**：`5 个四元数`（80 B）+ `15 路激光`（60 B）
  + `50 个机器人状态`（200 B）。遥测流，尽力而为（仅 FEC），由机器人以 200 Hz 发送。
- `Debug` 流承载 `EventLog`，优先级**最低**；接收时端点先分发控制，最后分发调试。
- **会话功能已暂停**（未调用 `beginConnect`/`setDeviceId`；
  `session_enabled_` 保持为 false）。
- 遥测目前填充的是**虚拟数据**（见
  `apps/robot_node.cpp` 中的 `fillVirtualTelemetry`）。

定义位于 `include/link/control.hpp` 和 `include/link/telemetry.hpp`
（`#pragma pack(1)` + 对精确大小的 `static_assert`）。

## WFB-ng 集成

WFB-ng 的 `wfb_tx` 从 UDP socket 读取原始帧，`wfb_rx` 将收到的帧转发到 UDP
socket。将 `link::UdpTransport` 指向这些服务端口，整条无线链路（及其
Reed-Solomon FEC）便透明地位于本协议之下。无需修改代码，只需配置 IP/端口。

## 实时性

机器人进程请求 `SCHED_FIFO`（优先级 50）。以 root 运行或授予
`CAP_SYS_NICE` 以获得确定性调度：

```sh
sudo setcap cap_sys_nice+ep ./build/robot_node
```

协议本身不做 ACK/重传，两个方向都保持纯尽力而为，可靠性交给底层的 WFB-ng FEC；
200 Hz 的预算与实时性得以保留。

## 协议速览

- 14 字节头部：magic、version、type、flags、stream、seq、len、头部 CRC。
- TLV 载荷（向后兼容，未知标签会被跳过）。
- 带优先级的流：**control（最高）→ telemetry → debug（最低）**。
- 下行：`ControlCmd`（joystick/estop/mode）以及可选的 `SetVelocity`、
  `MoveOrder`、`Stop`、`EmergencyStop`、`SetArmJoints` —— 尽力而为（仅 FEC），
  并按序列号去重。
- 上行：`Telemetry`（340 B）以及 `EventLog` —— 仅 FEC。
- 会话（`ConnectReq`/`ConnectAck`、心跳）已实现但**处于暂停状态**。
- 机器人失效保护：500 ms 未收到有效指令即自动停止。

## 许可证

MIT —— 见 [LICENSE](LICENSE)。
