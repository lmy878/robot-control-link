# omni-arm-link

Real-time control link for an **omnidirectional base + 4 arms** robot over a
**WFB-ng** radio: reliable downlink commands (ACK + retransmit), best-effort
uplink telemetry (FEC-only), hand-rolled binary framing, Linux C++17.

See [`docs/PROTOCOL.md`](docs/PROTOCOL.md) for the full protocol specification.

## Layout

```
include/link/   protocol headers (common, crc, frame, tlv, messages,
                transport, endpoint, kinematics, control, telemetry)
src/            implementation
apps/           robot_node (Jetson), controller (RPi), + sender/receiver demo
tests/          unit + ARQ + session + packet + priority tests
docs/           PROTOCOL.md
```

## Build

```sh
make            # builds build/sender, build/receiver, build/test_link
make test       # runs the unit + ARQ tests
```

Requires only `g++` (C++17) and `make`.

## Run (loopback, no radio)

Two modes are provided.

**Application mode (main task)** — Jetson robot node + RPi controller:

```sh
./build/robot_node &     # binds 5600, sends 200 Hz / 340 B telemetry
./build/controller       # binds 5601, sends 100 Hz control packets
```

**Demo mode** — the older ACK-per-command sender/receiver pair:

```sh
./build/receiver &       # binds 5600
./build/sender           # binds 5601
```

In application mode you should see the controller report `telem=200.0Hz` with a
growing `acked` count, and occasional `robot log[...]` debug lines.

## Application packets (main task)

- `ControlPacket` — downlink, **168 B**, control stream (highest priority),
  reliable (ACK + retransmit), sent by the controller at 100 Hz.
- `TelemetryPacket` — uplink, **340 B**, telemetry stream, best-effort
  (FEC-only), sent by the robot at 200 Hz.
- `Debug` stream carries `EventLog` at the **lowest priority**; on receive,
  endpoints dispatch control first, debug last.
- **Session features are paused** (`beginConnect`/`setDeviceId` are not called;
  `session_enabled_` stays false).

Defined in `include/link/control.hpp` and `include/link/telemetry.hpp`
(`#pragma pack(1)` + `static_assert` on exact size).

## WFB-ng integration

WFB-ng's `wfb_tx` reads raw frames from a UDP socket and `wfb_rx` forwards
received frames to a UDP socket. Point `link::UdpTransport` at those service
ports and the whole radio link (plus its Reed-Solomon FEC) sits transparently
below this protocol. No code change is required — only the IP/ports.

## Real-time

The robot process requests `SCHED_FIFO` (priority 50). Run it as root or grant
`CAP_SYS_NICE` for deterministic scheduling:

```sh
sudo setcap cap_sys_nice+ep ./build/receiver
```

Unlike an ACK per telemetry sample, only downlink commands are acknowledged, so
uplink stays purely best-effort and the 200 Hz budget is preserved.

## Protocol at a glance

- 16-byte header: magic, version, type, flags, stream, seq, ack, len, header CRC.
- TLV payloads (forward compatible, unknown tags skipped).
- Session: `ConnectReq`/`ConnectAck` handshake, 1 Hz heartbeats, link-loss
  detection (`Idle → Connecting → Active ⇄ Lost`).
- Downlink: `SetVelocity`, `MoveOrder`, `Stop`, `EmergencyStop`, `SetArmJoints`,
  `SetArmPose`, `SetArmMode`, `SetParam`
  — reliable via ACK + retransmit, deduplicated by sequence number.
- Uplink: `TelemetryBase`, `TelemetryArm`, `Status`, `EventLog` — FEC-only.
- Robot failsafe: auto-stop after 500 ms without a valid command.

## License

MIT — see [LICENSE](LICENSE).
