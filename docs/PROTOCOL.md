# Robot Control Link Protocol (RCLP) v1

A compact, real-time control protocol for an **omnidirectional base + 4 arms**
robot, carried over a **WFB-ng** (Wifibroadcast-ng) radio link.

- **Downlink** (controller → robot): movement orders / velocity setpoints / arm
  commands. Delivery is **reliable** (explicit ACK + retransmit).
- **Uplink** (robot → controller): telemetry + status. Delivery is
  **best-effort / FEC-only** (WFB-ng protects the bits; no retransmit).
- Rates: **downlink 100 Hz, uplink 200 Hz**, target end-to-end latency **< 20 ms**.
- Platform: **Linux, C++17**, hand-rolled binary encoding, low latency / RT friendly.

---

## 1. Layering

```
+--------------------------------------------------+
|  Application (orders, arm targets, telemetry)     |  apps/sender, apps/receiver
+--------------------------------------------------+
|  RCLP  (this spec)                                |  link::Endpoint
|  framing | seq | dup-suppress | ACK/NACK | ARQ    |
+--------------------------------------------------+
|  WFB-ng  (radio transport + Reed-Solomon FEC)     |  external service
|  wfb_tx <- UDP  |  wfb_rx -> UDP                  |
+--------------------------------------------------+
|  802.11 raw radio                                 |
+--------------------------------------------------+
```

WFB-ng's `wfb_tx` reads raw frames from a UDP socket; `wfb_rx` forwards received
frames to a UDP socket. RCLP therefore talks plain UDP to those service ports and
treats the radio, its FEC and link statistics as a black box
(`link::UdpTransport`).

---

## 2. Roles

| Role | Sends | Receives |
|------|-------|----------|
| **Controller (C)** | downlink commands | uplink telemetry |
| **Robot (D)** | uplink telemetry | downlink commands |

Direction is implied by the message type (`0x00..0x7F` = downlink,
`0x80..0xFF` = uplink).

---

## 3. Frame format

Every frame is a **16-byte header** followed by a payload:

| Offset | Size | Field | Notes |
|-------:|-----:|-------|-------|
| 0  | 2 | Magic | `0x52 0x4C` (`"RL"`) |
| 2  | 1 | Version | `1` |
| 3  | 1 | MsgType | see §7 |
| 4  | 1 | Flags | see §4 |
| 5  | 1 | StreamId | `0` control, `1` telemetry, `2` session |
| 6  | 2 | Seq | per-stream, little-endian |
| 8  | 2 | AckSeq | piggybacked ACK (0 = none) |
| 10 | 2 | PayloadLen | bytes after the header |
| 12 | 2 | HeaderCRC | CRC-16/CCITT-FALSE over bytes `0..11` |
| 14 | 2 | Reserved | 0 (future use) |

All multi-byte integers and IEEE-754 floats are **little-endian**.

If flag `PAYLOAD_CRC` is set, the last 2 bytes of the payload are a
CRC-16/CCITT-FALSE over the preceding payload bytes; `PayloadLen` includes them.
The header CRC always protects the header itself. Frames with a bad magic,
version, length, or either CRC are silently dropped.

---

## 4. Flags

| Bit | Name | Meaning |
|----:|------|---------|
| 0 | `ACK_REQ` | sender wants an ACK for this seq |
| 1 | `IS_ACK` | this frame is an ACK (`AckSeq` = acknowledged seq) |
| 2 | `IS_NACK` | this frame is a NACK (`AckSeq` = rejected seq) |
| 3 | `RETRANSMIT` | a retransmission |
| 4 | `FRAG_START` | first fragment of a message |
| 5 | `FRAG_MORE` | more fragments follow |
| 6 | `PAYLOAD_CRC` | trailing payload CRC-16 present |
| 7 | `EMERGENCY` | emergency / e-stop semantics |

---

## 5. Streams and sequence spaces

Each `StreamId` has its **own 16-bit wrapping sequence counter**, independent of
direction:

- **Control (0)** — downlink commands and their ACKs.
- **Telemetry (1)** — uplink telemetry.
- **Session (2)** — connect / heartbeat / status.

A single frame may both carry a new message and piggyback `AckSeq` for the
opposite direction.

---

## 6. Payload encoding (TLV)

Payloads are **type-length-value** triples for forward compatibility:

```
+--------+--------+------------------+
| tag u8 | len u8 | value[len] bytes |
+--------+--------+------------------+
```

A decoder **skips unknown tags**, so new fields can be appended without
breaking older peers. Values may be `u8`, `u16`, `f32`, or arrays of `f32`.

---

## 7. Message catalog

### 7.1 Downlink (Controller → Robot)

| Code | Name | TLV tags |
|-----:|------|----------|
| `0x01` | `ConnectReq` | `1=version u16`, `2=device_id u32`, `3=nonce u32` |
| `0x02` | `Disconnect` | — |
| `0x03` | `Heartbeat` | (controller → robot, empty) |
| `0x10` | `SetVelocity` | `1=vx f32`, `2=vy f32`, `3=wz f32` (m/s, rad/s) |
| `0x11` | `MoveOrder` | `1=dir u8`, `2=speed u8`, `3=duration_ms u16` |
| `0x12` | `Stop` | — |
| `0x13` | `EmergencyStop` | — |
| `0x14` | `SetArmJoints` | `1=arm_id u8`, `2=count u8`, `3=joints f32[]` (rad) |
| `0x15` | `SetArmPose` | `1=arm_id u8`, `2..7 = x,y,z,roll,pitch,yaw f32` |
| `0x16` | `SetArmMode` | `1=arm_id u8`, `2=mode u8` |
| `0x17` | `SetParam` | `1=param_id u16`, `2=value f32` |

`MoveOrder.dir`: `0` forward, `1` backward, `2` strafe-left, `3` strafe-right,
`4` rotate-left, `5` rotate-right, `6` stop. `speed` is `0..255`.

### 7.2 Uplink (Robot → Controller)

| Code | Name | TLV tags |
|-----:|------|----------|
| `0x81` | `ConnectAck` | `1=session_id u32`, `2=status u8`, `3=capabilities u16` |
| `0x82` | `Heartbeat` | (robot → controller, empty) |
| `0x83` | `TelemetryBase` | `1=x`, `2=y`, `3=yaw`, `4=vx`, `5=vy`, `6=wz` (f32), `7=battery u8` |
| `0x84` | `TelemetryArm` | `1=arm_id u8`, `2=count u8`, `3=joints f32[]` |
| `0x85` | `Status` | `1=mode u8`, `2=faults u16`, `3=estop u8` |
| `0x86` | `Ack` | `1=ack_seq u16`, `2=status u8` |
| `0x87` | `Nack` | `1=ack_seq u16`, `2=reason u8` |
| `0x88` | `EventLog` | `1=level u8`, `2=text bytes` |

---

## 8. Reliability

### 8.1 Downlink — ACK + retransmit (ARQ)

1. Sender assigns `Seq`, sets `ACK_REQ | PAYLOAD_CRC`, stores the frame in a
   pending map, transmits.
2. Receiver validates, **de-duplicates by `Seq`**, executes **once**, and
   replies `Ack` with `AckSeq = Seq`.
3. Sender removes the frame from pending on ACK.
4. If no ACK within **RTO = 30 ms**, the frame is retransmitted (flag
   `RETRANSMIT`) up to **5** times, then dropped and reported.

The controller limits in-flight velocity setpoints (bounded window, default 8)
because setpoints are **latching** on the robot: a skipped/stale setpoint is
harmless, an unbounded backlog is not.

### 8.2 Uplink — FEC only

Telemetry is periodic and idempotent, so it is sent **without** `ACK_REQ` and
carries no payload CRC by default (WFB-ng's FEC + link CRC protect it). Lost
samples are simply superseded by the next one.

### 8.3 Duplicate suppression

Per stream, a 64-bit sliding window tracks recently seen sequence numbers
(`seq` and `AckSeq` are compared modulo 65536). Duplicates are re-ACKed but
never re-dispatched.

---

## 9. Fragmentation

`FRAG_START` / `FRAG_MORE` support messages larger than the MTU. Each fragment
carries the same `Seq`; the receiver reassembles per stream. (Defined; used only
when a payload exceeds the link MTU.)

---

## 10. Session & timing

State machine: `Idle → Connecting → Active ⇄ Lost`.

- **Handshake** — the controller sends `ConnectReq` (protocol version, device
  id, nonce) and repeats it every heartbeat interval while `Connecting`. The
  robot answers `ConnectAck` with a `session_id` (the request nonce in v1) and
  enters `Active`; the controller enters `Active` on a `ConnectAck` with
  `status == 0`. Handshake frames are best-effort; the periodic repeat makes
  loss self-healing.
- **Heartbeat** — once `Active`, each side sends an empty heartbeat every
  **1 s** (controller `0x03`, robot `0x82`). Any valid frame refreshes the link.
- **Link loss** — if no valid frame arrives for **3 s** while `Active`, the
  endpoint transitions to `Lost` and notifies via the state callback; the next
  valid frame returns it to `Active`.
- **Failsafe on the robot** — if no valid command is received within **500 ms**,
  velocity is forced to zero. `EmergencyStop` latches regardless of traffic.
- Uplink telemetry: base 200 Hz, arm joints 50 Hz (round-robin), status 1 Hz.
- The controller only transmits commands while `Active`.

---

## 11. Real-time notes (Linux)

- `UdpTransport` sockets are non-blocking with `poll()`; the RX/control loop
  never blocks on I/O.
- The robot process attempts `SCHED_FIFO` priority 50 (`pthread_setschedparam`);
  run as root (or grant `CAP_SYS_NICE`) for guaranteed real-time scheduling.
- Hot paths use fixed-size stack buffers; no allocation per frame.
- Recommended loop cadence: 2 ms poll tick, which comfortably covers 200 Hz
  (5 ms) uplink.

---

## 12. Omnidirectional base kinematics

Mecanum inverse kinematics (`link::mecanumInverse`), with `vx` forward, `vy`
left, `wz` yaw rate, `k = half_length + half_width`, wheel radius `r`:

```
w_fl = ( vx - vy - k*wz) / r
w_fr = ( vx + vy + k*wz) / r
w_rl = ( vx + vy - k*wz) / r
w_rr = ( vx - vy + k*wz) / r
```

Joint targets are clamped to per-joint `[min, max]` before being applied.

---

## 13. Constants

| Name | Value |
|------|-------|
| Magic | `0x52 0x4C` |
| Version | `1` |
| Header size | 16 bytes |
| Max payload | 1024 bytes |
| CRC-16 poly | `0x1021`, init `0xFFFF` |
| RTO | 30 ms |
| Max retries | 5 |
| Failsafe timeout | 500 ms |
| Heartbeat interval | 1 s |
| Link timeout | 3 s |
| Downlink rate | 100 Hz |
| Uplink rate | 200 Hz |

---

## 14. Open items

- Session authentication / encryption (keys, replay protection) — `nonce` and
  `capabilities` fields are reserved for this.
- Fragment reassembly strategy under severe loss (`FRAG_START`/`FRAG_MORE`).
- Multi-robot addressing (currently one C ↔ one D).
- Multi-robot addressing (currently one C ↔ one D).
