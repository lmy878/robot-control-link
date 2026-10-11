# omni-arm-link

Real-time control link for an **omnidirectional base + 4 arms** robot over a
**WFB-ng** radio: reliable downlink commands (ACK + retransmit), best-effort
uplink telemetry (FEC-only), hand-rolled binary framing, Linux C++17.

See [`docs/PROTOCOL.md`](docs/PROTOCOL.md) for the full protocol specification.

## Layout

```
include/link/   protocol headers (common, crc, frame, tlv, messages,
                transport, endpoint, kinematics)
src/            implementation
apps/           sender (controller) and receiver (robot) demo programs
tests/          unit + ARQ tests
docs/           PROTOCOL.md
```

## Build

```sh
make            # builds build/sender, build/receiver, build/test_link
make test       # runs the unit + ARQ tests
```

Requires only `g++` (C++17) and `make`.

## Run (loopback, no radio)

`sender` binds 5601 and targets 5600; `receiver` binds 5600 and targets 5601.

```sh
./build/receiver &            # robot side
./build/sender                # controller side, Ctrl-C to stop
```

Expected: the robot logs decoded `CMD move ...` orders and wheel speeds; the
controller logs `TELEM ...` and `STATUS ...`, with `acked` counting commands and
`pending` staying near zero.

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
