#!/usr/bin/env python3
"""Emulated radio channel: packet loss + optional block FEC + wire statistics.

Sits between two UDP peers and forwards datagrams in both directions while
injecting Bernoulli loss. Optionally it applies a systematic Reed-Solomon block
FEC (see fec.py) to the edge0 -> edge1 direction, emulating what WFB-ng does
below the RCLP transport, and reports the FEC recovery rate.

Topology (edge0 is the "tx"/robot side, edge1 is the "rx"/controller side):

    edge0  --(0->1)-->  [ loss +/- FEC ]  --(0->1)-->  edge1
    edge0  <--(1->0)--  [     loss     ]  <--(1->0)--  edge1

Peers are learned from the first datagram received on each edge, or provided
explicitly with --peer0/--peer1. Writes a JSON statistics file on exit.
"""
import argparse
import json
import os
import random
import select
import signal
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fec import RSBlock  # noqa: E402

MAGIC = b"RL"
FLAG_ACK_REQ = 1 << 0
FLAG_IS_ACK = 1 << 1
FLAG_IS_NACK = 1 << 2
FLAG_RETRANSMIT = 1 << 3

_running = True


def _on_signal(_sig, _frm):
    global _running
    _running = False


def parse_header(b):
    if len(b) < 16 or b[0:2] != MAGIC:
        return None
    return {
        "version": b[2],
        "msg_type": b[3],
        "flags": b[4],
        "stream": b[5],
        "seq": struct.unpack_from("<H", b, 6)[0],
        "ack": struct.unpack_from("<H", b, 8)[0],
        "plen": struct.unpack_from("<H", b, 10)[0],
    }


def parse_addr(s):
    if not s:
        return None
    host, _, port = s.rpartition(":")
    return (host, int(port))


def _dir_stats():
    return {
        "in": 0,
        "out": 0,
        "dropped": 0,
        "retransmit": 0,
        "acks": 0,
        "nacks": 0,
        "dup_frames": 0,
        "streams": {},          # stream id -> count
        "fec_blocks": 0,
        "fec_data_frames": 0,
        "fec_data_dropped": 0,
        "fec_data_recovered": 0,
        "fec_residual_lost": 0,
        "fec_parity_sent": 0,
        "fec_parity_dropped": 0,
    }


class Channel:
    def __init__(self, args):
        self.args = args
        self.start = time.time()
        self.rng = random.Random(args.seed)

        self.e0 = self._bind(args.edge0)
        self.e1 = self._bind(args.edge1)
        self.peer0 = parse_addr(args.peer0)
        self.peer1 = parse_addr(args.peer1)

        self.k = args.fec_k
        self.m = args.fec_m
        self.rs = RSBlock(self.k, self.m) if (self.k > 0 and self.m > 0) else None
        self.fec_buf = []

        self.stats = {"0->1": _dir_stats(), "1->0": _dir_stats()}
        self.seen_seq = {"0->1": set(), "1->0": set()}

    @staticmethod
    def _bind(port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        s.setblocking(False)
        return s

    def _record(self, direction, data):
        st = self.stats[direction]
        st["in"] += 1
        h = parse_header(data)
        if not h:
            return
        st["streams"][str(h["stream"])] = st["streams"].get(str(h["stream"]), 0) + 1
        if h["flags"] & FLAG_RETRANSMIT:
            st["retransmit"] += 1
        if h["flags"] & FLAG_IS_ACK:
            st["acks"] += 1
        if h["flags"] & FLAG_IS_NACK:
            st["nacks"] += 1
        if h["stream"] == 0:  # control stream: track duplicates on the wire
            if h["seq"] in self.seen_seq[direction]:
                st["dup_frames"] += 1
            else:
                self.seen_seq[direction].add(h["seq"])

    def _drop_now(self, src_edge):
        if self.args.drop_after is None:
            return False
        if time.time() - self.start < self.args.drop_after:
            return False
        return src_edge == self.args.drop_edge

    def _forward(self, sock, peer, data, direction):
        if peer is None:
            return
        try:
            sock.sendto(data, peer)
            self.stats[direction]["out"] += 1
        except OSError:
            pass

    def on_edge0(self, data):
        self._record("0->1", data)
        if self._drop_now(0):
            self.stats["0->1"]["dropped"] += 1
            return
        if self.rs:
            self.fec_buf.append(data)
            if len(self.fec_buf) >= self.k:
                self._flush_fec()
        else:
            if self.rng.random() < self.args.loss0:
                self.stats["0->1"]["dropped"] += 1
                return
            self._forward(self.e1, self.peer1, data, "0->1")

    def on_edge1(self, data):
        self._record("1->0", data)
        if self._drop_now(1):
            self.stats["1->0"]["dropped"] += 1
            return
        if self.rng.random() < self.args.loss1:
            self.stats["1->0"]["dropped"] += 1
            return
        self._forward(self.e0, self.peer0, data, "1->0")

    def _flush_fec(self):
        k, m, st = self.k, self.m, self.stats["0->1"]
        frames = self.fec_buf
        self.fec_buf = []

        blocks = [struct.pack("<H", len(f)) + f for f in frames]
        length = max(len(b) for b in blocks)
        blocks = [b.ljust(length, b"\x00") for b in blocks]
        parity = self.rs.parity(blocks)

        wire = [(i, blocks[i]) for i in range(k)] + [(k + r, parity[r]) for r in range(m)]
        received, dropped_data, dropped_parity = {}, 0, 0
        for idx, block in wire:
            if self.rng.random() < self.args.loss0:
                if idx < k:
                    dropped_data += 1
                else:
                    dropped_parity += 1
            else:
                received[idx] = block

        st["fec_blocks"] += 1
        st["fec_data_frames"] += k
        st["fec_data_dropped"] += dropped_data
        st["fec_parity_sent"] += m
        st["fec_parity_dropped"] += dropped_parity

        delivered = [None] * k
        if len(received) >= k:
            data = self.rs.recover(received)
            if data is not None:
                delivered = data
                st["fec_data_recovered"] += dropped_data
        else:
            for i in range(k):
                if i in received:
                    delivered[i] = received[i]

        for i in range(k):
            if delivered[i] is None:
                st["fec_residual_lost"] += 1
                continue
            content = delivered[i]
            (flen,) = struct.unpack_from("<H", content, 0)
            self._forward(self.e1, self.peer1, content[2:2 + flen], "0->1")

    def run(self):
        duration = self.args.duration
        while _running:
            if duration and (time.time() - self.start) >= duration:
                break
            r, _, _ = select.select([self.e0, self.e1], [], [], 0.02)
            for s in r:
                try:
                    data, addr = s.recvfrom(65535)
                except OSError:
                    continue
                if s is self.e0:
                    self.peer0 = addr
                    self.on_edge0(data)
                else:
                    self.peer1 = addr
                    self.on_edge1(data)

    def summary(self):
        out = {"peer0": self.peer0, "peer1": self.peer1, "directions": self.stats}
        d = self.stats["0->1"]
        raw_loss = (d["dropped"] / d["in"]) if d["in"] else 0.0
        out["raw_loss_0to1"] = raw_loss
        if self.rs and d["fec_data_frames"]:
            out["fec"] = {
                "k": self.k,
                "m": self.m,
                "recovery_rate": (d["fec_data_recovered"] / d["fec_data_dropped"])
                if d["fec_data_dropped"] else 1.0,
                "residual_per": d["fec_residual_lost"] / d["fec_data_frames"],
                "overhead": (self.k + self.m) / self.k,
            }
        return out


def main():
    p = argparse.ArgumentParser(description="lossy UDP channel with FEC")
    p.add_argument("--edge0", type=int, required=True)
    p.add_argument("--edge1", type=int, required=True)
    p.add_argument("--peer0", default=None, help="host:port override for edge0 peer")
    p.add_argument("--peer1", default=None, help="host:port override for edge1 peer")
    p.add_argument("--loss0", type=float, default=0.0, help="loss on 0->1 direction")
    p.add_argument("--loss1", type=float, default=0.0, help="loss on 1->0 direction")
    p.add_argument("--fec-k", type=int, default=0)
    p.add_argument("--fec-m", type=int, default=0)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--duration", type=float, default=6.0)
    p.add_argument("--drop-after", type=float, default=None)
    p.add_argument("--drop-edge", type=int, default=1, choices=[0, 1])
    p.add_argument("--stats", default=None)
    args = p.parse_args()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    ch = Channel(args)
    ch.run()
    result = ch.summary()
    text = json.dumps(result)
    if args.stats:
        with open(args.stats, "w") as f:
            f.write(text)
    print(text)


if __name__ == "__main__":
    main()
