"""
broker.py — Gossip broker and orchestrator network state for L5 SCR simulation.

Functionally identical to tapestry-csm-sim/orchestrator/broker.py for L4
traffic (gossip routing, partition control, position error injection).

L5 addition: also receives SIM_MSG_SCR_METRIC datagrams (type=4) and
dispatches them to scr_metric_cb.

The telemetry writer receives both L4 and L5 metrics and combines them
into a single row per element per cycle.
"""

import asyncio
import logging
import math

from protocol import (
    ORCH_PORT, ELEMENT_BASE_PORT, LOOPBACK,
    CTRL_SET_PARTITION, CTRL_SET_POWER,
    decode, encode_gossip, encode_control,
)

log = logging.getLogger(__name__)


def _dist(a: dict, b: dict) -> float:
    dx = a['x'] - b['x']
    dy = a['y'] - b['y']
    return math.sqrt(dx * dx + dy * dy)


class _Protocol(asyncio.DatagramProtocol):
    def __init__(self, broker: 'GossipBroker'):
        self._broker = broker

    def connection_made(self, transport):
        self._broker._transport = transport

    def datagram_received(self, data: bytes, addr):
        self._broker._on_receive(data, addr)

    def error_received(self, exc):
        log.warning("UDP error: %s", exc)

    def connection_lost(self, exc):
        if exc:
            log.error("connection lost: %s", exc)


class GossipBroker:
    """
    Asyncio UDP gossip broker for the SCR simulation.

    Usage:
        broker = GossipBroker(n_elements=5,
                              metric_cb=on_l4_metric,
                              scr_metric_cb=on_scr_metric)
        await broker.start()
        broker.set_partition([[0,1,2],[3,4]])
        ...
        broker.stop()
    """

    def __init__(self, n_elements: int,
                 metric_cb=None,
                 scr_metric_cb=None):
        self._n             = n_elements
        self._metric_cb     = metric_cb
        self._scr_metric_cb = scr_metric_cb
        self._transport     = None
        self._islands       = {i: 0 for i in range(n_elements)}
        self._metrics       = {}
        self._scr_metrics   = {}

        self._ground_truth: dict[int, dict] = {}
        self._last_seen: dict[int, dict[int, dict]] = {
            i: {} for i in range(n_elements)
        }

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    async def start(self):
        loop = asyncio.get_event_loop()
        await loop.create_datagram_endpoint(
            lambda: _Protocol(self),
            local_addr=(LOOPBACK, ORCH_PORT),
        )
        log.info("broker listening on %s:%d", LOOPBACK, ORCH_PORT)

    def stop(self):
        if self._transport:
            self._transport.close()

    # ── Receive ───────────────────────────────────────────────────────────────

    def _on_receive(self, data: bytes, addr):
        msg = decode(data)
        if msg is None:
            return
        if msg['type'] == 'gossip':
            self._route_gossip(msg)
        elif msg['type'] == 'metric':
            self._handle_metric(msg)
        elif msg['type'] == 'scr_metric':
            self._handle_scr_metric(msg)

    def _route_gossip(self, msg: dict):
        sender_id     = msg['src_id']
        sender_island = self._islands.get(sender_id, 0)
        pos           = {'x': msg['x'], 'y': msg['y']}

        self._ground_truth[sender_id] = pos
        raw = encode_gossip(msg)

        for elem_id in range(self._n):
            if elem_id == sender_id:
                continue
            if self._islands.get(elem_id, 0) != sender_island:
                continue
            self._transport.sendto(raw, (LOOPBACK, ELEMENT_BASE_PORT + elem_id))
            self._last_seen[elem_id][sender_id] = pos

    def _handle_metric(self, msg: dict):
        observer_id = msg['element_id']
        msg['mean_position_error'] = self._compute_position_error(observer_id)
        self._metrics[observer_id] = msg
        if self._metric_cb:
            asyncio.ensure_future(self._metric_cb(msg))

    def _handle_scr_metric(self, msg: dict):
        elem_id = msg['element_id']
        self._scr_metrics[elem_id] = msg
        if self._scr_metric_cb:
            asyncio.ensure_future(self._scr_metric_cb(msg))

    # ── Position error ────────────────────────────────────────────────────────

    def _compute_position_error(self, observer_id: int) -> float:
        beliefs     = self._last_seen.get(observer_id, {})
        total_error = 0.0
        n_peers     = self._n - 1

        for peer_id in range(self._n):
            if peer_id == observer_id:
                continue
            believed_pos = beliefs.get(peer_id)
            actual       = self._ground_truth.get(peer_id)
            if believed_pos is None or actual is None:
                continue
            total_error += _dist(believed_pos, actual)

        return (total_error / n_peers) if n_peers > 0 else 0.0

    # ── Partition control ─────────────────────────────────────────────────────

    def set_partition(self, islands: list[list[int]]):
        for island_id, group in enumerate(islands):
            for elem_id in group:
                if elem_id >= self._n:
                    continue
                self._islands[elem_id] = island_id
                self._send_ctrl(elem_id, CTRL_SET_PARTITION, island_id)
        log.info("partition applied: %s", islands)

    def heal_partition(self):
        for elem_id in range(self._n):
            self._islands[elem_id] = 0
            self._send_ctrl(elem_id, CTRL_SET_PARTITION, 0)
        log.info("partition healed — all in island 0")

    def set_power(self, elem_id: int, power_state: int):
        self._send_ctrl(elem_id, CTRL_SET_POWER, power_state)
        log.info("element %d power → %d", elem_id, power_state)

    def _send_ctrl(self, elem_id: int, ctrl_type: int, value: int):
        if self._transport is None:
            log.warning("broker not started — cannot send control")
            return
        data = encode_control(0, ctrl_type, value)
        self._transport.sendto(data, (LOOPBACK, ELEMENT_BASE_PORT + elem_id))

    # ── Inspection ────────────────────────────────────────────────────────────

    @property
    def metrics(self) -> dict:
        return dict(self._metrics)

    @property
    def scr_metrics(self) -> dict:
        return dict(self._scr_metrics)

    @property
    def islands(self) -> dict:
        return dict(self._islands)
