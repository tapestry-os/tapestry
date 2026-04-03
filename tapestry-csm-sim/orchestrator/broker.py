"""
broker.py — Gossip broker and orchestrator network state.

Listens on ORCH_PORT for all element traffic (gossip + metrics).
Forwards gossip only to elements in the same partition island.
Sends control messages to individual elements on their own ports.
"""

import asyncio
import logging

from protocol import (
    ORCH_PORT, ELEMENT_BASE_PORT, LOOPBACK,
    CTRL_SET_PARTITION, CTRL_SET_POWER,
    decode, encode_gossip, encode_control,
)

log = logging.getLogger(__name__)


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
    Asyncio UDP gossip broker.

    Usage:
        broker = GossipBroker(n_elements=5, metric_cb=my_async_fn)
        await broker.start()
        broker.set_partition([[0,1,2],[3,4]])
        ...
        broker.stop()

    metric_cb, if provided, is called as `await metric_cb(metric_dict)`
    for every metric message received from any element.
    """

    def __init__(self, n_elements: int, metric_cb=None):
        self._n          = n_elements
        self._metric_cb  = metric_cb
        self._transport  = None
        # island assignment: element_id → island_id (all start in island 0)
        self._islands    = {i: 0 for i in range(n_elements)}
        self._metrics    = {}   # element_id → most recent metric dict

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

    def _route_gossip(self, msg: dict):
        """Forward gossip to every element in the same partition island."""
        sender_id     = msg['src_id']
        sender_island = self._islands.get(sender_id, 0)
        raw           = encode_gossip(msg)

        for elem_id in range(self._n):
            if elem_id == sender_id:
                continue
            if self._islands.get(elem_id, 0) != sender_island:
                continue   # partitioned — drop
            self._transport.sendto(raw, (LOOPBACK, ELEMENT_BASE_PORT + elem_id))

    def _handle_metric(self, msg: dict):
        self._metrics[msg['element_id']] = msg
        if self._metric_cb:
            asyncio.ensure_future(self._metric_cb(msg))

    # ── Partition control ────────────────────────────────────────────────────

    def set_partition(self, islands: list[list[int]]):
        """
        Assign elements to partition islands.

        Example: set_partition([[0,1,2],[3,4]])
            → elements 0,1,2 form island 0; elements 3,4 form island 1.
        """
        for island_id, group in enumerate(islands):
            for elem_id in group:
                if elem_id >= self._n:
                    continue
                self._islands[elem_id] = island_id
                self._send_ctrl(elem_id, CTRL_SET_PARTITION, island_id)
        log.info("partition applied: %s", islands)

    def heal_partition(self):
        """Merge all elements back into island 0."""
        for elem_id in range(self._n):
            self._islands[elem_id] = 0
            self._send_ctrl(elem_id, CTRL_SET_PARTITION, 0)
        log.info("partition healed — all in island 0")

    def set_power(self, elem_id: int, power_state: int):
        """Send a power-state control message to one element."""
        self._send_ctrl(elem_id, CTRL_SET_POWER, power_state)
        log.info("element %d power → %d", elem_id, power_state)

    def _send_ctrl(self, elem_id: int, ctrl_type: int, value: int):
        if self._transport is None:
            log.warning("broker not started — cannot send control")
            return
        data = encode_control(0, ctrl_type, value)
        self._transport.sendto(data, (LOOPBACK, ELEMENT_BASE_PORT + elem_id))

    # ── Inspection ───────────────────────────────────────────────────────────

    @property
    def metrics(self) -> dict:
        """Most recent metric snapshot keyed by element_id."""
        return dict(self._metrics)

    @property
    def islands(self) -> dict:
        """Current island assignment keyed by element_id."""
        return dict(self._islands)
