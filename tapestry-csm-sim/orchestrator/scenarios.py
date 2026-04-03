"""
scenarios.py — Timed partition / power injection scripts.

A scenario is an async generator that yields (delay_s, action_fn, kwargs)
tuples.  run() drives the generator, sleeping between events, then idles
until the requested duration elapses.

Power state constants mirror power_state_t in state.h:
    POWER_ACTIVE = 0   sensing, gossiping, updating position
    POWER_IDLE   = 1   gossiping but not updating position
    POWER_SLEEP  = 2   not gossiping; last state persists until expired

Built-in scenarios
──────────────────
  default    One clean split + heal: partition at 5 s, heal at 15 s.
  flapping   Five rapid partition/heal cycles (2 s each) to stress
             reconciliation and test convergence speed.
  sleep      Element 0 sleeps at 5 s (stops gossiping) and wakes at 12 s;
             tests staleness-driven world model expiry and recovery.
  asymmetric Three-way split: [0], [1,2], [3,4] at 5 s, partial heal
             ([0] rejoins [1,2]) at 12 s, full heal at 20 s.
"""

import asyncio
import logging

log = logging.getLogger(__name__)

POWER_ACTIVE = 0
POWER_IDLE   = 1
POWER_SLEEP  = 2

# ── Registry ──────────────────────────────────────────────────────────────────

_REGISTRY: dict = {}

def scenario(name: str):
    def decorator(fn):
        _REGISTRY[name] = fn
        return fn
    return decorator

def available() -> list[str]:
    return list(_REGISTRY.keys())

# ── Runner ────────────────────────────────────────────────────────────────────

async def run(name: str, broker, n_elements: int, duration_s: float):
    """
    Execute scenario `name`, then idle until `duration_s` from start.
    Logs each event as it fires.
    """
    if name not in _REGISTRY:
        raise ValueError(f"Unknown scenario '{name}'. "
                         f"Available: {available()}")

    start   = asyncio.get_event_loop().time()
    gen     = _REGISTRY[name](broker, n_elements)

    async for delay_s, action, kwargs in gen:
        await asyncio.sleep(delay_s)
        log.info("scenario '%s': %s(%s)", name, action.__name__,
                 ', '.join(f'{k}={v}' for k, v in kwargs.items()))
        action(broker, **kwargs)

    elapsed   = asyncio.get_event_loop().time() - start
    remaining = duration_s - elapsed
    if remaining > 0:
        await asyncio.sleep(remaining)

# ── Built-in scenarios ────────────────────────────────────────────────────────

@scenario('default')
async def _default(broker, n_elements: int):
    """Single partition at 5 s; heal at 15 s."""
    half = n_elements // 2
    yield 5.0,  _partition, {'islands': [list(range(half)),
                                          list(range(half, n_elements))]}
    yield 10.0, _heal,      {}


@scenario('flapping')
async def _flapping(broker, n_elements: int):
    """Five partition / heal cycles, 2 s apart."""
    half = n_elements // 2
    islands = [list(range(half)), list(range(half, n_elements))]
    for _ in range(5):
        yield 2.0, _partition, {'islands': islands}
        yield 2.0, _heal,      {}


@scenario('sleep')
async def _sleep(broker, n_elements: int):
    """Element 0 sleeps at 5 s, wakes at 12 s."""
    yield 5.0, _set_power, {'elem_id': 0, 'power_state': POWER_SLEEP}
    yield 7.0, _set_power, {'elem_id': 0, 'power_state': POWER_ACTIVE}


@scenario('asymmetric')
async def _asymmetric(broker, n_elements: int):
    """
    Three-way split at 5 s: [0] | [1,2] | [3,4].
    Partial heal at 12 s: [0,1,2] | [3,4].
    Full heal at 20 s.
    """
    if n_elements < 5:
        yield 1.0, _heal, {}   # not enough elements, just heal immediately
        return

    yield 5.0,  _partition, {'islands': [[0], [1, 2], [3, 4]]}
    yield 7.0,  _partition, {'islands': [[0, 1, 2], [3, 4]]}
    yield 8.0,  _heal,      {}

# ── Action helpers ────────────────────────────────────────────────────────────

def _partition(broker, *, islands):
    broker.set_partition(islands)

def _heal(broker, **_):
    broker.heal_partition()

def _set_power(broker, *, elem_id, power_state):
    broker.set_power(elem_id, power_state)
