"""
scenarios.py — Timed partition / power injection scripts for the SCR simulation.

Same runner contract as tapestry-csm-sim/orchestrator/scenarios.py:
each scenario is an async generator yielding (delay_s, action_fn, kwargs).

L4 scenarios (default, flapping, asymmetric, sleep) are carried over
unchanged — they remain valid for observing SCR behaviour under the same
network conditions.

L5-specific scenarios
─────────────────────
  leader_loss    Partition element 0 (always the initial leader — lowest ID)
                 at 5 s; heal at 15 s.  Measures: re-election convergence
                 time, leader_agreement during and after the partition.

  cascade        Element 0 partitioned at 5 s (element 1 becomes leader).
                 Element 1 then partitioned at 12 s (element 2 becomes leader).
                 Full heal at 20 s.  Measures: sequential re-elections,
                 whether the fleet converges on a single leader after each step.

Power state constants:
    POWER_ACTIVE = 0
    POWER_IDLE   = 1
    POWER_SLEEP  = 2
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
    if name not in _REGISTRY:
        raise ValueError(f"Unknown scenario '{name}'. "
                         f"Available: {available()}")

    start = asyncio.get_event_loop().time()
    gen   = _REGISTRY[name](broker, n_elements)

    async for delay_s, action, kwargs in gen:
        await asyncio.sleep(delay_s)
        log.info("scenario '%s': %s(%s)", name, action.__name__,
                 ', '.join(f'{k}={v}' for k, v in kwargs.items()))
        action(broker, **kwargs)

    elapsed   = asyncio.get_event_loop().time() - start
    remaining = duration_s - elapsed
    if remaining > 0:
        await asyncio.sleep(remaining)

# ── L4 scenarios (carried over) ───────────────────────────────────────────────

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
    half    = n_elements // 2
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
    """Three-way split at 5 s; partial heal at 12 s; full heal at 20 s."""
    if n_elements < 5:
        yield 1.0, _heal, {}
        return
    yield 5.0, _partition, {'islands': [[0], [1, 2], [3, 4]]}
    yield 7.0, _partition, {'islands': [[0, 1, 2], [3, 4]]}
    yield 8.0, _heal,      {}

# ── L5 scenarios ──────────────────────────────────────────────────────────────

@scenario('leader_loss')
async def _leader_loss(broker, n_elements: int):
    """
    Partition element 0 (initial leader — lowest ID) from the rest of the
    swarm at 5 s.  The remaining elements must elect a new leader from
    {1 .. n-1}.  Heal at 15 s; original leader 0 should reclaim leadership.

    Key observations:
      - How quickly does leader_id converge to 1 after the partition?
      - Does leader_agreement drop to ~(n-1)/n during the partition (the
        partitioned element still believes it is leader)?
      - After heal, how quickly does element 0 reclaim leadership?
    """
    if n_elements < 2:
        yield 1.0, _heal, {}
        return

    # Isolate element 0 (leader) from all others
    others = list(range(1, n_elements))
    yield 5.0,  _partition, {'islands': [[0], others]}
    yield 10.0, _heal,      {}


@scenario('cascade')
async def _cascade(broker, n_elements: int):
    """
    Sequential leader elimination, stress-testing re-election:

      t= 5 s  Partition element 0 → element 1 becomes leader
      t=12 s  Partition elements 0 and 1 → element 2 becomes leader
      t=20 s  Full heal → element 0 reclaims leadership

    Requires >= 3 elements; falls back to leader_loss with 2.

    Key observations:
      - Each partition wave triggers a fresh election.
      - After full heal, all elements should converge on element 0.
      - election_count shows how many leadership changes occurred.
    """
    if n_elements < 3:
        # Fall back: just do a simple leader_loss
        others = list(range(1, n_elements))
        yield 5.0,  _partition, {'islands': [[0], others]}
        yield 10.0, _heal,      {}
        return

    others_after_first  = list(range(1, n_elements))
    others_after_second = list(range(2, n_elements))

    yield 5.0,  _partition, {'islands': [[0], others_after_first]}
    yield 7.0,  _partition, {'islands': [[0, 1], others_after_second]}
    yield 8.0,  _heal,      {}

# ── Action helpers ────────────────────────────────────────────────────────────

def _partition(broker, *, islands):
    broker.set_partition(islands)

def _heal(broker, **_):
    broker.heal_partition()

def _set_power(broker, *, elem_id, power_state):
    broker.set_power(elem_id, power_state)
