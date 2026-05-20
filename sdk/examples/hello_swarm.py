#!/usr/bin/env python3
"""
hello_swarm.py — minimal Tapestry application example.

Demonstrates L7 SDK usage without a running simulation: submit a FORM
goal to a 4-element swarm snapshot and print each element's resulting
directive.

Run from any directory:
    python sdk/examples/hello_swarm.py
"""

import os
import sys

# Resolve SDK and BSE stub from repo root.
_HERE      = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))
sys.path.insert(0, os.path.join(_REPO_ROOT, 'sdk', 'python'))

from tapestry.choreo import Choreo, Goal, GoalType, GoalShape
from tapestry.bse import BSEDirectiveType

# ── Simulated world snapshot ──────────────────────────────────────────────────
# Four elements scattered across the 100×100 logical arena.

ELEMENTS = [
    {'id': 0, 'x': 10.0, 'y': 10.0, 'is_active': True, 'is_stale': False, 'is_self': False},
    {'id': 1, 'x': 90.0, 'y': 10.0, 'is_active': True, 'is_stale': False, 'is_self': False},
    {'id': 2, 'x': 90.0, 'y': 90.0, 'is_active': True, 'is_stale': False, 'is_self': False},
    {'id': 3, 'x': 10.0, 'y': 90.0, 'is_active': True, 'is_stale': False, 'is_self': False},
]

SCR_STATE = {
    'role': 1,           # FOLLOWER
    'quorum_state': 2,   # HEALTHY
    'leader_id': 0,
}

# ── Goal ──────────────────────────────────────────────────────────────────────

GOAL = Goal(
    type   = GoalType.FORM,
    target = (50.0, 50.0),
    radius = 30.0,
    shape  = GoalShape.CIRCLE,
)

# ── Run ───────────────────────────────────────────────────────────────────────

print("Tapestry SDK — hello_swarm")
print(f"Goal : {GOAL.type.name}  centre=(50, 50)  radius={GOAL.radius} logical units")
print(f"Arena: 100×100 logical units   Elements: {len(ELEMENTS)}")
print()
print(f"{'Element':>8}  {'Directive':>18}  {'Target':>20}")
print("-" * 52)

for elem in ELEMENTS:
    self_id = elem['id']
    peers   = [e for e in ELEMENTS if e['id'] != self_id]

    choreo = Choreo(element_id=self_id)
    choreo.submit_goal(GOAL)
    choreo.tick(peers, SCR_STATE)

    d = choreo.get_directive()
    if d.type == BSEDirectiveType.MOVE_TO_POINT:
        target_str = f"({d.target[0]:5.1f}, {d.target[1]:5.1f})"
    else:
        target_str = "—"

    print(f"{self_id:>8}  {d.type.name:>18}  {target_str:>20}")

print()
print("Note: currently, target positions are geometry-only (stub); no path planning.")
