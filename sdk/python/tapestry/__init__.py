"""
Tapestry Choreographer SDK — Python package.

Provides the L7 Choreographer interface (Choreo) backed by the L6
Behavior Synthesis Engine (BSE).  Import and use:

    from tapestry.choreo import Choreo, Goal, GoalType

See sdk/examples/hello_swarm.py for a worked example.
"""

from tapestry.choreo import (Choreo, Goal, GoalType, GoalShape,
                              ChoreoState, ChoreoCapabilities)

__all__ = ["Choreo", "Goal", "GoalType", "GoalShape",
           "ChoreoState", "ChoreoCapabilities"]
