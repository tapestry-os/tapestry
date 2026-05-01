"""
Tapestry Choreography SDK — Python package.

Provides the L7 choreography interface (Choreo) backed by the L6
BSE stub (BSEStub).  Import and use:

    from tapestry.choreo import Choreo, Goal, GoalType

See sdk/examples/hello_swarm.py for a worked example.
"""

from tapestry.choreo import Choreo, Goal, GoalType, GoalShape, GoalStatus

__all__ = ["Choreo", "Goal", "GoalType", "GoalShape", "GoalStatus"]
