"""
Tapestry Application SDK — Python package.

Provides the L7 application interface (TapestryApp) backed by the L6
BSE stub (BSEStub).  Import and use:

    from tapestry.app import TapestryApp, Goal, GoalType

See sdk/examples/hello_swarm.py for a worked example.
"""

from tapestry.app import TapestryApp, Goal, GoalType, GoalShape, GoalStatus

__all__ = ["TapestryApp", "Goal", "GoalType", "GoalShape", "GoalStatus"]
