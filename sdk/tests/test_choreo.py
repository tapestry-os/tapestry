"""
test_choreo.py — L7 Choreographer SDK (sdk/python/tapestry/choreo.py).

Covers the five-stage lifecycle, the capability gate, script validation and
advance rules, quorum suspension (including the per-goal exception that
keeps HOLD ticking), and the scope="all" collective predicate.  These are
the guarantees an application written against the SDK relies on, and the
mirror of tapestry-os/subsys/choreo/choreo.c.
"""

import errno

import pytest
from helpers import QUORUM_DEGRADED, QUORUM_HEALTHY, QUORUM_LOST, scr, solo, wm

from tapestry.bse import BSEDirectiveType
from tapestry.choreo import (Choreo, ChoreoCapabilities, ChoreoScope,
                             ChoreoState, ChoreoStep, Goal, GoalShape,
                             GoalType)

HEALTHY  = scr(QUORUM_HEALTHY)
DEGRADED = scr(QUORUM_DEGRADED)
LOST     = scr(QUORUM_LOST)

# SCR_CAP_* hardware bits (scr.h), mirrored privately by choreo.py.
CAP_RELAY, CAP_SENSOR, CAP_ACTUATOR = 0x01, 0x02, 0x04


def timed(goal_type=GoalType.CONVERGE, ms=300, **kw):
    """A step that can only advance on its time bound."""
    return ChoreoStep(goal=Goal(type=goal_type, **kw), max_duration_ms=ms)


# ── Lifecycle ────────────────────────────────────────────────────────────────

def test_a_fresh_choreo_is_idle_and_scriptless():
    c = Choreo(element_id=0)
    assert c.goal_status() == ChoreoState.IDLE
    assert c.current_goal_type() == GoalType.NONE
    assert c.script_step() == -1
    assert c.script_complete() is False


def test_configure_then_deploy_walks_idle_configured_running():
    c = Choreo(element_id=0)
    assert c.configure(Goal(type=GoalType.FORM)) == 0
    assert c.goal_status() == ChoreoState.CONFIGURED
    assert c.deploy() == 0
    assert c.goal_status() == ChoreoState.RUNNING
    assert c.current_goal_type() == GoalType.FORM


@pytest.mark.parametrize("goal", [None, Goal(type=GoalType.NONE)])
def test_configure_rejects_a_missing_or_empty_goal(goal):
    assert Choreo(element_id=0).configure(goal) == -1


def test_configure_rejects_a_second_goal_while_one_is_loaded():
    c = Choreo(element_id=0)
    c.configure(Goal(type=GoalType.FORM))
    assert c.configure(Goal(type=GoalType.HOLD)) == -1


def test_deploy_outside_configured_is_rejected():
    c = Choreo(element_id=0)
    assert c.deploy() == -1
    c.configure(Goal(type=GoalType.FORM))
    assert c.deploy() == 0
    assert c.deploy() == -1


def test_terminate_returns_to_idle_and_signals_quiescence():
    """TERMINATED transitions straight back to IDLE — a polling caller will
    never observe it (choreo.py's ChoreoState docstring)."""
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.FORM))
    c.terminate()
    assert c.goal_status() == ChoreoState.IDLE
    assert c.current_goal_type() == GoalType.NONE
    assert c.get_directive().type == BSEDirectiveType.IDLE


def test_terminate_is_valid_from_any_state():
    c = Choreo(element_id=0)
    c.terminate()                                  # from IDLE
    assert c.goal_status() == ChoreoState.IDLE
    c.configure(Goal(type=GoalType.HOLD))
    c.terminate()                                  # from CONFIGURED
    assert c.goal_status() == ChoreoState.IDLE


def test_cancel_goal_is_terminate():
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.FORM))
    c.cancel_goal()
    assert c.goal_status() == ChoreoState.IDLE


def test_submit_goal_replaces_an_active_goal():
    c = Choreo(element_id=0)
    assert c.submit_goal(Goal(type=GoalType.FORM)) == 0
    assert c.submit_goal(Goal(type=GoalType.CONVERGE, target=(1.0, 2.0))) == 0
    assert c.current_goal_type() == GoalType.CONVERGE


def test_submit_goal_rejects_none():
    assert Choreo(element_id=0).submit_goal(None) == -1


def test_current_goal_type_is_none_outside_running_and_suspended():
    c = Choreo(element_id=0)
    c.configure(Goal(type=GoalType.FORM))
    assert c.current_goal_type() == GoalType.NONE   # CONFIGURED, not running
    c.deploy()
    assert c.current_goal_type() == GoalType.FORM


# ── Capability gate ──────────────────────────────────────────────────────────

def test_unregistered_capabilities_skip_the_check_entirely():
    """capabilities=None mirrors choreo_register_scr() never being called."""
    c = Choreo(element_id=0, capabilities=None)
    assert c.configure(Goal(type=GoalType.FORM, required_caps=0xFF)) == 0


def test_capability_requirements_map_onto_scr_hardware_flags():
    c = Choreo(element_id=0, capabilities=CAP_ACTUATOR | CAP_SENSOR | CAP_RELAY)
    for cap in (ChoreoCapabilities.LOCOMOTION, ChoreoCapabilities.SENSING,
                ChoreoCapabilities.SIGNALING):
        fresh = Choreo(element_id=0,
                       capabilities=CAP_ACTUATOR | CAP_SENSOR | CAP_RELAY)
        assert fresh.configure(Goal(type=GoalType.FORM, required_caps=cap)) == 0
    assert c.configure(Goal(type=GoalType.FORM,
                            required_caps=ChoreoCapabilities.NONE)) == 0


@pytest.mark.parametrize("cap,hw", [
    (ChoreoCapabilities.LOCOMOTION, CAP_SENSOR | CAP_RELAY),
    (ChoreoCapabilities.SENSING,    CAP_ACTUATOR | CAP_RELAY),
    (ChoreoCapabilities.SIGNALING,  CAP_ACTUATOR | CAP_SENSOR),
])
def test_a_missing_hardware_flag_is_eperm(cap, hw):
    c = Choreo(element_id=0, capabilities=hw)
    assert c.configure(Goal(type=GoalType.FORM, required_caps=cap)) == -errno.EPERM
    assert c.goal_status() == ChoreoState.IDLE


def test_bonding_has_no_scr_mapping_and_can_never_be_satisfied():
    c = Choreo(element_id=0, capabilities=0xFF)
    assert c.configure(Goal(type=GoalType.FORM,
                            required_caps=ChoreoCapabilities.BONDING)) \
        == -errno.EPERM


# ── Script validation ────────────────────────────────────────────────────────

def test_submit_script_rejects_an_empty_script():
    assert Choreo(element_id=0).submit_script([]) == -1


def test_submit_script_rejects_a_step_with_no_exit_condition():
    """Neither achievement nor a time bound — it would stall by
    construction, so it never reaches the runtime."""
    step = ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=0,
                      advance_on_achieved=False)
    assert Choreo(element_id=0).submit_script([step]) == -1


def test_submit_script_rejects_an_empty_goal_in_any_position():
    good = timed()
    bad = ChoreoStep(goal=Goal(type=GoalType.NONE), max_duration_ms=100)
    assert Choreo(element_id=0).submit_script([good, bad]) == -1


def test_submit_script_validates_capabilities_up_front():
    """Every step is checked before step 0 deploys — a script must not run
    halfway and then discover it cannot finish."""
    c = Choreo(element_id=0, capabilities=CAP_ACTUATOR)
    steps = [timed(),
             timed(required_caps=ChoreoCapabilities.SENSING)]
    assert c.submit_script(steps) == -errno.EPERM
    assert c.goal_status() == ChoreoState.IDLE
    assert c.script_step() == -1


def test_a_rejected_script_leaves_the_previous_run_untouched():
    c = Choreo(element_id=0)
    assert c.submit_script([timed(ms=100)]) == 0
    c.tick(solo(), HEALTHY)
    assert c.script_complete() is True
    assert c.submit_script([]) == -1
    assert c.script_complete() is True


# ── Script advance ───────────────────────────────────────────────────────────

def test_a_step_advances_when_its_time_bound_elapses():
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=200), timed(ms=200, target=(9.0, 9.0))])
    c.tick(solo(), HEALTHY)
    assert c.script_step() == 0
    c.tick(solo(), HEALTHY)
    assert c.script_step() == 1
    # The advance happens after the BSE has already produced this cycle's
    # directive, so the new step's goal first reaches the substrate on the
    # NEXT tick.  One cycle of lag, by construction.
    assert c.get_directive().target == (50.0, 50.0)
    c.tick(solo(), HEALTHY)
    assert c.get_directive().target == (9.0, 9.0)


def test_completing_the_last_step_terminates_into_quiescence():
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=100)])
    c.tick(solo(), HEALTHY)
    assert c.script_complete() is True
    assert c.script_step() == -1
    assert c.goal_status() == ChoreoState.IDLE
    assert c.get_directive().type == BSEDirectiveType.IDLE


def test_script_complete_survives_the_terminate_it_triggers():
    """The flag is the application's cue to map IDLE to platform
    quiescence, so terminate() must not clear it."""
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=100)])
    c.tick(solo(), HEALTHY)
    c.terminate()
    assert c.script_complete() is True


def test_a_new_submission_clears_script_complete():
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=100)])
    c.tick(solo(), HEALTHY)
    assert c.script_complete() is True
    c.submit_script([timed(ms=100)])
    assert c.script_complete() is False


def test_a_step_advances_on_its_own_achievement():
    step = ChoreoStep(
        goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0),
                  achieve_eps=1.0, achieve_hold_ms=200),
        max_duration_ms=100_000, advance_on_achieved=True)
    c = Choreo(element_id=0)
    c.submit_script([step, timed(ms=100)])
    c.tick(solo(0.0, 0.0), HEALTHY)
    assert c.script_step() == 0
    c.tick(solo(0.0, 0.0), HEALTHY)
    assert c.script_step() == 1


def test_the_time_bound_still_fires_when_achievement_never_does():
    """The robustness net: a step that cannot be achieved must not hang."""
    step = ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0),
                                achieve_eps=0.01, achieve_hold_ms=100),
                      max_duration_ms=300, advance_on_achieved=True)
    c = Choreo(element_id=0)
    c.submit_script([step, timed(ms=100)])
    for _ in range(3):
        c.tick(solo(50.0, 50.0), HEALTHY)
    assert c.script_step() == 1


# ── scope="all" ──────────────────────────────────────────────────────────────

def collective_step(ms=100_000):
    return ChoreoStep(
        goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0),
                  achieve_eps=1.0, achieve_hold_ms=100),
        max_duration_ms=ms, advance_on_achieved=True, scope=ChoreoScope.ALL)


def test_scope_all_waits_for_every_fresh_peer():
    c = Choreo(element_id=0)
    c.submit_script([collective_step(), timed(ms=100)])
    behind = wm([(0, 0), (0, 0)], self_id=0, achieved=[False, False])
    for _ in range(5):
        c.tick(behind, HEALTHY)
        assert c.goal_achieved() is True            # own predicate fires
        assert c.script_step() == 0                 # collective one does not
    c.tick(wm([(0, 0), (0, 0)], self_id=0, achieved=[False, True]), HEALTHY)
    assert c.script_step() == 1


def test_scope_self_ignores_peers_that_have_not_achieved():
    step = ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0),
                                achieve_eps=1.0, achieve_hold_ms=100),
                      max_duration_ms=100_000, advance_on_achieved=True)
    c = Choreo(element_id=0)
    c.submit_script([step, timed(ms=100_000)])
    behind = wm([(0, 0), (0, 0)], self_id=0, achieved=[False, False])
    c.tick(behind, HEALTHY)
    assert c.script_step() == 1


def test_the_collective_predicate_requires_own_achievement_first():
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.CONVERGE, target=(0.0, 0.0)))
    entries = wm([(90, 90), (0, 0)], self_id=0, achieved=[False, True])
    c.tick(entries, HEALTHY)
    assert c.collective_achieved(entries) is False


def test_the_collective_predicate_is_vacuously_true_without_fresh_peers():
    """Eventually consistent, not a synchronization barrier — a solo
    element is not blocked by peers it cannot hear."""
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.HOLD))
    lonely = wm([(0, 0), (5, 5)], self_id=0, stale=[1], achieved=[False, False])
    c.tick(lonely, HEALTHY)
    assert c.goal_achieved() is True
    assert c.collective_achieved(lonely) is True


def test_a_peer_entry_without_an_achieved_key_counts_as_not_achieved():
    """Absent gossip is not consent — the key defaults to False."""
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.HOLD))
    entries = [{"id": 0, "x": 0.0, "y": 0.0, "is_active": True,
                "is_stale": False, "is_self": True},
               {"id": 1, "x": 1.0, "y": 1.0, "is_active": True,
                "is_stale": False, "is_self": False}]
    c.tick(entries, HEALTHY)
    assert c.collective_achieved(entries) is False


# ── Quorum suspension ────────────────────────────────────────────────────────

def test_quorum_loss_suspends_a_running_script():
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=100_000)])
    c.tick(solo(), HEALTHY)
    assert c.goal_status() == ChoreoState.RUNNING
    c.tick(solo(), LOST)
    assert c.goal_status() == ChoreoState.SUSPENDED


def test_suspension_freezes_the_step_timer():
    """A partition pauses the show rather than timing it out."""
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=300), timed(ms=300)])
    c.tick(solo(), LOST)                            # ticks once, then suspends
    for _ in range(50):
        c.tick(solo(), LOST)
    assert c.script_step() == 0
    assert c.goal_status() == ChoreoState.SUSPENDED


def test_quorum_recovery_resumes_the_script():
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=300), timed(ms=300)])
    c.tick(solo(), LOST)
    c.tick(solo(), DEGRADED)                        # DEGRADED is enough
    assert c.goal_status() == ChoreoState.RUNNING
    c.tick(solo(), HEALTHY)
    c.tick(solo(), HEALTHY)
    assert c.script_step() == 1


def test_hold_keeps_ticking_the_bse_while_suspended():
    """Station capture needs no peers, and deferring it to quorum recovery
    would capture a drifted position."""
    c = Choreo(element_id=1)
    c.submit_goal(Goal(type=GoalType.HOLD))
    c.tick(wm([(0, 0), (2, 2)], self_id=1), LOST)
    assert c.goal_status() == ChoreoState.SUSPENDED
    assert c.get_directive().target == (2.0, 2.0)
    c.tick(wm([(0, 0), (7, 7)], self_id=1), LOST)
    assert c.get_directive().target == (2.0, 2.0)


def test_a_peer_referential_goal_stays_frozen_while_suspended():
    """EXCHANGE references the collective, so a partition must not let it
    re-capture a snapshot from a world model it can no longer trust."""
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.EXCHANGE))
    c.tick(solo(-1.0, 0.0), LOST)                   # no peer → capture fails
    assert c.goal_status() == ChoreoState.SUSPENDED
    frozen = c.get_directive()
    c.tick(wm([(-1, 0), (1, 0)], self_id=0), LOST)  # peers reappear, still lost
    assert c.get_directive() is frozen


def test_tick_does_nothing_before_deploy():
    c = Choreo(element_id=0)
    c.configure(Goal(type=GoalType.CONVERGE, target=(5.0, 5.0)))
    c.tick(solo(), HEALTHY)
    assert c.get_directive().type == BSEDirectiveType.IDLE
    assert c.goal_status() == ChoreoState.CONFIGURED


def test_a_missing_quorum_state_is_treated_as_healthy():
    """scr_state.get('quorum_state', 2) — an incomplete state dict must not
    suspend the show."""
    c = Choreo(element_id=0)
    c.submit_script([timed(ms=100_000)])
    c.tick(solo(), {"role": 0, "leader_id": 0})
    assert c.goal_status() == ChoreoState.RUNNING


# ── Goal → intent translation ────────────────────────────────────────────────

def test_every_goal_field_reaches_the_bse_intent():
    goal = Goal(type=GoalType.FORM, target=(3.0, 4.0), radius=6.0,
                shape=GoalShape.GRID, slot_shift=2, direct_path=True,
                achieve_eps=0.25, achieve_hold_ms=1500, id=77)
    intent = Choreo._goal_to_intent(goal)
    assert (intent.target, intent.radius, int(intent.shape)) == \
        ((3.0, 4.0), 6.0, int(GoalShape.GRID))
    assert (intent.slot_shift, intent.direct_path) == (2, True)
    assert (intent.achieve_eps, intent.achieve_hold_ms) == (0.25, 1500)
    assert intent.id == 77


def test_an_unmapped_goal_type_becomes_an_idle_intent():
    from tapestry.bse import BSEIntentType
    intent = Choreo._goal_to_intent(Goal(type=GoalType.NONE))
    assert intent.type == BSEIntentType.IDLE
