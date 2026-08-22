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

from tapestry.bse import BSEDirectiveType, BSEFrame, BSEMotion, BSEAnchorSelector
from tapestry.choreo import (Choreo, ChoreoCapabilities, ChoreoEvent,
                             ChoreoScope, ChoreoState, ChoreoStep,
                             ChoreoTrack, ChoreoTrackFilter,
                             ChoreoTransition, ELEMENT_HEALTH_LOW_BATTERY,
                             Goal, GoalShape, GoalType)

HEALTHY  = scr(QUORUM_HEALTHY)
DEGRADED = scr(QUORUM_DEGRADED)
LOST     = scr(QUORUM_LOST)

# SCR_CAP_* hardware bits (scr.h), mirrored privately by choreo.py.
CAP_RELAY, CAP_SENSOR, CAP_ACTUATOR = 0x01, 0x02, 0x04
CAP_BONDING, CAP_ABS_POSITION       = 0x08, 0x10


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
    assert c.submit_goal(Goal(type=GoalType.CONVERGE, target=(1.0, 2.0, 0.0))) == 0
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
    # FORM defaults to frame=ABSOLUTE, which derives its own ABS_POSITION
    # requirement (see _derived_caps() tests below) — granted here
    # alongside the three under test so this test stays about LOCOMOTION/
    # SENSING/SIGNALING specifically.
    hw = CAP_ACTUATOR | CAP_SENSOR | CAP_RELAY | CAP_ABS_POSITION
    c = Choreo(element_id=0, capabilities=hw)
    for cap in (ChoreoCapabilities.LOCOMOTION, ChoreoCapabilities.SENSING,
                ChoreoCapabilities.SIGNALING):
        fresh = Choreo(element_id=0, capabilities=hw)
        assert fresh.configure(Goal(type=GoalType.FORM, required_caps=cap)) == 0
    assert c.configure(Goal(type=GoalType.FORM,
                            required_caps=ChoreoCapabilities.NONE)) == 0


@pytest.mark.parametrize("cap,hw", [
    (ChoreoCapabilities.LOCOMOTION,   CAP_SENSOR | CAP_RELAY | CAP_ABS_POSITION),
    (ChoreoCapabilities.SENSING,      CAP_ACTUATOR | CAP_RELAY | CAP_ABS_POSITION),
    (ChoreoCapabilities.SIGNALING,    CAP_ACTUATOR | CAP_SENSOR | CAP_ABS_POSITION),
    (ChoreoCapabilities.BONDING,      CAP_ACTUATOR | CAP_ABS_POSITION),
    (ChoreoCapabilities.ABS_POSITION, CAP_ACTUATOR),
])
def test_a_missing_hardware_flag_is_eperm(cap, hw):
    c = Choreo(element_id=0, capabilities=hw)
    assert c.configure(Goal(type=GoalType.FORM, required_caps=cap)) == -errno.EPERM
    assert c.goal_status() == ChoreoState.IDLE


def test_bonding_maps_onto_scr_cap_bonding():
    c = Choreo(element_id=0, capabilities=CAP_BONDING | CAP_ABS_POSITION)
    assert c.configure(Goal(type=GoalType.FORM,
                            required_caps=ChoreoCapabilities.BONDING)) == 0


def test_spin_derives_a_locomotion_floor_even_when_undeclared():
    """Choreo SDK Design doc §11: capability requirements are a derived
    floor — motion == SPIN demands LOCOMOTION whether or not required_caps
    declares it."""
    c = Choreo(element_id=0,
              capabilities=CAP_SENSOR | CAP_RELAY | CAP_ABS_POSITION)  # no actuator
    rc = c.configure(Goal(type=GoalType.FORM,
                          motion=BSEMotion.SPIN, spin_rate_radps=0.5,
                          required_caps=ChoreoCapabilities.NONE))
    assert rc == -errno.EPERM


def test_spin_derived_floor_is_satisfied_by_actuator_hardware():
    c = Choreo(element_id=0, capabilities=CAP_ACTUATOR | CAP_ABS_POSITION)
    rc = c.configure(Goal(type=GoalType.FORM,
                          motion=BSEMotion.SPIN, spin_rate_radps=0.5,
                          required_caps=ChoreoCapabilities.NONE))
    assert rc == 0


def test_absolute_frame_derives_an_abs_position_floor_even_when_undeclared():
    """A FORM/CONVERGE goal with frame == ABSOLUTE (the default) demands
    ABS_POSITION whether or not required_caps declares it."""
    c = Choreo(element_id=0, capabilities=CAP_ACTUATOR)   # no ABS_POSITION
    rc = c.configure(Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0)))
    assert rc == -errno.EPERM


def test_collective_frame_does_not_derive_an_abs_position_floor():
    """Opting into frame == COLLECTIVE avoids the ABSOLUTE-only floor."""
    c = Choreo(element_id=0, capabilities=CAP_ACTUATOR)   # no ABS_POSITION
    rc = c.configure(Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0),
                          frame=BSEFrame.COLLECTIVE))
    assert rc == 0


def test_hold_does_not_derive_an_abs_position_floor():
    """HOLD never reads frame at all (bse.py §5) — no floor to derive."""
    c = Choreo(element_id=0, capabilities=CAP_ACTUATOR)   # no ABS_POSITION
    assert c.configure(Goal(type=GoalType.HOLD)) == 0


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


def test_submit_script_rejects_spin_with_no_duration_bound():
    """A non-terminal motion never "completes" — until=achieved alone is
    not a sufficient exit, only a valid early-advance on top of a real
    duration bound."""
    bad = ChoreoStep(
        goal=Goal(type=GoalType.FORM, shape=GoalShape.CIRCLE, radius=3.0,
                 motion=BSEMotion.SPIN, spin_rate_radps=0.5),
        max_duration_ms=0, advance_on_achieved=True)
    assert Choreo(element_id=0).submit_script([bad]) == -1

    good = ChoreoStep(
        goal=Goal(type=GoalType.FORM, shape=GoalShape.CIRCLE, radius=3.0,
                 motion=BSEMotion.SPIN, spin_rate_radps=0.5),
        max_duration_ms=60_000)
    assert Choreo(element_id=0).submit_script([good]) == 0


def test_submit_script_rejects_too_many_transitions():
    bad = ChoreoStep(
        goal=Goal(type=GoalType.HOLD), max_duration_ms=1000,
        on=[ChoreoTransition(event=ChoreoEvent.ACHIEVED, goto_step_idx=0)] * 5)
    assert Choreo(element_id=0).submit_script([bad]) == -1


def test_submit_script_rejects_an_out_of_range_goto():
    bad = ChoreoStep(
        goal=Goal(type=GoalType.HOLD), max_duration_ms=1000,
        on=[ChoreoTransition(event=ChoreoEvent.ACHIEVED, goto_step_idx=5)])
    assert Choreo(element_id=0).submit_script([bad]) == -1


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
    c.submit_script([timed(ms=200), timed(ms=200, target=(9.0, 9.0, 0.0))])
    c.tick(solo(), HEALTHY)
    assert c.script_step() == 0
    c.tick(solo(), HEALTHY)
    assert c.script_step() == 1
    # The advance happens after the BSE has already produced this cycle's
    # directive, so the new step's goal first reaches the substrate on the
    # NEXT tick.  One cycle of lag, by construction.
    assert c.get_directive().target == (50.0, 50.0, 50.0)
    c.tick(solo(), HEALTHY)
    assert c.get_directive().target == (9.0, 9.0, 0.0)


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
        goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0),
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
    step = ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0),
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
        goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0),
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
    step = ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0),
                                achieve_eps=1.0, achieve_hold_ms=100),
                      max_duration_ms=100_000, advance_on_achieved=True)
    c = Choreo(element_id=0)
    c.submit_script([step, timed(ms=100_000)])
    behind = wm([(0, 0), (0, 0)], self_id=0, achieved=[False, False])
    c.tick(behind, HEALTHY)
    assert c.script_step() == 1


def test_the_collective_predicate_requires_own_achievement_first():
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0)))
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
    assert c.get_directive().target == (2.0, 2.0, 0.0)
    c.tick(wm([(0, 0), (7, 7)], self_id=1), LOST)
    assert c.get_directive().target == (2.0, 2.0, 0.0)


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
    c.configure(Goal(type=GoalType.CONVERGE, target=(5.0, 5.0, 0.0)))
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


# ── Goal queue: preemption + resume ─────────────────────────────────────────
# preempt_goal() saves the running goal (and, for a script, its exact
# step/timer position) instead of discarding it; terminate() (and therefore
# cancel_goal()) resumes it automatically once the preempting goal ends,
# instead of going to IDLE. Ports choreo_preempt_goal()'s ztest coverage
# (examples/cf21bl-formation/tests/src/main.c) to prove Python parity.

def test_preempt_resumes_hold_station():
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.HOLD))
    c.tick(wm([(5, 5)], self_id=0), HEALTHY)   # capture the (5,5) station
    assert not c.is_preempted()

    rth = Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0))
    assert c.preempt_goal(rth) == 0
    assert c.is_preempted()
    assert c.current_goal_type() == GoalType.CONVERGE

    # A second preempt while one is already parked is rejected — depth 1.
    another = Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0))
    assert c.preempt_goal(another) == -errno.EBUSY

    c.tick(wm([(5, 5)], self_id=0), HEALTHY)
    assert c.get_directive().target == (0.0, 0.0, 0.0)

    # Cancelling the preempting goal resumes HOLD at its ORIGINALLY
    # captured (5,5) station — not a fresh capture at wherever self is
    # now — proving the saved activation state, not just the goal type,
    # survives the round trip.
    c.cancel_goal()
    assert not c.is_preempted()
    assert c.current_goal_type() == GoalType.HOLD
    c.tick(wm([(5, 5)], self_id=0), HEALTHY)
    assert c.get_directive().target == (5.0, 5.0, 0.0)


def test_preempt_mid_script_resumes_at_same_step():
    c = Choreo(element_id=0)
    assert c.submit_script([timed(GoalType.HOLD, ms=100_000)]) == 0
    assert c.script_step() == 0

    for _ in range(5):
        c.tick(solo(), HEALTHY)   # 500 ms of the 100000 ms step elapsed

    rth = Goal(type=GoalType.CONVERGE, target=(9.0, 9.0, 0.0))
    assert c.preempt_goal(rth) == 0
    assert c.current_goal_type() == GoalType.CONVERGE

    # Run the preempting goal for far longer than the parked step's
    # remaining budget would tolerate if its timer kept accumulating.
    for _ in range(200):
        c.tick(solo(), HEALTHY)

    c.cancel_goal()   # resume the parked script
    assert c.current_goal_type() == GoalType.HOLD
    assert c.script_step() == 0, "must resume at the SAME step, not advance"

    # If step_ms had kept accumulating during the 200 preempting ticks
    # (20000 ms), 500+20000 = 20500 ms would already be well past this
    # check point; confirm the step is still short of its 100000 ms bound
    # after only ~99000 ms more (99500 ms since the step started, counting
    # only pre- and post-preemption HOLD ticks).
    for _ in range(990):
        c.tick(solo(), HEALTHY)
    assert c.script_step() == 0
    assert not c.script_complete()

    for _ in range(10):
        c.tick(solo(), HEALTHY)   # push past 100000 ms total HOLD time
    assert c.script_complete(), \
        "script must complete once its OWN accumulated time is due"


def test_preempt_from_idle_rejected():
    c = Choreo(element_id=0)
    g = Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0))
    assert c.preempt_goal(g) == -1


def test_ordinary_submit_while_preempted_drops_parked_goal():
    c = Choreo(element_id=0)
    c.submit_goal(Goal(type=GoalType.HOLD))
    c.tick(solo(), HEALTHY)

    rth = Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0))
    c.preempt_goal(rth)
    assert c.is_preempted()

    # An ORDINARY submit — not preempt_goal() — always fully replaces
    # everything, including anything parked.
    fresh = Goal(type=GoalType.CONVERGE, target=(3.0, 3.0, 0.0))
    assert c.submit_goal(fresh) == 0
    assert not c.is_preempted(), \
        "ordinary submit must drop the parked goal, not stack on it"

    # A subsequent preempt must succeed — the stack must genuinely be
    # empty, not just reporting empty while still logically full.
    another = Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0))
    assert c.preempt_goal(another) == 0

    # Cancelling now resumes `fresh`, not the long-gone HOLD.
    c.cancel_goal()
    assert c.current_goal_type() == GoalType.CONVERGE


def test_parked_goal_id_names_the_displaced_goal():
    c = Choreo(element_id=0)
    assert c.parked_goal_id() == 0   # nothing parked
    c.submit_goal(Goal(type=GoalType.HOLD, id=42))
    c.tick(solo(), HEALTHY)
    c.preempt_goal(Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0)))
    assert c.parked_goal_id() == 42
    c.cancel_goal()
    assert c.parked_goal_id() == 0   # nothing parked again after resume


# ── Goal → intent translation ────────────────────────────────────────────────

def test_every_goal_field_reaches_the_bse_intent():
    goal = Goal(type=GoalType.FORM, target=(3.0, 4.0, 0.0), radius=6.0,
                shape=GoalShape.GRID, slot_shift=2, direct_path=True,
                achieve_eps=0.25, achieve_hold_ms=1500, id=77)
    intent = Choreo._goal_to_intent(goal)
    assert (intent.target, intent.radius, int(intent.shape)) == \
        ((3.0, 4.0, 0.0), 6.0, int(GoalShape.GRID))
    assert (intent.slot_shift, intent.direct_path) == (2, True)
    assert (intent.achieve_eps, intent.achieve_hold_ms) == (0.25, 1500)
    assert intent.id == 77


def test_an_unmapped_goal_type_becomes_an_idle_intent():
    from tapestry.bse import BSEIntentType
    intent = Choreo._goal_to_intent(Goal(type=GoalType.NONE))
    assert intent.type == BSEIntentType.IDLE


# ── Events + transitions (Choreo SDK Design doc §8, single track) ──────────

def test_explicit_achieved_transition_skips_a_step():
    c = Choreo(element_id=0)
    steps = [
        ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000,
                  on=[ChoreoTransition(event=ChoreoEvent.ACHIEVED, goto_step_idx=2)]),
        ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(99.0, 99.0, 0.0)),
                  max_duration_ms=60_000),
        ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(7.0, 7.0, 0.0)),
                  max_duration_ms=60_000),
    ]
    assert c.submit_script(steps) == 0
    for _ in range(3):
        c.tick(solo(), HEALTHY)
    assert c.script_step() == 2
    assert c.get_directive().target == (7.0, 7.0, 0.0)


def test_element_joined_and_lost_cycle_the_welcome_dance():
    """The design doc's §8.3 flagship demo for this feature."""
    c = Choreo(element_id=0)
    steps = [
        ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=300_000,
                  on=[ChoreoTransition(event=ChoreoEvent.ELEMENT_JOINED, goto_step_idx=1)]),
        ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0)),
                  max_duration_ms=300_000,
                  on=[ChoreoTransition(event=ChoreoEvent.ELEMENT_LOST, goto_step_idx=0)]),
    ]
    assert c.submit_script(steps) == 0
    for _ in range(5):
        c.tick(solo(), HEALTHY)
    assert c.script_step() == 0

    entries = wm([(0, 0), (5, 5)], self_id=0)
    for _ in range(19):
        c.tick(entries, HEALTHY)
    assert c.script_step() == 0, "still debouncing the join"
    for _ in range(2):
        c.tick(entries, HEALTHY)
    assert c.script_step() == 1, "element_joined fired after the debounce"

    entries = wm([(0, 0), (5, 5)], self_id=0, stale=[1])
    for _ in range(21):
        c.tick(entries, HEALTHY)
    assert c.script_step() == 0, "element_lost cycled back to step 0"


def test_count_transitions_check_in_declaration_order():
    c = Choreo(element_id=0)
    steps = [
        ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000, on=[
            ChoreoTransition(event=ChoreoEvent.COUNT_EQ, threshold=3, goto_step_idx=2),
            ChoreoTransition(event=ChoreoEvent.COUNT_GTE, threshold=2, goto_step_idx=1),
        ]),
        ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(1.0, 1.0, 0.0)), max_duration_ms=60_000),
        ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(3.0, 3.0, 0.0)), max_duration_ms=60_000),
    ]
    assert c.submit_script(steps) == 0
    c.tick(wm([(0, 0), (1, 1), (2, 2)], self_id=0), HEALTHY)
    assert c.script_step() == 2, "count_eq(3), declared first, wins over count_gte(2)"


def test_anchor_lost_transition():
    c = Choreo(element_id=0)
    steps = [
        ChoreoStep(
            goal=Goal(type=GoalType.CONVERGE, frame=BSEFrame.ELEMENT,
                     anchor=BSEAnchorSelector.ID, anchor_id=9),
            max_duration_ms=60_000,
            on=[ChoreoTransition(event=ChoreoEvent.ANCHOR_LOST, goto_step_idx=1)]),
        ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(4.0, 4.0, 0.0)), max_duration_ms=60_000),
    ]
    assert c.submit_script(steps) == 0
    c.tick(solo(), HEALTHY)
    assert c.script_step() == 1, "an anchor that was never fresh transitions immediately"


def test_goto_end_completes_the_script_early():
    c = Choreo(element_id=0)
    steps = [
        ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000,
                  on=[ChoreoTransition(event=ChoreoEvent.ACHIEVED, goto_step_idx=1)]),
    ]
    assert c.submit_script(steps) == 0
    for _ in range(3):
        c.tick(solo(), HEALTHY)
    assert c.script_complete()


# ── Tracks (choreo.h §7) ─────────────────────────────────────────────────────

def test_track_capability_filter_falls_through_to_catchall():
    # no SENSOR cap, but ABS_POSITION so the catchall track's implicit-
    # ABSOLUTE CONVERGE below satisfies its derived floor — this test is
    # about capability-filtered track SELECTION, not the frame axis.
    c = Choreo(element_id=0, capabilities=CAP_ABS_POSITION)
    sensing = [ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000)]
    catchall = [ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(9.0, 9.0, 0.0)),
                            max_duration_ms=60_000)]
    tracks = [
        ChoreoTrack(filter=ChoreoTrackFilter(required_caps=ChoreoCapabilities.SENSING),
                    steps=sensing),
        ChoreoTrack(filter=ChoreoTrackFilter(), steps=catchall),
    ]
    assert c.submit_tracks(solo(), tracks) == 0
    assert c.current_track() == 1, "no SENSOR cap must fall through to the catch-all track"


def test_track_capability_filter_matches_first_track():
    c = Choreo(element_id=0, capabilities=CAP_SENSOR | CAP_ABS_POSITION)
    sensing = [ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000)]
    catchall = [ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(9.0, 9.0, 0.0)),
                            max_duration_ms=60_000)]
    tracks = [
        ChoreoTrack(filter=ChoreoTrackFilter(required_caps=ChoreoCapabilities.SENSING),
                    steps=sensing),
        ChoreoTrack(filter=ChoreoTrackFilter(), steps=catchall),
    ]
    assert c.submit_tracks(solo(), tracks) == 0
    assert c.current_track() == 0, "SENSOR cap must match the first declared track"


def test_track_no_match_and_no_catchall_is_rejected():
    c = Choreo(element_id=0, capabilities=0)
    steps = [ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000)]
    tracks = [ChoreoTrack(filter=ChoreoTrackFilter(required_caps=ChoreoCapabilities.SENSING),
                          steps=steps)]
    assert c.submit_tracks(solo(), tracks) == -1


def test_track_energy_low_migration_is_debounced():
    c = Choreo(element_id=0)
    low_battery = [ChoreoStep(goal=Goal(type=GoalType.CONVERGE, target=(0.0, 0.0, 0.0)),
                              max_duration_ms=60_000)]
    normal = [ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000)]
    tracks = [
        ChoreoTrack(filter=ChoreoTrackFilter(requires_energy_low=True), steps=low_battery),
        ChoreoTrack(filter=ChoreoTrackFilter(), steps=normal),
    ]
    entries = solo()
    assert c.submit_tracks(entries, tracks) == 0
    assert c.current_track() == 1, "starts on the catch-all track, not low"

    entries[0]["health_flags"] = ELEMENT_HEALTH_LOW_BATTERY
    for _ in range(19):
        c.tick(entries, HEALTHY)
    assert c.current_track() == 1, "still debouncing the low-battery switch"
    for _ in range(2):
        c.tick(entries, HEALTHY)
    assert c.current_track() == 0, "migrated to the low-battery track after the debounce hold"
    assert c.get_directive().target == pytest.approx((0.0, 0.0, 0.0))


def test_track_scoped_collective_excludes_other_track_peer():
    c = Choreo(element_id=0)
    entries = wm([(0.0, 0.0), (10.0, 0.0), (100.0, 100.0)], self_id=0)
    entries[1]["current_track"] = 0   # same track as self
    entries[2]["current_track"] = 1   # DIFFERENT track
    steps = [ChoreoStep(goal=Goal(type=GoalType.CONVERGE, frame=BSEFrame.COLLECTIVE),
                        max_duration_ms=60_000)]
    tracks = [ChoreoTrack(filter=ChoreoTrackFilter(), steps=steps)]
    assert c.submit_tracks(entries, tracks) == 0
    c.tick(entries, HEALTHY)
    assert c.get_directive().target == pytest.approx((5.0, 0.0, 0.0)), \
        "collective centroid must exclude the different-track peer"


def test_track_default_still_counts_default_track_peer():
    c = Choreo(element_id=0)
    entries = wm([(0.0, 0.0), (10.0, 0.0)], self_id=0)   # current_track defaults absent -> 0
    steps = [ChoreoStep(goal=Goal(type=GoalType.CONVERGE, frame=BSEFrame.COLLECTIVE),
                        max_duration_ms=60_000)]
    assert c.submit_script(steps) == 0
    assert c.current_track() == 0
    c.tick(entries, HEALTHY)
    assert c.get_directive().target == pytest.approx((5.0, 0.0, 0.0)), \
        "a no-tracks script still counts a default-track peer"


def test_track_ordinary_goal_drops_multitrack_mode():
    c = Choreo(element_id=0)
    steps = [ChoreoStep(goal=Goal(type=GoalType.HOLD), max_duration_ms=60_000)]
    tracks = [
        ChoreoTrack(filter=ChoreoTrackFilter(requires_energy_low=True), steps=steps),
        ChoreoTrack(filter=ChoreoTrackFilter(), steps=steps),
    ]
    assert c.submit_tracks(solo(), tracks) == 0
    assert c.current_track() == 1

    assert c.submit_goal(Goal(type=GoalType.HOLD)) == 0
    assert c.current_track() == 0, "an ordinary goal submission must drop multi-track mode"
