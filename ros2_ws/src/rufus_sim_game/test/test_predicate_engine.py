"""Unit tests for rufus_sim_game.predicate_engine.

These cover (a) the predicate scenarios called out in plan.md
Stage 5 (capture, region entry/exit, cone), (b) dwell-timer
semantics, and (c) every grammar-rejection path so that v2 can
relax them deliberately rather than by accident.
"""

import math

import pytest

from rufus_sim_game.predicate_engine import (
    DwellTimer, PredicateError, compile_predicate,
)


AGENTS = ('R0', 'R1')


def _vals(**overrides):
    base = {
        aid: {c: 0.0 for c in (
            'x', 'y', 'z', 'vx', 'vy', 'vz', 'psi',
            'qw', 'qx', 'qy', 'qz')}
        for aid in AGENTS
    }
    for aid, comps in overrides.items():
        base[aid].update(comps)
    return base


# ----- positive cases (every scenario from plan.md) -----


def test_capture_euclidean():
    p = compile_predicate(
        'capture',
        '(R0.x - R1.x)**2 + (R0.y - R1.y)**2 < 0.5**2',
        dwell_s=0.0, outcome='pursuer_win', agent_ids=AGENTS,
    )
    assert p.op == '<'
    assert p.evaluate(_vals(R0={'x': 0.0, 'y': 0.0},
                            R1={'x': 0.3, 'y': 0.1})) is True
    assert p.evaluate(_vals(R0={'x': 0.0, 'y': 0.0},
                            R1={'x': 1.0, 'y': 0.0})) is False
    # Margin is signed: lhs - rhs (negative when satisfied).
    margin_close = p.margin(_vals(R0={'x': 0.0, 'y': 0.0},
                                  R1={'x': 0.3, 'y': 0.0}))
    assert margin_close == pytest.approx(0.3 ** 2 - 0.5 ** 2)


def test_disk_exit():
    p = compile_predicate(
        'escape', 'R1.x**2 + R1.y**2 > 50**2',
        dwell_s=0.0, outcome='evader_win', agent_ids=AGENTS,
    )
    assert p.evaluate(_vals(R1={'x': 60.0, 'y': 0.0})) is True
    assert p.evaluate(_vals(R1={'x': 30.0, 'y': 30.0})) is False


def test_region_entry_compound_via_two_predicates():
    # Region entry: x in [0, 50] AND y in [0, 30] is the disjunction
    # of four boundary inequalities; in v1 grammar these must each
    # be expressed as a single predicate. Here we verify each side
    # individually and rely on the engine evaluating multiple
    # predicates per tick (composition is an episode-runner
    # concern, not the parser's, in v1).
    east_boundary = compile_predicate(
        'east', 'R0.x > 50.0', dwell_s=0.0, outcome='draw',
        agent_ids=AGENTS,
    )
    south_boundary = compile_predicate(
        'south', 'R0.y < 0.0', dwell_s=0.0, outcome='draw',
        agent_ids=AGENTS,
    )
    assert east_boundary.evaluate(
        _vals(R0={'x': 60.0})) is True
    assert south_boundary.evaluate(
        _vals(R0={'y': -1.0})) is True
    assert east_boundary.evaluate(
        _vals(R0={'x': 30.0})) is False


def test_cone_via_squared_cosine():
    # 2D forward cone: R1 is in R0's forward half-cone of half-angle
    # alpha when
    #   ((dx) cos psi + (dy) sin psi)**2 > cos(alpha)**2 * (dx^2 + dy^2)
    # AND the dot product (dx cos psi + dy sin psi) is positive.
    # cos(psi)/sin(psi) of a state symbol is non-polynomial, so we
    # parameterise the predicate on the velocity components instead:
    # R0 is moving toward R1 if vx*(R1.x-R0.x) + vy*(R1.y-R0.y) > 0
    # and the squared projection exceeds cos^2(alpha) * |d|^2 * |v|^2.
    cos_alpha_sq = math.cos(math.radians(20.0)) ** 2
    coef = float(cos_alpha_sq)
    expr = (
        f'(R0.vx*(R1.x-R0.x) + R0.vy*(R1.y-R0.y))**2 '
        f'> {coef} * '
        f'((R1.x-R0.x)**2 + (R1.y-R0.y)**2) * '
        f'(R0.vx**2 + R0.vy**2)'
    )
    p = compile_predicate(
        'in_cone', expr, dwell_s=0.0, outcome='pursuer_win',
        agent_ids=AGENTS,
    )
    # R0 facing along +x at unit speed, R1 directly ahead -> in cone.
    inside = _vals(
        R0={'vx': 1.0, 'vy': 0.0, 'x': 0.0, 'y': 0.0},
        R1={'x': 5.0, 'y': 0.1})
    assert p.evaluate(inside) is True
    # R1 well off-axis -> outside cone.
    outside = _vals(
        R0={'vx': 1.0, 'vy': 0.0, 'x': 0.0, 'y': 0.0},
        R1={'x': 5.0, 'y': 5.0})
    assert p.evaluate(outside) is False


# ----- dwell timer -----


def test_dwell_timer_fires_after_streak():
    t = DwellTimer(dwell_s=0.5)
    assert t.update(0.0, True) == (False, False)
    assert t.update(0.3, True) == (False, False)
    assert t.update(0.5, True) == (True, True)
    assert t.update(0.6, True) == (True, False)


def test_dwell_timer_resets_on_break():
    t = DwellTimer(dwell_s=0.5)
    t.update(0.0, True)
    t.update(0.3, True)
    assert t.update(0.4, False) == (False, False)
    assert t.update(0.5, True) == (False, False)
    # Streak only resumes; needs another full dwell.
    assert t.update(0.99, True) == (False, False)
    assert t.update(1.05, True) == (True, True)


def test_dwell_timer_zero_fires_on_first_true():
    t = DwellTimer(dwell_s=0.0)
    assert t.update(0.0, False) == (False, False)
    assert t.update(0.1, True) == (True, True)
    assert t.update(0.2, True) == (True, False)


def test_dwell_timer_does_not_re_fire_after_reset_streak():
    # Once fired, _fired stays set until reset(); this is the
    # contract the episode_runner depends on (it terminates the
    # episode on the single fire).
    t = DwellTimer(dwell_s=0.1)
    t.update(0.0, True)
    assert t.update(0.2, True) == (True, True)
    t.update(0.3, False)
    assert t.update(0.4, True) == (False, False)
    # After explicit reset, the timer can fire again.
    t.reset()
    t.update(1.0, True)
    assert t.update(1.2, True) == (True, True)


# ----- grammar rejection paths -----


def _expect_error(*, match=None, **kwargs):
    with pytest.raises(PredicateError, match=match):
        compile_predicate(**kwargs)


def test_reject_non_polynomial_function():
    _expect_error(
        pred_id='p', expr_str='cos(R0.x) < 0.5',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )


def test_reject_python_and():
    _expect_error(
        pred_id='p',
        expr_str='R0.x < 1 and R1.x > 0',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )


def test_accept_bitwise_and():
    # `&` between two relationals is the v2 conjunction. Verify it
    # compiles and evaluates as logical AND.
    p = compile_predicate(
        'and',
        '(R0.x < 1) & (R1.x > 0)',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )
    assert p.evaluate(_vals(R0={'x': 0.0}, R1={'x': 5.0})) is True
    assert p.evaluate(_vals(R0={'x': 5.0}, R1={'x': 5.0})) is False
    assert p.evaluate(_vals(R0={'x': 0.0}, R1={'x': -1.0})) is False
    # margin is informational for compound predicates.
    assert p.margin(_vals(R0={'x': 0.0}, R1={'x': 5.0})) == 0.0


def test_accept_bitwise_or():
    p = compile_predicate(
        'or',
        '(R0.x < 1) | (R1.x > 5)',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )
    assert p.evaluate(_vals(R0={'x': 0.0}, R1={'x': 0.0})) is True
    assert p.evaluate(_vals(R0={'x': 5.0}, R1={'x': 10.0})) is True
    assert p.evaluate(_vals(R0={'x': 5.0}, R1={'x': 0.0})) is False


def test_accept_bitwise_not():
    p = compile_predicate(
        'not',
        '~(R0.x > 5)',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )
    assert p.evaluate(_vals(R0={'x': 10.0})) is False
    assert p.evaluate(_vals(R0={'x': 0.0})) is True


def test_accept_compound_disjunction():
    # Multi-pursuer capture: any pursuer within capture range of
    # the evader. Mirrors the actual Stage 7 use case.
    p = compile_predicate(
        'capture',
        '((R0.x - R1.x)**2 + (R0.y - R1.y)**2 < 0.5**2) | '
        '((R0.x + R1.x)**2 < 0.1**2)',
        dwell_s=0.0, outcome='pursuer_win',
        agent_ids=AGENTS,
    )
    assert p.evaluate(_vals(R0={'x': 0.0, 'y': 0.0},
                            R1={'x': 0.3, 'y': 0.0})) is True
    assert p.evaluate(_vals(R0={'x': 0.0, 'y': 0.0},
                            R1={'x': 5.0, 'y': 5.0})) is False


def test_reject_arithmetic_above_inequality():
    # `(R0.x < 1) + 5` puts an arithmetic op above a Relational
    # — the collector reaches an unexpected node at boolean level.
    _expect_error(
        pred_id='p',
        expr_str='(R0.x < 1) + R1.x',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
        match=None,
    )


def test_reject_chained_comparison():
    _expect_error(
        pred_id='p', expr_str='0 < R0.x < 5',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )


def test_reject_unknown_agent_id():
    _expect_error(
        pred_id='p', expr_str='R2.x < 1',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )


def test_reject_unknown_component():
    _expect_error(
        pred_id='p', expr_str='R0.acceleration < 1',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
        match='not a valid agent state component',
    )


def test_reject_non_integer_power():
    _expect_error(
        pred_id='p', expr_str='R0.x**0.5 < 1',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
        match='non-negative integer',
    )


def test_reject_division_by_symbol():
    # `1/R0.x` has Pow(R0.x, -1) which is a negative-integer
    # exponent, rejected by the polynomial-form gate.
    _expect_error(
        pred_id='p', expr_str='1/R0.x < 1',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
        match='non-negative integer',
    )


def test_reject_constant_predicate():
    _expect_error(
        pred_id='p', expr_str='1 < 2',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
        match='constant',
    )


def test_reject_no_comparison():
    _expect_error(
        pred_id='p', expr_str='R0.x + R1.x',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
        match='boolean composition of comparisons',
    )


def test_reject_negative_dwell():
    _expect_error(
        pred_id='p', expr_str='R0.x < 1',
        dwell_s=-0.1, outcome='win', agent_ids=AGENTS,
        match='dwell_s',
    )


def test_reject_empty_outcome():
    _expect_error(
        pred_id='p', expr_str='R0.x < 1',
        dwell_s=0.0, outcome='', agent_ids=AGENTS,
        match='outcome',
    )


# ----- evaluation contract -----


def test_inputs_in_lambdify_order():
    p = compile_predicate(
        'p', 'R1.x*R0.y - R0.x*R1.y < 0',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )
    # Symbol order is alphabetical within the symbol name space
    # (R0__x, R0__y, R1__x, R1__y), and `inputs` mirrors that
    # exactly so the runner can populate values from a dict
    # without caring about the YAML field order.
    assert p.inputs == (
        ('R0', 'x'), ('R0', 'y'), ('R1', 'x'), ('R1', 'y'))


def test_evaluate_uses_only_listed_inputs():
    # The evaluator should not look up components it doesn't need
    # — guarantees the runner can hand it sparse dicts.
    p = compile_predicate(
        'p', 'R0.x + R0.y > 5',
        dwell_s=0.0, outcome='win', agent_ids=AGENTS,
    )
    # Don't pre-fill R1; evaluator must not touch it.
    sparse = {'R0': {'x': 3.0, 'y': 3.0}}
    assert p.evaluate(sparse) is True
