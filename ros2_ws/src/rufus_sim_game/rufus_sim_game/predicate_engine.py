"""Predicate parsing, validation, compilation, and dwell tracking
for the pursuit-evasion game engine.

Grammar
-------

A predicate is a boolean expression over polynomial inequalities
in the allowed agent state components:

  expr := rel
        | expr & expr
        | expr | expr
        | ~expr
        | (expr)
  rel  := poly OP poly
  OP   := < | <= | > | >= | == | !=
  poly := <numeric literal>
        | <agent_id>.<component>
        | poly + poly | poly - poly | poly * poly | poly / poly
        | poly ** <integer>
        | (poly)
        | -poly

`<component>` is one of: x, y, z, vx, vy, vz, psi, qw, qx, qy, qz.

The boolean operators are Python's bitwise `&`, `|`, `~` — the
keyword forms `and`, `or`, `not` cannot be used because
`Relational.__bool__` raises (sympy refuses to coerce a symbolic
inequality to a Python bool, which is what `and`/`or`/`not`
require). Chained comparisons (`0 < x < 5`) are rejected for the
same reason.

Each leaf inequality must be polynomial: at every `rel` site,
`lhs - rhs` must be a polynomial in the symbols that appear.
`sympy.Poly` enforces this per-leaf; non-polynomial leaves
(`sqrt`, `1/x`, `cos`, ...) are rejected at compile time.

Compiling produces a `CompiledPredicate` whose `evaluate(values)`
returns the boolean truth of the whole expression given a dict
of agent state values. For atomic predicates (single inequality)
`margin(values)` returns `lhs - rhs`; for compound predicates
that scalar isn't well-defined and `margin` returns 0.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

import sympy as sp
from sympy.parsing.sympy_parser import parse_expr


DEFAULT_COMPONENTS = (
    'x', 'y', 'z',
    'vx', 'vy', 'vz',
    'psi',
    'qw', 'qx', 'qy', 'qz',
)


# Inequality node types accepted at the top of the parsed expression.
_REL_TYPES = (sp.StrictLessThan, sp.LessThan,
              sp.StrictGreaterThan, sp.GreaterThan,
              sp.Equality, sp.Unequality)

_OP_STR = {
    sp.StrictLessThan: '<',
    sp.LessThan: '<=',
    sp.StrictGreaterThan: '>',
    sp.GreaterThan: '>=',
    sp.Equality: '==',
    sp.Unequality: '!=',
}

# Allowed AST nodes inside each side of the inequality.
_ALLOWED_ARITH = (
    sp.Add, sp.Mul, sp.Pow,
    sp.Symbol, sp.Integer, sp.Float, sp.Rational,
    sp.NumberSymbol,
)

# Component name mangling: `<agent_id>.<component>` parses into a
# sympy Symbol named `<agent_id>__<component>`. Two underscores keep
# the boundary unambiguous as long as agent_ids and component names
# don't themselves contain `__`.
_NAME_SEP = '__'


class PredicateError(ValueError):
    """Raised when a predicate fails to parse, validate, or compile.

    Always carries the predicate id so the episode loader can route
    the error back to the offending YAML entry.
    """


# --------------------------------------------------------------------
# Compilation
# --------------------------------------------------------------------


@dataclass(frozen=True)
class CompiledPredicate:
    id: str
    expr_str: str
    op: str
    dwell_s: float
    outcome: str
    is_polynomial: bool
    # (agent_id, component) for each free symbol, in lambdify order.
    inputs: tuple[tuple[str, str], ...]
    _eval: Callable[..., object]    # returns numpy bool/scalar
    _margin: Callable[..., float]   # signed lhs-rhs

    def evaluate(self, values: dict[str, dict[str, float]]) -> bool:
        args = [values[aid][comp] for aid, comp in self.inputs]
        return bool(self._eval(*args))

    def margin(self, values: dict[str, dict[str, float]]) -> float:
        args = [values[aid][comp] for aid, comp in self.inputs]
        return float(self._margin(*args))


class _AgentNamespace:
    """Stand-in for an agent during parsing.

    Attribute access returns a sympy `Symbol` whose name encodes
    `(agent_id, component)`; unknown components raise immediately so
    the parser surfaces a clear error rather than silently building
    a free symbol that no AgentState will ever fill in.
    """

    __slots__ = ('_agent_id', '_components')

    def __init__(self, agent_id: str,
                 components: Iterable[str]) -> None:
        self._agent_id = agent_id
        self._components = frozenset(components)

    def __getattr__(self, name: str):
        if name.startswith('_') or name not in self._components:
            raise PredicateError(
                f"'{self._agent_id}.{name}' is not a valid agent "
                f"state component (allowed: "
                f"{sorted(self._components)})"
            )
        return sp.Symbol(f'{self._agent_id}{_NAME_SEP}{name}')


def compile_predicate(
    pred_id: str,
    expr_str: str,
    dwell_s: float,
    outcome: str,
    *,
    agent_ids: Sequence[str],
    components: Sequence[str] = DEFAULT_COMPONENTS,
) -> CompiledPredicate:
    """Parse, validate, and compile one predicate.

    Raises `PredicateError` on any failure; the message is the only
    diagnostic surfaced to the user, so it must include the predicate
    id and a pointer to what is wrong.
    """

    if dwell_s < 0:
        raise PredicateError(
            f"predicate {pred_id!r}: dwell_s must be >= 0, "
            f"got {dwell_s}"
        )
    if not outcome:
        raise PredicateError(
            f"predicate {pred_id!r}: outcome string is empty"
        )
    if not expr_str.strip():
        raise PredicateError(
            f"predicate {pred_id!r}: expr is empty"
        )
    if any(_NAME_SEP in c for c in components):
        raise PredicateError(
            f"component names must not contain {_NAME_SEP!r}"
        )
    if any(_NAME_SEP in a for a in agent_ids):
        raise PredicateError(
            f"agent_ids must not contain {_NAME_SEP!r} "
            f"(would collide with the symbol-name mangling)"
        )

    local_dict = {
        aid: _AgentNamespace(aid, components) for aid in agent_ids
    }

    # parse_expr post-processes the input into Python that calls
    # `Integer(...)`, `Float(...)`, `Rational(...)` for numeric
    # literals; those constructors must be reachable in global_dict.
    # Anything not in this dict is unresolvable, which is exactly
    # what we want for `sin`, `cos`, etc.
    safe_globals = {
        'Integer': sp.Integer,
        'Float': sp.Float,
        'Rational': sp.Rational,
        'Symbol': sp.Symbol,
    }

    try:
        parsed = parse_expr(
            expr_str,
            local_dict=local_dict,
            global_dict=safe_globals,
            evaluate=True,
        )
    except PredicateError:
        raise
    except Exception as e:
        raise PredicateError(
            f"predicate {pred_id!r}: failed to parse "
            f"{expr_str!r}: {e}"
        ) from e

    # `0 < 1` and friends evaluate to bare booleans; reject these
    # because they cannot trigger on any AgentState input.
    if isinstance(parsed, sp.logic.boolalg.BooleanAtom):
        raise PredicateError(
            f"predicate {pred_id!r}: expression {expr_str!r} "
            f"evaluates to a constant {parsed!r}; the expression "
            f"references no agent state"
        )

    # Top-level must be either an inequality or a boolean
    # composition over inequalities.
    if not isinstance(parsed, _REL_TYPES + _BOOL_TYPES):
        raise PredicateError(
            f"predicate {pred_id!r}: expression must reduce to "
            f"a comparison or a boolean composition of "
            f"comparisons; got {parsed!r}"
        )

    relationals = _collect_relationals(parsed, pred_id)
    if not relationals:
        raise PredicateError(
            f"predicate {pred_id!r}: expression {expr_str!r} "
            f"contains no inequality leaves"
        )

    # Validate each leaf inequality: its sides must be arithmetic,
    # and `lhs - rhs` must be polynomial in the symbols that
    # appear in that leaf.
    for rel in relationals:
        _validate_ast(rel.lhs, pred_id)
        _validate_ast(rel.rhs, pred_id)
        leaf_margin = sp.expand(rel.lhs - rel.rhs)
        leaf_syms = tuple(sorted(leaf_margin.free_symbols,
                                 key=lambda s: s.name))
        if leaf_syms:
            try:
                sp.Poly(leaf_margin, *leaf_syms)
            except sp.PolynomialError as e:
                raise PredicateError(
                    f"predicate {pred_id!r}: leaf inequality "
                    f"{rel} is not polynomial in "
                    f"{[s.name for s in leaf_syms]}: {e}"
                ) from e

    free_syms = tuple(sorted(parsed.free_symbols,
                             key=lambda s: s.name))
    if not free_syms:
        raise PredicateError(
            f"predicate {pred_id!r}: expression {expr_str!r} has "
            f"no free agent-state symbols (would never depend on "
            f"agent state)"
        )

    inputs = tuple(_split_symbol_name(s.name, agent_ids, components,
                                      pred_id)
                   for s in free_syms)

    eval_fn = sp.lambdify(free_syms, parsed, modules=['numpy'])

    # Margin (signed `lhs - rhs`) is only well-defined for atomic
    # predicates; for boolean compositions it is informational and
    # we expose 0.0 as a placeholder. Strategies that need a
    # signed-margin diagnostic should split compound predicates
    # into named atoms and inspect each.
    if isinstance(parsed, _REL_TYPES):
        margin_expr = sp.expand(parsed.lhs - parsed.rhs)
        margin_fn = sp.lambdify(free_syms, margin_expr,
                                modules=['numpy'])
        op_str = _OP_STR[type(parsed)]
    else:
        def _zero_margin(*_args, **_kwargs):
            return 0.0
        margin_fn = _zero_margin
        op_str = type(parsed).__name__.lower()  # 'and'/'or'/'not'

    return CompiledPredicate(
        id=pred_id,
        expr_str=expr_str,
        op=op_str,
        dwell_s=dwell_s,
        outcome=outcome,
        is_polynomial=True,
        inputs=inputs,
        _eval=eval_fn,
        _margin=margin_fn,
    )


# Boolean-composition node types accepted at any level of the
# parsed expression. Concrete classes only — `Boolean` itself is
# too broad in sympy 1.14 (Symbol inherits from Boolean).
_BOOL_TYPES = (
    sp.And, sp.Or, sp.Not,
)


def _collect_relationals(node, pred_id: str) -> list:
    """Walk a parsed expression and return its inequality leaves.

    Recurses through `_BOOL_TYPES`. Reaching anything that's not a
    boolean op or a relational at this depth means the expression
    is malformed (e.g., an arithmetic node above an inequality
    isn't allowed because `(R0.x < 1) + R1.x` isn't a meaningful
    predicate)."""
    if isinstance(node, _REL_TYPES):
        return [node]
    if isinstance(node, _BOOL_TYPES):
        out: list = []
        for arg in node.args:
            out.extend(_collect_relationals(arg, pred_id))
        return out
    raise PredicateError(
        f"predicate {pred_id!r}: AST node "
        f"{type(node).__name__} ({node!r}) appears at boolean "
        f"level; only inequalities and `&`/`|`/`~` of "
        f"inequalities are allowed there"
    )


def _validate_ast(node, pred_id: str) -> None:
    # Order matters: in sympy 1.14 `Symbol` is a subclass of
    # `Boolean` (every Expr is Boolean-coercible), so we must accept
    # the arithmetic/symbol nodes *first* and only then look for the
    # genuine boolean-composition classes by their concrete types.
    if isinstance(node, _REL_TYPES):
        raise PredicateError(
            f"predicate {pred_id!r}: nested comparison not allowed "
            f"in v1 grammar; got {node!r}"
        )
    if isinstance(node, sp.Pow):
        # Reject Pow(_, non-integer) to keep polynomial form.
        exp = node.exp
        if not (exp.is_integer and exp.is_nonnegative):
            raise PredicateError(
                f"predicate {pred_id!r}: power exponent must be a "
                f"non-negative integer (got {exp})"
            )
        for arg in node.args:
            _validate_ast(arg, pred_id)
        return
    if isinstance(node, _ALLOWED_ARITH):
        for arg in getattr(node, 'args', ()):
            _validate_ast(arg, pred_id)
        return
    # Concrete boolean ops only (And/Or/Not/Xor/Nand/Nor/Implies/...)
    # — `Boolean` itself is too broad in sympy 1.14.
    if isinstance(node, sp.logic.boolalg.BooleanFunction):
        raise PredicateError(
            f"predicate {pred_id!r}: boolean composition "
            f"(and/or/not, &/|) is not supported in v1 grammar; "
            f"got {node!r}"
        )
    raise PredicateError(
        f"predicate {pred_id!r}: unsupported AST node "
        f"{type(node).__name__} ({node!r}); only +, -, *, /, **, "
        f"parentheses, numeric literals, and <agent_id>.<component> "
        f"symbols are allowed"
    )


def _split_symbol_name(
    mangled: str,
    agent_ids: Sequence[str],
    components: Sequence[str],
    pred_id: str,
) -> tuple[str, str]:
    if _NAME_SEP not in mangled:
        raise PredicateError(
            f"predicate {pred_id!r}: free symbol {mangled!r} did "
            f"not come from an <agent_id>.<component> reference"
        )
    agent_id, comp = mangled.split(_NAME_SEP, 1)
    if agent_id not in agent_ids:
        raise PredicateError(
            f"predicate {pred_id!r}: unknown agent_id "
            f"{agent_id!r} (manifest agents: "
            f"{sorted(agent_ids)})"
        )
    if comp not in components:
        raise PredicateError(
            f"predicate {pred_id!r}: unknown component {comp!r} "
            f"on agent {agent_id!r} (allowed: "
            f"{sorted(components)})"
        )
    return agent_id, comp


# --------------------------------------------------------------------
# Dwell timer
# --------------------------------------------------------------------


@dataclass
class DwellTimer:
    """Per-predicate dwell-time tracker.

    The episode runner calls `update(now_s, satisfied)` once per
    tick. `update` returns `(held, fired_now)`:

      held       — True if the predicate has been continuously
                   satisfied for at least `dwell_s` seconds.
      fired_now  — True exactly once, on the tick at which
                   `held` first becomes True.

    A single false sample resets the streak. Sim time should be
    monotonic; if it goes backwards (e.g. user resets gz), call
    `reset()` first.
    """

    dwell_s: float
    _t_first_true: float | None = None
    _fired: bool = False

    def update(self, now_s: float,
               satisfied: bool) -> tuple[bool, bool]:
        if not satisfied:
            self._t_first_true = None
            return False, False
        if self._t_first_true is None:
            self._t_first_true = now_s
        held = (now_s - self._t_first_true) >= self.dwell_s
        if held and not self._fired:
            self._fired = True
            return True, True
        return held, False

    def reset(self) -> None:
        self._t_first_true = None
        self._fired = False
