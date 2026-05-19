"""Name -> Strategy class registry.

v1 is a hard-coded dict; the strategy_runner takes a `type`
string from the episode YAML and looks it up here. The map sits
in its own module so reference strategies and tests can register
without import cycles. A v2 follow-up should switch to Python
entry_points so external packages can ship strategies without
editing this file.
"""

from __future__ import annotations

from typing import Type

from .strategy import Strategy


_REGISTRY: dict[str, Type[Strategy]] = {}


class UnknownStrategy(KeyError):
    """Raised when an episode YAML names a strategy type that has
    not been registered."""


def register(name: str, cls: Type[Strategy]) -> Type[Strategy]:
    """Decorator/function. Registers `cls` under `name`.

    Re-registration with the same (name, cls) pair is a no-op;
    re-registration with a different class raises so silent
    name collisions do not survive a refactor.
    """
    existing = _REGISTRY.get(name)
    if existing is None:
        _REGISTRY[name] = cls
    elif existing is not cls:
        raise ValueError(
            f"strategy name {name!r} already registered to "
            f"{existing.__module__}.{existing.__qualname__}"
        )
    return cls


def get(name: str) -> Type[Strategy]:
    try:
        return _REGISTRY[name]
    except KeyError as e:
        raise UnknownStrategy(
            f"unknown strategy type {name!r}; registered: "
            f"{sorted(_REGISTRY)}"
        ) from e


def names() -> list[str]:
    return sorted(_REGISTRY)
