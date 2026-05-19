"""rufus_sim_strategies — pursuit-evasion strategy ABC + reference
strategies.

Importing this package registers the reference strategies via
side effects so the strategy_runner can resolve them by name.
"""

from .strategy import Measurement, Strategy  # noqa: F401
from . import reference  # noqa: F401  (registers references)
