# -*- coding: utf-8 -*-
from __future__ import print_function

import os

from .run_components import DRIVER_DIR


PORTFOLIO_DIR = os.path.join(DRIVER_DIR, "portfolios")

ALIASES = {}

# blind search
ALIASES["blind"] = ["--search", "laostar(blind(cost_type=ONE), cost_type=ONE, max_time=1740000.0)"]

# ff 
ALIASES["ff"] = ["--search", "laostar(ff(),max_time=1740000.0)"]

# oss + ff
ALIASES["ff-oss"] = ["--symmetries", "sym=symmetry_state_pruning(symmetries=goal_only_orbit)",
                     "--search", "laostar(ff(cost_type=ONE), max_time=1740000.0, symmetry=sym)"]
PORTFOLIOS = {}
for portfolio in os.listdir(PORTFOLIO_DIR):
    name, ext = os.path.splitext(portfolio)
    assert ext == ".py", portfolio
    PORTFOLIOS[name.replace("_", "-")] = os.path.join(PORTFOLIO_DIR, portfolio)


def show_aliases():
    for alias in sorted(ALIASES.keys() + PORTFOLIOS.keys()):
        print(alias)


def set_options_for_alias(alias_name, args):
    """
    If alias_name is an alias for a configuration, set args.search_options
    to the corresponding command-line arguments. If it is an alias for a
    portfolio, set args.portfolio to the path to the portfolio file.
    Otherwise raise KeyError.
    """
    assert not args.search_options
    assert not args.portfolio

    if alias_name in ALIASES:
        args.search_options = [x.replace(" ", "").replace("\n", "")
                               for x in ALIASES[alias_name]]
    elif alias_name in PORTFOLIOS:
        args.portfolio = PORTFOLIOS[alias_name]
    else:
        raise KeyError(alias_name)
