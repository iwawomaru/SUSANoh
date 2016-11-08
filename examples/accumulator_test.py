#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Accumulator Tester
## Author: Daiki SHIMADA


import sys,os

# for Python3 style
import six

import numpy as np

# from SUSANoh
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from susanoh import components, environments


def rule_0(data, **kwargs):
    if np.mean(data) < 0.5:
        return 2
    return None
    
def rule_1(data, **kwargs):
    if np.mean(data) >= 0.5:
        return 3
    return None


if __name__ == '__main__':
    episodes = 1

    n_stat = environments.TestEnv.n_stat
    n_act = environments.TestEnv.n_act

    # setup model
    rules = [components.Rule(rule_0), components.Rule(rule_1)]
    model = components.RuleLayer(rules)

    env = environments.TestEnv(model)

    for i in six.moves.range(episodes):
        env.execute()

