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

action_table = {
        'stop':0,
        'forward':1,
        'backward':2,
        'right':3,
        'left':4
        }


def ball_rule(data, max_position=255):
    x, y = data
    # reject too low position
    if y < max_position*3/4.:
        if x < max_position*1/4.:
            return action_table['left']
        elif x < max_position*3/4.:
            return action_table['forward']
        else:
            return action_table['right']
    return None


if __name__ == '__main__':
    episodes = 15

    n_stat = environments.TestEnv.n_stat
    n_act = environments.TestEnv.n_act

    # setup model
    rules = [components.BallRule(ball_rule)]
    model = components.RuleLayer(rules)

    env = environments.TestEnv(model)

    for i in six.moves.range(episodes):
        env.execute()

