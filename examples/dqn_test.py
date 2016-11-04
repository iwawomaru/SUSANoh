#!/usr/bin/env python
# -*- coding: utf-8 -*-

## DQN Component Tester
## Author: Daiki SHIMADA


import sys,os

# for Python3 style
import six

import numpy as np

# from SUSANoh
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from susanoh import components, environments


if __name__ == '__main__':
    episodes = 1

    # Change to your environment
    n_stat = environments.TestEnv.n_stat
    n_act = environments.TestEnv.n_act

    # setup model
    # If you want to use L1 regularization, put the rate into 'L1_rate'.
    model = components.DQN(n_stat, n_act, L1_rate=None, on_gpu=False)

    # Change to your environment
    env = environments.TestEnv(model)

    for i in six.moves.range(episodes):
        env.execute()

