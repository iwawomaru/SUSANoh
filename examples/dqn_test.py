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

    # Change to your environment
    n_stat = environments.SoccerEnv.n_stat
    n_act = environments.SoccerEnv.n_act

    # setup model
    # If you want to use L1 regularization, put the rate into 'L1_rate'.
    model = components.DQN(n_stat, n_act, L1_rate=None, on_gpu=True)

    # Change to your environment
    env = environments.SoccerEnv(model)

    while True:
        env.execute()

