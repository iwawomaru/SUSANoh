#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Accumulator Tester
## Author: Daiki SHIMADA


import sys,os

# from SUSANoh
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from susanoh import components, environments

if __name__ == '__main__':
    n_stat = environments.TestEnv.n_stat
    n_act = environments.TestEnv.n_act

    model = components.Random(n_input=n_stat, n_output=n_act, output_type="id")
    env = environments.TestEnv(model)
    while True:
        env.execute()

