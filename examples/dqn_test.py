#!/usr/bin/env python
# -*- coding: utf-8 -*-

## DQN Component Tester
## Author: Daiki SHIMADA


import sys,os
from chainer import serializers
import argparse

# for Python3 style
import six

import numpy as np

# from SUSANoh
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from susanoh import components, environments


parser = argparse.ArgumentParser(description='SUSANoh:')
parser.add_argument('--initmodel', '-m', default='',
                   help='Initialize the model from given file')
args = parser.parse_args()


if __name__ == '__main__':

    # Change to your environment
    n_stat = environments.SoccerEnv.n_stat
    n_act = environments.SoccerEnv.n_act

    # setup model
    # If you want to use L1 regularization, put the rate into 'L1_rate'.
    model = components.DQN(n_stat, n_act, L1_rate=None, on_gpu=True)
    if args.initmodel:
        print('Load model from', args.initmodel)
        serializers.load_npz(args.initmodel, model.agent.q)

    # Change to your environment
    env = environments.SoccerEnv(model)
    i = 0
    while True:
        env.execute()

        i = i + 1
        if i % 10 == 0:
           print 'save the model'
           model.agent.save(index=i)

