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

    # Change to your environment
    n_stat = environments.SoccerEnv.n_stat
    n_act = environments.SoccerEnv.n_act

    # setup model
    # If you want to use L1 regularization, put the rate into 'L1_rate'.
    ball_rule = components.BallRule(ball_rule)
    dqn_rule = components.DQN(n_stat, n_act, L1_rate=None, on_gpu=True)
    random_rule = components.Random(n_stat, n_act)
    model = components.RuleLayer([dqn_rule, ball_rule,random_rule])
                                  
    if args.initmodel:
        print('Load model from', args.initmodel)
        serializers.load_npz(args.initmodel, dqn_rule.agent.q)

    # Change to your environment
    env = environments.SoccerEnv(model)
    i = 0
    while True:
        env.execute()

        i = i + 1
        if i % 10 == 0:
           print 'save the model'
           dqn_rule.agent.save(index=i)

