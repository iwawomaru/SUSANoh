# -*- coding: utf-8 -*-

import sys,os
from chainer import serializers
import argparse

# for Python3 style
import six

import numpy as np

# from SUSANoh
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from susanoh import components, environments

action_table = {
        'stop':0,
        'up':2,
        'down':3,
        }

def ball_rule(data, paddle_length=8):
    bx, by, py = data
    if by is not None: 
        if by < py - paddle_length/2:
            return 2
        elif by > py + paddle_length/2:
            return 3
    return None

if __name__ == '__main__':

    # Change to your environment
    n_stat = environments.Pong.n_stat
    n_act = environments.Pong.n_act

    # setup model
    ball_rule = components.PongRule(ball_rule)
    dqn_rule = components.DQN(n_stat, n_act, L1_rate=None, on_gpu=True)
    model = components.RuleLayer([dqn_rule, ball_rule])

    # Change to your environment
    env = environments.Pong(model, render=False)

    while True:
        env.execute()
