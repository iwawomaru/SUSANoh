# -*- coding: utf-8 -*-

from susanoh import Component
from susanoh import Visualizer

class RuleLayer(Component, Visualizer):
    def __init__(self, rules, default_action=0, bicamon_server=None):
        super(RuleLayer, self).__init__()
        self.rules = rules
        self.default_action = default_action
        # for BiCAmon
        self.server = bicamon_server
        # for debug
        self.call_counter = [0 for i in xrange(len(self.rules))]

    def __call__(self, data, **kwargs):
        for i,r in enumerate(self.rules):
            action = r(data, **kwargs)
            if action is not None:
                self.send_to_viewer('MOp')
                self.send_to_viewer('MOs')
                self.call_counter[i] += 1
                return action
        self.send_to_viewer('MOp')
        self.send_to_viewer('MOs')
        return self.default_action

    def accum_reinforcement_train(self, data, action):
        for r in self.rules:
            r.accum_reinforcement_train(data, action)

    def set_reward(self, reward):
        for r in self.rules:
            r.set_reward(reward)
        
    def reset_counter(self):
        self.call_counter = [0 for i in xrange(len(self.rules))]
