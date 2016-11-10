# -*- coding: utf-8 -*-

from susanoh import Component
from susanoh import Visualizer

class RuleLayer(Component, Visualizer):
    def __init__(self, rules, default_action=0, bicamon_server=None):
        super(RuleLayer, self).__init__(server=bicamon_server)
        self.rules = rules
        self.default_action = default_action


    def __call__(self, data, **kwargs):
        for r in self.rules:
            action = r(data, **kwargs)
            if action is not None:
                self.send_to_viewer('MOp')
                self.send_to_viewer('MOs')
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
        
