# -*- coding: utf-8 -*-

from susanoh import Component
from susanoh import Visualizer

class Rule(Component, Visualizer):
    def __init__(self, rule_func, bicamon_server=None):
        super(Rule, self).__init__()
        self.rule_func = rule_func
        # for BiCAmon
        self.server = bicamon_server
    
    def __call__(self, data, **kwargs):
        action = self.rule_func(data, **kwargs)
        
        if action is not None:
            # send to BiCAmon
            self.send_to_viewer('SCm')
            self.send_to_viewer('SCs')
        return action

    def supervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):  pass
