# -*- coding: utf-8 -*-

from susanoh import Component
from susanoh import Visualizer

class Rule(Component, Visualizer):
    def __init__(self, rule_func, bicamon_server=None):
        super(Rule, self).__init__(server=bicamon_server)
        self.rule_func = rule_func
    
    def __call__(self, data, **kwargs):
        return self.rule_func(data, **kwargs)

    def supervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):  pass
