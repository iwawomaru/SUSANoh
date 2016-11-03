# -*- coding: utf-8 -*-

from susanoh import Component

class RuleLayer(Component):
    def __init__(self, rules, default_action=0):
        super(RuleLayer, self).__init__()
        self.rules = rules
        self.default_action = default_action


    def __call__(self, data, **kwargs):
        for r in self.rules:
            action = r(data, **kwargs)
            if action is not None:
                return action
        return self.default_action
