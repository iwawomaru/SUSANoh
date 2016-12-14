# -*- coding: utf-8 -*-

import numpy
from susanoh.components import Rule


class PongRule(Rule):

    paddle_length = 8

    def __init__(self, rule_func, bicamon_server=None):
        super(PongRule, self).__init__(rule_func, bicamon_server=bicamon_server)
        self.i = 0

    def __call__(self, data, **kwargs):
        c = self.ball_detecotion(data)
        return self.rule_func(c, **kwargs)

    def ball_detecotion(self, data):
        # chack input channels
        if data.ndim != 2:
            raise ValueError('Invalid array format')
        # return (ball_x, ball_y, paddle_y)
        bx, by = self._detect_ball(data)
        py = self._detect_paddle(data)
        return bx, by, py

    def _format(self, image, area):
        im = image[:, area[0]:area[1]] # crop
        return im.astype(numpy.float32)

    def _detect_ball(self, observation):
        obs = self._format(observation, (40, 70))
        self.i += 1
        if not obs.max() == 0:
            y = obs.argmax(axis=0).max()
            x = obs.argmax(axis=0).argmax()
            return x, y
        else:
            return None, None

    def _detect_paddle(self, observation):
        obs = self._format(observation, (71, 72))
        return obs[:,0].argmax()+self.paddle_length/2

    def set_reward(self, reward): pass
    def supervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):  pass
    def accum_reinforcement_train(self, data=None, action=None): pass
