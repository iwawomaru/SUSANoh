# -*- coding: utf-8 -*-

import numpy
import skimage.draw, skimage.feature, skimage.color, skimage.transform
from susanoh.components import Rule

class BallRule(Rule):
    ####################################################################
    # !! Your rule_func should be able to take and process tuple (x, y)# 
    # which is center position of the ball.                            #
    ####################################################################
    def __init__(self, rule_func, t_size=32, t_r=12, t_top=0.75, proc_size=(256,256)):
        super(BallRule, self).__init__(rule_func)
        self.proc_size = proc_size
        # create template
        ones = numpy.ones((t_size, t_size), dtype=numpy.float32)
        rr, cc = skimage.draw.circle(ones.shape[0]/2, ones.shape[1]/2, t_r)
        ones[rr, cc] = 0
        self.template = ones[:int(t_size*t_top)]

    def __call__(self, data, **kwargs):
        c = self.ball_detecotion(data)
        return self.rule_func(c, **kwargs)

    def ball_detecotion(self, data):
        # chack input channels
        if data.ndim==3 and data.shape[-1] != 1:
            data = skimage.color.rgb2gray(data)
        elif data.ndim != 2:
            raise ValueError('Invalid array format')
        # resize image for detection
        d = skimage.transform.resize(data, self.proc_size)
        res = skimage.feature.match_template(d, self.template)
        ij = numpy.unravel_index(numpy.argmax(res), res.shape)
        x, y = ij[::-1]
        # return (x, y) in resized space
        return x+self.template.shape[1]/2, y+self.template.shape[0]

    def supervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):  pass

