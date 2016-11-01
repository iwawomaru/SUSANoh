# -*- coding: utf-8 -*-

# for Python3 style
from __future__ import print_function
import six

import numpy as np

# from SUSANoh
from susanoh.environment import Environment


class TestEnv(Environment):
    n_stat = 100
    n_act = 5
    episode_size = 10
    dummy_output = np.zeros((256, 256, 3), dtype=np.float32)

    def __init__(self, model):
        super(TestEnv, self).__init__(model, render=False)
        self.action_getter = TestAction()
        self.episode_number = 0

    def step(self, action):
        self.action_getter.control_action(action)
        obs = self.action_getter.get_image_array()
        reward = np.random.normal(0, 1)
        done = np.random.randn() < 0.1
        info = None
        return obs, reward, done, info

    def execute(self, epochs=None):
        episode_reward = 0
        observation = self.reset()
        done = False

        for frame in six.moves.range(self.__class__.episode_size):
            action = self.model(observation)
            print('action={}'.format(action))

            observation, reward, done, info = self.step(action)
            self.model.set_reward(reward)
            episode_reward += reward
            if done:
                break

        self.model.reinforcement_train()
        self.episode_number += 1
        print('ep {0:d}: game finished, reward: {1:f}'.format(
               self.episode_number, episode_reward))


    def reset(self):
        return self.dummy_output

    def print_stat(self):
        return self.stat



class TestAction(object):
    dummy_output = np.zeros((256, 256, 3), dtype=np.float32)

    def __init__(self):
        # NOP
        pass

    def control_action(self, action):
        # NOP
        pass

    def get_image_array(self):
        return self.dummy_output

