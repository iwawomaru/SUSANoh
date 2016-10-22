from susanoh.environment import Environment
import numpy as np


class GazeboEnv(Environment):

    n_stat = 0
    n_act = 0
    episode_size = 0

    @classmethod
    def gen_reward_reset_checker(cls):
        return lambda x: False

    def __init__(self, model, gazebo_launch_name, render=False):
        super(GazeboEnv, self).__init__(model, render)

        """ TODO: launch ros """

        self.action = None
        self.frame = 0
        self.default_positive_reward = 1.
        self.default_negative_reward = -1.
        self.episode_number = 0

    def step(self, action):
        return self.env.step(action)

    def reinforcement_train(self):
        """
        TODO: Consider how and when models should be updated.
        """
        pass

    def execute(self, epochs=None):
        episode_reward = 0

        observation = self.env.reset()
        done = False
        reward = 0

        for frame in xrange(self.__class__.episode_size):
            if self.render: self.print_stat()
            action = self.model(observation)
            observation, reward, done, info = self.step(action)
            self.model.set_reward(reward) # does model should have history of reward? environment is better?
            episode_reward += reward
            if done: break

        if not done:
            self.model.set_reward(self.default_negative_reward)
            episode_reward += (-reward + self.default_negative_reward)

        self.model.reinforcement_train()
        self.episode_number += 1
        print ('ep %d: game finished, reward: %f' %
               (self.episode_number, episode_reward))

    def print_stat(self):
        self.env.render()

    def reset(self):
        return self.env.reset()


def SoccerEnv(GazeboEnv):
    n_stat = 0
    n_act = 0
    episode_size = 0

    def __init__(self, model, render=False):
        super(GazeboEnv, self).__init__(model, "soccer_field.xml", render)

        """ TODO: launch ros """
