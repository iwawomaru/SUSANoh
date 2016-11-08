from susanoh.environment import Environment
from susanoh.environments.gazebo_action import GazeboAction
from susanoh.environments.gazebo_utils import get_ball_location, reset_world

import rospy
import roslaunch

import subprocess
import os
import time

import numpy as np

class GazeboEnv(Environment):

    n_stat = 0
    n_act = 0
    episode_size = 0

    def __init__(self, model, gazebo_launch_name, render=False):
        super(GazeboEnv, self).__init__(model, render)

        fullpath = os.path.join("/home/osawa/SUSANoh/worlds", gazebo_launch_name)
        dir_path = "dir:=/home/osawa/SUSANoh/worlds"
        subprocess.Popen(["roslaunch", fullpath, dir_path])

        self.action_getter = GazeboAction()
        self.episode_number = 0
        self.reward_list = []
        self.prev_ball_pos = 3.25

    def step(self, action, is_end=False):
        self.action_getter.control_action(action)
        obs = self.action_getter.get_image_array()
        ball_loc = get_ball_location()

        if ball_loc[0] > 4.25:
            reward = 1.
            done = True
            self.prev_ball_pos = 3.25
            print "==============================="
            print "======= GOOOOOOOOOOOL!! ======="
            print "==============================="
        elif is_end:
            reward = -1.
            done = True
            print "******* fail *******"
        else:
            reward = max(ball_loc[0] - self.prev_ball_pos, 0)
            # reward = 0
            self.prev_ball_pos = ball_loc[0]
            done = False
        info = None
        return obs, reward, done, info

    def reinforcement_train(self):
        """
        TODO: Consider how and when models should be updated.
        """
        pass

    def execute(self, epochs=None):
        episode_reward = 0
        observation, reward, done, info = self.reset()
        rate = rospy.Rate(5)
        self.prev_ball_pos = 3.25
        for frame in xrange(self.__class__.episode_size):
            if observation is not None:
                self.model.set_reward(reward)
                action = self.model.reinforcement_train(data=observation)
            else:
                print "observation was None"
                action = 0

            is_end = (frame == self.__class__.episode_size-1)
            observation, reward, done, info = self.step(action, is_end)
            episode_reward += reward

            if done: break
            rate.sleep()

        # self.model.reinforcement_train()
        self.reward_list.append(episode_reward)
        self.episode_number += 1
        print ('ep %d: game finished, reward: %f' %
               (self.episode_number, episode_reward))
        

    def reset(self):
        reset_world(robot_x=3.0-np.random.rand()*0.5, 
                    robot_y=np.random.rand()-0.5)
        obs = self.action_getter.get_image_array()
        reward = 0
        done = False
        info = None
        return obs, reward, done, info

    def __del__(self):
        f = open("res.txt", "w")
        for rew in self.reward_list:
            f.write(str(rew)+"\n")
        f.close()

        # Kill gzclient, gzserver and roscore
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count('gzclient')
        gzserver_count = tmp.count('gzserver')
        roscore_count = tmp.count('roscore')
        rosmaster_count = tmp.count('rosmaster')
        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        if rosmaster_count > 0:
            os.system("killall -9 rosmaster")
        if roscore_count > 0:
            os.system("killall -9 roscore")

        if (gzclient_count or gzserver_count or 
            roscore_count or rosmaster_count >0):
            os.wait()

class SoccerEnv(GazeboEnv):
    n_stat = 60 * 60 * 3
    n_act = 5
    episode_size = 50

    def __init__(self, model, render=False):
        super(SoccerEnv, self).__init__(model, "test.launch", render)

