from __future__ import print_function

from susanoh import Component
from susanoh import Visualizer

import os
import numpy as np
from chainer import initializers
from chainer import Chain
from chainer import Variable
from chainer import cuda
from chainer import serializers
from chainer import optimizer, optimizers
import chainer.functions as F
from susanoh.components.agent import Agent

import time

# This is Component 
class DQN(Component, Visualizer):
    def __init__(self, n_input, n_output, L1_rate=None, on_gpu=False, 
                 epsilon=1, model_path="",
                 bicamon_server=None):
        self.n_input = n_input
        self.n_output = n_output
        self.agent = DQNAgent(n_output, epsilon=epsilon, 
                              model_path=model_path, on_gpu=on_gpu,
                              bicamon_server=bicamon_server)
        self.trainer = DQNTrainer(self.agent, L1_rate=L1_rate, bicamon_server=bicamon_server)
        self.accum = [0, 0, 0, 0, 0]
        self.action = None
        # for BiCAmon
        self.server = bicamon_server
        
    def __call__(self, data, **kwargs):
        #self.rng = np.random.RandomState(123)
        action = int(self.trainer.start(data))

        # send to BiCAmon
        self.send_to_viewer('ACAd')
        self.send_to_viewer('ACAv')

        self.accum[action] += 0.3
        if self.accum[action] >= 1.0:
            #self.accums = [0, 0, 0, 0, 0]
            self.action = action
        else:
            self.action = None
        for i in xrange(len(self.accum)):
            self.accum[i] *= 0.8
        return self.action
    
    def set_reward(self, reward):
        if reward > 0:
            # send to BiCAmon
            self.send_to_viewer('ACB')
        elif reward < 0:
            # send to BiCAmon
            self.send_to_viewer('COAp')
            self.send_to_viewer('MEA')
            self.send_to_viewer('BLA')
            self.send_to_viewer('CEA')
            self.send_to_viewer('BMA')
            self.send_to_viewer('AAA')

        self.reward = reward
        
    def supervised_train(self, data=None, label=None, epoch=None, **kwargs): pass

    def unsupervised_train(self, data=None, label=None, epoch=None, **kwargs): pass

    def reinforcement_train(self, data=None, label=None, epoch=None, **kwargs):
        #print " learning..."
        return self.trainer.act(data, self.reward)

    # for accumrater
    def accum_reinforcement_train(self, data, action):
        return self.trainer.accum_act(data, action, self.reward)

    
# These class is for making DQN Component
class Q(Chain):
    """
    This is DQN written in chainer
    """

    SIZE = 60  # Observation SIZE

    def __init__(self, n_history, n_action, on_gpu=False):
        self.n_history = n_history
        self.n_action = n_action
        self.on_gpu = on_gpu
        # initializer = initializers.HeNormal()
        super(Q, self).__init__(
            l1 = F.Convolution2D(n_history, 64, ksize=5, stride=2, 
                                 nobias=False, wscale=np.sqrt(2)),
            l2 = F.Convolution2D(64, 64, ksize=3, stride=1, 
                                 nobias=False, wscale=np.sqrt(2)),
            l3 = F.Convolution2D(64, 64, ksize=3, stride=1, 
                                 nobias=False, wscale=np.sqrt(2)),
            #l4 = F.Linear(None, 2048, wscale=np.sqrt(2)),
            out = F.Linear(None, self.n_action, wscale=np.sqrt(2))
        )
        if on_gpu:
            self.to_gpu()

    def __call__(self, state):
        _state = self.arp_to_gpu(state)
        s = Variable(_state)
        h1 = F.relu(self.l1(s))
        h2 = F.relu(self.l2(h1))
        h3 = F.relu(self.l3(h2))
        # h4 = F.relu(self.l4(h3))
        q_value = self.out(h3)
        return q_value

    def arp_to_gpu(self, arp):
        return arp if not self.on_gpu else cuda.to_gpu(arp)


class DQNAgent(Agent, Visualizer):

    def __init__(self, actions, epsilon=1., n_history=4, on_gpu=False, 
                 model_path="", load_if_exist=True,
                 bicamon_server=None):
        self.actions = actions
        self.epsilon = epsilon
        self.q = Q(n_history, actions, on_gpu)
        self._state = []
        self._observations = [
            np.zeros((self.q.SIZE, self.q.SIZE), np.float32),
            np.zeros((self.q.SIZE, self.q.SIZE), np.float32)
        ] # now & pre
        self.last_action  = 0
        self.model_path = model_path if model_path else os.path.join(os.path.dirname(__file__), "./store")
        if not os.path.exists(self.model_path):
            print("make directory to store model at {0}".format(self.model_path))
            os.mkdir(self.model_path)
        else:
            models = self.get_model_files()
            """
            if load_if_exist and len(models) > 0:
                print("load model file {0}.".format(models[-1]))
                serializers.load_npz(os.path.join(self.model_path, models[-1]), self.q)
                serializers.load_npz(os.path.join(self.model_path, 
            models[-1]), self.q)
            """
        # for BiCAmon
        self.server = bicamon_server

    #stock 4 frame to send DQN
    def _update_state(self, observation):
        # get only Blue channel (observation is BGR)
        formatted = observation[:,:,0]
        # formatted = observation
        formatted = formatted.astype(np.float32)
        #formatted = observation.transpose(2, 0, 1).astype(np.float32)
        # state = formatted
        state = np.maximum(formatted, self._observations[0])
        self._state.append(state)
        if len(self._state) > self.q.n_history:
            self._state.pop(0)
        return formatted

    def start(self, observation):
        self._state = []
        self._observations = [
            np.zeros((self.q.SIZE, self.q.SIZE), np.float32),
            np.zeros((self.q.SIZE, self.q.SIZE), np.float32)
        ]
        self.last_action = 0

        action = self.act(observation, 0)
        return action

    def act(self, observation, reward):
        # send to BiCAmon
        self.send_to_viewer('VISp')
        self.send_to_viewer('VISam')
        self.send_to_viewer('VISpm')
        self.send_to_viewer('VISI')
        self.send_to_viewer('VISpl')

        o = self._update_state(observation)
        s = self.get_state()
        qv = self.q(np.array([s]))
        # print np.argmax(qv.data[-1])
        
        if np.random.rand() < self.epsilon:
            action  = np.random.randint(0, self.actions)
            # print "== DQN Random : ", action, "eps = ", self.epsilon
        else:
            action = np.argmax(qv.data[-1])
            print("== DQN argmax qv : ", qv.data[-1])
        # self.epsilon *= self.

        self._observations[-1] = self._observations[0].copy()
        self._observations[0] = o
        self.last_action = action

        return action

    def get_state(self):
        state = []
        for i in range(self.q.n_history):
            if i < len(self._state):
                state.append(self._state[i])
            else:
                state.append(np.zeros((self.q.SIZE, self.q.SIZE), dtype=np.float32))

        np_state = np.stack(state, axis=0)
        return np_state

    def save(self, index=0):
        fname = "dqn.model" if index == 0 else "dqn_{0}.model".format(index)
        path = os.path.join(self.model_path, fname)
        serializers.save_npz(path, self.q)

    def get_model_files(self):
        files = os.listdir(self.model_path)
        model_files = []
        for f in files:
            if f.startswith("pong") and f.endswith(".model"):
                model_files.append(f)
        model_files.sort()
        return model_files


class DQNTrainer(Agent, Visualizer):

    def __init__(self, agent, memory_size=10**4, replay_size=32, gamma=0.99, 
                 initial_exploration=2500, target_update_freq=2500,
                 learning_rate=0.00025, epsilon_decay=25000,
                 minimum_epsilon=0.1,L1_rate=None,
                 bicamon_server=None):
        self.agent = agent
        self.target = Q(self.agent.q.n_history, self.agent.q.n_action, 
                        on_gpu=self.agent.q.on_gpu)

        self.memory_size = memory_size
        self.replay_size = replay_size
        self.gamma = gamma
        self.initial_exploration = initial_exploration
        self.target_update_freq = target_update_freq
        self.laerning_rate = learning_rate
        self.epsilon_decay = epsilon_decay
        self.minimum_epsilon = minimum_epsilon
        self._step = 0
        model_path = ""
        self.model_path = model_path if model_path else os.path.join(os.path.dirname(__file__), "./store")

        # prepare for replay
        n_hist = self.agent.q.n_history
        size = self.agent.q.SIZE
        self.memory = [
            np.zeros((memory_size, n_hist, size, size), dtype=np.float32),
            np.zeros(memory_size, dtype=np.uint8),
            np.zeros((memory_size, 1),dtype=np.float32),
            np.zeros((memory_size, n_hist, size, size), dtype=np.float32),
            np.zeros((memory_size, 1), dtype=np.bool)]
        self.memory_text = ["state", "action", "reward", 
                            "next_state", "episode_end"]

        #prepare optimizer
        self.optimizer  = optimizers.RMSpropGraves(lr=learning_rate,alpha=0.95, momentum=0.95, eps=0.01)
        self.optimizer.setup(self.agent.q)
        if L1_rate is not None:
            self.optimizer.add_hook(optimizer.Lasso(L1_rate))
        self._loss = 9
        self._qv = 0

        # for BiCAmon
        self.server = bicamon_server
            
    def save(self, index=0):
        fname = "dqn.state" if index == 0 else "dqn_{0}.state".format(index)
        path = os.path.join(self.model_path, fname)
        serializers.save_npz(path, self.optimizer)

    def calc_loss(self, states, actions, rewards, next_states, episode_ends):
        # send to BiCAmon
        self.send_to_viewer('CP')
        self.send_to_viewer('LSc')

        qv = self.agent.q(states)
        q_t = self.target(next_states)

        # max_a Q(s', a)
        max_q_prime = np.array(list(map(np.max, q_t.data)), dtype=np.float32) 

        target = cuda.to_cpu(qv.data.copy())
        for i in range(self.replay_size):
            if episode_ends[i][0] is True:
                _r = np.sign(rewards[i])
            else:
                _r = np.sign(rewards[i]) + self.gamma * max_q_prime[i]

            target[i, actions[i]] = _r

        td = Variable(self.target.arp_to_gpu(target)) - qv
        td_tmp = td.data + 1000.0 * (abs(td.data) <= 1) # Avoid zero division
        td_clip = td * (abs(td.data) <= 1) + td/abs(td_tmp) * (abs(td.data) > 1)

        zeros = Variable(self.target.arp_to_gpu(np.zeros((self.replay_size, self.target.n_action), dtype=np.float32)))
        loss = F.mean_squared_error(td_clip, zeros)
        self._loss = loss.data
        self.__qv = np.max(qv.data)
        return loss

    def start(self, observation):
        return self.agent.start(observation)

    def act(self, observation, reward):
        if self.initial_exploration <= self._step:
            self.agent.epsilon -= self.epsilon_decay
            if self.agent.epsilon < self.minimum_epsilon:
                self.agent.epsilon = self.minimum_epsilon

        return self.train(observation, reward, episode_end=False)

    def accum_act(self, observation, action, reward):
        
        if self.initial_exploration <= self._step:
            self.agent.epsilon -= self.epsilon_decay
            if self.agent.epsilon < self.minimum_epsilon:
                self.agent.epsilon = self.minimum_epsilon

        return self.accum_train(observation, action, reward, episode_end=False)

    def train(self, observation, reward, episode_end):
        action = 0
        last_state = self.agent.get_state()
        last_action = self.agent.last_action
        if not episode_end:
            action = self.agent.act(observation, reward)
            result_state = self.agent.get_state()
            self.memorize(last_state, last_action, reward, result_state, False)
        else:
            self.memorize(last_state, last_action, reward, last_state, True)

        if self.initial_exploration <= self._step:
            self.experience_replay()

            if self._step % self.target_update_freq == 0:
                print("===== copy params ")
                self.target.copyparams(self.agent.q)

        self._step += 1
        return action
    
    def accum_train(self, observation, action, reward,  episode_end):
        last_state = self.agent.get_state()
        last_action = self.agent.last_action
        if not episode_end:
            dummy_action = self.agent.act(observation, reward)
            result_state = self.agent.get_state()
            self.memorize(last_state, action, reward, result_state, False)
        else:
            self.memorize(last_state, last_action, reward, last_state, True)

        if self.initial_exploration <= self._step:
            self.experience_replay()

            if self._step % self.target_update_freq == 0:
                print("===== copy params ")
                self.target.copyparams(self.agent.q)
        self._step += 1
        return action

    def memorize(self, state, action, reward, next_state, episode_end):
        _index = self._step % self.memory_size
        self.memory[0][_index] = state
        self.memory[1][_index] = action
        self.memory[2][_index] = reward
        if not episode_end:
            self.memory[3][_index] = next_state
        self.memory[4][_index] = episode_end

    def experience_replay(self):
        # send to BiCAmon
        self.send_to_viewer('ENTI')
        self.send_to_viewer('DG')
        self.send_to_viewer('CA3')
        self.send_to_viewer('CA1')
        self.send_to_viewer('ENTm')
        self.send_to_viewer('PAR')
        self.send_to_viewer('POST')

        indices = []
        if self._step < self.memory_size:
            indices = np.random.randint(0, self._step, (self.replay_size))
        else:
            indices = np.random.randint(0, self.memory_size, (self.replay_size))

        states = []
        actions = []
        rewards = []
        next_states = []
        episode_ends = []
        for i in indices:
            states.append(self.memory[0][i])
            actions.append(self.memory[1][i])
            rewards.append(self.memory[2][i])
            next_states.append(self.memory[3][i])
            episode_ends.append(self.memory[4][i])

        to_np = lambda arp: np.array(arp)
        self.optimizer.target.cleargrads()
        loss = self.calc_loss(to_np(states), to_np(actions), to_np(rewards), to_np(next_states), to_np(episode_ends))
        loss.backward()
        self.optimizer.update()

    def report(self, episode):
        s = "{0}: loss={1}, q value={2}, epsilon={3}".format(self._step, self._loss, self._qv, self.agent.epsilon)
        self.agent.save(episode)
        return s
    
        
