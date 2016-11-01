from susanoh import Component

import os
import numpy as np
from chainer import Chain
from chainer import Variable
from chainer import cuda
from chainer import serializers
from chainer import optimizer, optimizers
import chainer.functions as F
from agent import Agent


# This is Component 
class DQN(Component):
    def __init__(self, n_input, n_output, L1_rate=None):
        self.n_input = n_input
        self.n_output = n_output
        self.agent = DQNAgent(n_output, epsilon=0.01, model_path="", on_gpu=False)
        self.trainer = DQNTrainer(self.agent, L1_rate=L1_rate)
        
    def __call__(self, data, **kwargs):
        #self.rng = np.random.RandomState(123)
        self.action = self.trainer.start(data)
        return self.trainer.start(data)
        

    def set_reward(self, reward):
        self.reward = reward
        
    def supervised_train(self, data=None, label=None, epoch=None, **kwargs): pass

    def unsupervised_train(self, data=None, label=None, epoch=None, **kwargs): pass

    def reinforcement_train(self, data=None, label=None, epoch=None, **kwargs):
        #print " learning..."
        self.trainer.act(data,self.reward)
    
    


# These class is for making DQN Component
class Q(Chain):
    """
    This is DQN written in chainer
    """

    SIZE = 80  # Pong is 80*80 image to use

    def __init__(self, n_history, n_action, on_gpu=False):
        self.n_history = n_history
        self.n_action = n_action
        self.on_gpu = on_gpu
        super(Q, self).__init__(
            l1 = F.Convolution2D(n_history, 32, ksize=8, stride=4, nobias=False, wscale=np.sqrt(2)),
            l2 = F.Convolution2D(32, 64, ksize=3, stride=2, nobias=False, wscale=np.sqrt(2)),
            l3 = F.Convolution2D(64, 64, ksize=3, stride=1, nobias=False, wscale=np.sqrt(2)),
            l4 = F.Linear(3136, 512, wscale=np.sqrt(2)),
            out = F.Linear(512, self.n_action, wscale=np.sqrt(2))
        )
        if on_gpu:
            self.to_gpu()

    def __call__(self, state):
        _state = self.arp_to_gpu(state)
        s = Variable(_state)
        h1 = F.relu(self.l1(s))
        h2 = F.relu(self.l2(h1))
        h3 = F.relu(self.l3(h2))
        h4 = F.relu(self.l4(h3))
        q_value = self.out(h4)
        return q_value

    def arp_to_gpu(self, arp):
        return arp if not self.on_gpu else cuda.to_gpu(arp)


class DQNAgent(Agent):

    def __init__(self, 
            actions, 
            epsilon=1, 
            n_history=4, 
            on_gpu=False, 
            model_path="", 
            load_if_exist=True):
        self.actions = actions
        self.epsilon = epsilon
        self.q = Q(n_history, actions, on_gpu)
        self._state = []
        self._observation = [
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
            if load_if_exist and len(models) > 0:
                print("load model file {0}.".format(models[-1]))
                serializers.load_npz(os.path.join(self.model_path, models[-1]), self.q)

    #stock 4 frame to send DQN
    def _update_state(self, observation):
        formatted = observation.reshape((80,80)).astype(np.float32)
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
        o = self._update_state(observation)
        s = self.get_state()
        qv = self.q(np.array([s]))
        #print np.argmax(qv.data[-1])
        
        if np.random.rand() < self.epsilon:
            action  = np.random.randint(0, self.actions)
        else:
            action = np.argmax(qv.data[-1])

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

        np.state = np.array(state)
        return np.state

    def save(self, index=0):
        fname = "pong.model" if index == 0 else "pong_{0}.model".format(index)
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


class DQNTrainer(Agent):

    def __init__(self, 
            agent, 
            memory_size=10**4, 
            replay_size=32, 
            gamma=0.99, 
            initial_exploration=10**4, 
            target_update_freq=10**4,
            learning_rate=0.00025, 
            epsilon_decay=1e-6, 
            minimum_epsilon=0.1,
            L1_rate=None):
        self.agent = agent
        self.target = Q(self.agent.q.n_history, self.agent.q.n_action, on_gpu=self.agent.q.on_gpu)

        self.memory_size = memory_size
        self.replay_size = replay_size
        self.gamma = gamma
        self.initial_exploration = initial_exploration
        self.target_update_freq = target_update_freq
        self.laerning_rate = learning_rate
        self.epslon_decay = epsilon_decay
        self.minimum_epsilon = minimum_epsilon
        self._step = 0

        # prepare for replay
        n_hist = self.agent.q.n_history
        size = self.agent.q.SIZE
        self.memory = [
            np.zeros((memory_size, n_hist, size, size), dtype=np.float32),
            np.zeros(memory_size, dtype=np.uint8),
            np.zeros((memory_size, 1),dtype=np.float32),
            np.zeros((memory_size, n_hist, size, size), dtype=np.float32),
            np.zeros((memory_size, 1), dtype=np.bool)
        ]
        self.memory_text = [
            "state", "action", "reward", "next_state", "episode_end"
        ]

        #prepare optimizer
        self.optimizer  = optimizers.RMSpropGraves(lr=learning_rate, alpha=0.95, momentum=0.95, eps=0.01)
        self.optimizer.setup(self.agent.q)
        if L1_rate is not None:
            self.optimizer.add_hook(optimizer.Lasso(L1_rate))
        self._loss = 9
        self._qv = 0

    def calc_loss(self, states, actions, rewards, next_states, episode_ends):
        qv = self.agent.q(states)
        q_t = self.target(next_states)
        max_q_prime = np.array(list(map(np.max, q_t.data)), dtype=np.float32) # max_a Q(s', a)

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
            self.agent.epsilon -= 1.0/10**6
            if self.agent.epsilon < self.minimum_epsilon:
                self.agent.epsilon = self.minimum_epsilon

        return self.train(observation, reward, episode_end=False)

    def train(self, observation, reward, episode_end):
        action = 0
        last_state = self.agent.get_state()
        last_action = self.agent.last_action
        if not episode_end:
            action = self.agent.act(observation, reward)
            result_state = self.agent.get_state()
            self.memorize(
                last_state,
                last_action,
                reward,
                result_state,
                False
            )
        else:
            self.memorize(
                last_state,
                last_action,
                reward,
                last_state,
                True
            )

        if self.initial_exploration <= self._step:
            self.experience_replay()

            if self._step % self.target_update_freq == 0:
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
    
        
