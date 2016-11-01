import sys,os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')

from noh.components import Random, Const, DQN
from noh.environments import Pong

n_stat = Pong.n_stat
n_act = Pong.n_act

# model = Random(n_input=n_stat, n_output=n_act)
model = DQN(n_input=n_stat, n_output=n_act)
env = Pong(model, render=True)
while True:
    env.execute()
