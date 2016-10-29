import sys,os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')

from susanoh.components import Random, Const
from susanoh.environments import SoccerEnv


n_stat = SoccerEnv.n_stat
n_act = SoccerEnv.n_act

# model = Random(n_input=n_stat, n_output=n_act)
model = Random(n_input=n_stat, n_output=n_act, output_type="id")
env = SoccerEnv(model, render=True)
while True:
    env.execute()
