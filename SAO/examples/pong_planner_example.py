import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')

from noh.environments import Pong
from noh.components import SuppressionBoosting


if __name__ == "__main__":

    n_stat = Pong.n_stat
    n_act = Pong.n_act

    model = SuppressionBoosting.create(n_stat, n_act, n_learner=4)

    env = Pong(model, render=True)
    while True:
        env.execute()
