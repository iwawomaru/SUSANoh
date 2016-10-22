from abc import ABCMeta, abstractmethod
import warnings, pickle
import numpy as np

class Component(object):
    """ Callable Component for Circuits with training capability.
    Bind, Circuit, and all component implementations defined in :mod:
    `noh.components` inherit this class.
    :class: `Component`s are the basic building blocks for :class: `Circuits`
    which must provide interfaces to be called and trained. Every component is
    expected to have the :meth: `__call__` method return a meaningful value
    given some sort of input. This is because every :class: `Component` is
    treated as a function which calculates a mapping of a given input.
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        self.rng = np.random.RandomState(123)

    @abstractmethod
    def __call__(self, data, **kwargs):
        raise NotImplementedError("`__call__` must be explicitly overridden")

    def supervised_train(self, data=None, label=None, epochs=None, **kwargs):
        warnings.warn("supervised_train will do nothing")

    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs):
        warnings.warn("unsupervised_train will do nothing")

    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):
        warnings.warn("reinforcemnt_train will do nothing")

    def set_reward(self, reward):
        pass

    def save(self, filename):
        f = open(filename, 'w')
        pickle.dump(self, f)
        f.close()


    @staticmethod
    def load(filename):
        f = open(filename, 'r')
        loaded = pickle.load(f)
        f.close()
        return loaded