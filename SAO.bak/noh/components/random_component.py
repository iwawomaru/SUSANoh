from noh import Component
import numpy as np

class Random(Component):
    def __init__(self, n_input, n_output, output_type="id"):
        super(Random, self).__init__()
        self.n_input = n_input
        self.n_output = n_output
        self.output_type = output_type

    def __call__(self, data, **kwargs):

        if self.output_type == "one-hot":
            id = self.rng.randint(0, self.n_output)
            res = [0 if i == id else 1 for i in xrange(self.n_output)]
            return np.array(res)
        elif self.output_type == "binary-vector":
            res = self.rng.randint(0, 1, size=self.n_output)
            return res
        elif self.output_type == "continuous-value-vector":
            res = np.random.rand(self.n_output)
            return res
        elif self.output_type == "id":
            res = self.rng.randint(0, self.n_output)
            return res

        return None

    def supervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):  pass
