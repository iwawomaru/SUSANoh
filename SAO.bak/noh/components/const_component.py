from noh import Component

class Const(Component):
    def __init__(self, n_input, n_output, const_output):
        super(Const, self).__init__()
        self.n_input = n_input
        self.n_output = n_output
        self.const_output = const_output

    def __call__(self, data, **kwargs):
        return self.const_output

    def supervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def unsupervised_train(self, data=None, label=None, epochs=None, **kwargs): pass
    def reinforcement_train(self, data=None, label=None, epochs=None, **kwargs):  pass
