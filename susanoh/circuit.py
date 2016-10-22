from noh.component import Component


class Collection(object):
    keys = []
    values = []

    def __init__(self, collection):
        if isinstance(collection, dict):
            for key, value in collection.items():
                self.keys.append(key)
                self.values.append(key)

        if isinstance(collection, list):
            keys = [item.__class__.__name__.lower() for item in collection]

            counts = {}

            for key, value in zip(keys, collection):
                if keys.count(key) > 1:
                    if key not in counts:
                        counts[key] = 0
                    counts[key] += 1
                    key = '{}{}'.format(key, counts[key])
                self.keys.append(key)
                self.values.append(value)

    def __getitem__(self, key):
        if isinstance(key, int):
            index = key
        else:
            index = self.keys.index(key)
        return self.values[index]

    def __setitem__(self, key, value):
        if isinstance(key, int):
            index = key
        else:
            index = self.keys.index(key)
        self.values[index] = value

    def __delitem__(self, key):
        if isinstance(key, int):
            index = key
        else:
            index = self.keys.index(key)
        del self.keys[key]
        del self.values[key]

    def __iter__(self):
        return iter(self.keys)

    # def __getslice__(self, i, j):
    #     raise NotImplementedError("To be implemented")

    # def __setslice__(self, i, j, values):
    #     raise NotImplementedError("To be implemented")

    # def __delslice__(self, i, j):
    #     raise NotImplementedError("To be implemented")

    def __getattr__(self, key):
        return self.__getitem__(key)


class PropRule(Collection):
    def __init__(self, components):
        super(PropRule, self).__init__(components)
        self.components = components

    def __call__(self, data):
        raise NotImplementedError("`__call__` must be explicitly overridden")


class Circuit(Collection, Component):
    def __init__(self, components, RuleClassDict, default_prop_name=None, default_train_name=None):
        super(Circuit, self).__init__(components)
        self.components = components
        self.rules = {}
        self.default_prop = None
        self.default_train = None

        for name in RuleClassDict:
            RuleClass = RuleClassDict[name]
            self.rules[name] = RuleClass(components)

        if default_prop_name is not None:
            self.set_default_prop(default_prop_name)
        if default_train_name is not None:
            self.set_default_train(default_train_name)

    def __call__(self, data, **kwargs):
        return self.default_prop(data)

    def train(self, data, label, epochs):
        return self.default_train(data, label, epochs)

    def set_default_prop(self, name):
        self.default_prop = self.rules[name]

    def set_default_train(self, name):
        self.default_train = self.rules[name]

    def __getattr__(self, key):
        return self.rules[key]