from noh import Circuit
from noh.circuit import PropRule
from noh.components import Random, Const
import numpy as np


class SimpleProp(PropRule):
    component_id_list = range(100)

    def __init__(self, components):
        super(SimpleProp, self).__init__(components)
        self.id = self.component_id_list.pop(0)

    def __call__(self, data):
        return self.components[self.id](data)


class LearnerSet(Circuit):

    def __init__(self, components, RuleClassDict):
        super(LearnerSet, self).__init__(components, RuleClassDict)
        self.n_components = len(components)
        self.f_go = False

    @classmethod
    def create(cls, n_stat, n_act, n_learner):
        component_list = [Random(n_input=n_stat, n_output=n_act)] + \
                         [Const(n_input=n_stat, n_output=n_act, const_output=n) for n in xrange(1, n_learner)]

        PropRulesDict = {"prop"+str(i): SimpleProp for i in xrange(n_learner)}

        return LearnerSet(component_list, PropRulesDict)


class PropLearner(PropRule):

    name_list = []
    def __init__(self, components):
        super(PropLearner, self).__init__(components)

    def __call__(self, data):
        if not self.components["learner_set"].f_go:
            self.components["learner_set"].f_go = True
            self.components["learner_set"].set_default_prop(name=np.random.choice(self.name_list))

        res = self.components["learner_set"](data)

        """ kashikoku shitai here """
        if np.random.rand() < 0.1:
            self.components["learner_set"].f_go = False
        return res


class SuppressionBoosting(Circuit):
    def __init__(self, components, RuleClassDict):
        super(SuppressionBoosting, self).__init__(components, RuleClassDict, default_prop_name="prop_learner")
        self.f_go = False

    @classmethod
    def create(cls, n_stat, n_act, n_learner):
        components = {"learner_set": LearnerSet.create(n_stat, n_act, n_learner),
                      "suppressor": None}
        PropLearner.name_list = components["learner_set"].rules.keys()
        return SuppressionBoosting(components, {"prop_learner": PropLearner})

    def stop(self):
        self.f_go = False