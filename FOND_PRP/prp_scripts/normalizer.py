
from GeneralizedTAMP.FOND_PRP.prp_scripts.fondparser.action import Action
from GeneralizedTAMP.FOND_PRP.prp_scripts.fondparser.formula import *

from itertools import product, chain

def normalize(op):
    effs = flatten(op)
    if len(effs) > 1:
        for i in range(len(effs)):
            if not isinstance(effs[i], And):
                effs[i] = And([effs[i]])
        op.effect = Oneof(effs)

def flatten(op):
    return _flatten(op.effect)

def combine(eff_lists):
    return [And(list(filter(lambda x: x != And([]), list(choice)))) for choice in product(*eff_lists)]

def _flatten(eff):

    if isinstance(eff, And):
        if 0 == len(eff.args):
            return [eff]
        else:
            return combine(list(map(_flatten, eff.args)))

    elif isinstance(eff, Oneof):
        return list(chain(*(map(_flatten, eff.args))))

    elif isinstance(eff, When):
        return [When(eff.condition, res) for res in _flatten(eff.result)]

    else:
        return [eff]
