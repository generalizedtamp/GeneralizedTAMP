from GeneralizedTAMP.pddlstream.pddlstream.algorithms.common import evaluations_from_init, \
    add_certified, add_facts, UNKNOWN_EVALUATION

def process_instance(instantiator, evaluations, instance, verbose=False): #, **complexity_args):
    if instance.enumerated:
        return []
    new_results, new_facts = instance.next_results(verbose=verbose)
    #remove_blocked(evaluations, instance, new_results)
    for result in new_results:
        complexity = result.compute_complexity(evaluations)
        #complexity = instantiator.compute_complexity(instance)
        for evaluation in add_certified(evaluations, result):
            instantiator.add_atom(evaluation, complexity)
    fact_complexity = 0 # TODO: record the instance or treat as initial?
    for evaluation in add_facts(evaluations, new_facts, result=UNKNOWN_EVALUATION, complexity=fact_complexity):
        instantiator.add_atom(evaluation, fact_complexity)
    if not instance.enumerated:
        instantiator.push_instance(instance)
    return new_results


def process_stream_queue(instantiator, evaluations, complexity_limit=0, verbose=False):
    instances = []
    results = []
    num_successes = 0
    while instantiator and (instantiator.min_complexity() <= complexity_limit):
        instance = instantiator.pop_stream()
        if instance.enumerated:
            continue
        instances.append(instance)
        new_results = process_instance(instantiator, evaluations, instance, verbose=verbose)
        results.extend(new_results)
        num_successes += bool(new_results) # TODO: max_results?
    if verbose:
        print('Calls: {} | Successes: {} | Results: {} | Counts: {}'.format(
            len(instances), num_successes, len(results), Counter(instance.external.name for instance in instances)))
    return len(instances)