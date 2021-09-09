from utils import print_concept, QualitativeAction
from pddl import Conjunction

def save_domain(domain, fn):
	domain_str = ""
	domain_str+=("(define (domain {})\n".format(domain.name))
	reqs = [req for req in domain.requirements.requirements if req != ":typing"]

	domain_str+=("\t(:requirements {})\n".format(" ".join(reqs)))
	domain_str+=("\t(:predicates\n")
	for predicate in domain.predicates:
		domain_str+=("\t\t({} {})\n".format(predicate.name, " ".join([a.name for a in predicate.arguments] )))
	domain_str+=("\t)\n")
	domain_str+=("\t(:functions\n")
	for function in domain.functions:
		domain_str+=("\t\t({} {})\n".format(function.name, " ".join([a.name for a in function.arguments] )))
	domain_str+=("\t)\n")

	
	for action in domain.actions:
		domain_str+=("\t(:action {}\n".format(action.name))
		domain_str+=("\t\t:parameters ({})\n".format(" ".join([p.name for p in action.parameters])))
		domain_str+=("\t\t:precondition {}\n".format(print_concept(action.precondition)))
		domain_str+=("\t\t:effect (and {})\n".format(" ".join([print_concept(e) for e in action.effects])))
		domain_str+=("\t)\n")

	for axiom in domain.axioms:
		domain_str+=("\t(:derived ({} {})\n".format(axiom.name, " ".join([p.name for p in axiom.parameters ])))
		domain_str+=("\t\t{}\n".format(print_concept(axiom.condition)))
		domain_str+=("\t)\n")

	domain_str+=(")\n")

	text_file = open(fn, "w")
	n = text_file.write(domain_str)
	text_file.close()

def save_typed_domain(domain, fn):
	domain_str = ""
	domain_str+=("(define (domain {})\n".format(domain.name))	

	reqs = domain.requirements.requirements

	domain_str+=("\t(:requirements {})\n".format(" ".join(reqs)))
	domain_str+=("\t(:predicates\n")
	for predicate in domain.predicates:
		domain_str+=("\t\t({} {})\n".format(predicate.name, " ".join(["{} - {}".format(a.name, a.type_name) for a in predicate.arguments] )))
	domain_str+=("\t)\n")
	domain_str+=("\t(:functions\n")
	for function in domain.functions:
		domain_str+=("\t\t({} {})\n".format(function.name, " ".join([a.name for a in function.arguments] )))
	domain_str+=("\t)\n")

	
	for action in domain.actions:
		domain_str+=("\t(:action {}\n".format(action.name))
		domain_str+=("\t\t:parameters ({})\n".format(" ".join(["{} - {}".format(p.name, p.type_name) for p in action.parameters])))
		domain_str+=("\t\t:precondition {}\n".format(print_concept(action.precondition, typed=True)))
		domain_str+=("\t\t:effect (and {})\n".format(" ".join([print_concept(e) for e in action.effects])))
		if(isinstance(action, QualitativeAction)):
			if(len(action.incs)>0):
				domain_str+=("\t\t:inc (and {})\n".format(" ".join([print_concept(e) for e in action.incs])))
			if(len(action.decs)>0):
				domain_str+=("\t\t:dec (and {})\n".format(" ".join([print_concept(e) for e in action.decs])))
			if(len(action.order)>0):
				domain_str+=("\t\t:order (and {})\n".format(" ".join([print_concept(e) for e in action.order])))

		domain_str+=("\t)\n")

	
	for axiom in domain.axioms:
		domain_str+=("\t(:derived ({} {})\n".format(axiom.name, " ".join(["{} - {}".format(p.name, p.type_name) for p in axiom.parameters ])))
		domain_str+=("\t\t{}\n".format(print_concept(axiom.condition, typed=True)))
		domain_str+=("\t)\n")
	domain_str+=(")\n")

	text_file = open(fn, "w")
	n = text_file.write(domain_str)
	text_file.close()
