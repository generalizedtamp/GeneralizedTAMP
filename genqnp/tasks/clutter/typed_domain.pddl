(define (domain pr2-tamp)
  (:requirements :strips :equality :typing)
  (:predicates
    (stackable ?o - block ?r - region)
    (grasp ?o - block ?g - grasp)
    (kin ?o - block ?p - pose ?g - grasp ?t - traj)
    (trajto ?o - block ?p - pose ?t - traj)
    (supported ?o - block ?p - pose ?r - region)
    (target ?o - block)
    (nontarget ?o - block)

    (atpose ?o - block ?p - pose)
    (atgrasp ?o - block ?g - grasp)
    (handempty )
    (safe_traj ?t - traj)
    (traj_cfree ?t - traj ?o - block ?p - pose)
    (holding ?o - block)
    (attrajto ?t - traj)  
    (blockingtraj ?ob - block ?t - traj)
  )

  (:action pick
    :parameters (?o - block ?p - pose ?g - grasp ?t - traj)
    :precondition (and (kin ?o ?p ?g ?t)
                       (atpose ?o ?p) 
                       (handempty)
                       (safe_traj ?t) ) 
    :effect (and (atgrasp ?o ?g) 
                 (not (atpose ?o ?p))
                 (not (handempty))
             )
  )

  (:action place
    :parameters (?o - block ?p - pose ?g - grasp ?t - traj)
    :precondition (and (kin ?o ?p ?g ?t)
                       (atgrasp ?o ?g)
                       (safe_traj ?t))
    :effect (and (atpose ?o ?p) 
                 (handempty )
                 (not (atgrasp ?o ?g)))
  )
  
  (:derived (holding ?o - block)
    (exists (?g - grasp) (and (grasp ?o ?g)
                         (atgrasp ?o ?g)))
  )

  (:derived (safe_traj ?t - traj)
    (forall (?o - block ?p - pose) (or (not (atpose ?o ?p)) 
                                       (and (traj_cfree ?t ?o ?p))))
  )

  (:derived (attrajto ?t - traj)
    (exists (?p - pose ?o - block) (and (atpose ?o ?p)
                                        (trajto ?o ?p ?t)
                                        (target ?o))
    )
  )

  (:derived (blockingtraj ?ob - block ?t - traj)
      (forall (?pb - pose) 
           (and (attrajto ?t) (not (holding ?ob)) (or
             (not (atpose ?ob ?pb))
             (not (traj_cfree ?t ?ob ?pb))))
      )
  )





)