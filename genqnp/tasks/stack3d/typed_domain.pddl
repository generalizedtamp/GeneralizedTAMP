(define (domain pr2-tamp)
  (:requirements :strips :equality :typing)
  (:predicates
    (controllable )
    (stackable ?o - block_type ?r - block_type)
    (table ?r - block_type)
    (grasp ?o - block_type ?g - grasp_type)
    (kin ?o - block_type ?p - pose_type ?g - grasp_type ?q - conf_type ?t - traj_type)
    (basemotion ?q1 - conf_type ?t - traj_type ?q2 - conf_type)
    (armmotion ?q1 - conf_type ?t - traj_type ?q2 - conf_type)
    (supported ?o - block_type ?p - pose_type ?r - block_type)

    (atpose ?o - block_type ?p - pose_type)
    (atgrasp ?o - block_type ?g - grasp_type)
    (handempty )
    (atbconf ?q - conf_type)
    (ataconf ?q - conf_type)

    (traj_cfree ?t - traj_type ?o - block_type ?p - pose_type)
    (safe_traj ?t - traj_type)

    (on ?o - block_type ?r - block_type)
    (holding ?o - block_type)
  )

  (:action pick
    :parameters (?o - block_type ?p - pose_type ?g - grasp_type ?q - conf_type ?t - traj_type)
    :precondition (and (kin ?o ?p ?g ?q ?t)
                       (atpose ?o ?p)
                       ;(safe_traj ?t)
                       (handempty ))
    :effect (and (atgrasp ?o ?g) 
                 (not (atpose ?o ?p))
                 (not (handempty ))
                 )
  )
  (:action place
    :parameters (?o - block_type ?p - pose_type ?g - grasp_type ?q - conf_type ?t - traj_type)
    :precondition (and (kin ?o ?p ?g ?q ?t)
                       (atgrasp ?o ?g)
                       ;(safe_traj ?t)
                       (not (handempty )))
    :effect (and (atpose ?o ?p)
                 (not (atgrasp ?o ?g))
                 (handempty ))
  )

  (:derived (ontable ?o - block_type)
    (exists (?p - pose_type) (and (supported ?o ?p ?r)
                      (atpose ?o ?p) (table ?r)))
  )
  
  (:derived (on ?o - block_type ?r - block_type)
    (exists (?p - pose_type) (and (supported ?o ?p ?r)
                      (atpose ?o ?p)))
  )

  ;(:derived (safe_traj ?t - traj_type)
  ;  (forall (?o - block_type ?p - pose_type) (or (not (atpose ?o ?p)) 
  ;                                     (and (traj_cfree ?t ?o ?p))))
  ;)
  
  (:derived (holding ?o - block_type)
    (exists (?g - grasp_type) (and (grasp ?o ?g)
                      (atgrasp ?o ?g)))
  )

  
)