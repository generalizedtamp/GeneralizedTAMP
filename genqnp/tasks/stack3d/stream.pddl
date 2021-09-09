(define (stream pr2-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (stackable ?o ?r)
    :outputs (?p)
    :certified (and (pose ?o ?p) (supported ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (graspable ?o)
    :outputs (?g)
    :certified (grasp ?o ?g)
  )
  (:stream inverse-kinematics
    :inputs (?o ?p ?g ?q)
    :domain (and (pose ?o ?p) (grasp ?o ?g) (atbconf ?q))
    :outputs (?t)
    :certified (and (atraj ?t) (kin ?o ?p ?g ?q ?t))
  )
  ;(:stream test-cfree-traj-pose
  ;  :inputs (?t ?o ?p)
  ;  :domain (and (atraj ?t) (pose ?o ?p))
  ;  :certified (traj_cfree ?t ?o ?p)
  ;)

)