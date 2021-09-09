(define (stream pr2-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (and (stackable ?o ?r))
    :outputs (?p)
    :certified (and (pose ?o ?p) (supported ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (and (graspable ?o) (nontarget ?o))
    :outputs (?g)
    :certified (grasp ?o ?g)
  )
  (:stream sample-target-grasp
    :inputs (?o)
    :domain (and (graspable ?o) (target ?o))
    :outputs (?g)
    :certified (grasp ?o ?g)
  )
  (:stream inverse-kinematics
    :inputs (?o ?p ?g)
    :domain (and (pose ?o ?p) (grasp ?o ?g))
    :outputs (?t)
    :certified (and (atraj ?t) (kin ?o ?p ?g ?t) (trajto ?o ?p ?t) )
  )

  (:stream test-cfree-traj-pose
  :inputs (?t ?o ?p)
  :domain (and (atraj ?t) (pose ?o ?p))
  :certified (traj_cfree ?t ?o ?p))

)