(define (stream pr2-tamp)

  ;(:stream sample-pose
  ;  :inputs (?o ?r)
  ;  :domain (and (stackable ?o ?r) (table ?r))
  ;  :outputs (?rp)
  ;  :certified (and (relpose ?o ?rp ?r))
  ;)

  (:stream compute-kin-sink
    :inputs (?o1 ?rp ?o2 ?wp2)
    :domain (and (relpose ?o1 ?rp ?o2) (worldpose ?o2 ?wp2) (worldposebasesink ?wp2) )
    :outputs (?wp1)
    :certified (and (worldpose ?o1 ?wp1) (computedposekin ?o1 ?wp1 ?rp ?o2 ?wp2) (worldposebasesink ?wp1))
  )

  (:stream compute-kin-stove
    :inputs (?o1 ?rp ?o2 ?wp2)
    :domain (and (relpose ?o1 ?rp ?o2) (worldpose ?o2 ?wp2) (worldposebasestove ?wp2) )
    :outputs (?wp1)
    :certified (and (worldpose ?o1 ?wp1) (computedposekin ?o1 ?wp1 ?rp ?o2 ?wp2) (worldposebasestove ?wp1))
  )

  (:stream sample-grasp
    :inputs (?o)
    :domain (graspable ?o)
    :outputs (?g)
    :certified (grasp ?o ?g)
  )

  (:stream inverse-kinematics-stove
    :inputs (?o ?wp ?g)
    :domain (and (worldpose ?o ?wp) (grasp ?o ?g) (worldposebasestove ?wp))
    :outputs (?q ?t)
    :certified (and (bconf ?q) (atraj ?t) (posekin ?o ?wp ?g ?q ?t) (atstove ?q))
  )

  (:stream inverse-kinematics-sink
    :inputs (?o ?wp ?g)
    :domain (and (worldpose ?o ?wp) (grasp ?o ?g) (worldposebasesink ?wp))
    :outputs (?q ?t)
    :certified (and (bconf ?q) (atraj ?t) (posekin ?o ?wp ?g ?q ?t) (atsink ?q))
  )

  (:stream test-cfree
    :inputs (?o1 ?p1 ?r ?o2 ?p2)
    :domain (and (relpose ?o1 ?p1 ?r) (relpose ?o2 ?p2 ?r) (tray ?r))
    :certified (cfree ?o1 ?p1 ?o2 ?p2)
  )


)