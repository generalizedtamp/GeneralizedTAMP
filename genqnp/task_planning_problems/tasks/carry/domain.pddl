(define (domain pr2-tamp)

  (:requirements :strips :equality)
  (:predicates
    (Arm ?a)
    (Stackable ?o ?r)
    (Sink ?r)
    (Stove ?r)
    (At ?q ?o)

    (Grasp ?o ?g)
    (PoseKin ?a ?o ?rp ?g ?q ?t)
    (ComputedPoseKin ?o1 ?wp1 ?rp ?o2 ?wp2)

    (BaseMotion ?q1 ?t ?q2)
    (ArmMotion ?a ?q1 ?t ?q2)

    (WorldPoseBaseStove ?wp)
    (WorldPoseBaseSink ?wp)


    (AtWorldPose ?o ?wp)
    (WorldPose ?o ?wp)
    (RelPose ?o ?rp ?r)
    (AtRelPose ?o ?rp ?r)

    (AtStove ?q)
    (AtSink ?q)

    (AtGrasp ?a ?o ?g)
    (HandEmpty ?a)
    (AtBConf ?q)

    (AtAConf ?a ?q)
    (CanMove)
    (Cleaned ?o)
    (Cooked ?o)
    (Can ?o)
    (UnsafeWorldPose ?o1 ?p1)

    (CFree ?o1 ?p1 ?o2 ?p2)
    (On ?o ?r)
    (Holding ?a ?o)
  )


  (:functions (total-cost))


  ; Todo: make cost continuous with distance

  (:action move_at_stove
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1) (CanMove) (AtStove ?q1) (AtStove ?q2) )
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove)) (increase (total-cost) 1) )

  )

  (:action move_at_sink
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1) (CanMove) (AtSink ?q1) (AtSink ?q2) )
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove)) (increase (total-cost) 1))
  )

  (:action move_to_sink
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1) (CanMove) (AtStove ?q1) (AtSink ?q2) )
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove)) (increase (total-cost) 100))
  )

  (:action move_to_stove
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1) (CanMove) (AtSink ?q1) (AtStove ?q2) )
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)) (not (CanMove)) (increase (total-cost) 100))
  )


  (:action pick
    :parameters (?a ?o ?wp ?rp ?r ?rwp ?g ?q ?t)
    :precondition (and (PoseKin ?a ?o ?wp ?g ?q ?t)
                       (AtWorldPose ?o ?wp) (HandEmpty ?a) (AtBConf ?q) (AtRelPose ?o ?rp ?r) (ComputedPoseKin ?o ?wp ?rp ?r ?rwp) )
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtWorldPose ?o ?wp))
                 (not (AtRelPose ?o ?rp ?r))
                 (not (HandEmpty ?a)) (increase (total-cost) 1)
                 (forall (?o2 ?wp2 ?rp2) (when (and (ComputedPoseKin ?o2 ?wp2 ?rp2 ?o ?wp) (AtRelPose ?o2 ?rp2 ?o) (AtWorldPose ?o2 ?wp2))
                                              (not (AtWorldPose ?o2 ?wp2))))
                 )
  )

  (:action place
    :parameters (?a ?o ?wp ?rp ?r ?rwp ?g ?q ?t)
    :precondition (and (PoseKin ?a ?o ?wp ?g ?q ?t)
                       ;(AtGrasp ?a ?o ?g) (AtBConf ?q) (WorldPose ?o ?wp) (Stackable ?o ?r) (AtWorldPose ?r ?rwp) (RelPose ?o ?rp ?r) (ComputedPoseKin ?o ?wp ?rp ?r ?rwp) (not (UnsafeWorldPose ?o ?wp))
                       (AtGrasp ?a ?o ?g) (AtBConf ?q) (WorldPose ?o ?wp) (Stackable ?o ?r) (AtWorldPose ?r ?rwp) (RelPose ?o ?rp ?r) (ComputedPoseKin ?o ?wp ?rp ?r ?rwp)
                       )
    :effect (and (AtWorldPose ?o ?wp) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g)) (increase (total-cost) 1) (AtRelPose ?o ?rp ?r)
                 (forall (?o2 ?wp2 ?rp2) (when (and (ComputedPoseKin ?o2 ?wp2 ?rp2 ?o ?wp) (AtRelPose ?o2 ?rp2 ?o) (WorldPose ?o2 ?wp2))
                                               (AtWorldPose ?o2 ?wp2)))
                 )
  )

  (:derived (On ?o ?r)
    (exists (?rp) (and
                      (AtRelPose ?o ?rp ?r)
                   ))
  )

  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
  )

  (:derived (UnsafeWorldPose ?o1 ?p1)
    (exists (?o2 ?p2) (and (WorldPose ?o1 ?p1) (AtWorldPose ?o2 ?p2)
                           (not (CFree ?o1 ?p1 ?o2 ?p2))))
  )
)