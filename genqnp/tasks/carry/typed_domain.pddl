(define (domain pr2-tamp)

  (:requirements :strips :equality)
  (:predicates
    (stackable ?o - block_type ?r - block_type)
    (sink ?r  - block_type)
    (stove ?r - block_type)
    (tray ?r - block_type)
    (tray_relpose ?o - block_type ?rp - relative_pose_type)
    (table ?r - block_type)

    (grasp ?o - block_type ?g - grasp_type)
    (posekin ?o - block_type ?rp - world_pose_type ?g - grasp_type ?q - conf_type ?t - traj_type)
    (computedposekin ?o1 - block_type ?wp1 - world_pose_type ?rp - relative_pose_type ?o2 - block_type ?wp2 - world_pose_type)
    (worldposebasestove ?wp - world_pose_type)
    (worldposebasesink ?wp - world_pose_type)

    (atworldpose ?o - block_type ?wp - world_pose_type)
    (worldpose ?o - block_type ?wp - world_pose_type)
    (relpose ?o - block_type ?rp - relative_pose_type ?r - block_type)
    (atrelpose ?o - block_type ?rp - relative_pose_type)

    (atstove ?q - conf_type)
    (atsink ?q - conf_type)

    (atgrasp ?o - block_type ?g - grasp_type)
    (handempty )
    (holding_tray )
    (canmove )
    (atbconf ?q - conf_type)

    (ataconf ?q - conf_type)
    (cleaned ?o - block_type)
    (cooked ?o - block_type)

    (canfittray ?o - block_type)

    (unsafeworldpose ?o1 - block_type ?p1 - world_pose_type)

    ;(confcontains ?q - conf_type)
    ;(confcancontain ?q - conf_type ?o - block_type ?wp - world_pose_type)

    (cfree ?o1 - block_type ?p1 - relative_pose_type ?o2 - block_type ?p2 - relative_pose_type)
    (on ?o - block_type ?r - block_type)
    (holding ?o - block_type)
  )  

  (:action move_to_sink
    :parameters (?q1 - conf_type ?q2 - conf_type)
    :precondition (and (atbconf ?q1) (atstove ?q1) (atsink ?q2) (holding_tray ) (canmove ))
    :effect (and (atbconf ?q2)
                 (not (atbconf ?q1))
                 (not (canmove )))
  )

  (:action move_to_stove
    :parameters (?q1 - conf_type ?q2 - conf_type)
    :precondition (and (atbconf ?q1) (atsink ?q1) (atstove ?q2) (holding_tray ) (canmove ))
    :effect (and (atbconf ?q2)
                 (not (atbconf ?q1))
                 (not (canmove )))
  )

  (:action pick_at_sink
    :parameters (?o - block_type ?wp - world_pose_type ?rp - relative_pose_type ?r - block_type ?rwp - world_pose_type ?g - grasp_pose_type ?q - conf_type ?t - traj_type ?q2 - conf_type)
    :precondition (and (posekin ?o ?wp ?g ?q2 ?t)
                       (atsink ?q) 
                       (atsink ?q2)
                       (atworldpose ?o ?wp) 
                       (handempty )
                       (atbconf ?q) 
                       (relpose ?o ?rp ?r)
                       (atrelpose ?o ?rp)
                       (computedposekin ?o ?wp ?rp ?r ?rwp) )
    :effect (and (atgrasp ?o ?g)
                 (not (atworldpose ?o ?wp))
                 (not (atrelpose ?o ?rp))
                 (atbconf ?q2)
                 (not (atbconf ?q))
                 (not (handempty ))
                 (canmove )
                 (forall (?o2 - block_type ?wp2 - world_pose_type ?rp2 - relative_pose_type) (when (and (computedposekin ?o2 ?wp2 ?rp2 ?o ?wp) (atrelpose ?o2 ?rp2) (atworldpose ?o2 ?wp2) )
                                              (not (atworldpose ?o2 ?wp2))))
                 )
  )

  (:action place_at_sink
    :parameters (?o - block_type ?wp - world_pose_type ?rp - relative_pose_type ?r - block_type ?rwp - world_pose_type ?g - grasp_pose_type ?q - conf_type ?t - traj_type ?q2 - conf_type)
    :precondition (and (posekin ?o ?wp ?g ?q2 ?t)
                       (atsink ?q) 
                       (atsink ?q2)
                       (atgrasp ?o ?g) (atbconf ?q) (worldpose ?o ?wp) (stackable ?o ?r) (atworldpose ?r ?rwp) 
                       (relpose ?o ?rp ?r) (computedposekin ?o ?wp ?rp ?r ?rwp)
                       )
    :effect (and (atworldpose ?o ?wp) (handempty ) 
                 (not (atgrasp ?o ?g)) 
                 (atrelpose ?o ?rp)
                 (atbconf ?q2)
                 (not (atbconf ?q))
                 (canmove )
                 (forall (?o2 - block_type ?wp2 - world_pose_type ?rp2 - relative_pose_type) (when (and (computedposekin ?o2 ?wp2 ?rp2 ?o ?wp) (atrelpose ?o2 ?rp2) (worldpose ?o2 ?wp2))
                                               (atworldpose ?o2 ?wp2)))
                 )
  )

    (:action pick_at_stove
    :parameters (?o - block_type ?wp - world_pose_type ?rp - relative_pose_type ?r - block_type ?rwp - world_pose_type ?g - grasp_pose_type ?q - conf_type ?t - traj_type ?q2 - conf_type)
    :precondition (and (posekin ?o ?wp ?g ?q2 ?t)
                       (atstove ?q) 
                       (atstove ?q2)
                       (atworldpose ?o ?wp) (handempty ) (atbconf ?q) 
                       (relpose ?o ?rp ?r) 
                       (atrelpose ?o ?rp) 
                       (computedposekin ?o ?wp ?rp ?r ?rwp) )
    :effect (and (atgrasp ?o ?g) 
                 (not (atworldpose ?o ?wp))
                 (not (atrelpose ?o ?rp))
                 (atbconf ?q2)
                 (not (atbconf ?q))
                 (not (handempty ))
                 (canmove )
                 (forall (?o2 - block_type ?wp2 - world_pose_type ?rp2 - relative_pose_type) (when (and (computedposekin ?o2 ?wp2 ?rp2 ?o ?wp) (atrelpose ?o2 ?rp2) (atworldpose ?o2 ?wp2))
                                              (not (atworldpose ?o2 ?wp2))))
                 )
  )

  (:action place_at_stove
    :parameters (?o - block_type ?wp - world_pose_type ?rp - relative_pose_type ?r - block_type ?rwp - world_pose_type ?g - grasp_pose_type ?q - conf_type ?t - traj_type ?q2 - conf_type)
    :precondition (and (posekin ?o ?wp ?g ?q2 ?t)
                       (atstove ?q) 
                       (atstove ?q2)
                       (atgrasp ?o ?g) (atbconf ?q) (worldpose ?o ?wp) (stackable ?o ?r) (atworldpose ?r ?rwp) 
                       (relpose ?o ?rp ?r) (computedposekin ?o ?wp ?rp ?r ?rwp)
                       )
    :effect (and (atworldpose ?o ?wp) (handempty )
                 (not (atgrasp ?o ?g)) (atrelpose ?o ?rp)
                 (atbconf ?q2)
                 (not (atbconf ?q))
                 (canmove )
                 (forall (?o2 - block_type ?wp2 - world_pose_type ?rp2 - relative_pose_type) (when (and (computedposekin ?o2 ?wp2 ?rp2 ?o ?wp) (atrelpose ?o2 ?rp2) (worldpose ?o2 ?wp2))
                                               (atworldpose ?o2 ?wp2)))
                 )
  )

  (:derived (on ?o - block_type ?r - block_type)
    (exists (?rp - relative_pose_type) (and
                      (atrelpose ?o ?rp)
                      (relpose ?o ?rp ?r)
                   ))
  )


  (:derived (canfittray ?o - block_type)
    (exists (?rp - relative_pose_type) (forall (?o2 - block_type ?rp2 - relative_pose_type) (and (tray_relpose ?o ?rp) (or (cfree ?o ?rp ?o2 ?rp2) (not (atrelpose ?o2 ?rp2)) (not (tray_relpose ?o2 ?rp2))))))
  )


  (:derived (holding ?o - block_type)
    (exists (?g - grasp_type) (and (grasp ?o ?g)
                      (atgrasp ?o ?g)))
  )
  (:derived (holding_tray )
    (exists (?o - block_type) (and (holding ?o)
                      (tray ?o)))
  )



)