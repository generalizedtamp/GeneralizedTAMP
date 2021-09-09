(define (problem FR_2_5)
 (:domain first-response)
 (:objects  l1 l2  - location
	    f1 f2 - fire_unit
	    v1 v2 v3 v4 v5 - victim
	    m1 m2 - medical_unit
)
 (:init 
	;;strategic locations
     (hospital l2)
     (hospital l1)
     (water-at l2)
     (water-at l1)
	;;disaster info
     (fire l1)
     (victim-at v1 l2)
     (victim-status v1 hurt)
     (fire l1)
     (victim-at v2 l1)
     (victim-status v2 hurt)
     (fire l2)
     (victim-at v3 l1)
     (victim-status v3 hurt)
     (fire l2)
     (victim-at v4 l2)
     (victim-status v4 dying)
     (fire l1)
     (victim-at v5 l2)
     (victim-status v5 dying)
	;;map info
	(adjacent l1 l1)
	(adjacent l2 l2)
	(fire-unit-at f1 l2)
	(fire-unit-at f2 l2)
	(medical-unit-at m1 l1)
	(medical-unit-at m2 l1)
	)
 (:goal (and  (nfire l1) (nfire l1) (nfire l2) (nfire l2) (nfire l1)  (victim-status v1 healthy) (victim-status v2 healthy) (victim-status v3 healthy) (victim-status v4 healthy) (victim-status v5 healthy)))
 )
