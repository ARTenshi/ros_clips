
;************************************************
;*                                              *
;*      ros_clips.clp                           *
;*                                              *
;*      Jesus Savage                            *
;*                                              *
;*              Bio-Robotics Laboratory         *
;*              UNAM, 2020                      *
;*                                              *
;*                                              *
;************************************************


(defrule clips-alive
        ?f <- (alive clips)
        =>
        (retract ?f)
        (printout t "ROS clips alive ROS")
)

;(defrule send-ros
	;(declare (salience 100))
	;;?f <-  (step ?n)
	;?f1 <- (send-ROS ?system ?action ?command ?t ?num)
	;=>
        ;;(retract ?f ?f1)
        ;(retract ?f1)
        ;(printout t "ROS " ?system " " ?action " " ?command " " ?t " " ?num " ROS")
;)

(defrule send-ros-num_plans
        (declare (salience 100))
        ?f <-  (step ?n)
        ?f1 <- (send-ROS ?system num_plans ?npl ?t ?num)
        =>
        (retract ?f ?f1)
        ;(retract ?f1)
        (printout t "ROS " ?system " num_plans " ?npl " " ?t " " ?num " ROS")
)


;(defrule step-ros
	;?f <- (step ?num)
	;=>
	;(retract ?f)
	;(printout t "ROS step " ?num " ROS")
;)




(defrule send-plan-two-arguments
	;(declare (salience 100))
        ?f <-  (step ?nm)
        ?f1 <- (send-ROS ?system ?nm goto ?arg1 ?arg2)
        =>
        (retract ?f ?f1)
        (printout t "ROS " ?system " plan " ?nm " goto "  ?arg1 " " ?arg2 " ROS")
)



(defrule send-plan-one-argument
        ;(declare (salience 100))
        ?f <-  (step ?nm)
        ?f1 <- (send-ROS ?system ?nm ?action ?argument)
        =>
        (retract ?f ?f1)
        (printout t "ROS " ?system " plan " ?nm " " ?action " "  ?argument " ROS")
)




