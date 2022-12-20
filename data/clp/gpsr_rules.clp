;****************************************
;*                                      *
;*  gpsr_rules.clp                      *
;*                                      *
;*          University of Mexico        *
;*          Jesus Savage-Carmona        *
;*                                      *
;*          20 Dec 2022                 *
;*                                      *
;****************************************

(defglobal ?*plan_number* = 0)

(defrule go-zone-not-same-zone-ROS
    ?ptrans <- (ptrans (actor ?actor)(obj ?actor)(to ?room))
    ( Room (name ?room)(zone ?zone))
    ( item (type Robot) (name ?actor))
    =>
    (retract ?ptrans)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
    (assert (attempt (name ptrans) (id ?*plan_number*)(move robot)(room ?room)(zone ?zone)(on room)(number 1 )))
    (assert (finish-planner ptrans ?*plan_number*))
)

(defrule exec-ptrans-object-ROS
    ?ptrans <- (ptrans (actor ?actor)(obj ?obj)(to ?place))
    (item (type Robot) (name ?actor))
    (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
    ( Room (name ?place)(zone ?zone-place))
    =>
    (retract ?ptrans)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 2 )(actions find-object ?obj)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 3 )(actions grab ?obj )) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 4 )(actions goto ?place ?zone-place)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 5 )(actions find-object freespace)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 6 )(actions go freespace )) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 7 )(actions drop ?obj )) )
    (assert (attempt (name ptrans) (id ?*plan_number* ) (move ?obj)(room ?room)(zone ?zone)(on floor)(number 7 )))
    (assert (finish-planner ptrans ?*plan_number*))
)


(defrule exec-ptrans-human-ROS
    ?ptrans <- (ptrans (actor ?actor)(obj ?actor)(to ?human))
    (item (type Robot) (name ?actor))
    (item (type Human) (name ?human)(room ?room)(zone ?zone))
    ;( Room (name ?place)(zone ?zone-place))
    =>
    (retract ?ptrans)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 2 )(actions find-object ?human)) )
    ;(assert (plan (name ptrans)(id ?*plan_number* ) (number 3 )(actions mv ?human )) )
    (assert (attempt (name ptrans) (id ?*plan_number* ) (move ?actor)(room ?room)(zone ?zone)(on room)(number 2 )))
    (assert (finish-planner ptrans ?*plan_number*))
)


(defrule exec-atrans-object-recepient-ROS
    ?atrans <- (atrans (actor ?actor)(obj ?obj)(to ?human))
    (item (type Robot) (name ?actor))
    (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
    (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
    ( Room (name ?place)(zone ?zone-place))
    =>
    (retract ?atrans)
    (bind ?*plan_number* (+ 1 ?*plan_number*))
    (assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 2 )(actions find-object ?obj)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 3 )(actions grab ?obj )) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 4 )(actions goto ?room-human ?zone-human)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 5 )(actions find-object ?human)) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 6 )(actions go ?human )) )
    (assert (plan (name ptrans)(id ?*plan_number* ) (number 7 )(actions drop ?obj )) )
    (assert (attempt (name atrans) (id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 7 )))
    (assert (finish-planner ptrans ?*plan_number*))
)

