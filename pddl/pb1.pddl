(define (problem pb1)
  (:domain kitchen)
  (:init
    (at_agent_spawnpoint)
    (gripper_empty)
    (sugar_on_stovetop)
    (spam_on_countertop)
    (drawer_closed)
  )
  (:goal (and
    (spam_in_drawer)
    (sugar_on_countertop)
    (drawer_closed)
    (gripper_empty)
  ))
)