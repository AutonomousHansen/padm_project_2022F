; PDDL Domain for 16.410 Project
(define (domain kitchen)
  (:requirements :strips)
  (:predicates
    (at_agent_spawnpoint)
    (at_spawning_stovetop)
    (at_spawning_countertop)
    (gripper_empty)
    (gripping_spam)
    (gripping_sugar)
    (sugar_on_stovetop)
    (sugar_on_countertop)
    (spam_on_stovetop)
    (spam_on_countertop)
    (spam_in_drawer)
    (drawer_open)
    (drawer_closed)
  )
  (:action move_to_spawning_stovetop
    :precondition ()
    :effect (and
            (at_spawning_stovetop)
            (not (at_spawning_countertop))
            (not (at_agent_spawnpoint))
            )
  )
  (:action move_to_spawning_countertop
    :precondition ()
    :effect (and
        (at_spawning_countertop)
        (not (at_spawning_stovetop))
        (not (at_agent_spawnpoint))
        )
  )
  (:action grip_sugar
    :precondition (and
                  (gripper_empty)
                  (sugar_on_stovetop)
                  (at_spawning_stovetop)
                )
    :effect (and
      (not (gripper_empty))
      (gripping_sugar)
    )
  )
  (:action grip_spam
    :precondition (and
                  (gripper_empty)
                  (spam_on_countertop)
                  (at_spawning_countertop)
                )
    :effect (and
      (not (gripper_empty))
      (gripping_spam)
    )
  )
  (:action place_sugar_on_countertop
    :precondition (and
                  (gripping_sugar)
                  (at_spawning_stovetop)
                  )
    :effect (and 
            (not (gripping_sugar))
            (gripper_empty)
            (sugar_on_countertop))
  )
  (:action place_spam_in_drawer
    :precondition (and
                  (gripping_spam)
                  (at_spawning_countertop)
                  (drawer_open))
    :effect (and 
            (not (gripping_spam))
            (gripper_empty)
            (spam_in_drawer))
  )
  (:action open_drawer
    :precondition (and
                  (at_spawning_countertop)
                  (gripper_empty)
                  (drawer_closed)
    )
    :effect (and
            (drawer_open)
            (not (drawer_closed))
    )
  )
  (:action close_drawer
    :precondition (and
                  (at_spawning_countertop)
                  (gripper_empty)
                  (drawer_open)
    )
    :effect (and
            (drawer_closed)
            (not (drawer_open))
    )
  )
  (:action noop
    :precondition ()
    :effect ()
  )
)