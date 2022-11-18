# padm_project_2022F
Section 1:
I have implemented the domain and problem files for this section. I have yet to implement the actual planner, though I checked the default 
planner and found that my domain and problem give the expected results.

In constructing the domain, I assumed that the arm would only ever interact with the spam, sugar and drawer. I also assumed that the drawer
would be empty, as well as the countertop where the sugar is to be placed.

I kept the grammar relatively simple and didn't do any parameter based predicates or actions. Originally I didn't want to assume that 
the sugar had to be placed on the countertop next to the stovetop, wanting to use the condition (or (at_spawning_stovetop) (at_spawning_countertop))
to allow the arm to place it at either. However, disjunctive preconditions are not supported under this version of pddl.

Section 2:
I haven't made that much progress because I spent a lot of time trying to get the software working on my computer. However, I'm beginning to build a
toolset in pybullet/lib.py to sample free confs, find the closest vertex, and then project the position forward. At the moment I'm thinking I'll sample for the arm and base as distinct tasks, and have some of the actions in my grammar be 'arm actions', 'base actions', and 'gripper actions'.