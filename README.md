# padm_project_2022F
Section 1:
I have implemented the domain and problem files for this section. I have yet to implement the actual planner, though I checked the default 
planner and found that my domain and problem give the expected results.

In constructing the domain, I assumed that the arm would only ever interact with the spam, sugar and drawer. I also assumed that the drawer
would be empty, as well as the countertop where the sugar is to be placed.

I kept the grammar relatively simple and didn't do any parameter based predicates or actions. Originally I didn't want to assume that 
the sugar had to be placed on the countertop next to the stovetop, wanting to use the condition (or (at_spawning_stovetop) (at_spawning_countertop))
to allow the arm to place it at either. However, disjunctive preconditions are not supported under this version of pddl.