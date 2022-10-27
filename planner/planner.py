from pddl_parser import PDDL, action
from pprint import pprint
# from queue import PriorityQueue as priority_queue

DOMAIN_FILE = '/home/roomba1/16.413/padm_project_2022F/pddl/kitchen.pddl'
PROBLEM_FILE = '/home/roomba1/16.413/padm_project_2022F/pddl/pb1.pddl'

class Planner:
    def __init__(self, domain_file, problem_file):
        self.parser = PDDL.PDDL_Parser()
        self.parser.parse_domain(domain_file)
        self.parser.parse_problem(problem_file)
        self.goal_state = (self.parser.positive_goals, self.parser.negative_goals)
        self.init_state = self.parser.state

    def plan(self, search='bfs'):
        # Return empty plan when initial state matches goals
        init_state = self.parser.state
        if self.is_goal(init_state):
            return []

        #BFS
        if search == 'bfs':
            explored_states = set([init_state])
            queue = [(init_state, [])]
            return self.bfs(queue, explored_states, self.parser.actions, self.is_goal)

        elif search == 'ff':
            return self.ff(init_state, self.parser.actions)

        elif search == 'enforced_hill_climbing_with_ff':
            return self.enforced_hill_climbing_with_ff(init_state, self.parser.actions)


    def valid(self, state, p_precons, n_precons):
        return p_precons.issubset(state) and n_precons.isdisjoint(state)

    def forward(self, state, p_effects, n_effects):
        return state.difference(n_effects).union(p_effects)
    
    def fast_forward(self, state, p_effects, n_effects):
        return state.union(p_effects)
    
    def is_goal(self, state):
        return self.valid(state, self.goal_state[0], self.goal_state[1])

    def which_goals(self, state):
        return (state & self.goal_state[0]).union(state & self.goal_state[1])

    def goal_diff(self, state):
        return (self.goal_state[0].difference(state)).union(self.goal_state[1].difference(state))

    def action_from_name(self, name):
        for action in self.parser.actions:
            if action.name == name:
                return action

    def creating_action(self, state, prev_action_layer):
        for action in self.parser.actions:
            if action.name in set(prev_action_layer):
                if set((state,)).issubset(action.add_effects):
                    return action

    def backpropagate_preconditions(self, action, goals, level):
        conds = [*action.positive_preconditions] + [*action.negative_preconditions]
        conds = set(conds).difference(self.init_state)
        goals[level - 1] = [*(conds.union(set(goals[level - 1])))]
        return goals

    def backpropagate_add_effects(self, action, goals, level):
        effects = action.add_effects.union(set(goals[level - 1]))
        goals[level - 1] = [*effects]
        return goals

    def bfs(self, queue, explored_states, actions, terminal_funct):
        while queue:
            cur_state, cur_plan = queue.pop(0)
            for action in actions:
                plan = cur_plan.copy()
                p_precons = action.positive_preconditions
                n_precons = action.negative_preconditions
                if self.valid(cur_state, p_precons, n_precons):
                    p_effects = action.add_effects
                    n_effects = action.del_effects
                    plan.append(action.name)
                    next_state = self.forward(cur_state, p_effects, n_effects)
                    if next_state not in explored_states:
                        if terminal_funct(next_state):
                            return (next_state, plan)
                        else:
                            explored_states.add(next_state)
                            queue.insert(-1, (next_state, plan))
        return None, None

    def enforced_hill_climbing_with_ff(self, init_state, actions):
        explored_states = set([init_state])
        queue = [(init_state, [])]
        cur_state = init_state
        plan = []
        relaxed_plan, heuristic = self.ff(init_state, actions)
        while heuristic != 0:
            heur_funct = self.partial_ff_heuristic_comp(heuristic, actions)
            helpful_actions = self.helpful_actions(relaxed_plan, cur_state)
            # Try to use helpful actions as a first resort
            if len(helpful_actions) > 0:
                next_state, bfs_plan = self.bfs(queue.copy(), explored_states.copy(), helpful_actions, heur_funct)
            # All actions if no helpful actions or nothing returned 
            if len(helpful_actions) == 0 or next_state == None:
                next_state, bfs_plan = self.bfs(queue.copy(), explored_states.copy(), actions, heur_funct)
            cur_state = next_state
            queue = [(cur_state, [])]
            explored_states = set([cur_state])
            plan += bfs_plan
            relaxed_plan, heuristic = self.ff(cur_state, actions)
        return plan

    def helpful_actions(self, relaxed_plan, cur_state):
        helpful_actions = []
        for action_n in relaxed_plan[0]:
            action = self.action_from_name(action_n)
            p_precons = action.positive_preconditions
            n_precons = action.negative_preconditions
            if self.valid(cur_state, p_precons, n_precons):
                if len(self.which_goals(action.add_effects)) > 0:
                    helpful_actions.append(action)
        return helpful_actions

    def partial_ff_heuristic_comp(self, heuristic, actions):
        def better_ff(state):
            _, new_heuristic = self.ff(state, actions)
            if new_heuristic < heuristic:
                return True
            else:
                return False
        return better_ff

    def ff(self, init_state, actions):
        graphplan = self.ff_graph(init_state, actions)
        plan, heuristic = self.ff_heuristic(graphplan)
        return plan, heuristic

    def ff_graph(self, init_state, actions):
        graph = []
        cur_state = init_state
        while True:
            next_actions = []
            next_states = []
            for action in actions:
                p_precons = action.positive_preconditions
                n_precons = action.negative_preconditions
                if self.valid(cur_state, p_precons, n_precons):
                    p_effects = action.add_effects
                    n_effects = action.del_effects
                    next_actions.append(action.name)
                    next_state = self.fast_forward(cur_state, p_effects, n_effects)
                    next_states += [*next_state]
            if self.is_goal(set(next_states)):
                graph.append(([*cur_state], [*set(next_actions)]))
                graph.append(([*set(next_states)], None))
                return graph
            if len(graph) > 0:
                last_state_layer, _ = graph[-1]
                if set(next_states) == set(last_state_layer):
                    return graph
            graph.append(([*cur_state], [*set(next_actions)]))
            cur_state = set(next_states)

    def ff_heuristic(self, graphplan):
        goal_firsts = [[]] * len(graphplan)
        action_firsts = [[]] * len(graphplan)

        final_state_layer, _ = graphplan[-1]

        if not self.is_goal(set(final_state_layer)):
            return (None, float('inf'))

        # Find first levels at which goals and actions appear
        for i in range(0, len(graphplan)):
            state_layer, action_layer = graphplan[i]
            old_goals = [goal for level in goal_firsts for goal in level]
            are_goals = self.which_goals(set(state_layer))
            new_goals = are_goals.difference(set(old_goals))
            goal_firsts[i] = [*new_goals]
            if action_layer:
                old_actions = [action for level in action_firsts for action in level]
                new_actions = set(action_layer).difference(set(old_actions))
                action_firsts[i] = [*new_actions]

        # Backwards step
        goals = goal_firsts
        heuristic = 0
        relaxed_plan = [None] * len(graphplan)
        level = (len(graphplan) - 1)
        for i in range(level, 0, -1):
            first_actions = action_firsts[i - 1]
            actions = []
            for goal in goals[i]:
                action = self.creating_action(goal, first_actions)
                if not action:
                    continue
                if action:
                    heuristic += 1
                goals = self.backpropagate_preconditions(action, goals, i)
                goals = self.backpropagate_add_effects(action, goals, i)
                actions.append(action.name)
            relaxed_plan[i-1] = [*set(actions)]
        return (relaxed_plan, heuristic)

if __name__ == '__main__':
    p = Planner(DOMAIN_FILE, PROBLEM_FILE)
    plan = p.plan(search='enforced_hill_climbing_with_ff')
    pprint(plan)
    # pprint(heuristic)
