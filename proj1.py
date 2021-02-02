from typing import List, Tuple, Set, Dict, Optional, cast
from environments.environment_abstract import Environment, State
from environments.farm_grid_world import FarmState
from heapq import heappush, heappop


class Node:
    def __init__(self, state: State, path_cost: float, parent_action: Optional[int], parent, depth):
        self.state: State = state
        self.parent: Optional[Node] = parent
        self.path_cost: float = path_cost
        self.parent_action: Optional[int] = parent_action
        self.depth: int = depth

    def __hash__(self):
        return self.state.__hash__()

    def __gt__(self, other):
        return self.path_cost < other.path_cost

    def __eq__(self, other):
        return self.state == other.state


def get_next_state_and_transition_cost(env: Environment, state: State, action: int) -> Tuple[State, float]:
    rw, states_a, _ = env.state_action_dynamics(state, action)
    state: State = states_a[0]
    transition_cost: float = -rw

    return state, transition_cost


def expand_node(env: Environment, parent_node: Node) -> List[Node]:
    actions = env.get_actions()
    children = []
    for i in range(len(actions)):
        (state_c,cost_c) = get_next_state_and_transition_cost(env, parent_node.state, actions[i])
        child = Node(state_c, cost_c, actions[i],parent_node,parent_node.depth + 1)
        children.append(child)
    return children


def get_soln(node: Node) -> List[int]:
    path = []
    final = node
    path.append(final.parent_action)
    while node.parent:
        path.insert(0,node.parent_action)
        node = node.parent
       # if node.parent == None:
       #     path.append(node.parent_action)
       #     break

    return path

def is_cycle(node: Node) -> bool:
    origin = node
    while node.parent:
        if origin.state == node.parent.state:
            return True
        else:
            node = node.parent
    return False

def get_heuristic(node: Node) -> float:
    state: FarmState = cast(FarmState, node.state)
    x1, y1 = state.agent_idx
    x2, y2 = state.goal_idx
    xs = abs(x1-x2)
    ys = abs(y1-y2)
    return xs+ys


def get_cost(node: Node, heuristic: float, weight_g: float, weight_h: float) -> float:
    return weight_g*node.path_cost + weight_h*heuristic

class BreadthFirstSearch:

    def __init__(self, state: State, env: Environment):
        self.env: Environment = env

        self.open: Set[Node] = set()
        self.fifo: List[Node] = []
        self.closed_set: Set[State] = set()

        # compute cost
        root_node: Node = Node(state, 0.0, None, None, 0)

        # push to open
        self.fifo.append(root_node)
        self.closed_set.add(root_node.state)

    def step(self):
        #print("closed_set \n", self.closed_set)
        front_node = self.fifo[0]
        self.fifo.remove(self.fifo[0])
        children = expand_node(self.env,front_node)
        for child in children:
            s = child.state
            if self.env.is_terminal(s):
                return child
           # is_reached = False
           # for i in range(len(self.closed_set)):
           #     if s in self.closed_set:
           #         is_reached == True
           # if is_reached == False:
            if not s in self.closed_set:
                self.closed_set.add(s)
                self.fifo.append(child)

class DepthLimitedSearch:

    def __init__(self, state: State, env: Environment, limit: float):
        self.env: Environment = env
        self.limit: float = limit

        self.lifo: List[Node] = []
        self.goal_node: Optional[Node] = None

        root_node: Node = Node(state, 0.0, None, None, 0)

        self.lifo.append(root_node)

    def step(self):
        node = self.lifo.pop()
        if self.env.is_terminal(node.state):
            return node
        if node.depth >= self.limit:
            return None
        elif not is_cycle(node):
            children = expand_node(self.env,node)
            for child in children:
                self.lifo.append(child)

OpenSetElem = Tuple[float, Node]


class BestFirstSearch:

    def __init__(self, state: State, env: Environment, weight_g: float, weight_h: float):
        self.env: Environment = env
        self.weight_g: float = weight_g
        self.weight_h: float = weight_h

        self.priority_queue: List[OpenSetElem] = []
        self.closed_dict: Dict[State, Node] = dict()

        root_node: Node = Node(state, 0.0, None, None, 0)

        self.closed_dict[state] = root_node

        heuristic = get_heuristic(root_node)
        cost = get_cost(root_node, heuristic, self.weight_g, self.weight_h)
        heappush(self.priority_queue, (cost, root_node))

    def step(self):
        c, node = heappop(self.priority_queue)
        if self.env.is_terminal(node.state):
            return node
        children = expand_node(self.env,node)
        for child in children:
            s = child.state
            child.path_cost += child.parent.path_cost
            #cost + get_cost(child, cost, self.weight_g, self.weight_h)
            #get_cost(self.closed_dict[s], cost, self.weight_g, self.weight_h)
            if not s in self.closed_dict or child.path_cost < self.closed_dict[s].path_cost:
                heuristic = get_heuristic(child)
                cost = get_cost(child, heuristic, self.weight_g, self.weight_h)
                self.closed_dict[s] = child
                heappush(self.priority_queue, (cost, child))
