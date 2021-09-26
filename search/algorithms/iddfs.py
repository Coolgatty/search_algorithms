"""
Iterative deepening Depth-first Search specialization of a generic search algorithm.
"""

from typing import Optional

from search.algorithms.search import Node, SearchAlgorithm
from search.space import Space
from search.algorithms.dfs import DFS
import time
from math import sqrt, pi

class IDDFS(DFS):
    """Iterative deepening Depth-first Search."""

    def __init__(self, problem):
        super().__init__(problem)
        self.max_expansions = 2**64

    def __str__(self) -> str:
        """The string representation of this Node."""
        return "{}[]".format(
            self.__class__.__name__,
        )

    @classmethod
    def name(cls) -> str:
        """Returns the name of the Algorithm."""
        return "Iterative deepening Depth-first Search"

    # pylint: no-self-argument
    def create_starting_node(self, state: Space.State) -> Node:
        """Create an Starting Node."""
        self.nodes_created += 1
        return Node(state, action=None, parent=None)

    def reach(self, state: Space.State, action: Space.Action, parent: Node):
        """Reaches a state and updates Open."""
        if state in self.open:
            # If the state was already in Open, then we discard this new path
            # as we don't have a way of telling which one is better.
            return

        # depth = 0
        # node = parent
        # while node != None:
        #     depth += 1
        #     node = node.parent
        # if depth > self.max_depth:
        #     print("ignored a reach")
        #     return

        self.nodes_created += 1
        self.open.insert(Node(state, action, parent))

    def _actually_search(self, depth) -> Optional[Node]:
        """Finds a single goal Node."""
        node = self.open.pop()
        cost = 0
        parent = node
        while parent != None:
            cost += 1
            parent = parent.parent
        if cost > depth:
            for i in self.closed:
                if i == node.state:
                    self.closed.remove(i)
            return None

        if self.problem.is_goal(node.state):
            return node

        self.expansions += 1
        if self.expansions >= self.expansion_limit:
            print(str(self), ": giving up...")
            return None
        # Expand the node and consider all its neighboring states.
        self.closed.add(node.state)
        for action, state in self.problem.space.neighbors(node.state):
            self.states_generated += 1
            if state in self.closed:
                # Déjà vu, we reached an expanded state.
                continue  # Not falling for this (again?).
            #print(self.states_reached, self.max_states)
            self.states_reached += 1
            self.reach(state, action, parent=node)
            result = self._actually_search(depth)
            if result != None:
                return result

        return None

    def search(self) -> Optional[Node]:
        """Finds a single goal Node."""
        self.time_ns = time.perf_counter_ns()
        solution = None
        depth = 0
        while self.expansions < self.expansion_limit and solution == None:
            self.open = self.create_open()
            self.closed = set()
            for start in self.problem.starting_states:
                self.open.insert(self.create_starting_node(start))
            solution = self._actually_search(depth)
            depth += 5
        self.time_ns = time.perf_counter_ns() - self.time_ns

        return solution
