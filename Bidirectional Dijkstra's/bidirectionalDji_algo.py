# bidirectional_algo.py
# Algorithm module for Bidirectional Dijkstra + stepper for visualization.
# Contains:
#   - Graph: adjacency lists (forward & backward)
#   - bidirectional_dijkstra: standard function returning (best_distance, path)
#   - BiDijkstraStepper: generator that yields snapshots for each atomic action
#
# The stepper snapshots are dictionaries that the GUI can consume.

import heapq
from typing import Dict, List, Tuple, Optional, Any, Generator


class Graph:
    """
    Graph storing forward and backward adjacency lists.
    For nodes we accept any hashable (str/int).
    forward_graph[u] = list of (v, weight)
    backward_graph[v] = list of (u, weight)
    """
    def __init__(self):
        self.forward_graph: Dict[Any, List[Tuple[Any, float]]] = {}
        self.backward_graph: Dict[Any, List[Tuple[Any, float]]] = {}
        self.nodes: set = set()

    def add_edge(self, u: Any, v: Any, weight: float):
        """Add directed edge u -> v and record reverse for backward traversal."""
        self.forward_graph.setdefault(u, []).append((v, weight))
        self.forward_graph.setdefault(v, [])  # ensure v exists as key
        self.backward_graph.setdefault(u, [])
        self.backward_graph.setdefault(v, [])
        self.backward_graph[v].append((u, weight))
        self.nodes.add(u)
        self.nodes.add(v)

    def add_undirected_edge(self, u: Any, v: Any, weight: float):
        """Add both directions for undirected graphs."""
        self.add_edge(u, v, weight)
        self.add_edge(v, u, weight)


def bidirectional_dijkstra(graph: Graph, source: Any, target: Any) -> Tuple[Optional[float], Optional[List[Any]]]:
    """
    Standard bidirectional Dijkstra that returns (best_distance, path_list) or (None, None).
    Uses variable names that are easy to read:
      - distance_start: distances from source
      - distance_goal: distances from target
      - parent_start / parent_goal: parent pointers for reconstructing path
    """
    # Edge cases
    if source == target:
        return (0, [source])
    if source not in graph.nodes or target not in graph.nodes:
        return (None, None)

    # Initialization
    distance_start = {source: 0}
    distance_goal = {target: 0}
    parent_start = {source: None}
    parent_goal = {target: None}
    queue_start = [(0, source)]
    queue_goal = [(0, target)]
    visited_start = set()
    visited_goal = set()
    best_distance = float('inf')
    meet_node = None

    while queue_start and queue_goal:
        # termination lower bound
        if queue_start[0][0] + queue_goal[0][0] >= best_distance:
            break

        # forward expansion
        if queue_start:
            d_u, u = heapq.heappop(queue_start)
            if u not in visited_start:
                visited_start.add(u)
                if u in visited_goal:
                    cand = distance_start[u] + distance_goal[u]
                    if cand < best_distance:
                        best_distance = cand
                        meet_node = u
                for v, w in graph.forward_graph.get(u, []):
                    nd = distance_start[u] + w
                    if v not in distance_start or nd < distance_start[v]:
                        distance_start[v] = nd
                        parent_start[v] = u
                        heapq.heappush(queue_start, (nd, v))
                        if v in distance_goal:
                            cand = nd + distance_goal[v]
                            if cand < best_distance:
                                best_distance = cand
                                meet_node = v

        # backward expansion
        if queue_goal:
            d_u, u = heapq.heappop(queue_goal)
            if u not in visited_goal:
                visited_goal.add(u)
                if u in visited_start:
                    cand = distance_start[u] + distance_goal[u]
                    if cand < best_distance:
                        best_distance = cand
                        meet_node = u
                for v, w in graph.backward_graph.get(u, []):
                    nd = distance_goal[u] + w
                    if v not in distance_goal or nd < distance_goal[v]:
                        distance_goal[v] = nd
                        parent_goal[v] = u
                        heapq.heappush(queue_goal, (nd, v))
                        if v in distance_start:
                            cand = nd + distance_start[v]
                            if cand < best_distance:
                                best_distance = cand
                                meet_node = v

    if meet_node is None:
        return (None, None)

    # reconstruct path
    path_front = []
    cur = meet_node
    while cur is not None:
        path_front.append(cur)
        cur = parent_start[cur]
    path_front.reverse()  # from source to meet_node

    path_back = []
    cur = parent_goal.get(meet_node)
    while cur is not None:
        path_back.append(cur)
        cur = parent_goal.get(cur)

    full_path = path_front + path_back
    return best_distance, full_path


# -----------------------------
# Stepper generator for GUI
# -----------------------------
class BiDijkstraStepper:
    """
    Generator-based stepper that yields snapshots for each atomic action.
    Snapshots are dictionaries:
      'action' : description string
      'distance_start' : copy of start distances
      'distance_goal' : copy of goal distances
      'parent_start' / 'parent_goal' : copies of parents
      'queue_start' / 'queue_goal' : lists (heap internal state)
      'visited_start' / 'visited_goal' : sets
      'best' : current best_distance
      'meet_node' : current meeting node (or None)
      'highlight' : dict with keys like 'pop','relax','candidate','meeting','final_path'
    """
    def __init__(self, graph: Graph, source: Any, target: Any):
        self.g = graph
        self.source = source
        self.target = target

        # algorithm state (same names as in bidirectional_dijkstra)
        self.distance_start = {source: 0}
        self.distance_goal = {target: 0}
        self.parent_start = {source: None}
        self.parent_goal = {target: None}
        self.queue_start = [(0, source)]
        self.queue_goal = [(0, target)]
        self.visited_start = set()
        self.visited_goal = set()
        self.best = float('inf')
        self.meet_node = None

    def snapshot(self, action: str, highlight: dict = None):
        """Return a snapshot dict for GUI consumption."""
        return {
            'action': action,
            'distance_start': dict(self.distance_start),
            'distance_goal': dict(self.distance_goal),
            'parent_start': dict(self.parent_start),
            'parent_goal': dict(self.parent_goal),
            'queue_start': list(self.queue_start),
            'queue_goal': list(self.queue_goal),
            'visited_start': set(self.visited_start),
            'visited_goal': set(self.visited_goal),
            'best': self.best,
            'meet_node': self.meet_node,
            'highlight': highlight or {}
        }

    def step(self) -> Generator[dict, None, None]:
        """Yield snapshots for each atomic action (pop, relax, candidate, finalize...)."""
        # trivial cases
        if self.source == self.target:
            yield self.snapshot("Trivial: source == target", {'final_path': [self.source]})
            return
        if self.source not in self.g.nodes or self.target not in self.g.nodes:
            yield self.snapshot("Source or target not in graph", {})
            return

        while self.queue_start and self.queue_goal:
            # termination check
            if self.queue_start[0][0] + self.queue_goal[0][0] >= self.best:
                yield self.snapshot("Termination: lower bound >= best", {})
                break

            # forward pop
            if self.queue_start:
                d_u, u = heapq.heappop(self.queue_start)
                yield self.snapshot(f"Forward popped ({u}, {d_u})", {'pop': ('forward', u)})
                if u in self.visited_start:
                    yield self.snapshot(f"Forward skipped already visited {u}")
                else:
                    self.visited_start.add(u)
                    yield self.snapshot(f"Forward finalized {u}", {'finalize': ('forward', u)})

                    if u in self.visited_goal:
                        cand = self.distance_start[u] + self.distance_goal[u]
                        yield self.snapshot(f"Candidate via finalized node {u}: {cand}", {'candidate': (u, cand)})
                        if cand < self.best:
                            self.best = cand
                            self.meet_node = u
                            yield self.snapshot(f"Best updated -> {self.best} at meeting {u}", {'meeting': u, 'best': self.best})

                    for v, w in self.g.forward_graph.get(u, []):
                        yield self.snapshot(f"Forward consider relax {u} -> {v} (w={w})", {'consider_relax': (u, v)})
                        nd = self.distance_start[u] + w
                        old = self.distance_start.get(v)
                        if old is None or nd < old:
                            self.distance_start[v] = nd
                            self.parent_start[v] = u
                            heapq.heappush(self.queue_start, (nd, v))
                            yield self.snapshot(f"Forward relax {u} -> {v}: dist[{v}]={nd}", {'relax': (u, v)})
                            if v in self.distance_goal:
                                cand = nd + self.distance_goal[v]
                                yield self.snapshot(f"Candidate via {v}: {cand}", {'candidate': (v, cand)})
                                if cand < self.best:
                                    self.best = cand
                                    self.meet_node = v
                                    yield self.snapshot(f"Best updated -> {self.best} at meeting {v}", {'meeting': v, 'best': self.best})

            # backward pop
            if self.queue_goal:
                d_u, u = heapq.heappop(self.queue_goal)
                yield self.snapshot(f"Backward popped ({u}, {d_u})", {'pop': ('backward', u)})
                if u in self.visited_goal:
                    yield self.snapshot(f"Backward skipped already visited {u}")
                else:
                    self.visited_goal.add(u)
                    yield self.snapshot(f"Backward finalized {u}", {'finalize': ('backward', u)})

                    if u in self.visited_start:
                        cand = self.distance_start[u] + self.distance_goal[u]
                        yield self.snapshot(f"Candidate via finalized node {u}: {cand}", {'candidate': (u, cand)})
                        if cand < self.best:
                            self.best = cand
                            self.meet_node = u
                            yield self.snapshot(f"Best updated -> {self.best} at meeting {u}", {'meeting': u, 'best': self.best})

                    for v, w in self.g.backward_graph.get(u, []):
                        yield self.snapshot(f"Backward consider relax {u} <- {v} (w={w})", {'consider_relax': (v, u)})
                        nd = self.distance_goal[u] + w
                        old = self.distance_goal.get(v)
                        if old is None or nd < old:
                            self.distance_goal[v] = nd
                            self.parent_goal[v] = u
                            heapq.heappush(self.queue_goal, (nd, v))
                            yield self.snapshot(f"Backward relax (via) {v} -> {u}: dist_goal[{v}]={nd}", {'relax': (v, u)})
                            if v in self.distance_start:
                                cand = nd + self.distance_start[v]
                                yield self.snapshot(f"Candidate via {v}: {cand}", {'candidate': (v, cand)})
                                if cand < self.best:
                                    self.best = cand
                                    self.meet_node = v
                                    yield self.snapshot(f"Best updated -> {self.best} at meeting {v}", {'meeting': v, 'best': self.best})

        # after while
        if self.meet_node is not None:
            # reconstruct
            front = []
            cur = self.meet_node
            while cur is not None:
                front.append(cur)
                cur = self.parent_start.get(cur)
            front.reverse()
            back = []
            cur = self.parent_goal.get(self.meet_node)
            while cur is not None:
                back.append(cur)
                cur = self.parent_goal.get(cur)
            full = front + back
            yield self.snapshot(f"Final path found with cost {self.best}", {'final_path': full})
        else:
            yield self.snapshot("No path found", {})
