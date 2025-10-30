from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot
import math

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False

    ueue = deque()
    ueue.append(start)
    visited = {start}
    came_from = dict()

    while len(ueue) > 0:
        current = ueue.popleft()

        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                ueue.append(neighbor)
                neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False
    pass

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    stack = [start]
    visited = {start}
    came_from = dict()
    while len(stack) > 0:
        current = stack.pop()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)
                neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    d1 = abs(p1[0] - p2[0])
    d2 = (p1[1] - p2[1])
    return d1 + d2

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    d1 = p1[0] - p2[0]
    d2 = p1[1] - p2[1]
    # return int(abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]))

    return math.sqrt(d1 ** 2 + d2 ** 2)


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
        A* Pathfinding Algorithm.
        Args:
            draw (callable): A function to call to update the Pygame window.
            grid (Grid): The Grid object containing the spots.
            start (Spot): The starting spot.
            end (Spot): The ending spot.
        Returns:
            None
        """

    def h(p1: tuple[int, int], p2: tuple[int, int]) -> int:
        """
        Heuristic function for A* algorithm: uses the Manhattan distance between two points.
        Args:
            p1 (tuple[int, int]): The first point (x1, y1).
            p2 (tuple[int, int]): The second point (x2, y2).
        Returns:
            float: The Manhattan distance between p1 and p2.
        """
        d1 = p1[0] - p2[0]
        d2 = p1[1] - p2[1]
        # return int(abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]))

        return int(math.sqrt(d1 ** 2 + d2 ** 2))

    pq = PriorityQueue()
    pq.put((h((start.x, start.y), (end.x, end.y)), 0, start))
    came_from = dict()
    visited = {start}
    while not pq.empty():
        cost, distance, current = pq.get()
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                pq.put((h((neighbor.x, neighbor.y), (end.x, end.y)), distance + 1, neighbor))
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    pass

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss