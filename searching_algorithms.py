from xmlrpc.client import MAXINT

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
    d2 = abs(p1[1] - p2[1])
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
        return int(h_euclidian_distance(p1, p2))

        #return int(math.sqrt(d1 ** 2 + d2 ** 2))

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


def dls(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
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
    limit:int = 200
    stack = [(start, 0)]
    visited = {start}
    came_from = dict()
    while len(stack) > 0:
        current, l = stack.pop()
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
                came_from[neighbor] = current
                if l < limit:
                    stack.append((neighbor, l + 1))
                    neighbor.make_open()
                    visited.add(neighbor)


        draw()
        if current != start:
            current.make_closed()

    return False
def ucs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
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

        #return int(math.sqrt(d1 ** 2 + d2 ** 2))

    pq = PriorityQueue()
    pq.put((0, start))
    came_from = dict()
    visited = {start}
    while not pq.empty():
        distance, current = pq.get()
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
                pq.put((distance + 1, neighbor))
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    pass

def dijkstra(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
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

        #return int(math.sqrt(d1 ** 2 + d2 ** 2))

    map = dict()
    map[start] = 0

    pq = PriorityQueue()
    pq.put((0, start))
    came_from = dict()
    visited = {start}
    while not pq.empty():
        distance, current = pq.get()
        # if current == end:
        #     while current in came_from:
        #         current = came_from[current]
        #         current.make_path()
        #         draw()
        #     end.make_end()
        #     start.make_start()
        #     return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                if map.get(neighbor) is None:
                    map[neighbor] = distance
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    pq.put((distance + 1, neighbor))
                    neighbor.make_open()
                elif map[neighbor] < distance:
                    map[neighbor] = distance
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    pq.put((distance + 1, neighbor))
                    neighbor.make_open()

        draw()
        current.make_closed()

    if map.get(end) is not None :
        curr = end
        while curr in came_from:
            curr = came_from[curr]
            curr.make_path()
            draw()
        end.make_end()
        start.make_start()
        return True
    return False
    pass

def dls_limit(draw: callable, grid: Grid, start: Spot, end: Spot, limit:int) -> bool:
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
    stack = [(start, 0)]
    visited = {start}
    came_from = dict()
    while len(stack) > 0:
        current, l = stack.pop()
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
                came_from[neighbor] = current
                if l < limit:
                    stack.append((neighbor, l + 1))
                    neighbor.make_open()
                    visited.add(neighbor)


        draw()
        if current != start:
            current.make_closed()

    return False
def clear_grid(grid: Grid):
    for spot in grid.grid:
        for s in spot:
            s.make_open()
def clear_grid_not_start_end(grid: Grid, start: Spot, end: Spot):
    for spot in grid.grid:
        for s in spot:
            if not s == start and not s == end:
                s.make_open()
def clear_grid_general(grid: Grid, start: Spot, end: Spot, walls:bool):
    for spot in grid.grid:
        for s in spot:
            if (s.is_barrier() and walls) or (not s == start and not s == end and not s.is_barrier()):
                #print(s.make_open())
                s.color = COLORS["WHITE"]

def iddfs(draw: callable, grid: Grid, start: Spot, end: Spot):
    for deapth in range(2500):
        if dls_limit(draw, grid, start, end, deapth):
            break
        clear_grid_general(grid, start, end, False)

def dfs_limit(draw: callable, grid: Grid, start: Spot, end: Spot, limit:float) -> tuple[bool, float]:
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
    stack = [(start, 0.0)]
    visited = {start}
    came_from = dict()
    min_cost:float = MAXINT
    while len(stack) > 0:
        current, cost = stack.pop()
        f = h_manhattan_distance((current.y, current.x), (end.y, end.x))
        #f = h_euclidian_distance((current.y, current.x), (end.y, end.x))
        if cost + f > limit:
            min_cost = min(min_cost, cost + f)
            continue
        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return (True, 0.0)

        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append((neighbor, cost + 1))
                neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return (False, min_cost)

def ida(draw: callable, grid: Grid, start: Spot, end: Spot):
    f = 0
    while True:
        success, min_cost = dfs_limit(draw, grid, start, end, f)
        if success:
            break
        clear_grid_general(grid, start, end, False)
        print(min_cost)
        f = min_cost

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss