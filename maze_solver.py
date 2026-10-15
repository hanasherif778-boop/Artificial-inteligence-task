
from collections import deque
import heapq
import itertools
import time

maze = [
    "S...#.....",
    ".#..#..#..",
    ".#.....#..",
    ".###.#....",
    "...#.#.#..",
    ".#...#....",
    ".#####.##.",
    ".......#..",
    "###.#....#",
    "....#....E"
]

# Find the start and end positions
def find_position(symbol):
    for r in range(len(maze)):
        for c in range(len(maze[0])):
            if maze[r][c] == symbol:
                return (r, c)


start = find_position("S")
goal = find_position("E")

# Possible movements: up, down, left, right
directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# Reconstruct the path using parent positions
def get_path(parent, goal):
    path = []
    current = goal

    while current is not None:
        path.append(current)
        current = parent[current]

    return path[::-1]


# ---------------- BFS ----------------
def bfs():
    queue = deque([start])
    parent = {start: None}
    expanded = 0

    while queue:
        current = queue.popleft()
        expanded += 1

        if current == goal:
            break

        r, c = current

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            neighbor = (nr, nc)

            if (0 <= nr < len(maze)
                    and 0 <= nc < len(maze[0])
                    and maze[nr][nc] != "#"
                    and neighbor not in parent):

                parent[neighbor] = current
                queue.append(neighbor)

    if goal not in parent:
        return None, expanded

    return get_path(parent, goal), expanded


# ---------------- A* SEARCH ----------------
def heuristic(position):
    r, c = position
    gr, gc = goal

    # Manhattan distance
    return abs(r - gr) + abs(c - gc)


def astar():
    counter = itertools.count()

    # Each item contains f-score, tie-breaker, g-score, position
    priority_queue = [
        (heuristic(start), next(counter), 0, start)
    ]

    parent = {start: None}
    g_score = {start: 0}
    expanded = 0

    while priority_queue:
        f, _, g, current = heapq.heappop(priority_queue)

        # Ignore an outdated queue entry
        if g != g_score.get(current):
            continue

        expanded += 1

        if current == goal:
            break

        r, c = current

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            neighbor = (nr, nc)

            if (0 <= nr < len(maze)
                    and 0 <= nc < len(maze[0])
                    and maze[nr][nc] != "#"):

                new_g = g + 1

                if new_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = new_g
                    parent[neighbor] = current

                    new_f = new_g + heuristic(neighbor)

                    heapq.heappush(
                        priority_queue,
                        (new_f, next(counter), new_g, neighbor)
                    )

    if goal not in parent:
        return None, expanded

    return get_path(parent, goal), expanded


# Run BFS and measure its time
t1 = time.perf_counter()
bfs_path, bfs_expanded = bfs()
bfs_time = time.perf_counter() - t1

# Run A* and measure its time
t1 = time.perf_counter()
astar_path, astar_expanded = astar()
astar_time = time.perf_counter() - t1


# Display the results
print("BFS")
if bfs_path:
    print("Path length:", len(bfs_path) - 1)
else:
    print("No path found")
print("Nodes expanded:", bfs_expanded)
print("Time:", round(bfs_time, 6), "seconds")

print("\nA*")
if astar_path:
    print("Path length:", len(astar_path) - 1)
else:
    print("No path found")
print("Nodes expanded:", astar_expanded)
print("Time:", round(astar_time, 6) ,"seconds")

   