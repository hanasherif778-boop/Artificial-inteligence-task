
import heapq
import time
from itertools import count

# The goal board
GOAL = (1, 2, 3,
        4, 5, 6,
        7, 8, 0)


# Check if a board can reach the goal
def is_solvable(state):
    tiles = [x for x in state if x != 0]
    inversions = 0

    for i in range(len(tiles)):
        for j in range(i + 1, len(tiles)):
            if tiles[i] > tiles[j]:
                inversions += 1

    return inversions % 2 == 0


# Heuristic 1: Count misplaced tiles
def misplaced_tiles(state):
    misplaced = 0

    for i in range(9):
        if state[i] != 0 and state[i] != GOAL[i]:
            misplaced += 1

    return misplaced


# Heuristic 2: Manhattan distance
def manhattan_distance(state):
    distance = 0

    for i in range(9):
        tile = state[i]

        if tile != 0:
            current_row = i // 3
            current_col = i % 3

            goal_index = tile - 1
            goal_row = goal_index // 3
            goal_col = goal_index % 3

            distance += abs(current_row - goal_row)
            distance += abs(current_col - goal_col)

    return distance

# Heuristic 3: Linear Conflict
def linear_conflict(state):
    # Start with Manhattan distance
    distance = manhattan_distance(state)
    conflicts = 0

    # Check rows
    for row in range(3):
        used = set()

        for col1 in range(3):
            pos1 = row * 3 + col1
            tile1 = state[pos1]

            if tile1 == 0 or (tile1 - 1) // 3 != row:
                continue

            if pos1 in used:
                continue

            for col2 in range(col1 + 1, 3):
                pos2 = row * 3 + col2
                tile2 = state[pos2]

                if pos2 in used:
                    continue

                if tile2 == 0 or (tile2 - 1) // 3 != row:
                    continue

                if (tile1 - 1) % 3 > (tile2 - 1) % 3:
                    conflicts += 1
                    used.add(pos1)
                    used.add(pos2)
                    break

    # Check columns
    for col in range(3):
        used = set()

        for row1 in range(3):
            pos1 = row1 * 3 + col
            tile1 = state[pos1]

            if tile1 == 0 or (tile1 - 1) % 3 != col:
                continue

            if pos1 in used:
                continue

            for row2 in range(row1 + 1, 3):
                pos2 = row2 * 3 + col
                tile2 = state[pos2]

                if pos2 in used:
                    continue

                if tile2 == 0 or (tile2 - 1) % 3 != col:
                    continue

                if (tile1 - 1) // 3 > (tile2 - 1) // 3:
                    conflicts += 1
                    used.add(pos1)
                    used.add(pos2)
                    break

    return distance + 2 * conflicts


# Generate all valid next boards
def get_neighbors(state):
    neighbors = []
    blank = state.index(0)

    row = blank // 3
    col = blank % 3

    directions = [(-1, 0), (1, 0),
                  (0, -1), (0, 1)]

    for dr, dc in directions:
        new_row = row + dr
        new_col = col + dc

        if 0 <= new_row < 3 and 0 <= new_col < 3:
            new_blank = new_row * 3 + new_col

            new_state = list(state)
            new_state[blank], new_state[new_blank] = (
                new_state[new_blank], new_state[blank]
            )

            neighbors.append(tuple(new_state))

    return neighbors


# A* search algorithm
def a_star(start, heuristic):
    if not is_solvable(start):
        return None, 0

    # Each queue item contains:
    # (f score, tie breaker, g score, board)
    counter = count()
    queue = []

    heapq.heappush(
        queue,
        (heuristic(start), next(counter), 0, start)
    )

    best_g = {start: 0}
    parent = {start: None}
    expanded = 0

    while queue:
        f, _, g, current = heapq.heappop(queue)

        # Ignore an outdated queue entry
        if g != best_g.get(current):
            continue

        expanded += 1

        # Have we reached the goal?
        if current == GOAL:
            path = []
            node = current

            while node is not None:
                path.append(node)
                node = parent[node]

            path.reverse()
            return path, expanded

        # Explore the next possible boards
        for neighbor in get_neighbors(current):
            new_g = g + 1

            if new_g < best_g.get(neighbor, float("inf")):
                best_g[neighbor] = new_g
                parent[neighbor] = current

                new_f = new_g + heuristic(neighbor)

                heapq.heappush(
                    queue,
                    (new_f, next(counter), new_g, neighbor)
                )

    return None, expanded


# Five test cases
test_cases = {
    "Already solved": GOAL,
    "One move away": (1, 2, 3, 4, 5, 6, 7, 0, 8),
    "Two moves away": (1, 2, 3, 4, 0, 6, 7, 5, 8),
    "Another solvable state": (1, 2, 3, 5, 0, 6, 4, 7, 8),
    "Unsolvable state": (1, 2, 3, 4, 5, 6, 8, 7, 0),
    "Unsolvable state": (7, 2, 4, 5, 0, 8, 3, 1, 0)
    
}


heuristics = {
    "Misplaced Tiles": misplaced_tiles,
    "Manhattan Distance": manhattan_distance,
    "Linear Conflict": linear_conflict
}




# Run every test using every heuristic
for test_name, start in test_cases.items():
    print("\nTest:", test_name)
    print("Solvable:", is_solvable(start))

    for heuristic_name, heuristic in heuristics.items():
        start_time = time.perf_counter()

        path, expanded = a_star(start, heuristic)

        elapsed = time.perf_counter() - start_time

        if path is None:
            print(
                heuristic_name,
                "| No solution",
                "| Expanded:", expanded,
                "| Time:", round(elapsed, 6), "seconds"
            )
        else:
            print(
                heuristic_name,
                "| Moves:", len(path) - 1,
                "| Expanded:", expanded,
                "| Time:", round(elapsed, 6), "seconds"
            )
