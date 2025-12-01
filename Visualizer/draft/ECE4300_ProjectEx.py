import matplotlib.pyplot as plt
import numpy as np
import heapq

GRID_SIZE = 16
grid = np.zeros((GRID_SIZE, GRID_SIZE))

# Obstacles (row, col)
obstacles = [(5, i) for i in range(GRID_SIZE) if i != 12] + [(10, i) for i in range(GRID_SIZE) if i != 5]
for row, col in obstacles:
    grid[row][col] = 1

start = (0, 0)
goal = (14, 14)

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for d_row, d_col in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+d_row, current[1]+d_col)
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

path = astar(grid, start, goal)

# Check for obstacle violation
if path:
    for step in path:
        if grid[step[0]][step[1]] == 1:
            print("ERROR: Path goes through obstacle at", step)

# Visualization
fig, ax = plt.subplots(figsize=(8, 8))

# Major ticks centered in cells
ax.set_xticks(np.arange(GRID_SIZE) + 0.5)
ax.set_yticks(np.arange(GRID_SIZE) + 0.5)
ax.set_xticklabels(np.arange(GRID_SIZE))
ax.set_yticklabels(np.arange(GRID_SIZE))

# Minor ticks for grid lines
ax.set_xticks(np.arange(GRID_SIZE + 1) - 0.5, minor=True)
ax.set_yticks(np.arange(GRID_SIZE + 1) - 0.5, minor=True)
ax.grid(which="minor", color="gray", linestyle='-', linewidth=0.5)
ax.tick_params(which="minor", bottom=False, left=False)

ax.invert_yaxis()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE - 0.5)
ax.set_ylim(-0.5, GRID_SIZE - 0.5)

# Draw path first
if path:
    path_x = [col + 0.5 for row, col in path]
    path_y = [row + 0.5 for row, col in path]
    ax.plot(path_x, path_y, color="green", linewidth=1.5, zorder=1)

# Draw obstacles above path
for row in range(GRID_SIZE):
    for col in range(GRID_SIZE):
        if grid[row][col] == 1:
            ax.add_patch(plt.Rectangle((col, row), 1, 1, color="black", edgecolor="gray", zorder=2))

# Mark start and goal
ax.plot(start[1] + 0.5, start[0] + 0.5, "bo", markersize=10, label="Start", zorder=3)
ax.plot(goal[1] + 0.5, goal[0] + 0.5, "ro", markersize=10, label="Goal", zorder=3)

ax.legend()
plt.title("A* Pathfinding Grid")
plt.show()
