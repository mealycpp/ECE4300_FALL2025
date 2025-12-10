"""
8-Connected A* Pathfinding - Python Implementation
Shows Python metrics only for comparison with Verilog

Metrics displayed match Verilog testbench format
"""

import heapq
import time
import numpy as np

class AStarPathfinder:
    """8-connected A* pathfinding with metrics"""
    
    def __init__(self, grid_size=16):
        self.grid_size = grid_size
        self.nodes_expanded = 0
        self.path_length = 0
        self.computation_time = 0
    
    def octile_distance(self, x1, y1, x2, y2):
        """Octile distance heuristic for 8-connected grids"""
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        max_d = max(dx, dy)
        min_d = min(dx, dy)
        return max_d * 10 + min_d * 4
    
    def get_neighbors(self, x, y):
        """Get all 8 neighbors with costs"""
        neighbors = []
        
        # Cardinal directions (cost = 10)
        if y > 0:
            neighbors.append((x, y-1, 10))
        if y < self.grid_size - 1:
            neighbors.append((x, y+1, 10))
        if x > 0:
            neighbors.append((x-1, y, 10))
        if x < self.grid_size - 1:
            neighbors.append((x+1, y, 10))
        
        # Diagonal directions (cost = 14)
        if y > 0 and x > 0:
            neighbors.append((x-1, y-1, 14))
        if y > 0 and x < self.grid_size - 1:
            neighbors.append((x+1, y-1, 14))
        if y < self.grid_size - 1 and x > 0:
            neighbors.append((x-1, y+1, 14))
        if y < self.grid_size - 1 and x < self.grid_size - 1:
            neighbors.append((x+1, y+1, 14))
        
        return neighbors
    
    def find_path(self, start_x, start_y, goal_x, goal_y, obstacle_map):
        """Find path using 8-connected A*"""
        start_time = time.time()
        
        open_set = []
        counter = 0
        g_score = {}
        f_score = {}
        parent = {}
        closed_set = set()
        
        self.nodes_expanded = 0
        
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        
        g_score[start] = 0
        f_score[start] = self.octile_distance(start_x, start_y, goal_x, goal_y)
        heapq.heappush(open_set, (f_score[start], counter, start))
        counter += 1
        parent[start] = None
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            self.nodes_expanded += 1
            
            if current == goal:
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = parent.get(node)
                path.reverse()
                
                path_map = np.zeros((self.grid_size, self.grid_size), dtype=int)
                for x, y in path:
                    path_map[y, x] = 1
                
                self.path_length = len(path)
                self.computation_time = time.time() - start_time
                
                stats = {
                    'path_found': True,
                    'path_length': len(path),
                    'nodes_expanded': self.nodes_expanded,
                    'computation_time_ms': self.computation_time * 1000,
                    'path_cost': g_score[goal]
                }
                
                return path, path_map, stats
            
            current_x, current_y = current
            for nx, ny, move_cost in self.get_neighbors(current_x, current_y):
                neighbor = (nx, ny)
                
                if obstacle_map[ny, nx] == 1 or neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.octile_distance(nx, ny, goal_x, goal_y)
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1
        
        self.computation_time = time.time() - start_time
        
        stats = {
            'path_found': False,
            'path_length': 0,
            'nodes_expanded': self.nodes_expanded,
            'computation_time_ms': self.computation_time * 1000,
            'path_cost': 0
        }
        
        return None, np.zeros((self.grid_size, self.grid_size), dtype=int), stats


def visualize_path(obstacle_map, path_map, start, goal):
    """Print grid visualization"""
    grid_size = obstacle_map.shape[0]
    
    print("\nGrid Visualization:")
    print("S=Start, G=Goal, #=Obstacle, *=Path, .=Empty\n")
    
    for y in range(grid_size):
        for x in range(grid_size):
            if (x, y) == start:
                print("S ", end="")
            elif (x, y) == goal:
                print("G ", end="")
            elif obstacle_map[y, x] == 1:
                print("# ", end="")
            elif path_map[y, x] == 1:
                print("* ", end="")
            else:
                print(". ", end="")
        print()


def print_metrics(stats, test_num, start, goal, obstacle_count):
    """Print metrics matching Verilog format"""
    
    print(f"\nTest Configuration:")
    print(f"  Start Position:        ({start[0]}, {start[1]})")
    print(f"  Goal Position:         ({goal[0]}, {goal[1]})")
    print(f"  Grid Size:             16x16 (256 nodes)")
    print(f"  Obstacles:             {obstacle_count} ({obstacle_count*100.0/256:.1f}%)")
    
    print(f"\n[METRIC 1] Path Quality:")
    print(f"  Path Found:            {'YES' if stats['path_found'] else 'NO'}")
    if stats['path_found']:
        print(f"  Path Length:           {stats['path_length']} nodes")
        print(f"  Path Cost:             {stats['path_cost']}")
    
    print(f"\n[METRIC 2] Algorithm Efficiency:")
    print(f"  Nodes Expanded:        {stats['nodes_expanded']}")
    
    print(f"\n[METRIC 3] Computation Performance:")
    time_us = stats['computation_time_ms'] * 1000
    print(f"  Time (μs):             {time_us:.2f}")
    print(f"  Time (ms):             {stats['computation_time_ms']:.3f}")
    if stats['nodes_expanded'] > 0:
        time_per_node = time_us / stats['nodes_expanded']
        print(f"  Time per Node:         {time_per_node:.2f} μs/node")


# Test case builders
def build_test_case_1():
    obstacle_map = np.zeros((16, 16), dtype=int)
    start = (0, 0)
    goal = (7, 7)
    return obstacle_map, start, goal

def build_test_case_2():
    GRID_SIZE = 16
    obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    for i in range(2, 14):
        obstacle_map[3, i] = 1
        obstacle_map[12, i] = 1
        obstacle_map[i, 3] = 1
        obstacle_map[i, 12] = 1
    
    for i in range(5, 11):
        obstacle_map[6, i] = 1
        obstacle_map[9, i] = 1
    
    for i in range(6, 10):
        obstacle_map[i, 6] = 1
        obstacle_map[i, 10] = 1
    
    obstacle_map[3, 8] = 0
    obstacle_map[6, 10] = 0
    obstacle_map[9, 6] = 0
    
    start = (1, 1)
    goal = (14, 14)
    
    return obstacle_map, start, goal

def build_test_case_3():
    GRID_SIZE = 16
    obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    seed = 42
    rand_val = seed
    
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            rand_val = (rand_val * 1103515245 + 12345) % (2**31)
            if not ((i == 1 and j == 1) or (i == 14 and j == 14)):
                if (rand_val % 100) < 30:
                    obstacle_map[j, i] = 1
    
    start = (1, 1)
    goal = (14, 14)
    
    return obstacle_map, start, goal

def build_test_case_4():
    GRID_SIZE = 16
    obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    for j in range(2, GRID_SIZE, 3):
        for i in range(GRID_SIZE - 1):
            obstacle_map[j, i] = 1
    
    for j in range(4, GRID_SIZE, 3):
        for i in range(1, GRID_SIZE):
            obstacle_map[j, i] = 1
    
    start = (0, 0)
    goal = (15, 15)
    
    return obstacle_map, start, goal

def build_test_case_5():
    GRID_SIZE = 16
    obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    for i in range(1, 6):
        obstacle_map[1, i] = 1
        obstacle_map[6, i] = 1
        obstacle_map[i, 1] = 1
        obstacle_map[i, 6] = 1
    
    for i in range(9, 14):
        obstacle_map[1, i] = 1
        obstacle_map[6, i] = 1
        obstacle_map[i, 9] = 1
        obstacle_map[i, 14] = 1
    
    for i in range(1, 6):
        obstacle_map[9, i] = 1
        obstacle_map[14, i] = 1
        obstacle_map[i, 1] = 1
        obstacle_map[i, 6] = 1
    
    for i in range(9, 14):
        obstacle_map[9, i] = 1
        obstacle_map[14, i] = 1
        obstacle_map[i, 9] = 1
        obstacle_map[i, 14] = 1
    
    obstacle_map[3, 6] = 0
    obstacle_map[3, 9] = 0
    obstacle_map[6, 3] = 0
    obstacle_map[9, 3] = 0
    obstacle_map[11, 6] = 0
    obstacle_map[11, 9] = 0
    
    start = (2, 2)
    goal = (13, 13)
    
    return obstacle_map, start, goal

def build_test_case_6():
    GRID_SIZE = 16
    obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if j > i + 1 or j < i - 1:
                obstacle_map[j, i] = 1
    
    start = (0, 0)
    goal = (15, 15)
    
    return obstacle_map, start, goal

def build_test_case_7():
    GRID_SIZE = 16
    obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    for i in range(GRID_SIZE):
        obstacle_map[7, i] = 1
    
    start = (0, 0)
    goal = (15, 15)
    
    return obstacle_map, start, goal


def run_all_test_cases():
    """Run all test cases"""
    
    print("="*70)
    print("PYTHON A* PATHFINDING - 8-CONNECTED")
    print("="*70)
    print("Grid Size: 16x16")
    print("Costs: Straight=10, Diagonal=14")
    print("Heuristic: Octile Distance")
    print()
    
    test_cases = [
        (1, "Simple diagonal path", build_test_case_1),
        (2, "Spiral maze", build_test_case_2),
        (3, "Dense random obstacles", build_test_case_3),
        (4, "Snake pattern", build_test_case_4),
        (5, "Rooms and corridors", build_test_case_5),
        (6, "Long diagonal corridor", build_test_case_6),
        (7, "No path (complete wall)", build_test_case_7),
    ]
    
    for test_num, test_name, builder_func in test_cases:
        print("="*70)
        print(f"TEST CASE {test_num}: {test_name}")
        print("="*70)
        
        obstacle_map, start, goal = builder_func()
        obstacle_count = np.sum(obstacle_map)
        
        pathfinder = AStarPathfinder(grid_size=16)
        path, path_map, stats = pathfinder.find_path(
            start[0], start[1], goal[0], goal[1], obstacle_map
        )
        
        print_metrics(stats, test_num, start, goal, obstacle_count)
        visualize_path(obstacle_map, path_map, start, goal)
        
        print()
    
    print("="*70)
    print("ALL TEST CASES COMPLETE")
    print("="*70)


if __name__ == "__main__":
    run_all_test_cases()