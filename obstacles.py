"""
Obstacle Generation and A* Pathfinding
"""

import numpy as np
import heapq


class ObstacleMap:
    """Generate and manage obstacles in the environment"""

    def __init__(self, map_size=100, num_obstacles=5):
        """
        Initialize obstacle map

        Args:
            map_size (int): Size of the map
            num_obstacles (int): Number of rectangular obstacles
        """
        self.map_size = map_size
        self.num_obstacles = num_obstacles
        self.obstacles = []
        self.obstacle_grid = np.zeros((map_size, map_size), dtype=bool)

    def generate_obstacles(self):
        """Generate random rectangular obstacles"""
        self.obstacles = []
        self.obstacle_grid = np.zeros((self.map_size, self.map_size), dtype=bool)

        for _ in range(self.num_obstacles):
            # Random position
            x = np.random.randint(10, self.map_size - 30)
            y = np.random.randint(10, self.map_size - 30)

            # Random size
            width = np.random.randint(8, 20)
            height = np.random.randint(8, 20)

            # Store obstacle
            obstacle = {
                'x': x,
                'y': y,
                'width': width,
                'height': height
            }
            self.obstacles.append(obstacle)

            # Mark in grid
            x_end = min(x + width, self.map_size)
            y_end = min(y + height, self.map_size)
            self.obstacle_grid[y:y_end, x:x_end] = True

    def is_obstacle(self, x, y):
        """
        Check if position is inside an obstacle

        Args:
            x (float): X coordinate
            y (float): Y coordinate

        Returns:
            bool: True if obstacle
        """
        x_idx = int(np.clip(x, 0, self.map_size - 1))
        y_idx = int(np.clip(y, 0, self.map_size - 1))
        return self.obstacle_grid[y_idx, x_idx]

    def is_path_clear(self, x1, y1, x2, y2, step=1.0):
        """
        Check if line segment between two points is obstacle-free

        Args:
            x1, y1: Start point
            x2, y2: End point
            step: Step size for checking

        Returns:
            bool: True if path is clear
        """
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if distance == 0:
            return not self.is_obstacle(x1, y1)

        steps = int(distance / step) + 1
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.is_obstacle(x, y):
                return False
        return True


class AStarPathfinder:
    """A* pathfinding algorithm for navigation around obstacles"""

    def __init__(self, obstacle_map):
        """
        Initialize A* pathfinder

        Args:
            obstacle_map (ObstacleMap): Obstacle map
        """
        self.obstacle_map = obstacle_map
        self.map_size = obstacle_map.map_size

    def heuristic(self, a, b):
        """
        Heuristic function (Euclidean distance)

        Args:
            a: (x, y) tuple
            b: (x, y) tuple

        Returns:
            float: Distance
        """
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, node, grid_spacing=2):
        """
        Get valid neighbors of a node

        Args:
            node: (x, y) tuple
            grid_spacing: Grid resolution

        Returns:
            list: List of neighbor nodes
        """
        x, y = node
        neighbors = []

        # 8-connected grid (including diagonals)
        directions = [
            (grid_spacing, 0), (-grid_spacing, 0),
            (0, grid_spacing), (0, -grid_spacing),
            (grid_spacing, grid_spacing), (-grid_spacing, -grid_spacing),
            (grid_spacing, -grid_spacing), (-grid_spacing, grid_spacing)
        ]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            # Check bounds
            if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                # Check if not obstacle
                if not self.obstacle_map.is_obstacle(nx, ny):
                    neighbors.append((nx, ny))

        return neighbors

    def find_path(self, start, goal, grid_spacing=2):
        """
        Find path from start to goal using A*

        Args:
            start: (x, y) tuple
            goal: (x, y) tuple
            grid_spacing: Grid resolution for pathfinding

        Returns:
            list: Path as list of (x, y) tuples, or None if no path
        """
        # Snap to grid
        start = (round(start[0] / grid_spacing) * grid_spacing,
                 round(start[1] / grid_spacing) * grid_spacing)
        goal = (round(goal[0] / grid_spacing) * grid_spacing,
                round(goal[1] / grid_spacing) * grid_spacing)

        # Check if start or goal is in obstacle
        if self.obstacle_map.is_obstacle(start[0], start[1]):
            print("Start position is in obstacle!")
            return None
        if self.obstacle_map.is_obstacle(goal[0], goal[1]):
            print("Goal position is in obstacle!")
            return None

        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        closed_set = set()

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current in closed_set:
                continue

            # Check if reached goal
            if self.heuristic(current, goal) < grid_spacing * 1.5:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            closed_set.add(current)

            # Explore neighbors
            for neighbor in self.get_neighbors(current, grid_spacing):
                if neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # No path found
        print("No path found from start to goal!")
        return None

    def smooth_path(self, path):
        """
        Smooth path by removing unnecessary waypoints

        Args:
            path: List of (x, y) tuples

        Returns:
            list: Smoothed path
        """
        if not path or len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Look ahead to see if we can skip waypoints
            j = len(path) - 1
            while j > i + 1:
                if self.obstacle_map.is_path_clear(
                    path[i][0], path[i][1],
                    path[j][0], path[j][1]
                ):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed.append(path[i + 1])
                i += 1

        return smoothed
