"""
Simulation Logic for Auto and Manual Modes
Handles lawnmower coverage, RSSI measurement, and A* pathfinding with obstacles
"""

import numpy as np
from obstacles import ObstacleMap, AStarPathfinder


class Simulation:
    """Controls robot movement through multiple phases"""

    def __init__(self, robot, field, learning_rate=3.0, gradient_delta=2.0, noise_std=0.3):
        """
        Initialize simulation

        Args:
            robot (Robot): Robot instance
            field (np.ndarray): RSSI field
            learning_rate (float): Step size for gradient descent (η) - increased from 0.5
            gradient_delta (float): Delta for finite difference gradient (δ)
            noise_std (float): Standard deviation of Gaussian noise
        """
        self.robot = robot
        self.field = field
        self.learning_rate = learning_rate
        self.initial_learning_rate = learning_rate
        self.gradient_delta = gradient_delta
        self.noise_std = noise_std

        # Phase management
        self.phase = 'lawnmower'  # 'lawnmower', 'measuring', 'visualizing', 'descent', 'manual'
        self.phase_timer = 0

        # Obstacles and pathfinding (MUST BE FIRST - before lawnmower generation)
        self.obstacle_map = ObstacleMap(map_size=field.shape[0], num_obstacles=5)
        self.obstacle_map.generate_obstacles()
        self.pathfinder = AStarPathfinder(self.obstacle_map)
        self.planned_path = []
        self.path_index = 0

        # Lawnmower pattern parameters
        self.lawnmower_spacing = 5.0  # Grid spacing
        self.lawnmower_speed = 3.0  # Increased speed
        self.lawnmower_path = []
        self.lawnmower_index = 0
        self.lawnmower_detour_path = []  # Path around obstacle
        self.lawnmower_detour_index = 0
        self._generate_lawnmower_path()  # Now obstacle_map exists!

        # Measured RSSI points
        self.measured_points = []

        # Gradient descent momentum (to escape local minima)
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.momentum = 0.7  # Momentum coefficient

        # Adaptive learning and exploration
        self.descent_iterations = 0
        self.stuck_counter = 0
        self.last_rssi = 0.0
        self.rssi_improvement_threshold = 0.1  # Min improvement to not be "stuck"

        # Track lawnmower path end for visualization
        self.lawnmower_path_end_index = 0

        # Target location (global minimum)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_rssi = 0.0

    def _generate_lawnmower_path(self):
        """Generate lawnmower coverage pattern that avoids obstacles"""
        map_size = self.field.shape[0]
        spacing = self.lawnmower_spacing

        path = []
        y = spacing
        going_right = True

        while y < map_size - spacing:
            if going_right:
                # Move right across
                for x in np.arange(spacing, map_size - spacing, spacing):
                    # Only add waypoint if not in obstacle
                    if not self.obstacle_map.is_obstacle(x, y):
                        path.append((x, y))
            else:
                # Move left across
                for x in np.arange(map_size - spacing, spacing, -spacing):
                    # Only add waypoint if not in obstacle
                    if not self.obstacle_map.is_obstacle(x, y):
                        path.append((x, y))

            y += spacing
            going_right = not going_right

        self.lawnmower_path = path

    def update_field(self, new_field):
        """Update the RSSI field and reset phases"""
        self.field = new_field
        self.phase = 'lawnmower'
        self.phase_timer = 0
        self.lawnmower_index = 0
        self.measured_points = []
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.descent_iterations = 0
        self.stuck_counter = 0
        self.last_rssi = 0.0
        self.lawnmower_path_end_index = 0
        self.learning_rate = self.initial_learning_rate

        # Regenerate obstacles
        self.obstacle_map.generate_obstacles()
        self._generate_lawnmower_path()

    def compute_gradient(self):
        """
        Compute gradient using finite differences
        NEGATIVE gradient for descent (go to minimum/weakest RSSI)

        Returns:
            tuple: (grad_x, grad_y)
        """
        x, y = self.robot.x, self.robot.y
        delta = self.gradient_delta

        # Compute partial derivatives using central differences
        x_plus = self._get_rssi_safe(x + delta, y)
        x_minus = self._get_rssi_safe(x - delta, y)
        grad_x = (x_plus - x_minus) / (2 * delta)

        y_plus = self._get_rssi_safe(x, y + delta)
        y_minus = self._get_rssi_safe(x, y - delta)
        grad_y = (y_plus - y_minus) / (2 * delta)

        # NEGATE for descent (go to minimum)
        return -grad_x, -grad_y

    def _get_rssi_safe(self, x, y):
        """
        Get RSSI value with boundary checking

        Args:
            x (float): X coordinate
            y (float): Y coordinate

        Returns:
            float: RSSI value
        """
        map_size = self.field.shape[0]
        x_idx = int(np.clip(x, 0, map_size - 1))
        y_idx = int(np.clip(y, 0, map_size - 1))
        return self.field[y_idx, x_idx]

    def update_lawnmower(self):
        """Update robot position following lawnmower pattern with obstacle navigation"""
        if self.lawnmower_index >= len(self.lawnmower_path):
            # Lawnmower complete, switch to measuring phase
            self.phase = 'measuring'
            self.phase_timer = 0
            self.lawnmower_path_end_index = len(self.robot.path)  # Mark where lawnmower ended
            print("Lawnmower coverage complete. Measuring RSSI...")
            return

        # If we're on a detour around an obstacle, follow that first
        if self.lawnmower_detour_path and self.lawnmower_detour_index < len(self.lawnmower_detour_path):
            self._follow_lawnmower_detour()
            return

        # Get target position
        target_x, target_y = self.lawnmower_path[self.lawnmower_index]

        # Move toward target
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < self.lawnmower_speed:
            # Reached waypoint
            self.robot.update_position(target_x, target_y)
            self.lawnmower_index += 1

            # Record measurement at this point
            rssi = self.robot.get_rssi_at_position(self.field)
            self.measured_points.append((target_x, target_y, rssi))
        else:
            # Move toward waypoint
            dx = (dx / distance) * self.lawnmower_speed
            dy = (dy / distance) * self.lawnmower_speed

            new_x = self.robot.x + dx
            new_y = self.robot.y + dy

            # Check if path is blocked by obstacle
            if not self.obstacle_map.is_path_clear(self.robot.x, self.robot.y, target_x, target_y, step=1.0):
                # Plan detour around obstacle
                print(f"Obstacle blocking path to waypoint {self.lawnmower_index}, planning detour...")
                detour = self.pathfinder.find_path(
                    (self.robot.x, self.robot.y),
                    (target_x, target_y),
                    grid_spacing=3
                )
                if detour and len(detour) > 1:
                    self.lawnmower_detour_path = detour
                    self.lawnmower_detour_index = 0
                else:
                    # Can't reach this waypoint, skip it
                    print(f"Cannot reach waypoint {self.lawnmower_index}, skipping...")
                    self.lawnmower_index += 1
            elif self.obstacle_map.is_obstacle(new_x, new_y):
                # Next step would be in obstacle
                self.lawnmower_index += 1
            else:
                self.robot.update_position(new_x, new_y)
                self.robot.get_rssi_at_position(self.field)

    def _follow_lawnmower_detour(self):
        """Follow the detour path around an obstacle"""
        if self.lawnmower_detour_index >= len(self.lawnmower_detour_path):
            # Detour complete
            self.lawnmower_detour_path = []
            self.lawnmower_detour_index = 0
            return

        target_x, target_y = self.lawnmower_detour_path[self.lawnmower_detour_index]

        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < self.lawnmower_speed * 1.5:
            # Reached detour waypoint
            self.robot.update_position(target_x, target_y)
            self.lawnmower_detour_index += 1

            # Measure at detour points too
            rssi = self.robot.get_rssi_at_position(self.field)
            self.measured_points.append((target_x, target_y, rssi))
        else:
            # Move toward detour waypoint
            dx = (dx / distance) * self.lawnmower_speed
            dy = (dy / distance) * self.lawnmower_speed
            self.robot.update_position(self.robot.x + dx, self.robot.y + dy)
            self.robot.get_rssi_at_position(self.field)

    def update_measuring(self):
        """Simulate measuring phase (brief pause)"""
        self.phase_timer += 1

        if self.phase_timer > 60:  # 1 second at 60 FPS
            self.phase = 'visualizing'
            self.phase_timer = 0
            print(f"Measured {len(self.measured_points)} points. Visualizing heatmap...")

    def update_visualizing(self):
        """Simulate visualization phase (brief pause)"""
        self.phase_timer += 1

        if self.phase_timer > 120:  # 2 seconds at 60 FPS
            # Find global minimum from measured points
            self._find_global_minimum()

            # Plan path using A* to avoid obstacles
            start = (self.robot.x, self.robot.y)
            goal = (self.target_x, self.target_y)
            print(f"Found global minimum at ({self.target_x:.1f}, {self.target_y:.1f}) with RSSI {self.target_rssi:.1f} dBm")
            print("Planning path with A* algorithm...")

            path = self.pathfinder.find_path(start, goal, grid_spacing=2)
            if path:
                self.planned_path = self.pathfinder.smooth_path(path)
                self.path_index = 0
                print(f"Path planned with {len(self.planned_path)} waypoints")
            else:
                print("No path found! Will use gradient descent to navigate around obstacles.")
                self.planned_path = []

            self.phase = 'descent'
            self.phase_timer = 0

    def _find_global_minimum(self):
        """Find the global minimum RSSI from all measured points"""
        if not self.measured_points:
            # Fallback: scan entire field
            min_rssi = self.field.max()
            min_x, min_y = self.robot.x, self.robot.y

            for x in range(self.field.shape[1]):
                for y in range(self.field.shape[0]):
                    rssi = self.field[y, x]
                    if rssi < min_rssi:
                        min_rssi = rssi
                        min_x, min_y = x, y
        else:
            # Find minimum from measured points
            min_point = min(self.measured_points, key=lambda p: p[2])
            min_x, min_y, min_rssi = min_point

        self.target_x = min_x
        self.target_y = min_y
        self.target_rssi = min_rssi

    def update_descent(self):
        """
        Navigate to global minimum using A* path (if available) or gradient descent
        """
        # If we have a planned path, follow it
        if self.planned_path and self.path_index < len(self.planned_path):
            self._follow_astar_path()
        else:
            # Fallback: direct navigation or gradient descent if near obstacles
            self._navigate_to_target()

    def _follow_astar_path(self):
        """Follow the A* planned path"""
        # Get current waypoint
        target_x, target_y = self.planned_path[self.path_index]

        # Calculate distance to waypoint
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        distance = np.sqrt(dx**2 + dy**2)

        # Check if reached waypoint
        if distance < 3.0:
            self.path_index += 1
            if self.path_index >= len(self.planned_path):
                # Reached goal!
                if self.descent_iterations == 0:
                    print(f"Reached global minimum! RSSI: {self.robot.current_rssi:.1f} dBm")
                    self.descent_iterations = 1
                return

        # Move toward waypoint
        speed = min(2.5, distance * 0.5)
        if distance > 0:
            dx_norm = dx / distance
            dy_norm = dy / distance

            new_x = self.robot.x + dx_norm * speed
            new_y = self.robot.y + dy_norm * speed

            # Check if path is blocked (dynamic obstacle avoidance)
            if self.obstacle_map.is_obstacle(new_x, new_y):
                # Use gradient descent to navigate around
                self._gradient_descent_step()
            else:
                self.robot.update_position(new_x, new_y)
                self.robot.get_rssi_at_position(self.field)

    def _navigate_to_target(self):
        """Direct navigation to target (fallback if no A* path)"""
        dx = self.target_x - self.robot.x
        dy = self.target_y - self.robot.y
        distance = np.sqrt(dx**2 + dy**2)

        if distance < 2.0:
            if self.descent_iterations == 0:
                print(f"Reached global minimum! RSSI: {self.robot.current_rssi:.1f} dBm")
                self.descent_iterations = 1
            return

        speed = min(3.0, distance * 0.5)
        if distance > 0:
            dx_norm = dx / distance
            dy_norm = dy / distance

            new_x = self.robot.x + dx_norm * speed
            new_y = self.robot.y + dy_norm * speed

            # If path blocked, use gradient descent
            if self.obstacle_map.is_obstacle(new_x, new_y):
                self._gradient_descent_step()
            else:
                self.robot.update_position(new_x, new_y)
                self.robot.get_rssi_at_position(self.field)

    def _gradient_descent_step(self):
        """
        Single step of gradient descent (for obstacle avoidance)
        Combines RSSI gradient with repulsion from obstacles
        """
        # Compute RSSI gradient (descent toward minimum)
        grad_x, grad_y = self.compute_gradient()

        # Add obstacle repulsion
        repulsion_x, repulsion_y = self._compute_obstacle_repulsion()

        # Combine forces
        total_x = grad_x + 0.5 * repulsion_x
        total_y = grad_y + 0.5 * repulsion_y

        # Update with momentum
        self.velocity_x = 0.5 * self.velocity_x + total_x
        self.velocity_y = 0.5 * self.velocity_y + total_y

        # Move
        new_x = self.robot.x + 2.0 * self.velocity_x
        new_y = self.robot.y + 2.0 * self.velocity_y

        # Make sure we don't move into obstacle
        if not self.obstacle_map.is_obstacle(new_x, new_y):
            self.robot.update_position(new_x, new_y)
            self.robot.get_rssi_at_position(self.field)

    def _compute_obstacle_repulsion(self):
        """Compute repulsion force from nearby obstacles"""
        repulsion_x = 0.0
        repulsion_y = 0.0

        # Check nearby cells for obstacles
        check_distance = 5.0
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                check_x = self.robot.x + dx
                check_y = self.robot.y + dy

                if self.obstacle_map.is_obstacle(check_x, check_y):
                    # Compute repulsion vector
                    dist = np.sqrt(dx**2 + dy**2) + 0.1
                    if dist < check_distance:
                        strength = (check_distance - dist) / dist
                        repulsion_x -= dx * strength
                        repulsion_y -= dy * strength

        return repulsion_x, repulsion_y

    def update_manual(self, keys, speed=2.0, boost_speed=3.0):
        """
        Update robot position in manual mode based on keyboard input

        Args:
            keys (dict): Pygame key states
            speed (float): Normal movement speed
            boost_speed (float): Speed when shift is held
        """
        import pygame

        # Check if shift is held for speed boost
        shift_held = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        current_speed = boost_speed if shift_held else speed

        dx, dy = 0, 0

        # Arrow keys or WASD
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            dy -= current_speed
        if keys[pygame.K_DOWN] or keys[pygame.K_s]:
            dy += current_speed
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            dx -= current_speed
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            dx += current_speed

        # Only update if there's movement
        if dx != 0 or dy != 0:
            new_x = self.robot.x + dx
            new_y = self.robot.y + dy
            self.robot.update_position(new_x, new_y)

            # Update RSSI reading
            self.robot.get_rssi_at_position(self.field)

    def update(self, keys=None):
        """
        Main update function - dispatches to appropriate phase

        Args:
            keys (dict): Pygame key states (for manual mode)
        """
        if self.robot.mode == 'manual':
            self.phase = 'manual'
            if keys is not None:
                self.update_manual(keys)
        elif self.robot.mode == 'auto':
            # Multi-phase automatic operation
            if self.phase == 'lawnmower':
                self.update_lawnmower()
            elif self.phase == 'measuring':
                self.update_measuring()
            elif self.phase == 'visualizing':
                self.update_visualizing()
            elif self.phase == 'descent':
                self.update_descent()

    def get_phase_description(self):
        """Get human-readable phase description"""
        descriptions = {
            'lawnmower': f'COVERAGE: Lawnmower scan ({self.lawnmower_index}/{len(self.lawnmower_path)})',
            'measuring': 'MEASURING: Processing RSSI data...',
            'visualizing': 'VISUALIZING: Building heatmap...',
            'descent': 'DESCENT: Seeking weakest signal',
            'manual': 'MANUAL: User control'
        }
        return descriptions.get(self.phase, self.phase.upper())
