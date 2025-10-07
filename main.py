"""
RF Coverage Mapping Simulator - Main Entry Point
Interactive PyGame visualization with auto/manual robot navigation
"""

import pygame
import numpy as np
import sys

from map_generator import generate_rssi_map
from robot import Robot
from simulation import Simulation
from utils import (
    field_to_surface, map_to_screen, load_config,
    draw_info_panel, export_path_to_csv
)


class RFCoverageSimulator:
    """Main simulator application"""

    def __init__(self, config_path='config.json'):
        """
        Initialize the simulator

        Args:
            config_path (str): Path to configuration file
        """
        # Load configuration
        self.config = load_config(config_path)

        # Extract config values
        self.map_size = self.config['map_size']
        self.window_size = self.config['window_size']
        self.fps = self.config['fps']
        self.robot_radius = self.config['robot_radius']
        self.trail_thickness = self.config['trail_thickness']

        # Initialize PyGame
        pygame.init()
        self.screen = pygame.display.set_mode((self.window_size, self.window_size))
        pygame.display.set_caption('RF Coverage Mapping Simulator')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        # Generate initial RSSI map
        self.generate_new_map()

        # Initialize robot at random position
        start_x = np.random.randint(10, self.map_size - 10)
        start_y = np.random.randint(10, self.map_size - 10)
        self.robot = Robot(start_x, start_y, self.map_size, mode='auto')

        # Initialize simulation
        self.simulation = Simulation(
            self.robot,
            self.field,
            learning_rate=self.config['learning_rate'],
            gradient_delta=self.config['gradient_delta'],
            noise_std=self.config['noise_std']
        )

        # Running flag
        self.running = True

    def generate_new_map(self):
        """Generate a new random RSSI map"""
        self.field = generate_rssi_map(
            size=self.map_size,
            sigma=self.config['gaussian_sigma'],
            rssi_min=self.config['rssi_min'],
            rssi_max=self.config['rssi_max']
        )

        # Convert to PyGame surface
        self.field_surface = field_to_surface(self.field, self.config['colormap'])

        # Scale to window size
        self.field_surface = pygame.transform.scale(
            self.field_surface,
            (self.window_size, self.window_size)
        )

    def handle_events(self):
        """Handle PyGame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                # Toggle mode
                if event.key == pygame.K_m:
                    mode = self.robot.toggle_mode()
                    print(f"Switched to {mode.upper()} mode")

                # Regenerate map
                elif event.key == pygame.K_r:
                    self.generate_new_map()
                    self.simulation.update_field(self.field)
                    self.robot.reset_path()
                    print("Generated new RSSI map")

                # Export path to CSV
                elif event.key == pygame.K_e:
                    export_path_to_csv(self.robot)

                # Quit
                elif event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    self.running = False

    def update(self):
        """Update simulation state"""
        # Get keyboard state for manual control
        keys = pygame.key.get_pressed()

        # Update simulation
        self.simulation.update(keys)

    def draw_trail(self):
        """Draw robot path trail"""
        if len(self.robot.path) < 2:
            return

        # Convert path to screen coordinates
        screen_path = [
            map_to_screen(x, y, self.map_size, self.window_size)
            for x, y in self.robot.path
        ]

        # Check if we should draw lawnmower path as dotted
        if self.simulation.phase in ['descent', 'manual'] and hasattr(self.simulation, 'lawnmower_path_end_index'):
            # Draw lawnmower portion (dotted)
            lawnmower_end = self.simulation.lawnmower_path_end_index
            if lawnmower_end > 1:
                lawnmower_screen_path = screen_path[:lawnmower_end]
                # Draw dotted line
                for i in range(0, len(lawnmower_screen_path) - 1, 4):
                    if i + 1 < len(lawnmower_screen_path):
                        pygame.draw.line(
                            self.screen,
                            (150, 150, 150),  # Gray
                            lawnmower_screen_path[i],
                            lawnmower_screen_path[min(i + 2, len(lawnmower_screen_path) - 1)],
                            1
                        )

            # Draw descent portion (solid red)
            if len(screen_path) > lawnmower_end:
                descent_path = screen_path[lawnmower_end:]
                if len(descent_path) >= 2:  # Need at least 2 points
                    pygame.draw.lines(
                        self.screen,
                        (255, 50, 50),  # Red
                        False,
                        descent_path,
                        self.trail_thickness
                    )
        else:
            # Draw regular solid trail during lawnmower phase
            pygame.draw.lines(
                self.screen,
                (255, 50, 50),  # Red
                False,
                screen_path,
                self.trail_thickness
            )

    def draw_robot(self):
        """Draw robot on screen"""
        # Convert position to screen coordinates
        screen_x, screen_y = map_to_screen(
            self.robot.x,
            self.robot.y,
            self.map_size,
            self.window_size
        )

        # Draw robot circle
        pygame.draw.circle(
            self.screen,
            (255, 255, 255),  # White
            (screen_x, screen_y),
            self.robot_radius
        )

        # Draw black outline
        pygame.draw.circle(
            self.screen,
            (0, 0, 0),
            (screen_x, screen_y),
            self.robot_radius,
            2
        )

    def draw_obstacles(self):
        """Draw obstacles on screen"""
        for obstacle in self.simulation.obstacle_map.obstacles:
            x = obstacle['x']
            y = obstacle['y']
            width = obstacle['width']
            height = obstacle['height']

            # Convert to screen coordinates
            screen_x, screen_y = map_to_screen(x, y, self.map_size, self.window_size)
            screen_width = int(width * self.window_size / self.map_size)
            screen_height = int(height * self.window_size / self.map_size)

            # Draw obstacle as dark rectangle
            pygame.draw.rect(
                self.screen,
                (40, 40, 40),  # Dark gray
                (screen_x, screen_y, screen_width, screen_height)
            )
            # Draw border
            pygame.draw.rect(
                self.screen,
                (0, 0, 0),  # Black
                (screen_x, screen_y, screen_width, screen_height),
                2
            )

    def draw_planned_path(self):
        """Draw A* planned path if available"""
        if not self.simulation.planned_path or len(self.simulation.planned_path) < 2:
            return

        # Convert to screen coordinates
        screen_path = [
            map_to_screen(x, y, self.map_size, self.window_size)
            for x, y in self.simulation.planned_path
        ]

        # Draw planned path as dashed green line
        for i in range(0, len(screen_path) - 1, 2):
            pygame.draw.line(
                self.screen,
                (0, 255, 0),  # Green
                screen_path[i],
                screen_path[min(i + 1, len(screen_path) - 1)],
                2
            )

    def render(self):
        """Render everything to screen"""
        # Draw background based on phase
        if self.simulation.phase in ['lawnmower', 'measuring']:
            # Blank/white background during lawnmower and measuring
            self.screen.fill((240, 240, 240))
        else:
            # Show heatmap after measuring phase
            self.screen.blit(self.field_surface, (0, 0))

        # Draw obstacles (always visible)
        self.draw_obstacles()

        # Draw planned A* path (if in descent phase)
        if self.simulation.phase == 'descent':
            self.draw_planned_path()

        # Draw trail
        self.draw_trail()

        # Draw robot
        self.draw_robot()

        # Draw info panel
        draw_info_panel(self.screen, self.robot, self.font, self.simulation)

        # Update display
        pygame.display.flip()

    def run(self):
        """Main game loop"""
        print("=== RF Coverage Mapping Simulator ===")
        print("Controls:")
        print("  M - Toggle between Auto/Manual mode")
        print("  R - Regenerate RSSI map")
        print("  E - Export path to CSV")
        print("  Arrow Keys / WASD - Manual control")
        print("  Shift - Speed boost (manual mode)")
        print("  ESC / Q - Quit")
        print()
        print(f"Starting in {self.robot.mode.upper()} mode...")

        while self.running:
            # Handle events
            self.handle_events()

            # Update simulation
            self.update()

            # Render
            self.render()

            # Control frame rate
            self.clock.tick(self.fps)

        # Cleanup
        pygame.quit()
        print("Simulator closed.")


def main():
    """Entry point"""
    try:
        simulator = RFCoverageSimulator('config.json')
        simulator.run()
    except FileNotFoundError:
        print("Error: config.json not found!")
        print("Make sure you're running from the wifi_simulator directory.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
