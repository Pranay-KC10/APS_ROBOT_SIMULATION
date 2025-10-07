

import numpy as np
import pygame
from matplotlib import cm
import csv


def field_to_surface(field, colormap='plasma'):
    
    # Normalize field to [0, 1]
    field_min = field.min()
    field_max = field.max()
    field_norm = (field - field_min) / (field_max - field_min + 1e-10)

    # Apply colormap
    cmap = cm.get_cmap(colormap)
    heat_rgba = cmap(field_norm)

    # Convert to RGB (0-255)
    heat_rgb = (heat_rgba[:, :, :3] * 255).astype(np.uint8)

    # Create surface (transpose because pygame uses (width, height))
    heat_surface = pygame.surfarray.make_surface(np.transpose(heat_rgb, (1, 0, 2)))

    return heat_surface


def map_to_screen(x, y, map_size, window_size):
    
    scale = window_size / map_size
    screen_x = int(x * scale)
    screen_y = int(y * scale)
    return screen_x, screen_y


def screen_to_map(screen_x, screen_y, map_size, window_size):
    
    scale = map_size / window_size
    map_x = screen_x * scale
    map_y = screen_y * scale
    return map_x, map_y


def export_path_to_csv(robot, filename='path.csv'):
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Header
        writer.writerow(['iteration', 'x', 'y', 'rssi'])

        # Data
        for i, (x, y) in enumerate(robot.path):
            # Note: RSSI is only tracked for current position
            # For historical points, we'd need to store it separately
            writer.writerow([i, f'{x:.2f}', f'{y:.2f}', ''])

    print(f"Path exported to {filename}")


def load_config(config_path='config.json'):
    
    import json

    with open(config_path, 'r') as f:
        config = json.load(f)

    return config


def render_text(surface, text, position, font, color=(255, 255, 255), bg_color=None):
    
    text_surface = font.render(text, True, color)

    if bg_color:
        # Create background rectangle
        text_rect = text_surface.get_rect(topleft=position)
        bg_rect = text_rect.inflate(8, 4)
        pygame.draw.rect(surface, bg_color, bg_rect)
        pygame.draw.rect(surface, color, bg_rect, 1)

    surface.blit(text_surface, position)


def draw_info_panel(surface, robot, font, simulation=None, x=10, y=10, line_height=25):

    state = robot.get_state()

    # Get phase description if simulation provided
    phase_desc = simulation.get_phase_description() if simulation else state['mode'].upper()

    # Create semi-transparent background
    panel_height = 7 * line_height + 10
    panel_surface = pygame.Surface((450, panel_height))
    panel_surface.set_alpha(180)
    panel_surface.fill((0, 0, 0))
    surface.blit(panel_surface, (x - 5, y - 5))

    # Render text lines
    lines = [
        f"Phase: {phase_desc}",
        f"Position: ({state['x']:.1f}, {state['y']:.1f})",
        f"RSSI: {state['rssi']:.1f} dBm",
        f"Iteration: {state['iteration']}",
        f"Path Length: {state['path_length']}",
        "",
        "Controls: M=Manual | R=Reset | E=Export CSV | ESC=Quit"
    ]

    for i, line in enumerate(lines):
        render_text(surface, line, (x, y + i * line_height), font)
