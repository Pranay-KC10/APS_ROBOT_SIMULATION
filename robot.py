
import numpy as np


class Robot:

    def __init__(self, x, y, map_size, mode='auto'):
        
        self.x = x
        self.y = y
        self.map_size = map_size
        self.mode = mode

        # Path tracking
        self.path = [(x, y)]
        self.iteration = 0

        # Current RSSI reading
        self.current_rssi = 0.0

    def update_position(self, new_x, new_y):
       
        # Clamp within map boundaries
        self.x = np.clip(new_x, 0, self.map_size - 1)
        self.y = np.clip(new_y, 0, self.map_size - 1)

        # Record path
        self.path.append((self.x, self.y))
        self.iteration += 1

    def get_rssi_at_position(self, field):
        
        # Use integer coordinates for array indexing
        x_idx = int(np.clip(self.x, 0, self.map_size - 1))
        y_idx = int(np.clip(self.y, 0, self.map_size - 1))

        self.current_rssi = field[y_idx, x_idx]
        return self.current_rssi

    def toggle_mode(self):
        self.mode = 'manual' if self.mode == 'auto' else 'auto'
        return self.mode

    def reset_path(self):
        self.path = [(self.x, self.y)]
        self.iteration = 0

    def get_state(self):
       
        return {
            'x': self.x,
            'y': self.y,
            'mode': self.mode,
            'iteration': self.iteration,
            'rssi': self.current_rssi,
            'path_length': len(self.path)
        }
