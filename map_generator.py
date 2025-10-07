

import numpy as np
from scipy.ndimage import gaussian_filter


class RSSIMapGenerator:

    def __init__(self, size=100, sigma=8.0, rssi_min=-90, rssi_max=-30):
        
        self.size = size
        self.sigma = sigma
        self.rssi_min = rssi_min
        self.rssi_max = rssi_max

    def generate(self):
        
        # Generate random noise
        random_field = np.random.rand(self.size, self.size)

        # Apply Gaussian smoothing to create realistic signal propagation
        smoothed = gaussian_filter(random_field, sigma=self.sigma)

        # Normalize to RSSI range (dBm)
        field_normalized = self._normalize(smoothed)

        return field_normalized

    def _normalize(self, field):
       
        field_min = field.min()
        field_max = field.max()

        # Scale to [0, 1]
        normalized = (field - field_min) / (field_max - field_min)

        # Scale to [rssi_min, rssi_max]
        scaled = normalized * (self.rssi_max - self.rssi_min) + self.rssi_min

        return scaled


def generate_rssi_map(size=100, sigma=8.0, rssi_min=-90, rssi_max=-30):
   
    generator = RSSIMapGenerator(size, sigma, rssi_min, rssi_max)
    return generator.generate()
