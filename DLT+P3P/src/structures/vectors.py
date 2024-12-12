import numpy as np
from dataclasses import dataclass

@dataclass
class Vec_t:
    x: float
    y: float
    z: float
    
    def to_array(self):
        return np.array([self.x, self.y, self.z]) 