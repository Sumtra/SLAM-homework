import numpy as np
from dataclasses import dataclass, field
from typing import List

@dataclass
class Mat_K:
    """相机内参矩阵"""
    data: np.ndarray = field(default_factory=lambda: np.eye(3))
    
    def __post_init__(self):
        if isinstance(self.data, list):
            self.data = np.array(self.data)

@dataclass
class Mat_R:
    """旋转矩阵"""
    data: np.ndarray = field(default_factory=lambda: np.eye(3))
    
    def __post_init__(self):
        if isinstance(self.data, list):
            self.data = np.array(self.data) 