from dataclasses import dataclass
from .vectors import Vec_t
from .matrices import Mat_R

@dataclass
class Photo:
    """照片信息"""
    label: str
    position: Vec_t
    rotation: Mat_R
    euler_angles: Vec_t
    enabled: bool = True 