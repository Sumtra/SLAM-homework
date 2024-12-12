from dataclasses import dataclass, field
from typing import List
from .matrices import Mat_K
from .photo import Photo

@dataclass
class Camera:
    """相机参数"""
    intrinsic: Mat_K
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    photos: List[Photo] = field(default_factory=list)
    
    def __post_init__(self):
        if self.photos is None:
            self.photos = [] 