from dataclasses import dataclass, field
from typing import List, Optional
from .vectors import Vec_t

@dataclass
class Color:
    r: int
    g: int
    b: int

@dataclass
class Obser:
    """观测点"""
    photo_id: int
    x: float
    y: float

@dataclass
class TiePnt:
    """特征点"""
    position: Vec_t
    color: Optional[Color] = None
    observations: List[Obser] = field(default_factory=list)
    
    def __post_init__(self):
        if self.observations is None:
            self.observations = [] 