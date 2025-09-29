"""5 Sep CBPattanaik"""


import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Dict, Tuple


class Point:
    def __init__(self, x: float, y: float):
        self.x = round(x, 2)
        self.y = round(y, 2)
    
    def __repr__(self):
        return f"Point: ({self.x}, {self.y})"
    
Path = List[Point]

class Obstacle:
    def __init__(self, center: Point, radius: float, color='grey', alpha=0.5):
        self.center = center
        self.radius = radius
        self.color = color
        self.alpha = alpha

class Environment:
    def __init__(self, start: Point, target: Point, obstacles: List[Obstacle]):
        self.start = start
        self.target = target
        self.obstacles = obstacles


