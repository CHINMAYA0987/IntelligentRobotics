"""5 Sep CBPattanaik"""

from type_decls import *
import random
import time
import math
import matplotlib.pyplot as plt
import networkx as nx  

class PRM:
    def __init__(self, location: Point, n_samples=150, k=10):
        self.location = location
        self.path = [self.location]
        self.n_samples = n_samples
        self.k = k
        self.roadmap = nx.Graph()

    def is_in_obstacle(self, point: Point, env: Environment) -> bool:
        for obs in env.obstacles:
            cx, cy, r = obs.center[0], obs.center[1], obs.radius
            dx = point.x - cx
            dy = point.y - cy
            if dx*dx + dy*dy <= r*r:  
                return True
        return False

    def is_edge_valid(self, p1: Point, p2: Point, env: Environment) -> bool:
        for obs in env.obstacles:
            cx, cy, r = obs.center[0], obs.center[1], obs.radius
            dx, dy = p2.x - p1.x, p2.y - p1.y
            fx, fy = p1.x - cx, p1.y - cy

            a = dx*dx + dy*dy
            b = 2 * (fx*dx + fy*dy)
            c = (fx*fx + fy*fy) - r*r

            discriminant = b*b - 4*a*c
            if discriminant >= 0:
                disc = math.sqrt(discriminant)
                t1 = (-b - disc) / (2*a)
                t2 = (-b + disc) / (2*a)
                if (0 <= t1 <= 1) or (0 <= t2 <= 1):
                    return False
        return True

    def build_roadmap(self, env: Environment):
        samples = []

        samples.append(env.start)
        samples.append(env.target)

        while len(samples) < self.n_samples:
            x, y = random.uniform(0, 30), random.uniform(0, 30)
            p = Point(x, y)
            if not self.is_in_obstacle(p, env):
                samples.append(p)

        for i, p in enumerate(samples):
            dists = []
            for j, q in enumerate(samples):
                if i == j:
                    continue
                dist = math.dist((p.x, p.y), (q.x, q.y))
                dists.append((dist, j, q))
            dists.sort()
            for _, j, q in dists[:self.k]:
                if self.is_edge_valid(p, q, env):
                    self.roadmap.add_edge(i, j, weight=math.dist((p.x, p.y), (q.x, q.y)))

        self.samples = samples

    def find_path(self, env: Environment):
        self.build_roadmap(env)
        path_indices = nx.shortest_path(self.roadmap, 0, 1, weight='weight')
        path = [self.samples[i] for i in path_indices]
        self.path = path

    def compute_path_length(self)->float:
        path_length = 0.0
        for i in range(1, len(self.path)):
            if self.path[i] is not None and self.path[i-1] is not None:
                dx = self.path[i].x - self.path[i-1].x
                dy = self.path[i].y - self.path[i-1].y
                path_length += math.sqrt(dx*dx + dy*dy)
        return path_length

    def navigation(self, env: Environment):
        print("PRM Navigation")
        start_time = time.time()
        self.find_path(env)
        

        # plt.ion()
        fig, ax = plt.subplots()
        ax.scatter(env.start.x, env.start.y, color='green', label='Start', zorder=5)
        ax.scatter(env.target.x, env.target.y, color='red', label='Target', zorder=5)

        for obs in env.obstacles:
            circle = plt.Circle(obs.center, obs.radius, color=obs.color, alpha=obs.alpha, zorder=2)
            ax.add_patch(circle)

        xs = [p.x for p in self.samples]
        ys = [p.y for p in self.samples]
        ax.scatter(xs, ys, s=10, color='gray', alpha=0.4)

        for (i, j) in self.roadmap.edges:
            p1, p2 = self.samples[i], self.samples[j]
            ax.plot([p1.x, p2.x], [p1.y, p2.y], color='lightblue', linewidth=0.5)

        path_x = [p.x for p in self.path]
        path_y = [p.y for p in self.path]
        ax.plot(path_x, path_y, color='blue', linewidth=2)

        end_time = time.time()
        comp_time = end_time - start_time
        path_length = self.compute_path_length()
        print(f"Time: {comp_time:.2f}, Length: {path_length:.2f}")

        ax.set_xlim(0, 30)
        ax.set_ylim(0, 30)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("PRM Navigation")
        ax.legend()
        plt.show()
        plt.ioff()
        
       



start = Point(1, 1)
target = Point(20, 20)
obstacles = [Obstacle((4.5, 3.0), 2), Obstacle((3.0, 12.0), 2), Obstacle((15.0, 15.0), 2)]

env = Environment(start, target, obstacles)
dem_prm = PRM(start)
dem_prm.navigation(env)