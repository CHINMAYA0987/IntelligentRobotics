"""5 Sep CBPattanaik"""

from type_decls import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

class Bug:

    TOWARDS_GOAL = "TOWARDS_GOAL"
    BOUNDARY_FOLLOWING = "BOUNDARY_FOLLOWING"
    FINISHED = "FINISHED"

    def __init__(self, location: Point):
        self.location = location
        self.state = Bug.TOWARDS_GOAL
        self.path = [self.location]
        self.cost = 0.0
        self.heading = 0

    def is_goal_reached(self, target:Point)->bool:
        return math.sqrt((self.location.x - target.x) ** 2 + (self.location.y - target.y) ** 2) < 0.1
    
    def move_towards_target(self, target:Point):
        direction = math.atan2(target.y - self.location.y, target.x - self.location.x)
        self.location = Point(
            self.location.x + 0.1 * math.cos(direction),
            self.location.y + 0.1 * math.sin(direction)
        )
        self.path.append(self.location)
    
    def is_obstructed(self, env: Environment):
        for obs in env.obstacles:
            cx, cy, r = obs.center[0], obs.center[1], obs.radius
            dx = self.location.x - cx
            dy = self.location.y - cy
            if dx*dx + dy*dy <= r*r:
                return True
        return False

    def follow_boundary(self, env: Environment):
        step_size = 0.08
        found_obstacle = False

        for obs in env.obstacles:
            cx, cy, r = obs.center[0], obs.center[1], obs.radius
            dx = self.location.x - cx
            dy = self.location.y - cy

            if dx*dx + dy*dy <= r*r + 1e-6:  
                found_obstacle = True
                angle = math.atan2(dy, dx)

                for _ in range(int(math.pi / step_size)):  
                    angle += step_size 
                    # new_x = cx + (r + 0.1) * math.cos(angle)  
                    # new_y = cy + (r + 0.1) * math.sin(angle)
                    tx, ty = -dy, dx
                    norm = math.sqrt(tx*tx + ty*ty)
                    tx, ty = tx / norm, ty / norm

                    new_x = self.location.x + step_size * tx
                    new_y = self.location.y + step_size * ty

                    self.location = Point(new_x, new_y)
                    self.path.append(self.location)

                    if not self.is_obstructed(env):
                        self.state = Bug.TOWARDS_GOAL
                        return
                break

        if not found_obstacle:
            self.state = Bug.TOWARDS_GOAL
            
    
    def compute_path_length(self)->float:
        path_length = 0.0
        for i in range(1, len(self.path)):
            if self.path[i] is not None and self.path[i-1] is not None:
                dx = self.path[i].x - self.path[i-1].x
                dy = self.path[i].y - self.path[i-1].y
                path_length += math.sqrt(dx*dx + dy*dy)
        return path_length
    

    def navigation(self, env:Environment):
        print("Bug Navigation")
        plt.ion()  
        fig, ax = plt.subplots()
        ax.scatter(env.start.x, env.start.y, color='green', label='Start', zorder=5)
        ax.scatter(env.target.x, env.target.y, color='red', label='Stop', zorder=5)

        for obs in env.obstacles:
            circle = plt.Circle((obs.center[0], obs.center[1]), obs.radius,
                                color=obs.color, alpha=obs.alpha, zorder=2)
            ax.add_patch(circle)
            
        active_bug = self
        start_time = time.time()
        while active_bug:
            if bug.is_goal_reached(env.target):
                    bug.state = Bug.FINISHED
                    break
            
            bug.move_towards_target(env.target)

            if bug.is_obstructed(env):
                bug.state = Bug.BOUNDARY_FOLLOWING
                bug.follow_boundary(env)
                

            ax.plot([p.x for p in bug.path if p is not None], 
                        [p.y for p in bug.path if p is not None], 
                        color='blue')
            
            # plt.draw()
            # plt.pause(0.01)

        # to be included only when simulation is off
        end_time = time.time()
        comp_time = end_time - start_time
        path_length = bug.compute_path_length()
        print(f"Time: {comp_time:.2f}, Length: {path_length:.2f}")

        ax.set_xlim(0, 30)
        ax.set_ylim(0, 30)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Bug Navigation')
        ax.grid(False)
        ax.legend()

        plt.ioff()
        plt.show()



start = Point(1, 1)
target = Point(20, 20)
obstacles = [Obstacle((4.5, 3.0), 2), Obstacle((3.0, 12.0), 2), Obstacle((15.0, 15.0), 2)]

env = Environment(start, target, obstacles)
bug = Bug(start)
bug.navigation(env)