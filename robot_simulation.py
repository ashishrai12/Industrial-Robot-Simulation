import numpy as np
import matplotlib.pyplot as plt
import random

class Robot:
    def __init__(self, start_pos):
        self.pos = np.array(start_pos, dtype=float)
        self.holding_object = None
        self.state = 'IDLE' # IDLE, MOVING_TO_PICK, PICKING, MOVING_TO_PLACE, PLACING, RETURNING, ERROR
        self.target_pos = None
        self.target_object = None
        self.speed = 0.1
        self.success_count = 0
        self.failure_count = 0
        self.arm_failure_prob = 0.005 # 0.5% chance of mechanical failure per frame
        self.grasp_failure_prob = 0.1 # 10% chance of missing grasp

    def move_towards(self, target):
        direction = target - self.pos
        dist = np.linalg.norm(direction)
        if dist < self.speed:
            self.pos = target.copy()
            return True # Reached
        else:
            self.pos += (direction / dist) * self.speed
            return False

    def update(self, environment):
        if self.state == 'ERROR':
            # Simulate recovery time or manual reset
            if random.random() < 0.02: # 2% chance to recover per frame
                self.state = 'IDLE'
            return

        # Random mechanical failure simulation
        if random.random() < self.arm_failure_prob:
            self.state = 'ERROR'
            self.failure_count += 1
            if self.holding_object:
                # Drop object at current position
                self.holding_object.pos = self.pos.copy()
                # If it drops on conveyor, maybe it stays, but for simplicity let's say it's lost or becomes debris
                # We won't add it back to environment objects list to simulate "broken/lost" product
                self.holding_object = None
            return

        if self.state == 'IDLE':
            # Look for objects in pickup zone
            target_obj = environment.get_pickable_object()
            if target_obj:
                self.target_pos = target_obj.pos
                self.target_object = target_obj
                self.state = 'MOVING_TO_PICK'
        
        elif self.state == 'MOVING_TO_PICK':
            # Update target pos as object moves
            if self.target_object in environment.objects:
                 self.target_pos = self.target_object.pos
            else:
                # Object gone (maybe fell off or moved past)
                self.state = 'IDLE'
                self.target_object = None
                return

            if self.move_towards(self.target_pos):
                self.state = 'PICKING'

        elif self.state == 'PICKING':
            # Try to grasp
            if random.random() < self.grasp_failure_prob:
                # Failed to grasp
                self.state = 'IDLE' # Try again or wait
                self.target_object = None
                # Could increment failure count here if desired, but maybe just a "miss"
            else:
                self.holding_object = self.target_object
                environment.remove_object(self.target_object)
                self.state = 'MOVING_TO_PLACE'
                self.target_pos = environment.drop_zone

        elif self.state == 'MOVING_TO_PLACE':
            if self.move_towards(self.target_pos):
                self.state = 'PLACING'
            
            # Object moves with robot
            if self.holding_object:
                self.holding_object.pos = self.pos.copy()

        elif self.state == 'PLACING':
            self.holding_object = None
            self.success_count += 1
            self.state = 'RETURNING'
            self.target_pos = environment.home_pos

        elif self.state == 'RETURNING':
            if self.move_towards(self.target_pos):
                self.state = 'IDLE'

class Object:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.color = np.random.rand(3,)

class Environment:
    def __init__(self):
        self.conveyor_y = 0.5
        self.conveyor_start = 0.0
        self.conveyor_end = 2.0
        self.conveyor_speed = 0.03
        self.objects = []
        self.drop_zone = np.array([1.5, 1.5])
        self.home_pos = np.array([1.0, 0.0])
        self.pickup_x_range = (0.8, 1.2) # Zone where robot can pick

    def update(self):
        # Move objects
        for obj in self.objects:
            obj.pos[0] += self.conveyor_speed
        
        # Remove objects that fall off end
        self.objects = [obj for obj in self.objects if obj.pos[0] < self.conveyor_end]

        # Spawn new objects
        if random.random() < 0.03: # 3% chance per frame
            # Add some noise to y position (misplacement)
            y_noise = np.random.normal(0, 0.05)
            self.objects.append(Object([self.conveyor_start, self.conveyor_y + y_noise]))

    def get_pickable_object(self):
        # Find object closest to center of pickup zone
        candidates = []
        for obj in self.objects:
            if self.pickup_x_range[0] <= obj.pos[0] <= self.pickup_x_range[1]:
                candidates.append(obj)
        
        if candidates:
            # Return the one furthest along (closest to leaving zone)
            return max(candidates, key=lambda o: o.pos[0])
        return None

    def remove_object(self, obj):
        if obj in self.objects:
            self.objects.remove(obj)

def run_simulation():
    env = Environment()
    robot = Robot(env.home_pos)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.ion() # Interactive mode

    print("Starting simulation... Close the window to stop.")
    
    try:
        while plt.fignum_exists(fig.number):
            env.update()
            robot.update(env)

            ax.clear()
            ax.set_xlim(-0.2, 2.5)
            ax.set_ylim(-0.5, 2.0)
            ax.set_aspect('equal')

            # Draw conveyor
            ax.plot([env.conveyor_start, env.conveyor_end], [env.conveyor_y, env.conveyor_y], 'k-', lw=4, label='Conveyor', alpha=0.5)
            # Draw pickup zone markers
            ax.axvline(x=env.pickup_x_range[0], color='orange', linestyle='--', alpha=0.5)
            ax.axvline(x=env.pickup_x_range[1], color='orange', linestyle='--', alpha=0.5)
            ax.text(np.mean(env.pickup_x_range), env.conveyor_y - 0.2, "Pickup Zone", ha='center', color='orange')

            # Draw drop zone
            circle = plt.Circle(env.drop_zone, 0.15, color='green', alpha=0.3, label='Drop Zone')
            ax.add_patch(circle)
            ax.text(env.drop_zone[0], env.drop_zone[1] + 0.2, "Drop Zone", ha='center', color='green')

            # Draw objects on conveyor
            for obj in env.objects:
                ax.plot(obj.pos[0], obj.pos[1], 's', color=obj.color, markersize=10, markeredgecolor='black')
            
            # Draw robot held object
            if robot.holding_object:
                ax.plot(robot.pos[0], robot.pos[1], 's', color=robot.holding_object.color, markersize=10, markeredgecolor='black')

            # Draw robot arm
            # Base
            ax.plot(env.home_pos[0], env.home_pos[1], 'ko', markersize=10)
            # Arm link
            ax.plot([env.home_pos[0], robot.pos[0]], [env.home_pos[1], robot.pos[1]], 'b-', lw=3, alpha=0.7) 
            # End Effector
            effector_color = 'red' if robot.state == 'ERROR' else 'blue'
            ax.plot(robot.pos[0], robot.pos[1], 'o', color=effector_color, markersize=12, label='Robot End Effector')
            
            # Status text
            status_text = (f"State: {robot.state}\n"
                           f"Successes: {robot.success_count}\n"
                           f"Failures: {robot.failure_count}")
            
            # Calculate success rate
            total_attempts = robot.success_count + robot.failure_count
            if total_attempts > 0:
                rate = (robot.success_count / total_attempts) * 100
                status_text += f"\nSuccess Rate: {rate:.1f}%"

            ax.text(0.02, 0.98, status_text, transform=ax.transAxes, verticalalignment='top', 
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

            plt.title("Industrial Robot Simulation")
            plt.draw()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        plt.close()
        print("Simulation stopped.")

if __name__ == "__main__":
    run_simulation()
