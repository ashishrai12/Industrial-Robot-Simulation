import matplotlib.pyplot as plt
import numpy as np
from .models import Robot, Environment

def run_simulation():
    # Setup environment and robot
    # Base at (1.0, -0.5) to reach up to conveyor at 0.5
    env = Environment(conveyor_y=0.5)
    robot = Robot(base_pos=np.array([1.0, -0.2]), l1=0.8, l2=0.7)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.ion()

    print("Starting Industrial Robot Simulation (Kinematics-enabled)...")
    
    try:
        while plt.fignum_exists(fig.number):
            env.update()
            robot.update(env)

            ax.clear()
            ax.set_xlim(-0.5, 2.5)
            ax.set_ylim(-0.5, 2.0)
            ax.set_aspect('equal')

            # Draw conveyor
            ax.plot([env.conveyor_start, env.conveyor_end], [env.conveyor_y, env.conveyor_y], 'k-', lw=6, label='Conveyor', alpha=0.3)
            
            # Pickup zone
            ax.axvspan(env.pickup_x_range[0], env.pickup_x_range[1], color='orange', alpha=0.1, label='Pickup Zone')

            # Drop zone
            circle = plt.Circle(env.drop_zone, 0.15, color='green', alpha=0.2, label='Drop Zone')
            ax.add_patch(circle)

            # Draw objects
            for obj in env.objects:
                ax.plot(obj.pos[0], obj.pos[1], 's', color=obj.color, markersize=12, markeredgecolor='black')
            
            # Robot Arm Visualization
            p0, p1, p2 = robot.get_absolute_positions()
            
            # Links
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'o-', color='gray', lw=8, markersize=10) # Link 1
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'o-', color='blue', lw=6, markersize=8) # Link 2
            
            # Held object
            if robot.holding_object:
                ax.plot(p2[0], p2[1], 's', color=robot.holding_object.color, markersize=12, markeredgecolor='black')

            # Status
            status_text = (f"State: {robot.state}\n"
                           f"Success: {robot.success_count}\n"
                           f"Failure: {robot.failure_count}\n")
            
            total = robot.success_count + robot.failure_count
            if total > 0:
                status_text += f"Rate: {(robot.success_count / total)*100:.1f}%"

            ax.text(0.02, 0.98, status_text, transform=ax.transAxes, verticalalignment='top', 
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

            plt.title("Advanced Industrial Robot Simulation")
            plt.draw()
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        plt.close()
        print("Simulation stopped.")

if __name__ == "__main__":
    run_simulation()
