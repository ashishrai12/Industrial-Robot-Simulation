import numpy as np
import random
from typing import List, Optional, Tuple
from .kinematics import PlanarKinematics2D

class SimulationObject:
    def __init__(self, pos: np.ndarray):
        self.pos = np.array(pos, dtype=float)
        self.color = np.random.rand(3,)

class Robot:
    def __init__(self, base_pos: np.ndarray, l1: float = 1.0, l2: float = 0.8):
        self.base_pos = np.array(base_pos, dtype=float)
        self.kinematics = PlanarKinematics2D(l1, l2)
        
        # Current joint angles
        self.theta1 = 0.0
        self.theta2 = 0.0
        
        # End effector position (relative to base)
        self.pos = self.kinematics.forward(self.theta1, self.theta2)
        
        self.holding_object: Optional[SimulationObject] = None
        self.state = 'IDLE' # IDLE, MOVING_TO_PICK, PICKING, MOVING_TO_PLACE, PLACING, RETURNING, ERROR
        self.target_pos: Optional[np.ndarray] = None
        self.target_object: Optional[SimulationObject] = None
        self.speed = 0.05 # Angular speed in radians
        self.success_count = 0
        self.failure_count = 0
        self.arm_failure_prob = 0.005 
        self.grasp_failure_prob = 0.1 

    def get_absolute_positions(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        p0, p1, p2 = self.kinematics.get_joint_positions(self.theta1, self.theta2)
        return p0 + self.base_pos, p1 + self.base_pos, p2 + self.base_pos

    def update(self, environment: 'Environment'):
        if self.state == 'ERROR':
            if random.random() < 0.02:
                self.state = 'IDLE'
            return

        if random.random() < self.arm_failure_prob:
            self.state = 'ERROR'
            self.failure_count += 1
            if self.holding_object:
                # Drop object at current EE position
                _, _, ee_pos = self.get_absolute_positions()
                self.holding_object.pos = ee_pos
                self.holding_object = None
            return

        if self.state == 'IDLE':
            target_obj = environment.get_pickable_object()
            if target_obj:
                self.target_object = target_obj
                self.state = 'MOVING_TO_PICK'
        
        elif self.state == 'MOVING_TO_PICK':
            if self.target_object in environment.objects:
                 self.target_pos = self.target_object.pos
            else:
                self.state = 'IDLE'
                self.target_object = None
                return

            if self.move_towards(self.target_pos):
                self.state = 'PICKING'

        elif self.state == 'PICKING':
            if random.random() < self.grasp_failure_prob:
                self.state = 'IDLE'
                self.target_object = None
            else:
                self.holding_object = self.target_object
                environment.remove_object(self.target_object)
                self.state = 'MOVING_TO_PLACE'
                self.target_pos = environment.drop_zone

        elif self.state == 'MOVING_TO_PLACE':
            if self.move_towards(self.target_pos):
                self.state = 'PLACING'
            
            if self.holding_object:
                _, _, ee_pos = self.get_absolute_positions()
                self.holding_object.pos = ee_pos

        elif self.state == 'PLACING':
            self.holding_object = None
            self.success_count += 1
            self.state = 'RETURNING'
            self.target_pos = environment.home_pos

        elif self.state == 'RETURNING':
            if self.move_towards(self.target_pos):
                self.state = 'IDLE'

    def move_towards(self, target_abs: np.ndarray) -> bool:
        """
        Moves joints towards the inverse kinematics solution for target_abs.
        Returns True if reached.
        """
        target_rel = target_abs - self.base_pos
        solutions = self.kinematics.inverse(target_rel[0], target_rel[1])
        
        if solutions is None:
            # Cannot reach target
            self.state = 'ERROR'
            return False
            
        t1_target, t2_target = solutions
        
        # Smoothly move joints
        d1 = t1_target - self.theta1
        d2 = t2_target - self.theta2
        
        # Simple wrap-around handling would be better, but for now:
        dist = np.sqrt(d1**2 + d2**2)
        if dist < self.speed:
            self.theta1 = t1_target
            self.theta2 = t2_target
            return True
        else:
            self.theta1 += (d1 / dist) * self.speed
            self.theta2 += (d2 / dist) * self.speed
            return False

class Environment:
    def __init__(self, conveyor_y: float = 0.5):
        self.conveyor_y = conveyor_y
        self.conveyor_start = -0.5
        self.conveyor_end = 2.5
        self.conveyor_speed = 0.02
        self.objects: List[SimulationObject] = []
        self.drop_zone = np.array([1.8, 1.2])
        self.home_pos = np.array([1.0, 0.5]) # Initial EE target pos
        self.pickup_x_range = (0.8, 1.2)

    def update(self):
        for obj in self.objects:
            obj.pos[0] += self.conveyor_speed
        
        self.objects = [obj for obj in self.objects if obj.pos[0] < self.conveyor_end]

        if random.random() < 0.02:
            y_noise = np.random.normal(0, 0.05)
            self.objects.append(SimulationObject([self.conveyor_start, self.conveyor_y + y_noise]))

    def get_pickable_object(self) -> Optional[SimulationObject]:
        candidates = [obj for obj in self.objects if self.pickup_x_range[0] <= obj.pos[0] <= self.pickup_x_range[1]]
        if candidates:
            return max(candidates, key=lambda o: o.pos[0])
        return None

    def remove_object(self, obj: SimulationObject):
        if obj in self.objects:
            self.objects.remove(obj)
