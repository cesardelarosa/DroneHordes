import pygame
import numpy as np
import itertools
import random
import config

class Behavior:
    def update(self, drones, *args):
        raise NotImplementedError("You must implement the 'update' method")

    def handle_input(self, key):
        pass

    def get_controls_status(self):
        return "(N/A)"

    def get_params_status(self):
        return "(N/A)"

class GasIdealBehavior(Behavior):
    def update(self, drones):
        pass

class FollowMouseBehavior(Behavior):
    def update(self, drones):
        mouse_pos = np.array(pygame.mouse.get_pos(), dtype=np.float64)

        for drone in drones:
            drone.velocity *= config.FRICTION

            direction_vec = mouse_pos - drone.position
            dist = np.linalg.norm(direction_vec)
            
            if dist > config.DRONE_RADIUS:
                direction_vec /= dist
                acceleration = direction_vec * config.MOUSE_ATTRACTION_FORCE
                drone.velocity += acceleration

            speed = np.linalg.norm(drone.velocity)
            if speed > config.MOUSE_MAX_SPEED:
                drone.velocity = (drone.velocity / speed) * config.MOUSE_MAX_SPEED
        
class BoidsBehavior(Behavior):
    def __init__(self):
        self.separation_weight = config.BOIDS_SEPARATION_WEIGHT
        self.alignment_weight = config.BOIDS_ALIGNMENT_WEIGHT
        self.cohesion_weight = config.BOIDS_COHESION_WEIGHT
        self.perception_radius = config.BOIDS_PERCEPTION_RADIUS

        self.param_order = ['S', 'A', 'C', 'R']
        self.selected_index = 0
        
        self.param_steps = {
            'S': 0.1,
            'A': 0.1,
            'C': 0.02,
            'R': 5
        }

    def modify_param(self, direction):
        param_name = self.param_order[self.selected_index]
        step = self.param_steps[param_name]
        
        if param_name == 'S':
            self.separation_weight = max(0, self.separation_weight + step * direction)
        elif param_name == 'A':
            self.alignment_weight = max(0, self.alignment_weight + step * direction)
        elif param_name == 'C':
            self.cohesion_weight = max(0, self.cohesion_weight + step * direction)
        elif param_name == 'R':
            self.perception_radius = max(1, self.perception_radius + step * direction)

    def handle_input(self, key):
        if key == pygame.K_s:
            self.selected_index = 0
        elif key == pygame.K_a:
            self.selected_index = 1
        elif key == pygame.K_c:
            self.selected_index = 2
        elif key == pygame.K_r:
            self.selected_index = 3
        
        elif key == pygame.K_LEFT:
            self.selected_index = (self.selected_index - 1) % len(self.param_order)
        elif key == pygame.K_RIGHT:
            self.selected_index = (self.selected_index + 1) % len(self.param_order)
        
        elif key == pygame.K_UP:
            self.modify_param(1)
        elif key == pygame.K_DOWN:
            self.modify_param(-1)

    def get_controls_status(self):
        return "CONTROLS: (S,A,C,R) Select | (←/→) Cycle | (↑/↓) Adjust"

    def get_params_status(self):
        params_str = []
        for i, param in enumerate(self.param_order):
            val_str = ""
            if param == 'S':
                val_str = f"S:{self.separation_weight:.1f}"
            elif param == 'A':
                val_str = f"A:{self.alignment_weight:.1f}"
            elif param == 'C':
                val_str = f"C:{self.cohesion_weight:.2f}"
            elif param == 'R':
                val_str = f"R:{self.perception_radius}"
            
            if i == self.selected_index:
                params_str.append(f"[{val_str}]")
            else:
                params_str.append(f" {val_str} ")
        
        return " | ".join(params_str)

    def update(self, drones):
        
        for drone in drones:
            separation_vec = np.zeros(2, dtype=np.float64)
            alignment_vec = np.zeros(2, dtype=np.float64)
            cohesion_vec = np.zeros(2, dtype=np.float64)
            perception_neighbors = 0
            separation_neighbors = 0

            for other in drones:
                if drone == other:
                    continue
                
                dist_vec = other.position - drone.position
                dist_mag = np.linalg.norm(dist_vec)

                if dist_mag == 0:
                    continue
                
                if dist_mag < self.perception_radius:
                    perception_neighbors += 1
                    alignment_vec += other.velocity
                    cohesion_vec += other.position

                    if dist_mag < config.BOIDS_SEPARATION_RADIUS:
                        separation_neighbors += 1
                        separation_vec -= (dist_vec / dist_mag) / dist_mag

            acceleration = np.zeros(2, dtype=np.float64)
            
            if separation_neighbors > 0:
                separation_vec /= separation_neighbors
                acceleration += self.steer(separation_vec, drone.velocity) * self.separation_weight
            
            if perception_neighbors > 0:
                alignment_vec /= perception_neighbors
                acceleration += self.steer(alignment_vec, drone.velocity) * self.alignment_weight
                
                cohesion_vec /= perception_neighbors
                cohesion_force = self.steer(cohesion_vec - drone.position, drone.velocity)
                acceleration += cohesion_force * self.cohesion_weight

            acceleration += self.avoid_walls(drone) * config.BOIDS_WALL_TURN_STRENGTH
            
            drone.velocity += acceleration
            
            speed = np.linalg.norm(drone.velocity)
            if speed > config.BOIDS_MAX_SPEED:
                drone.velocity = (drone.velocity / speed) * config.BOIDS_MAX_SPEED
            elif speed < config.BOIDS_MIN_SPEED:
                drone.velocity = (drone.velocity / speed) * config.BOIDS_MIN_SPEED

    def steer(self, desired_direction, current_velocity):
        desired_norm = np.linalg.norm(desired_direction)
        if desired_norm > 0:
            desired_direction = (desired_direction / desired_norm) * config.BOIDS_MAX_SPEED
        
        steer_force = desired_direction - current_velocity
        return steer_force
    
    def avoid_walls(self, drone):
        steer = np.zeros(2, dtype=np.float64)
        margin = config.BOIDS_WALL_MARGIN
        
        if drone.position[0] < margin:
            steer[0] = 1.0
        elif drone.position[0] > config.SCREEN_WIDTH - margin:
            steer[0] = -1.0
            
        if drone.position[1] < margin:
            steer[1] = 1.0
        elif drone.position[1] > config.SCREEN_HEIGHT - margin:
            steer[1] = -1.0
        
        return steer

class DirectedFlockingBehavior(BoidsBehavior):
    def __init__(self):
        super().__init__()
        self.target_weight = config.BOIDS_TARGET_WEIGHT
        
        self.param_order.append('T')
        self.param_steps['T'] = 0.1

    def handle_input(self, key):
        super().handle_input(key)
        if key == pygame.K_t:
            self.selected_index = self.param_order.index('T')

    def get_controls_status(self):
        return "CONTROLS: (S,A,C,R,T) Select | (←/→) Cycle | (↑/↓) Adjust"

    def get_params_status(self):
        params_str = []
        for i, param in enumerate(self.param_order):
            val_str = ""
            if param == 'S':
                val_str = f"S:{self.separation_weight:.1f}"
            elif param == 'A':
                val_str = f"A:{self.alignment_weight:.1f}"
            elif param == 'C':
                val_str = f"C:{self.cohesion_weight:.2f}"
            elif param == 'R':
                val_str = f"R:{self.perception_radius}"
            elif param == 'T':
                val_str = f"T:{self.target_weight:.1f}"
            
            if i == self.selected_index:
                params_str.append(f"[{val_str}]")
            else:
                params_str.append(f" {val_str} ")
        
        return " | ".join(params_str)

    def update(self, drones):
        mouse_pos = np.array(pygame.mouse.get_pos(), dtype=np.float64)
        
        for drone in drones:
            separation_vec = np.zeros(2, dtype=np.float64)
            alignment_vec = np.zeros(2, dtype=np.float64)
            cohesion_vec = np.zeros(2, dtype=np.float64)
            perception_neighbors = 0
            separation_neighbors = 0

            for other in drones:
                if drone == other:
                    continue
                
                dist_vec = other.position - drone.position
                dist_mag = np.linalg.norm(dist_vec)

                if dist_mag == 0:
                    continue
                
                if dist_mag < self.perception_radius:
                    perception_neighbors += 1
                    alignment_vec += other.velocity
                    cohesion_vec += other.position

                    if dist_mag < config.BOIDS_SEPARATION_RADIUS:
                        separation_neighbors += 1
                        separation_vec -= (dist_vec / dist_mag) / dist_mag

            acceleration = np.zeros(2, dtype=np.float64)
            
            if separation_neighbors > 0:
                separation_vec /= separation_neighbors
                acceleration += self.steer(separation_vec, drone.velocity) * self.separation_weight
            
            if perception_neighbors > 0:
                alignment_vec /= perception_neighbors
                acceleration += self.steer(alignment_vec, drone.velocity) * self.alignment_weight
                
                cohesion_vec /= perception_neighbors
                cohesion_force = self.steer(cohesion_vec - drone.position, drone.velocity)
                acceleration += cohesion_force * self.cohesion_weight

            acceleration += self.avoid_walls(drone) * config.BOIDS_WALL_TURN_STRENGTH
            
            target_force = self.steer(mouse_pos - drone.position, drone.velocity)
            acceleration += target_force * self.target_weight
            
            drone.velocity += acceleration
            
            speed = np.linalg.norm(drone.velocity)
            if speed > config.BOIDS_MAX_SPEED:
                drone.velocity = (drone.velocity / speed) * config.BOIDS_MAX_SPEED
            elif speed < config.BOIDS_MIN_SPEED:
                drone.velocity = (drone.velocity / speed) * config.BOIDS_MIN_SPEED
