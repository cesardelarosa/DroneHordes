import pygame
import numpy as np
import itertools
import random
import config

class Behavior:
    def update(self, drones, *args):
        raise NotImplementedError("You must implement the 'update' method")

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
    def update(self, drones):
        new_velocities = []

        for drone in drones:
            neighbors = self.get_neighbors(drone, drones)
            
            if not neighbors:
                new_velocities.append(np.copy(drone.velocity))
                continue

            separation_vec = self.separation(drone, neighbors)
            alignment_vec = self.alignment(drone, neighbors)
            cohesion_vec = self.cohesion(drone, neighbors)
            wander_vec = self.wander()
            
            acceleration = np.zeros(2, dtype=np.float64)
            acceleration += separation_vec * config.BOIDS_SEPARATION_WEIGHT
            acceleration += alignment_vec * config.BOIDS_ALIGNMENT_WEIGHT
            acceleration += cohesion_vec * config.BOIDS_COHESION_WEIGHT
            acceleration += wander_vec * config.BOIDS_WANDER_STRENGTH
            
            new_velocity = drone.velocity + acceleration
            
            speed = np.linalg.norm(new_velocity)
            if speed > config.BOIDS_MAX_SPEED:
                new_velocity = (new_velocity / speed) * config.BOIDS_MAX_SPEED
            elif speed < config.BOIDS_MIN_SPEED:
                new_velocity = (new_velocity / speed) * config.BOIDS_MIN_SPEED
            
            new_velocities.append(new_velocity)

        for i, drone in enumerate(drones):
            drone.velocity = new_velocities[i]

    def get_neighbors(self, drone, all_drones):
        neighbors = []
        for other in all_drones:
            if drone == other:
                continue
            dist = np.linalg.norm(drone.position - other.position)
            if dist < config.BOIDS_PERCEPTION_RADIUS:
                neighbors.append(other)
        return neighbors

    def separation(self, drone, neighbors):
        steer = np.zeros(2, dtype=np.float64)
        for other in neighbors:
            dist_vec = drone.position - other.position
            dist_mag = np.linalg.norm(dist_vec)
            if dist_mag > 0:
                steer += (dist_vec / dist_mag) / dist_mag
        return steer
    
    def alignment(self, drone, neighbors):
        avg_velocity = np.mean([n.velocity for n in neighbors], axis=0)
        steer = avg_velocity - drone.velocity
        return steer

    def cohesion(self, drone, neighbors):
        center_of_mass = np.mean([n.position for n in neighbors], axis=0)
        direction = center_of_mass - drone.position
        
        dist = np.linalg.norm(direction)
        if dist > 0:
            steer = (direction / dist) * config.BOIDS_MAX_SPEED - drone.velocity
            return steer
        
        return np.zeros(2, dtype=np.float64)
    
    def wander(self):
        return np.array([random.uniform(-1, 1), random.uniform(-1, 1)])
