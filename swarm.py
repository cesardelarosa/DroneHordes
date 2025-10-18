import pygame
import numpy as np
import random
import itertools
import config
from behaviors import GasIdealBehavior, BoidsBehavior

class Drone:
    def __init__(self):
        self.radius = config.DRONE_RADIUS
        self.mass = config.DRONE_MASS
        
        self.position = np.array([
            random.uniform(self.radius, config.SCREEN_WIDTH - self.radius),
            random.uniform(self.radius, config.SCREEN_HEIGHT - self.radius)
        ], dtype=np.float64)
        
        angle = random.uniform(0, 2 * np.pi)
        self.velocity = np.array([np.cos(angle), np.sin(angle)], dtype=np.float64) * config.DRONE_SPEED

    def draw(self, screen):
        pygame.draw.circle(
            screen, 
            config.COLOR_DRONE, 
            self.position.astype(int), 
            self.radius
        )

class Swarm:
    def __init__(self, n):
        self.drones = [Drone() for _ in range(n)]
        self.behavior = GasIdealBehavior()

    def set_behavior(self, behavior_instance):
        self.behavior = behavior_instance

    def reset_drone_velocities(self, speed=None):
        if speed is None:
            speed = config.DRONE_SPEED

        for drone in self.drones:
            if speed == 0:
                drone.velocity = np.zeros(2, dtype=np.float64)
            else:
                angle = random.uniform(0, 2 * np.pi)
                drone.velocity = np.array([np.cos(angle), np.sin(angle)], dtype=np.float64) * speed

    def handle_wall_collisions(self, drone):
        if drone.position[0] <= drone.radius:
            drone.velocity[0] *= -1
            drone.position[0] = drone.radius
        elif drone.position[0] >= config.SCREEN_WIDTH - drone.radius:
            drone.velocity[0] *= -1
            drone.position[0] = config.SCREEN_WIDTH - drone.radius
        
        if drone.position[1] <= drone.radius:
            drone.velocity[1] *= -1
            drone.position[1] = drone.radius
        elif drone.position[1] >= config.SCREEN_HEIGHT - drone.radius:
            drone.velocity[1] *= -1
            drone.position[1] = config.SCREEN_HEIGHT - drone.radius

    def handle_drone_collisions(self):
        for drone_a, drone_b in itertools.combinations(self.drones, 2):
            dist_vec = drone_a.position - drone_b.position
            dist_mag = np.linalg.norm(dist_vec)
            min_dist = drone_a.radius + drone_b.radius

            if dist_mag < min_dist and dist_mag > 0:
                normal = dist_vec / dist_mag
                overlap = (min_dist - dist_mag) * 0.5
                
                drone_a.position += normal * overlap
                drone_b.position -= normal * overlap

    def update(self):
        self.behavior.update(self.drones)
        
        for drone in self.drones:
            drone.position += drone.velocity
            
            if not isinstance(self.behavior, BoidsBehavior):
                self.handle_wall_collisions(drone)
        
        self.handle_drone_collisions()

        if not self.drones:
            return {} 

        positions = np.array([d.position for d in self.drones])
        velocities = np.array([d.velocity for d in self.drones])

        com = np.mean(positions, axis=0)

        speeds_sq = np.sum(velocities**2, axis=1)
        kinetic_energy = 0.5 * config.DRONE_MASS * np.sum(speeds_sq)
        
        spatial_disorder = np.mean(np.linalg.norm(positions - com, axis=1))

        velocity_disorder = np.mean(np.std(velocities, axis=0))

        return {
            "kinetic_energy": kinetic_energy,
            "spatial_disorder": spatial_disorder,
            "velocity_disorder": velocity_disorder
        }

    def draw(self, screen):
        for drone in self.drones:
            drone.draw(screen)
