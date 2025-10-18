import pygame
import numpy as np
import random
import itertools
import config

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

    def update_movement(self):
        self.position += self.velocity

    def check_wall_collision(self):
        if self.position[0] <= self.radius or self.position[0] >= config.SCREEN_WIDTH - self.radius:
            self.velocity[0] *= -1
            self.position[0] = np.clip(self.position[0], self.radius, config.SCREEN_WIDTH - self.radius)
        
        if self.position[1] <= self.radius or self.position[1] >= config.SCREEN_HEIGHT - self.radius:
            self.velocity[1] *= -1
            self.position[1] = np.clip(self.position[1], self.radius, config.SCREEN_HEIGHT - self.radius)

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

    def handle_drone_collisions(self):
        for drone_a, drone_b in itertools.combinations(self.drones, 2):
            dist_vec = drone_a.position - drone_b.position
            dist_mag = np.linalg.norm(dist_vec)
            min_dist = drone_a.radius + drone_b.radius

            if dist_mag < min_dist:
                normal = dist_vec / dist_mag
                tangent = np.array([-normal[1], normal[0]])
                
                overlap = min_dist - dist_mag
                drone_a.position += normal * overlap / 2
                drone_b.position -= normal * overlap / 2

                v1n = np.dot(drone_a.velocity, normal)
                v1t = np.dot(drone_a.velocity, tangent)
                v2n = np.dot(drone_b.velocity, normal)
                v2t = np.dot(drone_b.velocity, tangent)

                v1n_new = (v1n * (drone_a.mass - drone_b.mass) + 2 * drone_b.mass * v2n) / (drone_a.mass + drone_b.mass)
                v2n_new = (v2n * (drone_b.mass - drone_a.mass) + 2 * drone_a.mass * v1n) / (drone_a.mass + drone_b.mass)

                v1_new_vec = v1n_new * normal + v1t * tangent
                v2_new_vec = v2n_new * normal + v2t * tangent

                drone_a.velocity = v1_new_vec
                drone_b.velocity = v2_new_vec

    def update(self):
        for drone in self.drones:
            drone.update_movement()
            drone.check_wall_collision()
        self.handle_drone_collisions()

    def draw(self, screen):
        for drone in self.drones:
            drone.draw(screen)
