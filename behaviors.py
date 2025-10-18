import pygame
import numpy as np
import itertools
import config

class Behavior:
    def update(self, drones, *args):
        raise NotImplementedError("You must implement the 'update' method")

class GasIdealBehavior(Behavior):
    def update(self, drones):
        for drone in drones:
            drone.position += drone.velocity
            self.check_wall_collision(drone)
        self.handle_drone_collisions(drones)

    def check_wall_collision(self, drone):
        if drone.position[0] <= drone.radius or drone.position[0] >= config.SCREEN_WIDTH - drone.radius:
            drone.velocity[0] *= -1
            drone.position[0] = np.clip(drone.position[0], drone.radius, config.SCREEN_WIDTH - drone.radius)
        
        if drone.position[1] <= drone.radius or drone.position[1] >= config.SCREEN_HEIGHT - drone.radius:
            drone.velocity[1] *= -1
            drone.position[1] = np.clip(drone.position[1], drone.radius, config.SCREEN_HEIGHT - drone.radius)

    def handle_drone_collisions(self, drones):
        for drone_a, drone_b in itertools.combinations(drones, 2):
            dist_vec = drone_a.position - drone_b.position
            dist_mag = np.linalg.norm(dist_vec)
            min_dist = drone_a.radius + drone_b.radius

            if dist_mag < min_dist and dist_mag > 0:
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

            drone.position += drone.velocity
            self.check_wall_collision(drone)

    def check_wall_collision(self, drone):
        if drone.position[0] <= drone.radius or drone.position[0] >= config.SCREEN_WIDTH - drone.radius:
            drone.velocity[0] *= -0.5 
            drone.position[0] = np.clip(drone.position[0], drone.radius, config.SCREEN_WIDTH - drone.radius)
        
        if drone.position[1] <= drone.radius or drone.position[1] >= config.SCREEN_HEIGHT - drone.radius:
            drone.velocity[1] *= -0.5 
            drone.position[1] = np.clip(drone.position[1], drone.radius, config.SCREEN_HEIGHT - drone.radius)
