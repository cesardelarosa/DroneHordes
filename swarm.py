import pygame
import numpy as np
import random
import config

class Drone:
    def __init__(self):
        self.position = np.array([
            random.uniform(config.DRONE_RADIUS, config.SCREEN_WIDTH - config.DRONE_RADIUS),
            random.uniform(config.DRONE_RADIUS, config.SCREEN_HEIGHT - config.DRONE_RADIUS)
        ], dtype=np.float64)
        
        angle = random.uniform(0, 2 * np.pi)
        self.velocity = np.array([np.cos(angle), np.sin(angle)], dtype=np.float64) * config.DRONE_SPEED

    def update(self):
        self.position += self.velocity
        
        if self.position[0] <= config.DRONE_RADIUS or self.position[0] >= config.SCREEN_WIDTH - config.DRONE_RADIUS:
            self.velocity[0] *= -1
        
        if self.position[1] <= config.DRONE_RADIUS or self.position[1] >= config.SCREEN_HEIGHT - config.DRONE_RADIUS:
            self.velocity[1] *= -1

    def draw(self, screen):
        pygame.draw.circle(
            screen, 
            config.COLOR_DRONE, 
            self.position.astype(int), 
            config.DRONE_RADIUS
        )
