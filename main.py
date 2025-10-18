import pygame
import sys
import argparse
import config
from swarm import Swarm
from behaviors import GasIdealBehavior, FollowMouseBehavior

def main(drone_count):
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    clock = pygame.time.Clock()

    swarm = Swarm(n=drone_count)
    
    behaviors = {
        'Ideal Gas': GasIdealBehavior(),
        'Follow Mouse': FollowMouseBehavior()
    }
    current_mode = 'Ideal Gas'
    swarm.set_behavior(behaviors[current_mode])
    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: {current_mode}")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    running = False
                
                if event.key == pygame.K_1:
                    current_mode = 'Ideal Gas'
                    swarm.set_behavior(behaviors[current_mode])
                    swarm.reset_drone_velocities(speed=config.DRONE_SPEED)
                
                if event.key == pygame.K_2:
                    current_mode = 'Follow Mouse'
                    swarm.set_behavior(behaviors[current_mode])
                    swarm.reset_drone_velocities(speed=0)
                
                pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: {current_mode}")

        
        stats = swarm.update()

        screen.fill(config.COLOR_BACKGROUND)
        swarm.draw(screen)
        pygame.display.flip()

        if stats:
            stats_str = (
                f"\x1b[2K\rMODE: {current_mode.ljust(15)} | "
                f"K.E.: {stats['kinetic_energy']:.1f} | "
                f"Spatial D: {stats['spatial_disorder']:.1f} | "
                f"Vel. D: {stats['velocity_disorder']:.1f}"
            )
            sys.stdout.write(stats_str)
            sys.stdout.flush()

        clock.tick(config.FPS)

    pygame.quit()
    print()
    sys.exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone swarm simulation.")
    parser.add_argument(
        '-n', '--count', 
        type=int, 
        default=config.DEFAULT_DRONE_COUNT, 
        help='Number of drones in the simulation.'
    )
    args = parser.parse_args()
    
    main(drone_count=args.count)
