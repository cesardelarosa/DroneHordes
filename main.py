import pygame
import sys
import argparse
import config
from swarm import Swarm
from behaviors import GasIdealBehavior, FollowMouseBehavior, BoidsBehavior

def main(drone_count):
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    clock = pygame.time.Clock()

    swarm = Swarm(n=drone_count)
    
    behaviors = {
        'Ideal Gas': GasIdealBehavior(),
        'Follow Mouse': FollowMouseBehavior(),
        'Boids Flocking': BoidsBehavior()
    }
    current_mode = 'Ideal Gas'
    swarm.set_behavior(behaviors[current_mode])
    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: {current_mode}")

    terminal_width = 80
    
    sys.stdout.write("\n" * 4)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    running = False
                
                elif event.key == pygame.K_1:
                    current_mode = 'Ideal Gas'
                    swarm.set_behavior(behaviors[current_mode])
                    swarm.reset_drone_velocities(speed=config.DRONE_SPEED)
                
                elif event.key == pygame.K_2:
                    current_mode = 'Follow Mouse'
                    swarm.set_behavior(behaviors[current_mode])
                    swarm.reset_drone_velocities(speed=0)
                
                elif event.key == pygame.K_3:
                    current_mode = 'Boids Flocking'
                    swarm.set_behavior(behaviors[current_mode])
                    swarm.reset_drone_velocities(speed=config.BOIDS_MAX_SPEED)
                
                else:
                    swarm.behavior.handle_input(event.key)
                
                pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: {current_mode}")

        
        stats = swarm.update()

        screen.fill(config.COLOR_BACKGROUND)
        swarm.draw(screen)
        pygame.display.flip()

        if stats:
            sys.stdout.write("\x1b[4A")

            mode_str = f"--- MODE: {current_mode} ---"
            controls_str = swarm.behavior.get_controls_status()
            params_str = swarm.behavior.get_params_status()
            physics_str = (
                f"K.E.: {stats['kinetic_energy']:.1f} | "
                f"Spatial D: {stats['spatial_disorder']:.1f} | "
                f"Vel. D: {stats['velocity_disorder']:.1f}"
            )

            sys.stdout.write(f"\x1b[2K{mode_str.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{controls_str.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{params_str.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{physics_str.center(terminal_width)}\n")
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
