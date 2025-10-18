import pygame
import sys
import argparse
import config
from swarm import Swarm
from behaviors import GasIdealBehavior, FollowMouseBehavior, BoidsBehavior, DirectedFlockingBehavior

def main(drone_count):
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    clock = pygame.time.Clock()

    swarm = Swarm(n=drone_count)
    
    behaviors = {
        '1': ('Ideal Gas', GasIdealBehavior()),
        '2': ('Follow Mouse', FollowMouseBehavior()),
        '3': ('Boids Flocking', BoidsBehavior()),
        '4': ('Directed Flocking', DirectedFlockingBehavior())
    }
    
    current_mode_key = '1'
    current_mode_name, current_behavior = behaviors[current_mode_key]
    swarm.set_behavior(current_behavior)
    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: {current_mode_name}")

    terminal_width = 80
    
    banner = r"""             _______  .______        ______   .__   __.  _______         
            |       \ |   _  \      /  __  \  |  \ |  | |   ____|        
            |  .--.  ||  |_)  |    |  |  |  | |   \|  | |  |__           
            |  |  |  ||      /     |  |  |  | |  . `  | |   __|          
            |  '--'  ||  |\  \----.|  `--'  | |  |\   | |  |____         
            |_______/ | _| `._____| \______/  |__| \__| |_______|        
       __    __    ______   .______       _______   _______      _______.
      |  |  |  |  /  __  \  |   _  \     |       \ |   ____|    /       |
      |  |__|  | |  |  |  | |  |_)  |    |  .--.  ||  |__      |   (----`
      |   __   | |  |  |  | |      /     |  |  |  ||   __|      \   \    
      |  |  |  | |  `--'  | |  |\  \----.|  '--'  ||  |____ .----)   |   
      |__|  |__|  \______/  | _| `._____||_______/ |_______||_______/    
                                                                         """
    print(banner)
    
    num_ui_lines = 11
    sys.stdout.write("\n" * num_ui_lines)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    running = False
                
                key_pressed = None
                if event.key == pygame.K_1:
                    key_pressed = '1'
                elif event.key == pygame.K_2:
                    key_pressed = '2'
                elif event.key == pygame.K_3:
                    key_pressed = '3'
                elif event.key == pygame.K_4:
                    key_pressed = '4'
                
                if key_pressed and key_pressed in behaviors:
                    current_mode_key = key_pressed
                    current_mode_name, current_behavior = behaviors[current_mode_key]
                    swarm.set_behavior(current_behavior)
                    
                    if current_mode_key == '1':
                        swarm.reset_drone_velocities(speed=config.DRONE_SPEED)
                    elif current_mode_key == '2':
                        swarm.reset_drone_velocities(speed=0)
                    elif current_mode_key == '3' or current_mode_key == '4':
                        swarm.reset_drone_velocities(speed=config.BOIDS_MAX_SPEED)
                    
                    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: {current_mode_name}")
                
                else:
                    swarm.behavior.handle_input(event.key)
        
        stats = swarm.update()

        screen.fill(config.COLOR_BACKGROUND)
        swarm.draw(screen)
        pygame.display.flip()

        if stats:
            sys.stdout.write(f"\x1b[{num_ui_lines}A")
            
            sys.stdout.write(f"\x1b[2K{'--- MODES ---'.center(terminal_width)}\n")
            for key, (name, behavior) in behaviors.items():
                line_str = f"({key}) {name}"
                if key == current_mode_key:
                    prefix = "\x1b[1m\x1b[32m-> \x1b[0m"
                    sys.stdout.write(f"\x1b[2K {prefix}\x1b[1m{line_str}\x1b[0m\n")
                else:
                    prefix = "   "
                    sys.stdout.write(f"\x1b[2K {prefix}{line_str}\n")
            
            controls_str = swarm.behavior.get_controls_status()
            params_str = swarm.behavior.get_params_status()
            physics_str = (
                f"K.E.: {stats['kinetic_energy']:.1f} | "
                f"Spatial D: {stats['spatial_disorder']:.1f} | "
                f"Vel. D: {stats['velocity_disorder']:.1f}"
            )

            sys.stdout.write(f"\x1b[2K{'--- CONTROLS ---'.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{controls_str.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{'--- PARAMS ---'.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{params_str.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{'--- PHYSICS ---'.center(terminal_width)}\n")
            sys.stdout.write(f"\x1b[2K{physics_str.center(terminal_width)}\n")
            sys.stdout.flush()

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
