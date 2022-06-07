import sys
from simulation.Simulation import Simulation
from utils.Resolution import Resolution

if __name__ == "__main__":
    argc = len(sys.argv)
    load_data_file = None
    if argc > 1:
        load_data_file = sys.argv[1]
    #screen_resolution = Resolution(1920, 1024)
    screen_resolution = Resolution(1024, 720)

    attraction_strength = (screen_resolution.height**2 +
                           screen_resolution.width**2) /4 # / 100
    attraction_point = (screen_resolution.width / 2,
                        screen_resolution.height / 2, attraction_strength)

    Simulation(screen_resolution, 40, 55, 2, True, attraction_point,
               10, load_data_file).run()  #change clock tick to 100
# Simulation(screen_resolution, 100, 44, 2, True, attraction_point, 8).run()
